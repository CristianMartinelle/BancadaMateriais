/* =====================================================================
   VERSÃO C.2.2: CORRIGIDA NA AUDITORIA TÉCNICA (NÃO TESTADA EM BANCADA)
   Microcontrolador: Arduino Uno Rev3 | Driver A4988 | Motor 17HS4401
   Correções: MAX_TRAVEL_MM = 30.0 mm | CAL_DEFAULT = 4300.0 counts/N
   Exigências de Hardware: Pull-up externo no ENABLE e MS1..MS3 em 5V
   ===================================================================== */
#include <EEPROM.h>

const uint8_t PIN_STEP = 2; // Pulso de passo A4988
const uint8_t PIN_DIR  = 3; // Sentido de giro A4988
const uint8_t PIN_EN   = 4; // Habilitação do driver (LOW = Ativo, HIGH = Livre)
const uint8_t PIN_DOUT = 12; // Dados seriais HX711
const uint8_t PIN_SCK  = 13; // Clock serial HX711

const float STEPS_PER_REV = 200.0; // NEMA 17: 1,8 graus por passo
const uint16_t MICROSTEPS = 16;    // Exige pinos MS1=MS2=MS3 conectados a 5 V na PCB
const float LEAD_MM       = 2.0;   // Fuso trapezoidal TR8 (passo 2,0 mm)
const float STEPS_PER_MM  = (STEPS_PER_REV * (float)MICROSTEPS) / LEAD_MM; // 1600 passos/mm
const uint8_t TENSION_DIR = HIGH;  // Sentido de tração

float speedMMmin          = 5.0;   // Velocidade de ensaio padrão (mm/min)
const float SPEED_MIN     = 0.5;   const float SPEED_MAX = 30.0; const float SPEED_STEP = 0.5;
const float DROP_FRACTION = 0.50;  // Parada na queda de 50% do pico de força
const uint8_t DROP_SAMPLES= 2;     // Amostras consecutivas para confirmação de ruptura
const float MIN_PEAK_N    = 2.0;   // Limiar mínimo para considerar ativa a detecção de ruptura
const float MAX_FORCE_N   = 200.0; // Limite de corte por sobrecarga por software (N)
const float MAX_TRAVEL_MM = 30.0;  // Curso máximo seguro corrigido (bloqueio mecânico a ~30 mm)
const float FORCE_ALPHA   = 0.40;  // Constante do filtro exponencial
const bool AUTO_TARE_ON_START = true;

const float CAL_DEFAULT   = 4300.0;// Ganho analítico corrigido (V_AVDD = 4,3 V, canal A ganho 128)
const uint16_t EEPROM_MAGIC = 0x5A71; const int EEPROM_ADDR_MAGIC = 0; const int EEPROM_ADDR_CAL = 2;

enum State : uint8_t { ST_IDLE, ST_RUN, ST_DONE };
enum Reason : uint8_t { R_NONE, R_BREAK, R_OVERFORCE, R_TRAVEL, R_MANUAL, R_NOSENSOR };
State state = ST_IDLE; uint8_t stopReason = R_NONE;

float countsPerN = CAL_DEFAULT; long offsetCounts = 0; float forceSign = 1.0; bool calValid = false;
float forceN = 0.0; float forceFilt = 0.0; float peakN = 0.0; float travelMM = 0.0; uint8_t dropCount = 0;
unsigned long stepCount = 0; unsigned long stepIntervalUs = 0; unsigned long lastStepUs = 0;
bool stepPinHigh = false; unsigned long testStartMs = 0; unsigned long lastPrintMs = 0;
unsigned long lastIdleMs = 0; unsigned long lastSampleMs = 0; bool csvMode = true;
const unsigned long PRINT_PERIOD_MS = 200; const unsigned long SENSOR_TIMEOUT_MS = 3000;

void hxInit() { pinMode(PIN_DOUT, INPUT); pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW); }

bool hxRead(long &value) {
  if (digitalRead(PIN_DOUT) == HIGH) return false;
  unsigned long v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_SCK, LOW); delayMicroseconds(1);
  }
  digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1); digitalWrite(PIN_SCK, LOW); // 25º pulso: Ganho 128 canal A
  if (v & 0x800000UL) v |= 0xFF000000UL;
  value = (long)v; return true;
}

bool hxWaitReady(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (digitalRead(PIN_DOUT) == HIGH) { if (millis() - t0 >= timeoutMs) return false; delay(1); }
  return true;
}

void motorInit() {
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW); digitalWrite(PIN_DIR, TENSION_DIR); digitalWrite(PIN_EN, HIGH);
}
void motorOn()  { digitalWrite(PIN_EN, LOW); }
void motorOff() { digitalWrite(PIN_EN, HIGH); }

void computeStepInterval() {
  float stepsPerSec = (speedMMmin / 60.0) * STEPS_PER_MM;
  if (stepsPerSec < 1.0) stepsPerSec = 1.0;
  stepIntervalUs = (unsigned long)(1000000.0 / stepsPerSec);
}

void loadCalibration() {
  uint16_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_CAL, countsPerN);
    if (isnan(countsPerN) || countsPerN < 100.0 || countsPerN > 50000.0) {
      countsPerN = CAL_DEFAULT; calValid = false;
    } else { calValid = true; }
  } else { countsPerN = CAL_DEFAULT; calValid = false; }
}

void saveCalibration(float val) {
  countsPerN = val; calValid = true;
  EEPROM.put(EEPROM_ADDR_CAL, countsPerN);
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
}

void doTare() {
  const uint8_t N_TARE = 10; long sum = 0; uint8_t valid = 0;
  for (uint8_t i = 0; i < N_TARE; i++) {
    if (hxWaitReady(150)) { long raw = 0; if (hxRead(raw)) { sum += raw; valid++; } }
    delay(10);
  }
  if (valid > 0) { offsetCounts = sum / valid; forceN = 0.0; forceFilt = 0.0; }
}

void updateForce() {
  long raw = 0;
  if (hxRead(raw)) {
    lastSampleMs = millis();
    long net = raw - offsetCounts;
    float instN = ((float)net / countsPerN) * forceSign;
    forceFilt = (FORCE_ALPHA * instN) + ((1.0 - FORCE_ALPHA) * forceFilt);
    forceN = forceFilt;
    if (state == ST_RUN) {
      if (forceN > peakN) { peakN = forceN; dropCount = 0; }
      if (peakN >= MIN_PEAK_N && forceN <= peakN * (1.0 - DROP_FRACTION)) {
        dropCount++;
        if (dropCount >= DROP_SAMPLES) stopTest(R_BREAK);
      } else { dropCount = 0; }
      if (forceN >= MAX_FORCE_N) stopTest(R_OVERFORCE);
    }
  } else {
    if (state == ST_RUN && (millis() - lastSampleMs > SENSOR_TIMEOUT_MS)) stopTest(R_NOSENSOR);
  }
}

void updateStepper() {
  if (state != ST_RUN) return;
  unsigned long now = micros();
  if (!stepPinHigh && (now - lastStepUs >= stepIntervalUs)) {
    digitalWrite(PIN_STEP, HIGH); stepPinHigh = true; lastStepUs = now;
  } else if (stepPinHigh && (now - lastStepUs >= 5)) {
    digitalWrite(PIN_STEP, LOW); stepPinHigh = false; stepCount++;
    travelMM = (float)stepCount / STEPS_PER_MM;
    if (travelMM >= MAX_TRAVEL_MM) stopTest(R_TRAVEL);
  }
}

void startTest() {
  if (state == ST_RUN) return;
  if (AUTO_TARE_ON_START) doTare();
  peakN = 0.0; dropCount = 0; stepCount = 0; travelMM = 0.0; stopReason = R_NONE;
  motorOn(); computeStepInterval(); lastStepUs = micros(); stepPinHigh = false;
  testStartMs = millis(); lastPrintMs = millis(); lastSampleMs = millis();
  state = ST_RUN;
  Serial.println(F("# INICIO DE ENSAIO DIDATICO"));
  if (csvMode) Serial.println(F("tempo_s,forca_N,pico_N,deslocamento_mm"));
}

void stopTest(uint8_t reason) {
  if (state != ST_RUN) return;
  state = ST_DONE; stopReason = reason; motorOff();
  Serial.print(F("# FIM DE ENSAIO - MOTIVO: "));
  switch (reason) {
    case R_BREAK:     Serial.println(F("RUPTURA_CONFIRMADA")); break;
    case R_OVERFORCE: Serial.println(F("SOBRECARGA_SEGURANCA_200N")); break;
    case R_TRAVEL:    Serial.println(F("CURSO_MAXIMO_30MM")); break;
    case R_MANUAL:    Serial.println(F("PARADA_MANUAL_OPERADOR")); break;
    case R_NOSENSOR:  Serial.println(F("TIMEOUT_SENSOR_HX711")); break;
    default:          Serial.println(F("DESCONHECIDO")); break;
  }
}

void setup() {
  Serial.begin(115200); hxInit(); motorInit(); loadCalibration(); computeStepInterval();
  Serial.println(F("# BANCADA DIDATICA UCDB - FIRMWARE CORRIGIDO C.2.2 PRONTO"));
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') startTest();
    else if (c == 'x' || c == 'X') stopTest(R_MANUAL);
    else if (c == 't' || c == 'T') doTare();
    else if (c == 'm' || c == 'M') { stopTest(R_MANUAL); motorOff(); }
  }
  updateForce(); updateStepper();
}
