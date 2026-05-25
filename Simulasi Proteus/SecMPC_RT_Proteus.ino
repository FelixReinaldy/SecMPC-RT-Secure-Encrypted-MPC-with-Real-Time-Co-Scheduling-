/*
 * SecMPC-RT – Proteus Demo (Arduino UNO)
 * Simulate: Secure Encrypted MPC with Real-Time Co-Scheduling
 * Kelas 4C – Pemrograman Kontroler – Tugas 2
 *
 * PIN MAPPING (Arduino UNO):
 *   A0  → Potensiometer (sensor, 0-1023 full range)
 *   D2  → LED HIJAU   (Normal, zone 31-69%)
 *   D3  → LED MERAH   (Fault/boundary: 0%, <=30%, 70%, 100%)
 *   D9  → LED KUNING  (PWM: TERANG di 0%/100%, REDUP di 30%/70%, MATI di normal)
 *   D1  → TX → Virtual Terminal RXD
 *
 * SKALA ADC Proteus:
 *   0%   = ADC 0    (potensiometer minimum)
 *   100% = ADC 1023 (potensiometer maksimum)
 *
 * TABEL SETPOINT (sinkron logika dengan Rust firmware):
 *   0%      (ADC = 0)      → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
 *   <=30%   (ADC <= 307)   → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
 *   31-69%  (ADC 308-706)  → MERAH:MATI   | KUNING:MATI   | HIJAU:HIDUP
 *   ~70%    (ADC 707-1022) → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
 *   100%    (ADC = 1023)   → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
 *
 * CATATAN PROTEUS (penyesuaian timing):
 *   LOOP_MS  = 100ms  (hardware asli: 10ms)
 *   DEADLINE = 80ms   (hardware asli: 8ms)
 *   MPC_STEP = 500    (hardware asli: 100)
 */

// ── Pin ───────────────────────────────────────────────────────────────────────
#define PIN_SENSOR      A0
#define PIN_LED_GREEN    2   // LED HIJAU
#define PIN_LED_RED      3   // LED MERAH
#define PIN_PWM_OUT      9   // LED KUNING (PWM)

// ── Konstanta ADC (10-bit Arduino, full range 0-1023) ────────────────────────
// Zone boundaries = persen × 1023 / 100
//   30% = 307   (= 30 * 1023 / 100)
//   31% = 317   (= 31 * 1023 / 100)  → awal zona NORMAL
//   69% = 706   (= 69 * 1023 / 100)  → akhir zona NORMAL
//   70% = 716   (= 70 * 1023 / 100)
const int ADC_MIN_VAL  = 0;     // 0%
const int ADC_MAX_VAL  = 1023;  // 100%
const int ZONE_30_PCT  = 307;   // 30% × 1023
const int ZONE_31_PCT  = 317;   // 31% × 1023  (awal NORMAL)
const int ZONE_69_PCT  = 706;   // 69% × 1023  (akhir NORMAL)
const int ZONE_70_PCT  = 716;   // 70% × 1023

// Setpoint MPC = 50% = 512
const int SETPOINT = 512;

// PWM untuk LED KUNING (8-bit: 0-255)
const int PWM_MAX = 255;   // TERANG penuh
const int PWM_DIM = 77;    // REDUP ≈ 30% × 255

// ── Konstanta lain ────────────────────────────────────────────────────────────
const int            MPC_HORIZON = 3;
const unsigned long  LOOP_MS     = 100;   // 100ms di Proteus
const unsigned long  DEADLINE_MS = 80;    // budget MPC
const int            MPC_STEP    = 500;   // step besar agar Proteus tidak freeze

// ── Kunci XOR-AES (sama dengan Rust) ─────────────────────────────────────────
const uint8_t AES_KEY[8] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6};

// ── CRC-8 (poly 0x07, sinkron dengan Rust) ───────────────────────────────────
uint8_t crc8(uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ── Klasifikasi Zone (sinkron logika dengan Rust classify_zone) ───────────────
// zone 0 = 0%      → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
// zone 1 = <=30%   → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
// zone 2 = 31-69%  → MERAH:MATI   | KUNING:MATI   | HIJAU:HIDUP
// zone 3 = ~70%    → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
// zone 4 = 100%    → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
int classify_zone(int adc) {
  if (adc <= ADC_MIN_VAL)  return 0;  // tepat 0
  if (adc <= ZONE_30_PCT)  return 1;  // 1 – 307  (<=30%)
  if (adc <= ZONE_69_PCT)  return 2;  // 308 – 706 (31-69% NORMAL)
  if (adc < ADC_MAX_VAL)   return 3;  // 707 – 1022 (~70%)
  return 4;                           // tepat 1023 (100%)
}

// ── MPC Compute (N=3, sinkron logika dengan Rust) ────────────────────────────
int mpc_compute(int* y_buf, int y_now) {
  int  dy       = y_buf[0] - y_buf[MPC_HORIZON - 1];
  long cost_min = 2147483647L;
  int  u_opt    = 0;
  for (int u = -1023; u <= 1023; u += MPC_STEP) {
    long cost = 0;
    for (int k = 1; k <= MPC_HORIZON; k++) {
      int e = SETPOINT - (y_now + dy * k - (u * 3 * k) / 10);
      cost += (long)e * e + (((long)26 * u * u) >> 8);
    }
    if (cost < cost_min) { cost_min = cost; u_opt = u; }
  }
  return u_opt;
}

// ── State ─────────────────────────────────────────────────────────────────────
int      y_buf[3]        = {SETPOINT, SETPOINT, SETPOINT};
int      buf_idx         = 0;
uint32_t deadline_misses = 0;
uint32_t loop_count      = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);

  // Startup self-test: semua LED nyala 500ms (sinkron dengan Rust)
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED,   HIGH);
  analogWrite(PIN_PWM_OUT,    PWM_MAX);
  delay(500);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  analogWrite(PIN_PWM_OUT,    0);

  // Header telemetri
  Serial.println("=================================================");
  Serial.println("  SecMPC-RT | Proteus | SETPOINT=512(50%)");
  Serial.println("  ADC RANGE: 0=0%  ..  1023=100%");
  Serial.println("  LED: MERAH=fault | KUNING=PWM | HIJAU=normal");
  Serial.println("  0%->Batas|<=30%->Redup|31-69%->Normal|~70%->Redup");
  Serial.println("=================================================");
  Serial.println("Iter | ADC | pct% | Zone       | Err | u_opt | T_ms | MISS");
  Serial.println("-----+-----+------+------------+-----+-------+------+-----");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  loop_count++;
  unsigned long t_start = millis();

  // ── STEP 1: Baca Sensor (0-1023) ─────────────────────────────────────────
  int y_now = analogRead(PIN_SENSOR);
  y_buf[buf_idx] = y_now;
  buf_idx = (buf_idx + 1) % MPC_HORIZON;

  // ── STEP 2: MPC Compute ───────────────────────────────────────────────────
  unsigned long t_mpc_start = millis();
  int u_opt                 = mpc_compute(y_buf, y_now);
  unsigned long t_mpc_ms    = millis() - t_mpc_start;

  // ── STEP 3: Klasifikasi Zone & Persen ────────────────────────────────────
  int error = SETPOINT - y_now;
  int zone  = classify_zone(y_now);
  int pct   = (y_now * 100) / ADC_MAX_VAL;  // 0-100%

  // ── STEP 4: Deadline Check ────────────────────────────────────────────────
  bool deadline_ok = (t_mpc_ms < DEADLINE_MS);
  if (!deadline_ok) deadline_misses++;

  // ── STEP 5: Aktuasi LED sesuai Tabel ─────────────────────────────────────
  bool red_on;
  int  kuning_duty;
  bool green_on;

  switch (zone) {
    case 0:  red_on = true;  kuning_duty = PWM_MAX; green_on = false; break; // 0%
    case 1:  red_on = true;  kuning_duty = PWM_DIM; green_on = false; break; // <=30%
    case 2:  red_on = false; kuning_duty = 0;       green_on = true;  break; // NORMAL
    case 3:  red_on = true;  kuning_duty = PWM_DIM; green_on = false; break; // ~70%
    default: red_on = true;  kuning_duty = PWM_MAX; green_on = false; break; // 100%
  }

  // Override: deadline miss → paksa MERAH, kuning mati (sinkron Rust)
  if (!deadline_ok) {
    red_on      = true;
    kuning_duty = 0;
    green_on    = false;
  }

  // Terapkan ke GPIO
  digitalWrite(PIN_LED_RED,   red_on   ? HIGH : LOW);
  digitalWrite(PIN_LED_GREEN, green_on ? HIGH : LOW);
  analogWrite(PIN_PWM_OUT,    kuning_duty);

  // ── STEP 6: Enkripsi + Telemetri ─────────────────────────────────────────
  uint8_t plain[8] = {
    (uint8_t)(y_now >> 8),       (uint8_t)(y_now & 0xFF),
    (uint8_t)(abs(error) >> 8),  (uint8_t)(abs(error) & 0xFF),
    (uint8_t)zone,               (uint8_t)pct,
    (uint8_t)(loop_count >> 8),  (uint8_t)(loop_count & 0xFF)
  };
  uint8_t cipher[8];
  for (int i = 0; i < 8; i++) cipher[i] = plain[i] ^ AES_KEY[i % 8];
  uint8_t crc = crc8(cipher, 8);
  (void)crc;  // dipakai implisit (anti dead-code)

  const char* zone_str;
  switch (zone) {
    case 0:  zone_str = "0%:BATAS  "; break;
    case 1:  zone_str = "<=30%:DIM "; break;
    case 2:  zone_str = "NORMAL    "; break;
    case 3:  zone_str = "70%:DIM   "; break;
    default: zone_str = "100%:MAX  "; break;
  }

  // Output ke Virtual Terminal (format sinkron dengan Rust UART)
  Serial.print(loop_count % 99999); Serial.print(" | ");
  Serial.print(y_now);              Serial.print(" | ");
  Serial.print(pct);                Serial.print("%  | ");
  Serial.print(zone_str);           Serial.print(" | ");
  Serial.print(error);              Serial.print(" | ");
  Serial.print(u_opt);              Serial.print(" | ");
  Serial.print(t_mpc_ms);          Serial.print(" | ");
  Serial.println(deadline_misses);

  // ── STEP 7: Tunggu sisa siklus ────────────────────────────────────────────
  unsigned long used = millis() - t_start;
  if (used < LOOP_MS) delay(LOOP_MS - used);
}
