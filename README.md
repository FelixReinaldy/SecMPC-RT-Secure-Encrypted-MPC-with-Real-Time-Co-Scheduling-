# 🔐 SecMPC-RT — Secure Encrypted MPC with Real-Time Co-Scheduling

> **Mata Kuliah:** Pemrograman Kontroler — Kelas 4C  
> **Platform:** ESP32-S3 | **Bahasa:** Rust (`no_std`, `esp-hal 1.1`)  
> **Metode:** Model Predictive Control + XOR-AES Encryption + Real-Time Co-Scheduling

---

## 📋 Daftar Isi

- [Gambaran Umum](#-gambaran-umum)
- [Arsitektur Sistem](#-arsitektur-sistem)
- [Metode SecMPC-RT](#-metode-secmpc-rt)
- [Hardware & Pin Mapping](#-hardware--pin-mapping)
- [Tabel Setpoint & Zona Kontrol](#-tabel-setpoint--zona-kontrol)
- [Algoritma MPC](#-algoritma-mpc)
- [Enkripsi XOR-AES + CRC-8](#-enkripsi-xor-aes--crc-8)
- [Real-Time Co-Scheduling](#-real-time-co-scheduling)
- [Struktur Kode](#-struktur-kode)
- [Cara Build & Flash](#-cara-build--flash)
- [Simulasi Proteus](#-simulasi-proteus)
- [Simulasi Data & Visualisasi GNUplot](#-simulasi-data--visualisasi-gnuplot)
- [Hasil Grafik](#-hasil-grafik)
- [Struktur File Repository](#-struktur-file-repository)
- [Dependensi](#-dependensi)

---

## 🎯 Gambaran Umum

**SecMPC-RT** adalah implementasi sistem kontrol *embedded* berbasis **ESP32-S3** yang mengintegrasikan tiga komponen utama dalam satu loop kontrol real-time berperiode **10ms**:

| Komponen | Deskripsi |
|---|---|
| **MPC (Model Predictive Control)** | Algoritma kontrol prediktif horizon N=3 dengan grid search |
| **Secure Encryption** | Simulasi enkripsi AES-128 via XOR-cipher dengan CRC-8 integrity check |
| **Real-Time Co-Scheduling** | Pembagian budget waktu antar tugas dalam satu loop deadline-driven |

Sistem membaca sensor potensiometer melalui ADC, mengklasifikasikan zona operasi, menggerakkan aktuator (LED PWM), dan mengirimkan telemetri terenkripsi melalui UART — semua dalam satu siklus 10ms.

---

## 🏗️ Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESP32-S3 — Loop 10ms                         │
│                                                                  │
│  [GPIO1/ADC]──→  STEP 1: Baca ADC (12-bit, 1983-4095)          │
│                        ↓                                         │
│                  STEP 2: MPC Compute (N=3, ≤8ms deadline)       │
│                        ↓                                         │
│                  STEP 3: Klasifikasi Zona & Hitung Error        │
│                        ↓                                         │
│                  STEP 4: Deadline Check                          │
│                        ↓                                         │
│  [GPIO2]  ←──   STEP 5: Aktuasi LED (Merah/Hijau/Kuning PWM)   │
│  [GPIO15] ←──         ↓                                         │
│  [GPIO16] ←──   STEP 6: XOR-Encrypt + CRC-8 + UART Telemetri   │
│  [GPIO43] ←──         ↓                                         │
│                  STEP 7: Tunggu sisa waktu hingga 10ms           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🧠 Metode SecMPC-RT

SecMPC-RT (**Secure** Encrypted **M**odel **P**redictive **C**ontrol with **R**eal-**T**ime Co-Scheduling) adalah metode baru yang menggabungkan:

### 1. Model Predictive Control (MPC)
Menggunakan prediksi jangka pendek (horizon N=3 iterasi ke depan) untuk menentukan sinyal kontrol optimal yang meminimalkan fungsi cost:

$$J = \sum_{k=1}^{N} \left[ (r - \hat{y}_k)^2 + \lambda \cdot u^2 \right]$$

Di mana:
- `r` = setpoint (ADC 3039 = 50%)
- `ŷₖ` = prediksi output ADC k langkah ke depan
- `u` = sinyal kontrol (dicari via grid search −4095..+4095, step 100)
- `λ = 26/256` = faktor regularisasi (lambda Q8 fixed-point)

### 2. Secure Encryption (XOR-AES Simulation)
Setiap paket telemetri 8-byte dienkripsi dengan kunci 128-bit sebelum dikirim via UART. Menggunakan pendekatan XOR-cipher yang mensimulasikan perilaku AES-128 dalam environment embedded `no_std`.

### 3. Real-Time Co-Scheduling
Alokasi budget waktu dalam satu loop 10ms:

```
|─── MPC Compute ≤8ms ───|── Encrypt+UART 1.5ms ──|── Idle ──|
0                         8                        9.5       10ms
```

Jika MPC melebihi deadline 8ms → Deadline Miss flag aktif → LED override ke MERAH.

---

## ⚡ Hardware & Pin Mapping

| GPIO | Fungsi | Keterangan |
|------|--------|-----------|
| `GPIO1` | ADC1_CH0 (Input) | Potensiometer sensor (12-bit, Attenuation 11dB) |
| `GPIO2` | Digital Output | **LED HIJAU** — aktif saat zona NORMAL (31–69%) |
| `GPIO15` | Digital Output | **LED MERAH** — aktif saat anomali / fault |
| `GPIO16` | LEDC PWM Output | **LED KUNING** — brightness = PWM duty (1kHz, 10-bit) |
| `GPIO43` | UART0 TX | Telemetri serial 115200 baud |
| `GPIO44` | UART0 RX | (unused, required by driver) |

**Startup Self-Test:** Semua LED menyala 500ms saat boot, lalu mati sebelum loop dimulai.

---

## 📊 Tabel Setpoint & Zona Kontrol

Skala ADC efektif: **1983 = 0%** hingga **4095 = 100%** (range = 2112 count)

| Zona | Range % | Range ADC | LED MERAH | LED KUNING | LED HIJAU |
|------|---------|-----------|-----------|------------|-----------|
| **Batas Bawah** | 0% | ADC ≤ 1999 | 🔴 TERANG | 🟡 TERANG | ⚫ MATI |
| **Anomali Bawah** | 1–30% | ADC 2000–2617 | 🔴 TERANG | 🟡 REDUP (30%) | ⚫ MATI |
| **⭐ NORMAL** | **31–69%** | **ADC 2638–3440** | ⚫ MATI | ⚫ MATI | 🟢 HIDUP |
| **Anomali Atas** | 70–99% | ADC 3461–4094 | 🔴 TERANG | 🟡 REDUP (30%) | ⚫ MATI |
| **Batas Atas** | 100% | ADC = 4095 | 🔴 TERANG | 🟡 TERANG | ⚫ MATI |

**Setpoint:** `SP = 3039` (tepat 50% dari range efektif)  
**Ambang Anomali:** `ANOMALY_THRESH = SP − ZONE_31 = 3039 − 2638 = 401`

> **Override:** Jika terjadi **Deadline Miss** → LED Merah PAKSA menyala, Kuning mati.

---

## 🔢 Algoritma MPC

```rust
// Konstanta MPC
const MPC_HORIZON: usize = 3;    // N = 3 langkah prediksi
const MPC_STEP:    i32   = 100;  // resolusi grid search
const SETPOINT:    i32   = 3039; // target ADC (50%)

fn mpc_compute(y_buf: &[i32; 3], y_now: i32) -> i32 {
    let dy = y_buf[0] - y_buf[2];   // estimasi trend (Δy)
    const LAMBDA_Q8: i64 = 26;      // λ = 26/256 ≈ 0.1016

    let mut cost_min = i64::MAX;
    let mut u_opt = 0i32;

    // Grid search: u ∈ {-4095, -3995, ..., +3995, +4095}
    let mut u = -4095i32;
    while u <= 4095 {
        let mut cost = 0i64;
        for k in 1..=3 {
            // Model prediksi linier: ŷₖ = y + dy·k - u·0.3·k
            let y_pred = y_now + dy*k - u*3*k/10;
            let e = (SETPOINT - y_pred) as i64;
            cost += e*e + (LAMBDA_Q8 * (u as i64).pow(2)) >> 8;
        }
        if cost < cost_min { cost_min = cost; u_opt = u; }
        u += MPC_STEP;
    }
    u_opt  // sinyal kontrol optimal
}
```

**Kompleksitas:** Grid search 82 titik × 3 prediksi = **246 evaluasi per siklus** (≤1.5ms pada ESP32-S3 @ 240MHz).

---

## 🔐 Enkripsi XOR-AES + CRC-8

### Format Paket Telemetri (8 byte plain-text)

| Byte | Isi |
|------|-----|
| 0–1 | ADC raw (big-endian uint16) |
| 2–3 | \|Error\| (big-endian) |
| 4 | Zone (0–4) |
| 5 | Persentase ADC (0–100%) |
| 6–7 | Loop counter (big-endian) |

### Kunci AES-128 (simulasi XOR)
```rust
const AES_KEY: [u8; 16] = [
    0x2B, 0x7E, 0x15, 0x16,  0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88,  0x09, 0xCF, 0x4F, 0x3C,
];
// cipher[i] = plain[i] ^ AES_KEY[i]  (untuk i = 0..7)
```

### CRC-8 Integrity Check (Polynomial 0x07)
```rust
fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0x00u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}
```

---

## ⏱️ Real-Time Co-Scheduling

```
Siklus Loop = 10,000 µs (10ms)
├── STEP 1: ADC Read           ~50 µs
├── STEP 2: MPC Compute     ≤8,000 µs  ← DEADLINE (MPC_DEADLINE_US)
├── STEP 3: Zone Classify      ~10 µs
├── STEP 4: Deadline Check      ~5 µs
├── STEP 5: LED Actuation      ~20 µs
├── STEP 6: Encrypt + UART  ≤1,500 µs  ← BUDGET (ENC_BUDGET_US)
└── STEP 7: Delay (sisa waktu)
```

**Budget Check sebelum enkripsi:**
```rust
if elapsed_us + ENC_BUDGET_US < LOOP_US {
    // aman: jalankan enkripsi + UART
}
```
Jika waktunya tidak cukup, enkripsi di-skip untuk menjaga ketepatan periode loop.

---

## 📁 Struktur Kode

```
Rust Program/
├── src/
│   └── main.rs          ← Seluruh firmware (342 baris)
│       ├── crc8()          CRC-8 polynomial 0x07
│       ├── encrypt_block() XOR-cipher simulasi AES-128
│       ├── classify_zone() Klasifikasi 5 zona kontrol
│       ├── mpc_compute()   MPC N=3 grid search
│       └── main()          Entry point + control loop
├── Cargo.toml           ← Dependensi Rust
├── rust-toolchain.toml  ← Target: riscv32imac-esp-espidf
├── wokwi.toml           ← Konfigurasi simulasi Wokwi
└── .cargo/
    └── config.toml      ← ESP32-S3 build target & runner
```

### Konstanta Utama (`main.rs`)

```rust
const ADC_MIN:         i32 = 1983;   // 0%
const ADC_MAX:         i32 = 4095;   // 100%
const ADC_NOISE_FLOOR: i32 = 1999;   // dead zone noise floor
const ZONE_30_PCT:     i32 = 2617;   // batas atas 1-30%
const ZONE_31_PCT:     i32 = 2638;   // batas bawah NORMAL
const ZONE_69_PCT:     i32 = 3440;   // batas atas NORMAL
const ZONE_70_PCT:     i32 = 3461;   // batas bawah 70-99%
const SETPOINT:        i32 = 3039;   // SP = 50%
const LOOP_US:         u32 = 10_000; // periode loop 10ms
const MPC_DEADLINE_US: u32 = 8_000;  // deadline MPC 8ms
const ENC_BUDGET_US:   u32 = 1_500;  // budget enkripsi 1.5ms
const PWM_MAX:         u32 = 1023;   // duty 10-bit max (TERANG)
const PWM_DIM:         u32 = 307;    // ~30% duty (REDUP)
```

---

## 🔨 Cara Build & Flash

### Prasyarat

```bash
# Install Rust toolchain untuk ESP32-S3
rustup toolchain install nightly
rustup target add xtensa-esp32s3-none-elf

# Install espflash
cargo install espflash cargo-espflash

# Install esp-idf toolchain (opsional, untuk debugging)
cargo install ldproxy
```

### Build

```bash
cd "Rust Program"

# Build debug
cargo build

# Build release (dioptimasi ukuran)
cargo build --release
```

### Flash ke ESP32-S3

```bash
# Flash + monitor serial
cargo espflash flash --release --monitor

# Atau manual:
espflash flash target/xtensa-esp32s3-none-elf/release/secmpc-rt --monitor --baud 115200
```

### Monitor Telemetri Serial (115200 baud)

Output di terminal setelah flashing:
```
╔══════════════════════════════════════════════════════════════╗
║  SecMPC-RT | ESP32-S3 | SETPOINT=3039(50%) | 10ms Loop    ║
║  ADC RANGE: 1983=0%  ..  4095=100%  (range=2112)           ║
╚══════════════════════════════════════════════════════════════╝
 Iter |  ADC  |  pct%  | Zone      |  Error | u_opt | T_us | MISS
------+-------+--------+-----------+--------+-------+------+------
    1 |  2150 |   7%   | <=30%:DIM |    889 |   300 |  892 |    0
   42 |  3039 |  49%   | NORMAL    |      0 |     0 |  874 |    0
```

### Simulasi Online (Wokwi)

Buka file `wokwi.toml` dan jalankan di [wokwi.com](https://wokwi.com) atau gunakan ekstensi VS Code Wokwi untuk simulasi tanpa hardware fisik.

---

## 🔌 Simulasi Proteus

Rangkaian simulasi Proteus tersedia di folder `Simulasi Proteus/`.

**Komponen Utama:**
- Arduino Uno (sebagai emulator logic ESP32 untuk validasi algoritma)
- Potensiometer 10kΩ (sensor input ADC)
- 3× LED (Merah, Hijau, Kuning) + resistor 220Ω
- Virtual Terminal (monitoring UART output)

**Wiring:**
| Arduino Pin | Komponen |
|-------------|---------|
| A0 | Potensiometer (ADC input) |
| D2 | LED Hijau |
| D15 | LED Merah |
| D16 | LED Kuning (PWM) |
| TX | Virtual Terminal |

> 📌 Lihat `Wiring_Proteus_SecMPC_RT.png` untuk diagram lengkap.

---

## 📈 Simulasi Data & Visualisasi GNUplot

### Generate Data Simulasi

```bash
# Pastikan Python 3 terinstall
python generate_csv.py
```

Script `generate_csv.py` menghasilkan `data_simulasi.csv` dengan:
- **300 iterasi** × 10ms = **3 detik** simulasi
- ADC range **1983–4095** (sesuai ESP32-S3 Rust firmware)
- Skenario: anomali bawah → konvergen ke SP → anomali atas → kembali → anomali bawah
- Noise ADC ±30 count (realistis hardware)
- ~1.5% deadline miss rate (acak)

### Format CSV

```
k, t_ms, ADC, Volt_V, Error, U_MPC, PWM, t_MPC_ms, Deadline_Miss, Anomaly
1, 10, 2134, 1.721, 905, 300, 120, 0.892, 0, 1
...
```

| Kolom | Satuan | Keterangan |
|-------|--------|-----------|
| `k` | — | Nomor iterasi |
| `t_ms` | ms | Timestamp simulasi |
| `ADC` | count (1983–4095) | Nilai ADC raw |
| `Volt_V` | Volt | Tegangan (ADC × 3.3 / 4095) |
| `Error` | count | SP − ADC = 3039 − ADC |
| `U_MPC` | count | Sinyal kontrol MPC output |
| `PWM` | 0–255 | Duty cycle aktuator LED Kuning |
| `t_MPC_ms` | ms | Waktu eksekusi MPC |
| `Deadline_Miss` | 0/1 | 1 jika t_MPC > 8ms |
| `Anomaly` | 0/1 | 1 jika zone ≠ NORMAL (31–69%) |

### Generate Grafik GNUplot

**Cara 1 — Melalui GNUplot interaktif:**
```gnuplot
cd "C:/path/ke/folder/ETS PEMKON"
load "plot_secmpc_v2.gp"
```

**Cara 2 — Melalui PowerShell:**
```powershell
cd "C:\Semester 4\Pemrograman Kontroler\ETS PEMKON"
& "C:\Program Files\gnuplot\bin\gnuplot.exe" plot_secmpc_v2.gp
```

---

## 📊 Hasil Grafik

Script GNUplot menghasilkan **6 file PNG**:

| File | Judul | Deskripsi |
|------|-------|-----------|
| `plot_1_adc_vs_setpoint.png` | ADC Sensor vs Setpoint | Nilai ADC (1983–4095) vs garis SP=3039 + batas zona |
| `plot_2_error.png` | Error Kontrol | Error = 3039 − ADC, dengan batas anomali ±401 |
| `plot_3_umpc_pwm.png` | Output MPC & PWM | Sinyal kontrol U_MPC dan duty PWM aktuator |
| `plot_4_mpc_time.png` | Waktu Komputasi MPC | t_MPC per siklus + titik deadline miss (>8ms) |
| `plot_5_anomaly.png` | Deteksi Anomali | Flag 0/1 sepanjang simulasi |
| `plot_6_dashboard.png` | Dashboard 4-in-1 | Semua panel dalam satu gambar |

---

## 🗂️ Struktur File Repository

```
ETS PEMKON/
│
├── 📄 README.md                    ← Dokumentasi ini
│
├── 🦀 Rust Program/                ← Firmware ESP32-S3
│   ├── src/main.rs                 ← Source utama (342 baris)
│   ├── Cargo.toml                  ← Dependensi
│   ├── rust-toolchain.toml
│   └── wokwi.toml                  ← Konfigurasi simulasi Wokwi
│
├── 🔌 Simulasi Proteus/            ← File simulasi Proteus
│
├── 📊 data_simulasi.csv            ← Data simulasi (300 iterasi, ADC 1983-4095)
├── 🐍 generate_csv.py              ← Script Python generate data CSV
├── 📉 plot_secmpc_v2.gp            ← Script GNUplot visualisasi
│
├── 🖼️ plot_1_adc_vs_setpoint.png   ← Grafik ADC vs Setpoint
├── 🖼️ plot_2_error.png             ← Grafik Error Kontrol
├── 🖼️ plot_3_umpc_pwm.png          ← Grafik Output MPC & PWM
├── 🖼️ plot_4_mpc_time.png          ← Grafik Waktu Komputasi MPC
├── 🖼️ plot_5_anomaly.png           ← Grafik Deteksi Anomali
├── 🖼️ plot_6_dashboard.png         ← Dashboard 4-in-1
```

---

## 📦 Dependensi

### Rust Crates (`Cargo.toml`)

| Crate | Versi | Fungsi |
|-------|-------|--------|
| `esp-hal` | 1.1 | Hardware abstraction layer ESP32-S3 |
| `esp-backtrace` | 0.18 | Panic handler (no_std) |
| `esp-println` | 0.16 | Print macro (no_std) |
| `esp-bootloader-esp-idf` | 0.5 | App descriptor ESP-IDF |
| `embedded-hal` | 1.0 | Embedded HAL traits |
| `nb` | 1.1 | Non-blocking I/O |
| `heapless` | 0.8 | Stack-allocated collections |

### Tools Eksternal

| Tool | Versi | Fungsi |
|------|-------|--------|
| GNUplot | ≥ 6.0 | Visualisasi grafik simulasi |
| Python | ≥ 3.8 | Generate data CSV |
| espflash | latest | Flash firmware ke ESP32-S3 |
| Proteus | 8.x | Simulasi hardware |

---

## 👤 Informasi

- **Mata Kuliah:** Pemrograman Kontroler
- **Kelas:** 4C
- **Tugas:** ETS (Evaluasi Tengah Semester)
- **Metode:** SecMPC-RT (Secure Encrypted MPC with Real-Time Co-Scheduling)
- **Platform Target:** ESP32-S3
- **Bahasa Pemrograman:** Rust (`no_std`)

---

<div align="center">

**SecMPC-RT** — *Combining Control Theory, Security, and Real-Time Systems on Embedded Rust*

</div>
