//! SecMPC-RT – Secure Encrypted MPC with Real-Time Co-Scheduling
//! Platform  : ESP32-S3 (no_std, esp-hal 1.1)
//! Kelas 4C  : Pemrograman Kontroler – Tugas 2
//!
//! PIN MAPPING:
//!   GPIO1  → ADC1_CH0  : Potensiometer (sensor, 1983-4095)
//!   GPIO2  → Output    : LED HIJAU   (Normal, setpoint 31-69%)
//!   GPIO15 → Output    : LED MERAH   (Fault/anomali/boundary)
//!   GPIO16 → LEDC PWM  : LED KUNING  (Aktuator, brightness = PWM duty)
//!   GPIO43 → UART0 TX  : Telemetri serial 115200 baud
//!   GPIO44 → UART0 RX  : (unused, required by driver)
//!
//! SKALA ADC EFEKTIF:
//!   ADC_MIN = 1983  → 0%   (potensiometer minimum)
//!   ADC_MAX = 4095  → 100% (potensiometer maksimum)
//!   Range efektif   = 4095 - 1983 = 2112 count
//!
//! TABEL SETPOINT (berdasarkan skala 0%-100%, per zona):
//!   0%        (ADC ≤ 1999)      → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
//!             (1983-1999 = dead zone noise floor hardware)
//!   1-30%     (ADC 2000-2617)   → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
//!   31-69%    (ADC 2638-3440)   → MERAH:MATI   | KUNING:MATI   | HIJAU:HIDUP
//!   70-99%    (ADC 3461-4094)   → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
//!   100%      (ADC = 4095)      → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig},
    ledc::{
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc,
    },
    time::Instant,
    uart::{Config as UartConfig, Uart},
};
use core::fmt::Write;

// ── Konstanta ADC (skala efektif: 1983=0% hingga 4095=100%) ─────────────────
const ADC_MIN:        i32 = 1983;   // 0%   → potensiometer minimum (untuk kalkulasi %)
const ADC_MAX:        i32 = 4095;   // 100% → potensiometer maksimum
const ADC_RANGE:      i32 = ADC_MAX - ADC_MIN;  // 2112 count

// Dead zone noise floor: ADC 1983-1999 masih dianggap 0% (zone TERANG)
// Hardware ESP32-S3 fluktuasi di ~1989-1993 saat potensiometer minimum
const ADC_NOISE_FLOOR: i32 = 1999;  // batas atas dead zone (inklusif)

// Batas zone dihitung dari: ADC_MIN + (pct / 100) × ADC_RANGE
// 0%  = 1983       → batas bawah (ADC_MIN)
// 30% = 1983 + 634 = 2617
// 31% = 1983 + 655 = 2638
// 69% = 1983 + 1457= 3440
// 70% = 1983 + 1478= 3461
// 100%= 4095       → batas atas (ADC_MAX)
const ZONE_30_PCT:    i32 = 2617;   // ADC_MIN + 30% × ADC_RANGE
const ZONE_31_PCT:    i32 = 2638;   // ADC_MIN + 31% × ADC_RANGE
const ZONE_69_PCT:    i32 = 3440;   // ADC_MIN + 69% × ADC_RANGE
const ZONE_70_PCT:    i32 = 3461;   // ADC_MIN + 70% × ADC_RANGE

// Setpoint MPC = tengah zona normal (50%) = 1983 + 50% × 2112 = 3039
const SETPOINT:       i32 = 3039;

// Ambang anomali dokumentasi (referensi tabel setpoint)
#[allow(dead_code)]
const ANOMALY_THRESH: i32 = SETPOINT - ZONE_31_PCT;  // ≈ 401

// Loop & deadline
const LOOP_US:        u32 = 10_000;
const MPC_DEADLINE_US:u32 = 8_000;
const ENC_BUDGET_US:  u32 = 1_500;

// MPC
const MPC_HORIZON: usize = 3;
const MPC_STEP:    i32   = 100;

// Kunci XOR-cipher (simulasi AES-128)
const AES_KEY: [u8; 16] = [
    0x2B,0x7E,0x15,0x16, 0x28,0xAE,0xD2,0xA6,
    0xAB,0xF7,0x15,0x88, 0x09,0xCF,0x4F,0x3C,
];

// PWM duty 10-bit max
const PWM_MAX: u32 = 1023;
// PWM redup = 30% dari max (untuk state REDUP)
const PWM_DIM: u32 = 307;   // ≈ 30% × 1023

// ── CRC-8 (poly 0x07) ─────────────────────────────────────────────────────────
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}

// ── XOR-Cipher (simulasi AES-128) ─────────────────────────────────────────────
#[inline(always)]
fn encrypt_block(plain: &[u8; 8], cipher: &mut [u8; 8]) {
    for i in 0..8 {
        cipher[i] = plain[i] ^ AES_KEY[i];
    }
}

// ── Klasifikasi zone berdasarkan tabel setpoint (skala 1983-4095) ────────────
/// Mengembalikan zone_label:
///   0 = 0% + noise floor  (ADC ≤ 1999)   → KUNING TERANG (stabil)
///   1 = 1–30%             (ADC 2000–2617) → KUNING REDUP
///   2 = 31–69%            (ADC 2638–3440) → HIJAU HIDUP (NORMAL)
///   3 = 70–99%            (ADC 3461–4094) → KUNING REDUP
///   4 = tepat 100%         (ADC = 4095)   → KUNING TERANG
fn classify_zone(adc: i32) -> u8 {
    if adc <= ADC_NOISE_FLOOR {
        0  // 0% + noise floor (1983-1999) → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
    } else if adc <= ZONE_30_PCT {
        1  // 1–30%      → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
    } else if adc <= ZONE_69_PCT {
        2  // 31–69% NORMAL → MERAH:MATI | KUNING:MATI  | HIJAU:HIDUP
    } else if adc < ADC_MAX {
        3  // 70–99%     → MERAH:TERANG | KUNING:REDUP  | HIJAU:MATI
    } else {
        4  // tepat 100% → MERAH:TERANG | KUNING:TERANG | HIJAU:MATI
    }
}

// ── MPC Compute (N=3, quadratic cost, grid search) ───────────────────────────
fn mpc_compute(y_buf: &[i32; MPC_HORIZON], y_now: i32) -> i32 {
    let dy = y_buf[0].wrapping_sub(y_buf[MPC_HORIZON - 1]);
    const LAMBDA_Q8: i64 = 26;
    let mut cost_min = i64::MAX;
    let mut u_opt = 0i32;
    let mut u = -4095i32;
    while u <= 4095 {
        let mut cost = 0i64;
        for k in 1..=(MPC_HORIZON as i32) {
            let y_pred = y_now
                .wrapping_add(dy.wrapping_mul(k))
                .wrapping_sub(u.wrapping_mul(3).wrapping_mul(k) / 10);
            let e = (SETPOINT - y_pred) as i64;
            cost = cost.saturating_add(e.saturating_mul(e));
            cost = cost.saturating_add(
                (LAMBDA_Q8.saturating_mul((u as i64).saturating_mul(u as i64))) >> 8,
            );
        }
        if cost < cost_min {
            cost_min = cost;
            u_opt = u;
        }
        u = u.wrapping_add(MPC_STEP);
    }
    u_opt
}

// ── Entry Point ───────────────────────────────────────────────────────────────
#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // ── LED MERAH (GPIO15) ────────────────────────────────────────────────────
    let mut led_red = Output::new(
        peripherals.GPIO15,
        Level::Low,
        OutputConfig::default(),
    );

    // ── LED HIJAU (GPIO2) ─────────────────────────────────────────────────────
    let mut led_green = Output::new(
        peripherals.GPIO2,
        Level::Low,
        OutputConfig::default(),
    );

    // Startup self-test: semua LED nyala 500ms lalu mati
    led_red.set_high();
    led_green.set_high();
    delay.delay_millis(500u32);
    led_red.set_low();
    led_green.set_low();

    // ── UART0 115200 baud (TX=GPIO43, RX=GPIO44) ──────────────────────────────
    let mut uart0 = Uart::new(peripherals.UART0, UartConfig::default())
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    // ── ADC1 GPIO1 (12-bit, Attenuation 11dB → full 0-3.3V range) ────────────
    let mut adc1_config = AdcConfig::new();
    let mut adc_pin = adc1_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    // ── LEDC PWM GPIO16 (LED KUNING, 10-bit, 1kHz) ───────────────────────────
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty:         timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency:    esp_hal::time::Rate::from_hz(1000),
        })
        .unwrap();

    let mut pwm_ch = ledc.channel(channel::Number::Channel0, peripherals.GPIO16);
    pwm_ch
        .configure(channel::config::Config {
            timer:      &lstimer0,
            duty_pct:   0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // ── State ─────────────────────────────────────────────────────────────────
    let mut y_buf:           [i32; MPC_HORIZON] = [SETPOINT; MPC_HORIZON];
    let mut buf_idx:          usize = 0;
    let mut deadline_misses:  u32   = 0;
    let mut loop_count:       u32   = 0;
    let mut _u_prev:           i32   = 0;

    // Header telemetri via UART
    writeln!(uart0, "\r\n╔══════════════════════════════════════════════════════════════╗").ok();
    writeln!(uart0, "║  SecMPC-RT | ESP32-S3 | SETPOINT=3039(50%) | 10ms Loop    ║").ok();
    writeln!(uart0, "║  ADC RANGE: 1983=0%  ..  4095=100%  (range=2112)           ║").ok();
    writeln!(uart0, "║  LED: MERAH=fault | KUNING=PWM aktuator | HIJAU=normal     ║").ok();
    writeln!(uart0, "║  Zone: 0%→Batas | ≤30%→Redup | 31-69%→Normal | 70%→Redup ║").ok();
    writeln!(uart0, "╚══════════════════════════════════════════════════════════════╝").ok();
    writeln!(uart0, " Iter |  ADC  |  pct%  | Zone      |  Error | u_opt | T_us | MISS").ok();
    writeln!(uart0, "------+-------+--------+-----------+--------+-------+------+------").ok();

    // ── Main Control Loop 10ms ────────────────────────────────────────────────
    loop {
        loop_count = loop_count.wrapping_add(1);
        let t0 = Instant::now();

        // ── STEP 1: Baca ADC (12-bit: 0-4095) ────────────────────────────────
        let adc_raw: u16 = nb::block!(adc1.read_oneshot(&mut adc_pin)).unwrap_or(2048);
        let y_now = adc_raw as i32;
        y_buf[buf_idx] = y_now;
        buf_idx = (buf_idx + 1) % MPC_HORIZON;

        // ── STEP 2: MPC Compute ───────────────────────────────────────────────
        let t_mpc = Instant::now();
        let u_opt = mpc_compute(&y_buf, y_now);
        let t_mpc_us = t_mpc.elapsed().as_micros() as u32;

        // ── STEP 3: Klasifikasi Zone & Error ─────────────────────────────────
        let error  = SETPOINT - y_now;
        let zone   = classify_zone(y_now);
        // Hitung persen berdasarkan skala efektif: 1983=0%, 4095=100%
        let pct = if y_now <= ADC_MIN {
            0i32
        } else if y_now >= ADC_MAX {
            100i32
        } else {
            ((y_now - ADC_MIN) * 100) / ADC_RANGE  // 0–100
        };

        // ── STEP 4: Deadline Check ────────────────────────────────────────────
        let deadline_ok = t_mpc_us < MPC_DEADLINE_US;
        if !deadline_ok {
            deadline_misses = deadline_misses.wrapping_add(1);
        }

        // ── STEP 5: Aktuasi LED sesuai Tabel Setpoint ─────────────────────────
        // Tabel:
        //   zone 0 (0%)      → MERAH:TERANG | KUNING:TERANG  | HIJAU:MATI
        //   zone 1 (≤30%)    → MERAH:TERANG | KUNING:REDUP   | HIJAU:MATI
        //   zone 2 (31-69%)  → MERAH:MATI   | KUNING:MATI    | HIJAU:HIDUP
        //   zone 3 (70%)     → MERAH:TERANG | KUNING:REDUP   | HIJAU:MATI
        //   zone 4 (100%)    → MERAH:TERANG | KUNING:TERANG  | HIJAU:MATI
        let (red_on, kuning_duty, green_on) = match zone {
            0 => (true,  PWM_MAX,  false),  // tepat 0%:   MERAH TERANG, KUNING TERANG, HIJAU MATI
            1 => (true,  PWM_DIM,  false),  // 1–30%:      MERAH TERANG, KUNING REDUP,  HIJAU MATI
            2 => (false, 0,        true),   // 31–69% NORMAL: MERAH MATI, KUNING MATI, HIJAU HIDUP
            3 => (true,  PWM_DIM,  false),  // 70–99%:     MERAH TERANG, KUNING REDUP,  HIJAU MATI
            _ => (true,  PWM_MAX,  false),  // tepat 100%: MERAH TERANG, KUNING TERANG, HIJAU MATI
        };

        // Override: deadline miss → paksa MERAH menyala, kuning mati
        let (red_on, kuning_duty, green_on) = if !deadline_ok {
            (true, 0u32, false)
        } else {
            (red_on, kuning_duty, green_on)
        };

        // Set LED MERAH
        if red_on { led_red.set_high(); } else { led_red.set_low(); }
        // Set LED HIJAU
        if green_on { led_green.set_high(); } else { led_green.set_low(); }
        // Set LED KUNING (PWM) — duty 10-bit raw
        let _ = pwm_ch.set_duty_hw(kuning_duty);

        // Simpan u_prev untuk telemetri
        if deadline_ok && zone == 2 { _u_prev = u_opt; }

        // ── STEP 6: Co-Scheduled Enkripsi + Telemetri ────────────────────────
        let elapsed_us = t0.elapsed().as_micros() as u32;
        if elapsed_us + ENC_BUDGET_US < LOOP_US {
            // XOR-encrypt 8-byte blok ADC+error (simulasi AES)
            let plain: [u8; 8] = [
                (adc_raw >> 8) as u8, adc_raw as u8,
                (error.abs() >> 8) as u8, error.abs() as u8,
                zone, pct as u8,
                (loop_count >> 8) as u8, loop_count as u8,
            ];
            let mut cipher = [0u8; 8];
            encrypt_block(&plain, &mut cipher);
            let crc = crc8(&cipher);
            let _ = crc; // digunakan implisit (anti dead-code)

            let zone_str = match zone {
                0 => "0%:BATAS  ",
                1 => "<=30%:DIM",
                2 => "NORMAL   ",
                3 => "70%:DIM  ",
                _ => "100%:MAX ",
            };
            let iter_disp = loop_count % 99999;
            writeln!(uart0,
                "{:>5} | {:>5} | {:>5}%  | {} | {:>6} | {:>5} | {:>4} | {:>4}",
                iter_disp, adc_raw, pct, zone_str, error, u_opt, t_mpc_us, deadline_misses
            ).ok();
        }

        // ── STEP 7: Tunggu sisa waktu loop 10ms ──────────────────────────────
        let used_us = t0.elapsed().as_micros() as u32;
        if used_us < LOOP_US {
            delay.delay_micros(LOOP_US - used_us);
        }
    }
}
