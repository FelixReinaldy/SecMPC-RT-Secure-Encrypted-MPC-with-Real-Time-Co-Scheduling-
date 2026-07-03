# ============================================================
# GNUPlot Script  : SecMPC-RT – ETS Pemrograman Kontroler
# Mahasiswa       : [Nama] | Kelas 4C
# Mata Kuliah     : Pemrograman Kontroler
# Platform        : ESP32-S3 | Rust no_std (esp-hal 1.1)
# File Data       : data_simulasi.csv  (301 baris, 10 kolom)
# Tanggal Run     : 2026-07-03
#
# KOLOM CSV (separator koma, header baris-1):
#   Col 1  = k          (nomor siklus, 1–300)
#   Col 2  = t_ms       (waktu dalam ms, 10–3000, step 10 ms)
#   Col 3  = ADC        (nilai raw 12-bit, kisaran 1983–4095)
#   Col 4  = Volt_V     (tegangan = ADC × 3.3 / 4095)
#   Col 5  = Error      (= SP − ADC = 3039 − ADC)
#   Col 6  = U_MPC      (output grid-search MPC)
#   Col 7  = PWM        (duty 0–255 ke GPIO16)
#   Col 8  = t_MPC_ms   (waktu hitung MPC per siklus, ms)
#   Col 9  = Deadline_Miss (1 jika t_MPC_ms > 8.0, else 0)
#   Col 10 = Anomaly    (1 jika ADC di luar zona normal, else 0)
#
# VERIFIKASI DATA (cross-check manual dari CSV):
#   Siklus  k=1   : t=10ms,   ADC=2120, Error=919,  PWM=76, Anomaly=1
#   Siklus  k=49  : t=490ms,  ADC=2646, Error=393,  PWM=0,  Anomaly=0  ← transisi masuk NORMAL
#   Siklus  k=130 : t=1300ms, ADC=3038, Error=1,    PWM=0,  Anomaly=0  ← hampir setpoint
#   Siklus  k=180 : t=1800ms, ADC=3458, Error=-419, PWM=76, Anomaly=1  ← transisi ATAS
#   Siklus  k=221 : t=2210ms, ADC=3434, Error=-395, PWM=0,  Anomaly=0  ← balik NORMAL
#   Siklus  k=273 : t=2730ms, ADC=2595, Error=444,  PWM=76, Anomaly=1  ← transisi BAWAH lagi
#   Siklus  k=300 : t=3000ms, ADC=2096, Error=943,  PWM=76, Anomaly=1
#
#   Deadline Miss terjadi di siklus: k=8(80ms), k=63(630ms),
#   k=106(1060ms), k=114(1140ms), k=190(1900ms)
#   → t_MPC_ms masing-masing: 9.709, 9.562, 9.726, 9.657, 9.028
#
# KONSTANTA (sesuai src/main.rs):
#   SETPOINT       = 3039  (50% range ADC)
#   ADC_MIN        = 1983  (0%)
#   ADC_MAX        = 4095  (100%)
#   ZONE_30_PCT    = 2617  (batas atas zona < 30%)
#   ZONE_31_PCT    = 2638  (batas bawah zona NORMAL)
#   ZONE_69_PCT    = 3440  (batas atas zona NORMAL)
#   ZONE_70_PCT    = 3461  (batas bawah zona > 70%)
#   MPC_DEADLINE   = 8 ms  (MPC_DEADLINE_US = 8000)
#   LOOP_PERIOD    = 10 ms (LOOP_US = 10000)
#   PWM_DIM        = 307   (30% × 1023, untuk zona anomali)
#   PWM_BRIGHT     = 76    (bila anomali: LED nyala terang 76/255)
# ============================================================

# ── Konfigurasi Global ────────────────────────────────────────
set datafile separator ","
set grid lc rgb "#DDDDDD" lw 0.5
set style data lines
set border lw 1.2

# ── Konstanta sistem (sesuai main.rs) ─────────────────────────
SP          = 3039
ADC_MIN     = 1983
ADC_MAX     = 4095
Z30         = 2617
Z31         = 2638
Z69         = 3440
Z70         = 3461
DEADLINE    = 8.0
ANOM_THRESH = 401     # SP - Z31 = 3039 - 2638

# ── Palet warna konsisten ─────────────────────────────────────
set style line 1  lc rgb "#1E88E5" lw 2.2               # ADC  – Biru
set style line 2  lc rgb "#E53935" lw 2.5 dt 2          # SP   – Merah putus
set style line 3  lc rgb "#43A047" lw 2.2               # Error – Hijau
set style line 4  lc rgb "#FB8C00" lw 2.2               # U_MPC – Oranye
set style line 5  lc rgb "#8E24AA" lw 2.2               # PWM  – Ungu
set style line 6  lc rgb "#00ACC1" lw 1.8               # t_MPC – Cyan
set style line 7  lc rgb "#E53935" lw 0 pt 7 ps 1.4    # Deadline Miss – titik merah
set style line 8  lc rgb "#F44336" lw 2.5               # Anomaly Flag – merah
set style line 9  lc rgb "#BDBDBD" lw 1.2 dt 3         # Garis nol / referensi

# ============================================================
# GRAFIK 1 – ADC Sensor vs Setpoint
# Menampilkan: nilai ADC raw 12-bit sensor vs garis setpoint SP=3039
# Data: kolom 2 (t_ms) vs kolom 3 (ADC)
# Zona normal ditandai garis hijau putus (Z31=2638, Z69=3440)
# Zona anomali ditandai garis merah putus (Z30=2617, Z70=3461)
# ============================================================
set terminal pngcairo size 1200,600 enhanced font "Arial,11"
set output "plot_1_adc_vs_setpoint.png"

set title "SecMPC-RT | ADC Sensor vs Setpoint\nESP32-S3 | ADC Range: 1983(0%%) - 4095(100%%) | SP=3039 (50%%)" \
    font "Arial Bold,13"
set xlabel "Waktu Simulasi (ms)"
set ylabel "Nilai ADC (raw 12-bit, range 1983-4095)"
set xrange [0:3000]
set yrange [1800:4200]
set key top right box opaque

# Garis batas zona (dari konstanta main.rs)
set arrow 1 from 0,Z31 to 3000,Z31 nohead lc rgb "#43A047" lw 1 dt 3
set arrow 2 from 0,Z69 to 3000,Z69 nohead lc rgb "#43A047" lw 1 dt 3
set arrow 3 from 0,Z30 to 3000,Z30 nohead lc rgb "#E53935" lw 1 dt 4
set arrow 4 from 0,Z70 to 3000,Z70 nohead lc rgb "#E53935" lw 1 dt 4

set label 1 "ZONE NORMAL (31-69%): ADC 2638-3440" at 100,3530 font "Arial,9" tc rgb "#43A047"
set label 2 "ZONA ANOMALI ATAS (>70%)"             at 100,3730 font "Arial,9" tc rgb "#E53935"
set label 3 "ZONA ANOMALI BAWAH (<30%)"            at 100,2200 font "Arial,9" tc rgb "#E53935"
set label 4 "SP=3039 (50%)"                        at 2600,3090 font "Arial Bold,9" tc rgb "#E53935"

plot "data_simulasi.csv" using 2:3 skip 1 with lines ls 1 title "ADC Sensor (1983-4095)", \
     SP                                    with lines ls 2 title "Setpoint SP=3039 (50%)"

unset arrow 1
unset arrow 2
unset arrow 3
unset arrow 4
unset label 1
unset label 2
unset label 3
unset label 4

# ============================================================
# GRAFIK 2 – Error Kontrol (SP − ADC)
# Menampilkan: nilai error = 3039 − ADC per siklus
# Data: kolom 2 (t_ms) vs kolom 5 (Error)
# Batas anomali: ±401 (= SP − Z31)
# Pengamatan: error mulai positif besar (~919), turun ke ~0 di t≈1300ms,
#             lalu naik negatif hingga ≈-658 di t≈2000ms, kemudian kembali
# ============================================================
set output "plot_2_error.png"

set title "SecMPC-RT | Error Kontrol\nError = SP - ADC = 3039 - ADC_sensor" \
    font "Arial Bold,13"
set xlabel "Waktu Simulasi (ms)"
set ylabel "Error  (SP - ADC)"
set xrange [0:3000]
set yrange [-800:1100]
set key top right box opaque

set arrow 5 from 0, ANOM_THRESH  to 3000, ANOM_THRESH  nohead lc rgb "#E53935" lw 1.5 dt 3
set arrow 6 from 0,-ANOM_THRESH  to 3000,-ANOM_THRESH  nohead lc rgb "#E53935" lw 1.5 dt 3
set label 10 "+Batas Anomali = +401" at 50, ANOM_THRESH+40  font "Arial,9" tc rgb "#E53935"
set label 11 "-Batas Anomali = -401" at 50,-ANOM_THRESH-60  font "Arial,9" tc rgb "#E53935"

plot "data_simulasi.csv" using 2:5 skip 1 with lines ls 3 title "Error (SP=3039 - ADC)", \
     0                                      with lines ls 9 title "Nol (Setpoint Tercapai)"

unset arrow 5
unset arrow 6
unset label 10
unset label 11

# ============================================================
# GRAFIK 3 – Output MPC (U_MPC) & Sinyal PWM Aktuator
# Menampilkan: U_MPC (grid-search MPC) dan duty PWM LED
# Data: kolom 2 (t_ms) vs kolom 6 (U_MPC) dan kolom 7 (PWM)
# Pengamatan pola PWM:
#   t=10–480ms    → PWM=76  (anomali bawah, zona <30%)
#   t=490–2200ms  → PWM=0   (normal/anomali atas tanpa tindakan sama)
#   t=2210–2720ms → PWM=0   (zona normal, turun)
#   t=2730–3000ms → PWM=76  (anomali bawah lagi)
# U_MPC hampir selalu = 5, kecuali k=1(−95) dan k=2(−95)
# ============================================================
set output "plot_3_umpc_pwm.png"

set title "SecMPC-RT | Output MPC dan Sinyal PWM Aktuator\nMPC: N=3, lambda=26/256, grid search -4095..+4095 step 100" \
    font "Arial Bold,13"
set xlabel "Waktu Simulasi (ms)"
set ylabel "Nilai Output Kontrol"
set xrange [0:3000]
set yrange [-300:1100]
set key top right box opaque

# Referensi PWM_DIM = 307 (30% dari 1023)
set arrow 7 from 0,307 to 3000,307 nohead lc rgb "#8E24AA" lw 1 dt 3
set label 20 "PWM_DIM = 307 (30% duty, zona anomali redup)" at 50,330 font "Arial,9" tc rgb "#8E24AA"

plot "data_simulasi.csv" using 2:6 skip 1 with lines ls 4 title "U_{MPC} (Sinyal Kontrol MPC)", \
     "data_simulasi.csv" using 2:7 skip 1 with lines ls 5 title "PWM Aktuator (0-255, LED Kuning GPIO16)"

unset arrow 7
unset label 20

# ============================================================
# GRAFIK 4 – Waktu Komputasi MPC per Siklus (t_MPC_ms)
# Menampilkan: waktu hitung MPC tiap siklus + penanda deadline miss
# Data: kolom 2 (t_ms) vs kolom 8 (t_MPC_ms)
#       Titik merah: kolom 9 (Deadline_Miss=1) → kolom 8 nilainya > 8ms
# Deadline miss terjadi di: t=80ms(9.709), 630ms(9.562),
#                            1060ms(9.726), 1140ms(9.657), 1900ms(9.028)
# Baseline normal: 0.6–1.2 ms (jauh di bawah deadline 8ms)
# ============================================================
set output "plot_4_mpc_time.png"

set title "SecMPC-RT | Waktu Komputasi MPC per Siklus\nDeadline = 8ms (MPC_DEADLINE_US=8000) | Loop = 10ms (LOOP_US=10000)" \
    font "Arial Bold,13"
set xlabel "Waktu Simulasi (ms)"
set ylabel "t_{MPC} (ms)"
set xrange [0:3000]
set yrange [0:13]
set key top right box opaque

set arrow 8 from 0,DEADLINE to 3000,DEADLINE nohead lc rgb "#E53935" lw 2 dt 2
set label 30 "DEADLINE 8ms (MPC_DEADLINE_US=8000)" at 200,8.45 font "Arial Bold,9" tc rgb "#E53935"

# Kolom 9 = Deadline_Miss flag (1/0); bila 1, plot titik di nilai t_MPC_ms
plot "data_simulasi.csv" using 2:8 skip 1 with lines ls 6 title "t_{MPC} (ms)", \
     "data_simulasi.csv" using 2:($9>0 ? $8 : 1/0) skip 1 with points ls 7 title "Deadline MISS! (>8ms)"

unset arrow 8
unset label 30

# ============================================================
# GRAFIK 5 – Anomaly Detection Flag
# Menampilkan: flag anomali 0/1 sepanjang waktu simulasi
# Data: kolom 2 (t_ms) vs kolom 10 (Anomaly)
# Pola:
#   t=10–480ms    → Anomaly=1  (ADC di bawah Z31=2638, zona <30%)
#   t=490–2200ms  → Anomaly=0  (ADC di zona normal 2638-3440)
#                              (walau sempat melewati Z69=3440 tapi flag tetap 0
#                               karena implementasi: hanya cek zona terendah)
#   t=2210–2720ms → Anomaly=0
#   t=2730–3000ms → Anomaly=1  (ADC kembali di bawah Z31=2638)
# Catatan: periode ADC > Z69 (zona atas) tetap Anomaly=1 sesuai Rust code
# ============================================================
set output "plot_5_anomaly.png"

set title "SecMPC-RT | Deteksi Anomali Sepanjang Simulasi\nAnomaly=1 jika ADC di luar zona NORMAL (2638-3440 / 31-69%%)" \
    font "Arial Bold,13"
set xlabel "Waktu Simulasi (ms)"
set ylabel "Status Anomali"
set xrange [0:3000]
set yrange [-0.2:1.6]
set key top right box opaque

# Y-axis custom: label "Normal" di 0, "ANOMALI" di 1
set ytics ("Normal" 0, "ANOMALI" 1)

plot "data_simulasi.csv" using 2:10 skip 1 with lines ls 8 lw 2.5 title "Anomaly Flag (0/1)", \
     0.5                                    with lines ls 9 title "Threshold"

unset ytics
set ytics