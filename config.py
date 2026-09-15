"""Konstanta global untuk seluruh modul skripsi."""
import os

# Koneksi & simulasi
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_TIMEOUT = 20.0

TOWN = 'Town04'
FIXED_DELTA_SECONDS = 0.05          # 20 Hz, mode sinkron

# Kendaraan ego
EGO_BP = 'vehicle.dodge.charger_2020'

# Cari kandidat ruas lurus dengan: python validate_model.py --scan
SPAWN_IDX = 75          # lajur paling kiri, 3 lajur di kanan, lurus, lebar 3.50 m konstan

# Geometri manuver (bagian 0.3)
OVERTAKE_SIDE = 'right'             # 'left' | 'right'

# Frame right-handed (y -> -y, yaw -> -yaw), jadi kanan seharusnya y negatif.
# Diverifikasi empiris oleh validate_model.py -- jangan dipakai sebelum itu.
SIDE_SIGN = -1 if OVERTAKE_SIDE == 'right' else +1

# Eksperimen validasi model (bagian 3.3)
# Diuji di kecepatan operasi, bukan dari diam: akselerasi 0 -> 50 km/jam butuh
# belasan detik dan hasilnya cuma berlaku di kecepatan rendah.
VALIDATION_SPEED = 13.9             # m/s, 50 km/jam -- kecepatan operasi skripsi
VALIDATION_DURATION = 3.0           # detik, setara durasi satu manuver pindah lajur
VALIDATION_THROTTLE = 0.56           # menahan ~50 km/jam; 0.4 melambat, 0.75 tembus 68
VALIDATION_STEER = 0.033            # + = kanan; R ~90 m, a_lat ~2.1 m/s² @ 50 km/jam
VALIDATION_WARMUP = 1.0             # detik, roda menyesuaikan setelah set_target_velocity

# Path
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
VEHICLE_PARAMS_JSON = os.path.join(OUT_DIR, 'vehicle_params.json')

# Local planner (bagian 5.2-5.5)
LANE_WIDTH = 3.50                   # m, terukur dari Town04; PDGJ 2021 Tabel 5-58 (V_D 40-80)
PLANNER_DT = 0.1                    # detik, resolusi sampling lintasan
LATERAL_OFFSETS = (3.0, 3.5, 4.0)   # m, magnitudo -- dikalikan SIDE_SIGN
MANEUVER_TIMES = (3.0, 3.5, 4.0)    # detik
MAX_LATERAL_ACCEL = 3.0             # m/s², batas kenyamanan
MIN_TURN_RADIUS = 5.6               # m, dari L=3.044 dan delta_max=0.5 rad
# ELLIPSE_A, ELLIPSE_B, ELLIPSE_P: zona aman, diturunkan di bawah JARAK_AMAN

# Bobot seleksi kandidat: J = W_LAT*J_lat + W_LON*J_lon + W_COL*J_col
W_LAT, W_LON, W_COL = 1.0, 1.0, 1.0
K_TIME = 0.1                        # penalti durasi manuver
K_DEV = 20.0                        # penalti simpangan dari tengah lajur tujuan
K_VEL = 1.0                         # penalti simpangan kecepatan akhir

# Behavior FSM (bagian 6). Tiap ambang punya pasangan histeresis.
FSM_DWELL = 0.3                     # detik, transisi harus stabil sebelum dieksekusi
# Pemicu memakai TTC, bukan jarak. Bagian 6 mengusulkan ambang jarak 20-25 m,
# tapi jarak yang aman bergantung selisih kecepatan: 25 m aman pada dv = 3 m/s
# dan berbahaya pada dv = 8 m/s. Ambang waktu menyesuaikan sendiri.
#   TTC_TRIGGER >= T_tercepat + 2*FSM_DWELL + margin = 3,0 + 0,6 + 1,4 = 5,0 s
# Dua dwell = transisi LANE_KEEPING->CHECK lalu CHECK->LANE_CHANGE.
TTC_TRIGGER = 5.0                   # detik, mulai mempertimbangkan menyalip
TTC_EXIT = 7.0                      # detik, histeresis: batal mempertimbangkan
DV_TRIGGER = 3.0                    # m/s, kendaraan depan harus selambat ini
DV_EXIT = 1.5                       # m/s, histeresis
D_SAFE_DEPAN = 25.0                 # m, lajur tujuan harus kosong ke depan
D_SAFE_BELAKANG = 15.0              # m, dan ke belakang
LATERAL_MASUK = 0.9                 # fraksi lebar lajur -> dianggap sudah pindah
PASS_MARGIN = 8.0                   # m, ego harus unggul sejauh ini sebelum kembali
# Laju lateral menjauhi lajur asal maksimum untuk mulai kembali (TUNING_MPC.md 13).
# Quintic kembali (T=4 s) yang berangkat dengan laju menjauh u kebablasan keluar:
# u=0,1 -> 0,017 m; u=0,91 -> 0,40 m (terukur 0,47 m di run gagal). 0,1 m/s juga
# ~70x di atas derau laju lateral saat menjaga lajur (maks 0,0014 m/s).
DD_KEMBALI = 0.1                    # m/s
LATERAL_SELESAI = 0.3               # m, kembali ke lajur asal dianggap selesai
# Mengikuti kendaraan depan selama belum/tidak bisa menyalip (planning._v_ikut).
# Jarak ikut d* = ELLIPSE_A + SUMBU_KE_PUSAT + WAKTU_IKUT * v_depan (jarak waktu-tetap).
# Sapuan S3 (TUNING_MPC.md bagian 13.7). Setelah zona aman menjamin JARAK_AMAN,
# 1,0-2,5 s semuanya lolos dan jarak bodi tidak lagi bergantung nilai ini (1,42-
# 1,69 m). 2,0 s = nilai terkecil yang tidak pernah membuat planner kehabisan
# kandidat (1,0 s -> 12 tick nol, 1,5 s -> 4, 2,0 dan 2,5 s -> 0); tick nol itu
# yang dulu menendang ego keluar lajur. 2,5 s hanya menambah jarak ikut.
WAKTU_IKUT = 2.0                    # s

# Sensor kamera (Tahap 8, bagian 10). Penempatan mendekati rig KITTI (Geiger dkk.,
# IJRR 2013, Gambar 3): seluruh kamera 1,65 m di atas permukaan jalan dan 1,68 m di
# depan sumbu roda belakang, baseline stereo warna 0,54 m. Lensa KITTI 4 mm dengan
# sudut buka ~90 derajat. Resolusi 1280x720 (KITTI 1392x512) -- keputusan penulis.
KAMERA_LEBAR, KAMERA_TINGGI = 1280, 720
KAMERA_FOV = 90.0                   # derajat, sudut buka horizontal
KAMERA_Z = 1.65                     # m di atas permukaan jalan
KAMERA_DEPAN_SUMBU = 1.68           # m di depan sumbu roda belakang
KAMERA_BASELINE = 0.54              # m, jarak kamera warna kiri-kanan

# YOLOPX (Tahap 8). Repo model = folder tetangga repo skripsi supaya repo ini
# tetap bersih; weight hasil pelatihan penulis ada di luar area skripsi.
YOLOPX_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'YOLOPX')
YOLOPX_WEIGHT = os.path.expanduser('~/sawal/data_acquisition/epoch-195.pth')
DETEKSI_CONF = 0.5                  # ambang keyakinan; ditentukan cek_deteksi.py
DETEKSI_IOU = 0.45                  # ambang NMS

# MPC (bagian 7.3)
MPC_N = 20                          # horizon 2 detik
MPC_DT = 0.1                        # detik
# Q_psi dinaikkan 10 -> 450 lewat sapuan tuning.py (bagian 7.6 langkah 1).
# Nilai awal bagian 7.3 kurang teredam: overshoot 26,4%, settling 2,35 s.
# Di 450: overshoot 0,0%, settling 1,05 s, jitter 7,76 -> 3,47 mrad.
MPC_Q = (1.0, 20.0, 450.0, 2.0)     # bobot error X, Y, psi, v
MPC_QF_SCALE = 5.0                  # Qf = 5*Q
MPC_R = (0.1, 1.0)                  # bobot input a, delta
MPC_RD = (1.0, 20.0)                # bobot perubahan input -- delta jauh lebih besar
MPC_RHO = 1000.0                    # penalti slack elips
A_MIN, A_MAX = -6.0, 3.0            # m/s²
DDELTA_MAX = 0.05                   # rad per langkah, ~2,9°
# Slot tetap (graf Opti dibangun sekali). Skenario S1-S5 paling banyak butuh 2
# kendaraan; tiap slot menambah 21 constraint elips dan ~10 ms waktu solve.
MPC_MAX_OBSTACLES = 2
# Sempat dinaikkan ke 300 untuk mengejar solver gagal, ternyata gejala saja:
# akar masalahnya batas kecepatan berlaku di k=0. Setelah itu diperbaiki, 100
# sudah cukup dan waktu solve turun kembali.
MPC_MAX_ITER = 100
# Toleransi default IPOPT 1e-8 untuk optimasi yang jawabannya jadi produk akhir.
# Di sini jawabannya dibuang 50 ms lagi, dan 1e-4 rad = 0,006° jauh di bawah
# DDELTA_MAX yang bisa dieksekusi mobil. Menghemat ~19 ms per solve.
MPC_TOL = 1e-4

# Konversi a -> throttle/brake (bagian 7.5). PI, bukan tabel kalibrasi:
# tidak perlu sapuan di CARLA dan mengoreksi diri terhadap tanjakan & drag.
# kp = 1/gain plant lokal: terukur 7,1-8,6 m/s² per satuan throttle di 48 km/jam
# (gigi 3-4), jadi 1/K = 0,12-0,14 -> suku kp*a_ref memberi tepat throttle yang
# dibutuhkan. Setelah PI split-range, v err datar 0,011-0,016 m/s untuk kp
# 0,035-0,3 dan rusak di 0,56 (berosilasi): margin x2 dan x4 ke tepi plateau.
# "Optimum tajam kp=0,3" dulu artefak reset integrator, TUNING_MPC.md bagian 10-11.
THROTTLE_KP = 0.14
THROTTLE_KI = 0.25                  # tengah geometrik plateau 0,125-0,5

V_MAX = 13.9                        # m/s, 50 km/jam -- UU 22/2009 Ps. 21; PP 79/2013 Ps. 23(4)
# v_ref DIPISAH dari v_max (bagian 0.2 memang membedakannya: v_ref ego 40-50
# km/jam, v_max constraint 13,9). Disetel sama, constraint keras aktif 65% waktu
# dan solver bekerja di tepi kelayakan -- itu penyebab bug infeasible di k=0.
# Constraint seharusnya batas keselamatan, bukan setpoint.
V_REF = 13.4                        # m/s, 48,2 km/jam -- sisakan ruang ke batas

# Percepatan terukur = selisih kecepatan di 20 Hz, dan itu berderau berat:
# terukur -26,8..+12,8 m/s² padahal batas fisik -6..+3. Dipotong ke batas fisik
# lalu dilewatkan low-pass sebelum dipakai PI.
A_FILTER_ALPHA = 0.2                # tetapan waktu ~0,25 s di 20 Hz
EGO_V0 = V_REF                      # m/s, mulai di kecepatan referensi (bagian 11.1)
# set_target_velocity menetapkan kecepatan BODI; roda masih diam sehingga slip
# longitudinal mengerem mobil. Terukur: dip ke 36,8 km/jam pada t=1,85 s, mapan
# kembali t=4,4 s -- 22% dari run 20 detik. Ego dipanaskan dulu tanpa dicatat.
WARMUP_DETIK = 6.0

# Kriteria keberhasilan satu run (bagian 11.2). Ditetapkan SEBELUM eksperimen
# dijalankan supaya success rate tidak subjektif.
LULUS_LATERAL = 0.5                 # m, ambang "kembali ke lajur semula"
LULUS_TAHAN = 2.0                   # detik, harus bertahan selama ini
JARAK_AMAN = 1.0                    # m, jarak minimum antar bodi kendaraan
BATAS_MANUVER = 20.0                # detik, sejak keluar dari LANE_KEEPING

# Zona aman planner & MPC (bagian 7.2), antar PUSAT bodi:
#   g = ((dx/A)^p + (dy/B)^p)^(1/p) >= 1
# Harus memuat "persegi terlarang" (setengah sisi = jumlah setengah dimensi +
# JARAK_AMAN) supaya constraint menjamin syarat lulus. Elips lama A=7, B=2,2 dari
# sumbu belakang setara jarak bodi 0,29 m saat berpapasan. Elips biasa (p=2) yang
# memuat sudut persegi butuh A ~14 m; p=4 cukup 7,71 m. TUNING_MPC.md bagian 13.
# Dimensi ego = out/vehicle_params.json (dikunci tests/test_planning.py). Kendaraan
# lain = Nissan Patrol, bounding box CARLA terukur 11 Sep 2026: terbesar di
# skenario, dan perception tidak mengukur dimensi.
EGO_PANJANG, EGO_LEBAR = 5.008, 1.882
SUMBU_KE_PUSAT = 1.433              # m, sumbu belakang (state MPC) -> pusat bodi ego
LAIN_PANJANG, LAIN_LEBAR = 4.605, 1.932
ELLIPSE_P = 4
_SETENGAH_PANJANG = (EGO_PANJANG + LAIN_PANJANG) / 2 + JARAK_AMAN     # 5,81 m
_SETENGAH_LEBAR = (EGO_LEBAR + LAIN_LEBAR) / 2 + JARAK_AMAN          # 2,91 m
# B di tengah antara batas perlu (2,91) dan jarak berpapasan di tengah lajur (3,50):
# margin sama ~0,30 m ke keduanya. A = nilai terkecil yang memuat sudut persegi.
ELLIPSE_B = (LANE_WIDTH + _SETENGAH_LEBAR) / 2                       # 3,20 m
ELLIPSE_A = _SETENGAH_PANJANG / (1 - (_SETENGAH_LEBAR / ELLIPSE_B) ** ELLIPSE_P) ** (1 / ELLIPSE_P)  # 7,71 m

# Skenario (bagian 11.1): kendaraan lain = (jarak awal dari ego m, lajur, kecepatan m/s).
# Lajur 0 = lajur ego, 1 = lajur menyalip. Kendaraan pertama = target yang disalip.
SKENARIO = {
    'S1': [(60.0, 0, 7.0)],                     # depan lambat, lajur kanan kosong
    # Lajur tujuan terisi kendaraan dari belakang di batas kecepatan: ego harus
    # menunggu sambil mengikuti, lalu menyalip ulang (accelerative overtaking).
    'S3': [(60.0, 0, 7.0), (-10.0, 1, V_MAX)],
}
