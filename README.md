# Skripsi — MPC untuk Manuver Overtaking di CARLA

Implementasi Model Predictive Control untuk skenario manuver menyalip pada
kendaraan otonom, di CARLA Simulator 0.9.16.

Dokumen pendamping:

| File | Isi |
|---|---|
| `WRITING_SUMMARY.md` | angka, sitasi, dan keputusan terverifikasi — untuk menulis skripsi |
| `TUNING_MPC.md` | seluruh proses tuning bobot: teori, data percobaan, alasan berhenti |
| `NOTES.md` | catatan kerja kronologis lengkap, termasuk bug dan jalan buntu |

---

## Setup

```bash
# 1. dependensi sistem (hanya untuk record_path.py & record_maneuver.py;
#    overlay.py memakai cv2, tidak butuh ffmpeg)
sudo apt install ffmpeg

# 2. dependensi Python (3.10)
pip install -r requirements.txt

# 3. jalankan server CARLA 0.9.16 terpisah (kualitas Low: hasil kendali identik), lalu:
#    ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low
python extract_params.py      # hanya kalau out/vehicle_params.json hilang
```

Versi paket `carla` **wajib sama persis** dengan versi server. Lihat catatan di
`requirements.txt`.

## Menjalankan

**Uji otomatis — tidak butuh server CARLA.** 127 uji, semuanya lolos.

```bash
for f in tests/*.py; do python "$f"; done
```

Uji dijalankan sebagai skrip, bukan lewat pytest. `tests/test_mpc.py` butuh
`out/vehicle_params.json` (sudah ikut di repo).

**Butuh server CARLA:**

| Perintah | Fungsi |
|---|---|
| `python main.py` | skenario S1 lengkap, loop tertutup, vonis berhasil/gagal |
| `python main.py --skenario S3 --detik 25` | lajur tujuan terisi: mengikuti, lalu menyalip ulang |
| `python main.py --perception vision --rekam` | S1 dengan YOLOPX + depth, plus video overlay |
| `python main.py --perception vision --rekam --akhiran _sesudah` | sama, tapi keluarannya tidak menimpa berkas pembanding |
| `python check_estimation.py` | ketelitian jarak & kecepatan vision vs ground truth |
| `python tuning.py --sweep Q_PSI 300,450,600` | harness tuning step response |
| `python tune_vision.py --sweep K_DEV 10,20,40` | sapuan parameter di skenario penuh + vision |
| `python experiment.py --perception vision --ulang 10` | Tahap 9: success rate + sebaran metrik |
| `python metrics.py --layer --eksperimen` | metrik per layer/fase dari log (tidak butuh server) |
| `python metrics.py --layer --eksperimen --akhiran _sebelum` | metrik yang sama SEBELUM perbaikan bagian 27 |
| `python validate_model.py` | validasi bicycle model terhadap plant |
| `python validate_model.py --steer` | verifikasi konversi kemudi |
| `python validate_model.py --scan` | cari spawn point ruas lurus |
| `python record_maneuver.py --kamera atas` | video dengan overlay kandidat |
| `python show_lanes.py` | gambar lingkungan uji dan kandidat planner |
| `python plot_run.py --skenario S3` | grafik hasil run dari log (tidak butuh server) |
| `python plot_compare.py` | grafik pembanding GT vs vision (tidak butuh server) |
| `python record_path.py` | video lintasan acuan global planner (butuh ffmpeg) |
| `python check_sensors.py` | pasang rig kamera, verifikasi penempatan, simpan contoh frame |
| `python show_rig.py` | konfigurasi sensor ala KITTI: foto ego + skema berdimensi |
| `python plot_concepts.py` | gambar konsep: rig, frame, skenario, pipeline, MPC (tanpa server) |
| `python check_detection.py --lajur 1` | ukur deteksi YOLOPX terhadap ground truth simulator |
| `python check_lanes.py` | ketelitian geometri lajur (lebar, simpangan, sudut hadap) vs peta HD |

## Arsitektur

```
localization.py  ground truth CARLA -> frame right-handed, titik sumbu belakang
perception.py    GroundTruth + VisionPerception -> halangan dalam FRAME EGO
overlay.py        overlay video: deteksi, kandidat planner, HUD  [butuh cv2]
planning.py      quintic/quartic, local planner, BehaviorFSM      [tanpa carla]
tracking.py      asosiasi dua tahap + Kalman filter halangan        [tanpa carla]
lanes.py         masker lajur YOLOPX -> IPM -> kisi -> lebar & simpangan [tanpa carla]
control.py       MPC CasADi + IPOPT, ThrottlePI, konversi kemudi  [tanpa carla]
evaluation.py    sensor tabrakan + kriteria keberhasilan 11.2
simulation.py    koneksi, mode sinkron, spawn, reference path
main.py          main loop: localization/perception 20 Hz, planner 10 Hz, MPC 20 Hz
config.py        semua konstanta
```

`planning.py` dan `control.py` **tidak boleh** mengimpor `carla` (aturan 2.4
rencana kerja). Ditegakkan oleh `tests/test_architecture.py`.

---

## Status

| Tahap | Isi | Status |
|---|---|---|
| 1 | Parameter kendaraan + validasi bicycle model | selesai |
| 2 | Localization + global planner | selesai |
| 3 | Local planner (quintic + quartic) | selesai |
| 4 | Behavior FSM | selesai |
| 5 | MPC + tuning bobot | selesai |
| 6 | Integrasi end-to-end | selesai: S1 dan S3 BERHASIL, deterministik |
| 7 | Baseline Pure Pursuit/Stanley | **dibatalkan** (keputusan penulis) |
| 8 | Perception lengkap (YOLOPX) | S1 selesai: BERHASIL, terulang, sudah dituning ulang |
| 9 | Eksperimen penuh | S1 selesai: GT 5/5, vision 10/10 (100%); S2-S5 belum |

Hasil terakhir (MPC + GT perception, 16 Sep 2026), identik bit-per-bit antar-run:

| Skenario | Vonis | Jarak min antar bodi | Deviasi lajur | Durasi manuver | Catatan |
|---|---|---|---|---|---|
| S1 | **BERHASIL** | 1,42 m | 0,015 m | 11,6 s | flying overtaking |
| S3 | **BERHASIL** | 1,47 m | 0,018 m | 19,0 s | mengikuti, lalu menyalip ulang; FSM lama GAGAL (0,00 m) |

Hasil S1 setelah perception tanpa peta HD (28 Sep 2026, `WRITING_SUMMARY.md`
bagian 29). Kendaraan yang disalip **Lincoln MKZ 2020**, render `quality-level=Low`:

| Metrik | MPC + vision (10 run) | MPC + GT (5 run) |
|---|---|---|
| Vonis | **10/10 BERHASIL** | **5/5 BERHASIL** |
| Jarak min antar bodi | 1,797 ± 0,017 m | 1,432 m (sd 0,000) |
| Durasi manuver | 12,12 ± 0,05 s | 11,65 s |
| **Deviasi lajur, SEBELUM manuver** | **0,0176 ± 0,0016 m** | **0,0000 m** |
| Deviasi lajur, ekor SESUDAH manuver | 0,1303 ± 0,0012 m | 0,0286 m |
| Galat prediksi @ 0,5 s, RMS | 0,0394 m | 0,0203 m |
| XTE ke lajur terdekat, RMS | 0,570 m | 0,535 m |
| Kegagalan solver | 0 dari 4.000 | 0 dari 2.000 |

Deviasi lajur **dipisah sebelum/sesudah manuver**: digabung, angkanya hampir
seluruhnya berisi ekor transien kembali, bukan kualitas menjaga lajur (bagian
15.5 dan 29.1).

Jalur GT tetap deterministik penuh: lima run identik bit-per-bit. Tabel S1/S3 di
atas memakai kendaraan target LAMA (Nissan Patrol) dan belum diukur ulang.

Angka GT di atas setelah perbaikan jangkar halangan 1,433 m (16 Sep). Sebelumnya
1,43 / 1,51 m: zona aman dulu lebih konservatif daripada rancangannya.

Deviasi turun 0,161 -> 0,011 m setelah syarat awal percepatan lateral planner
diambil dari rencana, bukan hasil ukur (`TUNING_MPC.md` 13.5). S3 wajib
`--detik 25`: lebih lama dari itu ego melewati ujung ruas lurus 400 m dan
menabrak guardrail.

---

## Pekerjaan yang Belum Selesai

Diurutkan dari yang paling mendesak. Terakhir diperbarui 17 September 2026.

### 1. Skenario S2, S4, S5 tidak punya definisi
Bukan "belum diimplementasikan" — **naskahnya tidak ada di repo ini sama sekali**.
Yang tercatat hanya sifat S5 (kendaraan depan mengerem mendadak), yang butuh
profil kecepatan terjadwal di `main.spawn_kendaraan`. Definisi S1 dan S3 di
`config.SKENARIO` pun rekonstruksi, bukan salinan bagian 11.3 rencana kerja —
cocokkan dulu sebelum ditulis di skripsi.

### 2. Data leakage YOLOPX — masalah KEABSAHAN, bukan performa
Split per-frame membuat frame berurutan dari sesi rekaman yang sama masuk train
dan val sekaligus. Angka pelatihan (mAP50 0,991, `WRITING_SUMMARY.md` 18.1)
**tidak boleh diklaim apa adanya**. Dua jalan: latih ulang dengan split per-sesi,
atau nyatakan eksplisit bahwa angka pelatihan tidak sah dan bersandar sepenuhnya
pada pengukuran terhadap simulator (bagian 18.2-18.4 dan 19.2), yang memang
bersih karena diukur di lingkungan uji, bukan di data latih.

### 3. Kamera belakang — sisa dari dua cacat yang sudah diperbaiki
Dua cacat jalur vision ditemukan dan **diperbaiki** 17 September 2026
(`WRITING_SUMMARY.md` bagian 26 mendiagnosis, bagian 27 memperbaiki dan mengukur
ulang 10 run). Yang tersisa bermuara ke satu hal: rig hanya punya kamera depan.

| Sisa | Sesudah perbaikan | Acuan GT |
|---|---|---|
| tick tanpa kandidat planner | 33,8 (dari 39,2) | 4,0 |
| `g` zona aman yang DILIHAT MPC | 0,931 (sesungguhnya 1,070) | 1,022 |
| jarak saat memutuskan kembali | -18,0 m (konservatif, buta 3,3 s) | -10,4 m |

Ongkos kamera belakang 14,4 ms per tick; anggaran masih cukup
(19,4 + 14,4 = 33,8 dari 50 ms). **S3 dengan vision juga menunggu ini**:
kendaraan lajur tujuan mulai 10 m di belakang ego dan tidak pernah terlihat,
sehingga gerbang `D_SAFE_BELAKANG` selalu lolos bukan karena aman melainkan
karena tidak terlihat.

### 4. Perbandingan bagian 27 versus 29 tidak bersih
Kendaraan target berganti (Nissan Patrol -> Lincoln MKZ 2020) **bersamaan** dengan
perombakan jalur perception, jadi selisih angka antara `WRITING_SUMMARY.md`
bagian 27 dan 29 memuat dua sebab sekaligus. Yang bisa disimpulkan hanya yang
kasar: menghapus peta HD tidak menurunkan tingkat keberhasilan, jarak aman,
maupun kualitas prediksi.

Untuk perbandingan bersih, konfigurasi lama harus dijalankan dengan MKZ. Belum
dikerjakan, dan harus dinyatakan bila selisihnya dikutip.

### 5. Tabel hasil S1/S3 ground truth memakai kendaraan lama
Baris S1 dan S3 di bagian Status masih Nissan Patrol; S3 belum pernah diukur ulang
dengan MKZ sama sekali.

### 6. Validasi perception saat berdampingan belum terkendali
`check_estimation.py` menyapu 55 → 9 m tetapi seluruhnya di lajur ego dengan ego
berjalan lurus. Angka untuk kasus berdampingan (bias +0,39 m, maks +2,41 m)
diambil dari log run loop tertutup — bukan sapuan yang dirancang. Padahal di
situlah `perception.koreksi_muka` bekerja paling keras, dan asumsi "ego dan
target sehadap" melemah saat yaw ego mencapai 10,8°.

### 7. 36,6 tick tanpa kandidat planner (vision) versus 4 (ground truth)
Terurai jadi tiga sebab berbeda (`WRITING_SUMMARY.md` bagian 26.3). Sepuluh tick
"halangan hantu" **sudah hilang** setelah perbaikan bagian 27, persis seperti
diramalkan. Diukur ulang 28 Sep 2026: 36,6 ± 0,9. Sisanya:

| Sebab | Tick | Status |
|---|---|---|
| jepitan awal pindah lajur — ada juga di GT | ~6 | wajar |
| asimetri planner-MPC (bagian 19.9): planner menolak keras di sepanjang horizon, MPC menerima lunak | ~22 | **perubahan rancangan**, bukan tuning |
| halangan hantu dari galat estimasi melintang | 10 → **0** | selesai |

Klaim lama "38-44 tick bertahan di seluruh sapuan, jadi ini geometri bukan
tuning" benar untuk kelompok kedua, dan terbukti salah untuk kelompok ketiga.

Tidak menurunkan keselamatan: jarak bodi 1,797 m terhadap syarat 1,0 m, karena
sejak planner berkomitmen pada rencana terakhirnya, replan yang gagal bukan lagi
kehilangan arah.

### 8. Skrip rekam lama masih playback dan hardcode
`main.py --perception vision --rekam` sudah merekam **hasil kendali sungguhan**
dengan overlay deteksi dan kandidat planner, jadi kebutuhan utamanya tertutupi.
Yang tersisa: `record_maneuver.py` masih playback (physics mati, ego ditempel ke
lintasan planner) dan hardcode 13,9 / 7,0 / 50 m, serta menuliskan offset sumbu
belakang `-1.4329...` alih-alih membaca `out/vehicle_params.json`.

**Keduanya juga tidak bisa dijalankan di mesin ini**: `record_path.py` dan
`record_maneuver.py` memanggil ffmpeg, yang tidak terpasang. `overlay.py` sudah
memakai `cv2.VideoWriter` dan tidak butuh ffmpeg.

### 9. Sitasi
- **ByteTrack** — strateginya dipakai `tracking.py`, tapi sumbernya belum
  dibuka, jadi sengaja tidak ditulis sebagai entri pustaka. Terbit 2022, tepat
  di batas aturan empat tahun.
- **Flash & Hogan (1985)** — dasar quintic minimum-jerk, belum diverifikasi ke
  sumber primer.
- **Kebijakan aturan 4 tahun** untuk sumber asal konsep (Werling 2010,
  Hayward 1972, Flash & Hogan 1985, KITTI 2013) belum diputuskan. Lihat
  `WRITING_SUMMARY.md` bagian 16.

### 10. Transien kembali ke lajur belum ditelusuri
Setelah kembali, ego melampaui tengah lajur sampai **+0,30 m** dan butuh lebih
dari 6 detik mengendap; jendela run 20 detik berakhir sebelum selesai
(`WRITING_SUMMARY.md` bagian 29.5). Tidak menurunkan keselamatan dan tidak
menggagalkan syarat lulus, tetapi ia yang mendominasi IAE dan ITAE.

Menelusurinya menuntut percobaan yang dirancang untuk itu. Empat dugaan sudah
gugur pada 28 Sep; jangan menambah dugaan kelima tanpa mengukur.

### 11. Waktu solve maksimum perlu diukur di mesin senggang
Terukur 36,18 ± 9,97 ms dengan satu run menyentuh **55,20 ms**, melewati anggaran
tick 50 ms. Diambil saat mesin menjalankan 13 langkah beruntun, dan rata-ratanya
justru turun ke 17,21 ms -- jadi kemungkinan besar kontensi, bukan regresi
solver. Harus diulang sebelum dikutip.

### 12. Lain-lain
- Klaim real-time sudah diukur (`WRITING_SUMMARY.md` 21.3), tetapi mesin **tidak
  benar-benar senggang** saat pengukuran (load ~3-5). Angka di mesin sepi
  kemungkinan sedikit lebih baik, bukan lebih buruk.
- Kasus terburuk anggaran tick praktis menyentuh batas: 14,4 + 34,5 = 48,9 ms
  dari 50 ms. Harus ditulis apa adanya, bukan dilaporkan sebagai "17,9 dari 50".
- `Q[v]` dan `R[a]` sengaja tidak dituning; `ThrottlePI` sudah menangani
  kecepatan (`TUNING_MPC.md` 6.4).
- Ego melebar sampai −4,28 m saat menyalip dengan vision (tepi lajur −5,25 m);
  marginnya 0,97 m. Sebelum perbaikan bagian 27 angkanya −4,71 m dengan margin
  0,54 m — lambungan berlebih itu buah dari halangan hantu, bukan rancangan.

---

## Hal yang Perlu Diperhatikan

**Lingkungan**

- **Semua perintah aktor wajib lewat `simulation.tick`.** `apply_control`,
  `set_target_velocity`, `set_transform` asinkron dan balapan dengan
  `world.tick()`: tanpa ini lima run identik memberi lima hasil berbeda, satu
  gagal. Ditegakkan `test_architecture.py`.

- **Server CARLA menurun setelah berjalan berjam-jam.** Terukur: kode identik,
  waktu solve 26–31 ms saat senggang vs 70 ms saat CARLA memakai 141% CPU.
  **Restart server** sebelum mengukur apa pun yang berbasis waktu.
- **Peringatan `Unable to import Axes3D`** muncul di setiap run. Sebabnya
  matplotlib ganda (sistem 3.5.1 di `/usr/lib/python3/dist-packages` + pip
  3.10.9). Tidak berbahaya — plot 3D tidak dipakai.
- Uji waktu solve sengaja meng-assert **jumlah iterasi**, bukan milidetik. Uji
  berbasis waktu gagal karena beban mesin, bukan karena kode.

**Koordinat dan tanda**

- **Konversi kemudi wajib menegasikan.** delta right-handed positif = belok
  **kiri**; steer CARLA positif = belok **kanan**. Tanpa negasi mobil keluar
  jalan 13,9 m. Dikunci `test_steer_membalik_tanda`.
- **Penyebut konversi kemudi = `delta_max_phys × curve(v)`**, bukan
  `delta_max`. Rumus bagian 7.5 rencana kerja salah (understeer 2,4×). Sumbu-x
  `steering_curve` dalam **km/jam**.
- **Jangan pernah mengurangi dua sudut tanpa `localization.wrap`.** Peta Town04
  menyimpan arah ruas 902 sebagai 450,22° (= 90,22 + 360); tanpa normalisasi
  error arah terbaca 359° padahal 0,8°.
- **Perception keluar frame ego**, konversi di `localization.halangan_ego_ke_jalan`.
  FSM memakai `x` relatif ego; planner & MPC memakai `x` absolut frame jalan.
  Main loop menggeser di antaranya.

**MPC dan pengukuran**

- **Batas state MPC tidak boleh berlaku di `k = 0`.** State awal dikunci ke hasil
  ukur; kelebihan 0,0003 m/s saja membuat solver `Infeasible_Problem_Detected`.
- **`V_REF` (13,4) sengaja dibedakan dari `V_MAX` (13,9).** Disamakan, constraint
  keras aktif 65% waktu dan solver bekerja di tepi kelayakan.
- **Percepatan terukur berderau berat** (−26,8..+12,8 m/s²). Dipotong ke batas
  fisik lalu low-pass. `a0` quartic memakai percepatan **yang diperintahkan**.
- **Kolom log diakses lewat nama** (`main.KOLOM`), bukan angka. Menyisipkan kolom
  sudah dua kali menggeser indeks tanpa error.
- **Jarak antar kendaraan untuk penilaian diukur antar bodi**, bukan antar pusat,
  dan dari **ground truth** — bukan perception. Yang dinilai harus benar; yang
  dipakai mobil boleh berisik.
- **Sensor tabrakan hanya di `evaluation.py`.** Instrumen pengukuran, bukan
  masukan kendali. Ditegakkan `test_architecture.py`.
- **`tuning.py` men-spawn ulang ego tiap konfigurasi.** Jangan diganti
  `set_transform` — putaran roda terbawa dan hasilnya bergantung pada urutan sapuan.
- **Periksa alasan di balik setiap perbaikan angka.** Selama pengembangan, metrik
  tiga kali memberi kesimpulan yang keliru. Detail di `TUNING_MPC.md` bagian 9.

**Skenario**

- Spawn point `SPAWN_IDX = 75`, lurus 400 m, lebar lajur 3,50 m. 40% jalurnya
  area junction (ramp highway). Kalau TrafficManager berperilaku aneh di Tahap
  9, **spawn 79** cadangannya.
- Ego perlu **fase pemanasan** (`WARMUP_DETIK = 6`): `set_target_velocity` hanya
  menetapkan kecepatan bodi, roda masih diam, dan transiennya memakan 22% run.
- Simulasi **deterministik bit-per-bit** untuk hasil kendali. Kalau dua run
  identik memberi angka berbeda, ada yang salah.
