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

**Uji otomatis — tidak butuh server CARLA.** 135 uji, semuanya lolos.

```bash
for f in tests/*.py; do python "$f"; done
```

Uji dijalankan sebagai skrip, bukan lewat pytest. `tests/test_mpc.py` butuh
`out/vehicle_params.json` (sudah ikut di repo).

**Butuh server CARLA:**

| Perintah | Fungsi |
|---|---|
| `python main.py` | skenario S1 lengkap, loop tertutup, vonis berhasil/gagal |
| `python main.py --scenario S3 --seconds 25` | lajur tujuan terisi: mengikuti, lalu menyalip ulang |
| `python main.py --perception vision --record` | S1 dengan YOLOPX + depth, plus video overlay |
| `python main.py --perception vision --record --suffix _after` | sama, tapi keluarannya tidak menimpa berkas pembanding |
| `python check_estimation.py` | ketelitian jarak & kecepatan vision vs ground truth |
| `python tuning.py --sweep Q_PSI 300,450,600` | harness tuning step response |
| `python tune_vision.py --sweep K_DEV 10,20,40` | sapuan parameter di skenario penuh + vision |
| `python experiment.py --perception vision --repeat 10` | Tahap 9: success rate + sebaran metrik |
| `python metrics.py --layer --experiment` | metrik per layer/fase dari log (tidak butuh server) |
| `python metrics.py --layer --experiment --suffix _before` | metrik yang sama SEBELUM perbaikan bagian 27 |
| `python validate_model.py` | validasi bicycle model terhadap plant |
| `python validate_model.py --steer` | verifikasi konversi kemudi |
| `python validate_model.py --scan` | cari spawn point ruas lurus |
| `python record_maneuver.py --camera top` | video dengan overlay kandidat |
| `python show_lanes.py` | gambar lingkungan uji dan kandidat planner |
| `python plot_run.py --scenario S3` | grafik hasil run dari log (tidak butuh server) |
| `python plot_compare.py` | grafik pembanding GT vs vision (tidak butuh server) |
| `python record_path.py` | video lintasan acuan global planner (butuh ffmpeg) |
| `python check_sensors.py` | pasang rig kamera, verifikasi penempatan, simpan contoh frame |
| `python show_rig.py` | konfigurasi sensor ala KITTI: foto ego + skema berdimensi |
| `python plot_concepts.py` | gambar konsep: rig, frame, skenario, pipeline, MPC (tanpa server) |
| `python check_detection.py --lane 1` | ukur deteksi YOLOPX terhadap ground truth simulator |
| `python check_lanes.py` | ketelitian geometri lajur (lebar, simpangan, sudut hadap) vs peta HD |
| `python check_lanes.py --weight ~/sawal/model/yolopx-continuous.pt` | sapuan yang sama pakai checkpoint lain (bagian 30.7) |
| `python check_ipm.py` | pembuktian IPM: f, bolak-balik, matriks CARLA, kamera depth |
| `python plot_ipm.py` | IPM dijelaskan di atas frame kamera + warp pandangan atas |
| `python plot_lane_fit.py` | shear salah vs tercocok, plus pencocokan kisi |
| `python plot_lane_pipeline.py` | tiga tahap lanes.py, dua model anotasi berdampingan |

**Checkpoint YOLOPX.** Sejak 2 Oktober 2026 dipakai `weights/best.pth` **epoch 92**,
pengganti checkpoint epoch 263; angka perbandingannya di `WRITING_SUMMARY.md`
bagian 31. Sebelumnya dipakai `yolopx-marking.pt` (anotasi per-marka). Varian
`yolopx-continuous.pt` (anotasi batas lajur menerus) diuji dan **ditolak**: ia
menarik garis lajur di tempat yang tidak ada markanya -- termasuk di atas rel
kereta -- dan ketelitian geometri lajurnya dua kali lebih buruk meski pikselnya
3,6 kali lebih banyak. Angka lengkap `WRITING_SUMMARY.md` bagian 30.7.

## Struktur File

Seluruh nama file, fungsi, class, variabel, konstanta, argumen CLI, kolom log,
dan teks keluaran memakai bahasa Inggris. Komentar dan docstring tetap
berbahasa Indonesia.

```
carla-ads-overtaking/
├── Modul inti (dipanggil main loop)
│   ├── config.py           semua konstanta: skenario, bobot MPC, ambang FSM, kamera
│   ├── simulation.py       koneksi, mode sinkron, spawn, reference path, tick()
│   ├── localization.py     ground truth CARLA -> frame right-handed, titik sumbu belakang
│   ├── sensors.py          CameraRig: kamera RGB + depth terpasang di ego
│   ├── yolopx.py           pembungkus model YOLOPX (deteksi + segmentasi)       [butuh torch]
│   ├── perception.py       GroundTruthPerception + VisionPerception -> halangan FRAME EGO
│   ├── tracking.py         asosiasi dua tahap + Kalman filter halangan          [tanpa carla]
│   ├── lanes.py            garis lajur di area jalan -> IPM -> kisi; porsi lajur tujuan & tepi jalan [tanpa carla]
│   ├── planning.py         quintic/quartic, local planner, BehaviorFSM          [tanpa carla]
│   ├── control.py          MPCController (CasADi + IPOPT), ThrottlePI, steer    [tanpa carla]
│   ├── evaluation.py       sensor tabrakan + kriteria keberhasilan 11.2
│   ├── overlay.py          Recorder: video overlay deteksi, kandidat, HUD       [butuh cv2]
│   └── main.py             main loop: perception 20 Hz, planner 10 Hz, MPC 20 Hz
│
├── Eksperimen & tuning (butuh server CARLA)
│   ├── experiment.py       Tahap 9: N ulangan, success rate + sebaran metrik
│   ├── tuning.py           harness step response untuk bobot MPC
│   ├── tune_vision.py      sapuan parameter di skenario penuh + vision
│   ├── extract_params.py   ekstraksi parameter fisik ego -> out/vehicle_params.json
│   └── validate_model.py   validasi bicycle model, konversi kemudi, cari spawn lurus
│
├── Pemeriksaan (butuh server CARLA)
│   ├── check_sensors.py    penempatan rig kamera vs angka KITTI
│   ├── check_detection.py  deteksi YOLOPX vs ground truth simulator
│   ├── check_estimation.py jarak & kecepatan vision vs ground truth
│   ├── check_lanes.py      geometri lajur YOLOPX vs peta HD
│   └── check_ipm.py        pembuktian IPM: f, bolak-balik, matriks CARLA, depth
│
├── Gambar & video
│   ├── metrics.py          metrik bab 4 per fase/layer dari log        [tanpa server]
│   ├── plot_run.py         grafik satu run tertutup                    [tanpa server]
│   ├── plot_compare.py     MPC + GT versus MPC + vision                [tanpa server]
│   ├── plot_concepts.py    gambar konsep: rig, frame, skenario, MPC    [tanpa server]
│   ├── plot_ipm.py         IPM di atas frame kamera + warp pandangan atas
│   ├── plot_lane_fit.py    kemiringan bersama + pencocokan kisi
│   ├── plot_lane_pipeline.py  tiga tahap lanes.py, dua model anotasi
│   ├── show_lanes.py       lingkungan uji dan kandidat planner
│   ├── show_rig.py         konfigurasi sensor ala KITTI
│   ├── record_maneuver.py  video playback manuver                      [butuh ffmpeg]
│   └── record_path.py      video lintasan acuan global planner         [butuh ffmpeg]
│
├── tests/                  135 uji, dijalankan sebagai skrip           [tanpa server]
│   ├── test_architecture.py  aturan 2.4: modul numerik tidak menyentuh carla
│   ├── test_bicycle_model.py, test_dimensions.py, test_evaluation.py, test_fsm.py
│   └── test_lanes.py, test_localization.py, test_mpc.py, test_perception.py,
│       test_planning.py, test_tracking.py
│
├── out/                    keluaran: gambar bab 3-4, log run (.npz), vehicle_params.json
│
└── README.md, WRITING_SUMMARY.md, TUNING_MPC.md, NOTES.md
```

`planning.py` dan `control.py` **tidak boleh** mengimpor `carla` (aturan 2.4
rencana kerja). Ditegakkan oleh `tests/test_architecture.py`.

**Log `.npz` lama.** Kunci dan nama kolom di `out/*.npz` sudah dimigrasi ke nama
Inggris (`columns`, `vehicle_positions`, `lane_dev`, `n_feasible`, `x_map`,
`min_dist`, `verdict`, ...). Nilainya tidak berubah: `metrics.py` memberi angka
yang identik dengan sebelum migrasi.

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
| 8 | Perception lengkap (YOLOPX) | S1 selesai: BERHASIL, terulang; drivable area dipakai kendali (bagian 31) |
| 9 | Eksperimen penuh | S1 selesai: GT 5/5, vision 10/10 (100%); S2-S5 belum |

Hasil terakhir (MPC + GT perception, 2 Okt 2026, kendaraan target **Lincoln MKZ
2020**), identik bit-per-bit antar-run:

| Skenario | Vonis | Jarak min antar bodi | Deviasi lajur | Durasi manuver | Catatan |
|---|---|---|---|---|---|
| S1 | **BERHASIL** | 1,43 m | 0,015 m | 11,6 s | flying overtaking |
| S3 | **BERHASIL** | 1,52 m | 0,018 m | 19,1 s | mengikuti, lalu menyalip ulang; FSM lama GAGAL (0,00 m) |

Hasil S1 dengan perception tanpa peta HD, checkpoint epoch 92, drivable area, dan
perbaikan kembali ke tengah lajur, tanpa fallback ke konstanta peta (3 Okt 2026,
`WRITING_SUMMARY.md` bagian 32 dan 34). Render `quality-level=Low`:

| Metrik | MPC + vision (10 run) | MPC + GT (5 run) |
|---|---|---|
| Vonis | **10/10 BERHASIL** | **5/5 BERHASIL** |
| Jarak min antar bodi | 1,812 ± 0,026 m | 1,432 m (sd 0,000) |
| Durasi manuver | 12,52 ± 0,09 s | 11,65 s |
| **Deviasi lajur, SEBELUM manuver** | **0,0110 ± 0,0004 m** | **0,0000 m** |
| Deviasi lajur, ekor SESUDAH manuver | 0,0501 ± 0,0012 m | 0,0286 m |
| Galat prediksi @ 0,5 s, RMS | 0,0407 m | 0,0203 m |
| XTE ke lajur terdekat, RMS | 0,563 m | 0,535 m |
| Kegagalan solver | 0 dari 4.000 | 0 dari 2.000 |
| Waktu solve rata-rata / maks | 16,81 / 31,37 ms | 15,67 / 28,83 ms |
| Lebar lajur yang dipakai FSM | 3,512-3,559 m (ukur) | 3,50 m (peta) |

Deviasi lajur **dipisah sebelum/sesudah manuver**: digabung, angkanya hampir
seluruhnya berisi ekor transien kembali, bukan kualitas menjaga lajur (bagian
15.5 dan 29.1).

Jalur GT tetap deterministik penuh: lima run identik bit-per-bit.

Angka GT di atas setelah perbaikan jangkar halangan 1,433 m (16 Sep). Sebelumnya
1,43 / 1,51 m: zona aman dulu lebih konservatif daripada rancangannya.

Deviasi turun 0,161 -> 0,011 m setelah syarat awal percepatan lateral planner
diambil dari rencana, bukan hasil ukur (`TUNING_MPC.md` 13.5). S3 wajib
`--seconds 25`: lebih lama dari itu ego melewati ujung ruas lurus 400 m dan
menabrak guardrail.

---

## Pekerjaan yang Belum Selesai

Diurutkan dari yang paling mendesak. Terakhir diperbarui 17 September 2026.

### 1. Skenario S2, S4, S5 tidak punya definisi
Bukan "belum diimplementasikan" — **naskahnya tidak ada di repo ini sama sekali**.
Yang tercatat hanya sifat S5 (kendaraan depan mengerem mendadak), yang butuh
profil kecepatan terjadwal di `main.spawn_vehicles`. Definisi S1 dan S3 di
`config.SCENARIOS` pun rekonstruksi, bukan salinan bagian 11.3 rencana kerja —
cocokkan dulu sebelum ditulis di skripsi.

### 2. ~~Data leakage YOLOPX~~ -- selesai 2 Okt 2026
Checkpoint epoch 92 dilatih dengan split per REKAMAN (27 train / 7 val / 1 test).
Angka test (Town04, rekaman 32): mAP@0,5 0,994, mAP@0,5:0,95 0,943, IoU area jalan
0,985 (`WRITING_SUMMARY.md` bagian 33). Catatan yang harus ikut ditulis: test
hanya satu rekaman, dan Town04 juga ada di train/val -- evaluasi simulator
mengukur kinerja dalam domain latih, bukan generalisasi ke peta baru.

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
sehingga gerbang `D_SAFE_REAR` selalu lolos bukan karena aman melainkan
karena tidak terlihat.

### 4. Perbandingan bagian 27 versus 29 tidak bersih
Kendaraan target berganti (Nissan Patrol -> Lincoln MKZ 2020) **bersamaan** dengan
perombakan jalur perception, jadi selisih angka antara `WRITING_SUMMARY.md`
bagian 27 dan 29 memuat dua sebab sekaligus. Yang bisa disimpulkan hanya yang
kasar: menghapus peta HD tidak menurunkan tingkat keberhasilan, jarak aman,
maupun kualitas prediksi.

Untuk perbandingan bersih, konfigurasi lama harus dijalankan dengan MKZ. Belum
dikerjakan, dan harus dinyatakan bila selisihnya dikutip.

### 5. ~~Tabel hasil S1/S3 ground truth memakai kendaraan lama~~ -- selesai 2 Okt 2026
S1 dan S3 diukur ulang dengan MKZ: 1,43 m / 11,6 s dan 1,52 m / 19,1 s, keduanya
BERHASIL. Tabel di bagian Status sudah memakai angka ini.

### 6. Validasi perception saat berdampingan belum terkendali
`check_estimation.py` menyapu 55 → 9 m tetapi seluruhnya di lajur ego dengan ego
berjalan lurus. Angka untuk kasus berdampingan (bias +0,39 m, maks +2,41 m)
diambil dari log run loop tertutup — bukan sapuan yang dirancang. Padahal di
situlah `perception.face_correction` bekerja paling keras, dan asumsi "ego dan
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
`main.py --perception vision --record` sudah merekam **hasil kendali sungguhan**
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

### 10. ~~Transien kembali ke lajur belum ditelusuri~~ -- sebagian besar selesai 2 Okt 2026
Sebabnya bukan pengendali: frame jalan vision berputar pelan oleh bias arah,
sedangkan tengah lajur asal dibekukan sebagai koordinat selama manuver. Dua
perbaikan (`WRITING_SUMMARY.md` bagian 32): arah jalan kini dari tick yang sama,
dan yang dikunci identitas lajur asal, bukan koordinatnya. Lampauan setelah
kembali **+0,37 -> +0,11 m**, ekor deviasi **0,132 -> 0,050 m**.

Yang tersisa: bias arah statis 0,1-0,2 deg yang bergantung lajur. Tiga dugaan
gugur (pitch, roll, jalan menurun); sebabnya belum ketemu.

### 11. ~~Waktu solve maksimum perlu diukur di mesin senggang~~ -- selesai 3 Okt 2026
Diulang dengan server CARLA baru dinyalakan, tanpa beban lain: maksimum **31,37 ms**
(vision, 10 run) dan 28,83 ms (GT, 5 run), semua di bawah anggaran 50 ms. Angka
55,20 ms lama memang kontensi (`WRITING_SUMMARY.md` bagian 34).

### 12. Kalibrasi tinggi kamera belum dilakukan
`check_ipm.py` mengukur tinggi kamera efektif **1,6368 m** terhadap 1,6500 m yang
dikonfigurasi -- ego duduk di suspensi. Selisih 0,8% itu masuk ke seluruh jarak
memanjang dan bisa dikalibrasi keluar dengan satu konstanta. Belum dikerjakan.

Terukur juga: asumsi jalan datar berbiaya **~0,73 m galat jarak di 42 m**
(kemiringan jalan ~0,076%). Itu bukan cacat melainkan harga asumsinya, dan kini
ada angkanya untuk ditulis di batasan masalah.

### 13. Lain-lain
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
  jalan 13,9 m. Dikunci `test_steer_flips_sign`.
- **Penyebut konversi kemudi = `delta_max_phys × curve(v)`**, bukan
  `delta_max`. Rumus bagian 7.5 rencana kerja salah (understeer 2,4×). Sumbu-x
  `steering_curve` dalam **km/jam**.
- **Jangan pernah mengurangi dua sudut tanpa `localization.wrap`.** Peta Town04
  menyimpan arah ruas 902 sebagai 450,22° (= 90,22 + 360); tanpa normalisasi
  error arah terbaca 359° padahal 0,8°.
- **Perception keluar frame ego**, konversi di `localization.obstacles_ego_to_road`.
  FSM memakai `x` relatif ego; planner & MPC memakai `x` absolut frame jalan.
  Main loop menggeser di antaranya.

**MPC dan pengukuran**

- **Batas state MPC tidak boleh berlaku di `k = 0`.** State awal dikunci ke hasil
  ukur; kelebihan 0,0003 m/s saja membuat solver `Infeasible_Problem_Detected`.
- **`V_REF` (13,4) sengaja dibedakan dari `V_MAX` (13,9).** Disamakan, constraint
  keras aktif 65% waktu dan solver bekerja di tepi kelayakan.
- **Percepatan terukur berderau berat** (−26,8..+12,8 m/s²). Dipotong ke batas
  fisik lalu low-pass. `a0` quartic memakai percepatan **yang diperintahkan**.
- **Kolom log diakses lewat nama** (`main.COLUMNS`), bukan angka. Menyisipkan kolom
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
- Ego perlu **fase pemanasan** (`WARMUP_SECONDS = 6`): `set_target_velocity` hanya
  menetapkan kecepatan bodi, roda masih diam, dan transiennya memakan 22% run.
- Simulasi **deterministik bit-per-bit** untuk hasil kendali. Kalau dua run
  identik memberi angka berbeda, ada yang salah.
