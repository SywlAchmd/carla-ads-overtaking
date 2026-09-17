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

**Uji otomatis — tidak butuh server CARLA.** 91 uji, semuanya lolos.

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
| `python check_estimation.py` | ketelitian jarak & kecepatan vision vs ground truth |
| `python tuning.py --sweep Q_PSI 300,450,600` | harness tuning step response |
| `python tune_vision.py --sweep K_DEV 10,20,40` | sapuan parameter di skenario penuh + vision |
| `python experiment.py --perception vision --ulang 10` | Tahap 9: success rate + sebaran metrik |
| `python metrics.py --layer --eksperimen` | metrik per layer/fase dari log (tidak butuh server) |
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

## Arsitektur

```
localization.py  ground truth CARLA -> frame right-handed, titik sumbu belakang
perception.py    GroundTruth + VisionPerception -> halangan dalam FRAME EGO
overlay.py        overlay video: deteksi, kandidat planner, HUD  [butuh cv2]
planning.py      quintic/quartic, local planner, BehaviorFSM      [tanpa carla]
tracking.py      asosiasi dua tahap + Kalman filter halangan        [tanpa carla]
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

Angka di atas setelah perbaikan jangkar halangan 1,433 m (16 Sep). Sebelumnya
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

### 3. S3 dengan vision butuh kamera belakang
Kendaraan lajur tujuan di S3 mulai **10 m di belakang ego**, dan rig satu kamera
depan (fov 90°) baru melihatnya setelah ia melewati bumper ego. Gerbang
`D_SAFE_BELAKANG` karena itu selalu lolos — bukan karena lajurnya aman,
melainkan karena tidak terlihat. Ongkos kamera belakang 14,4 ms per tick;
anggaran masih cukup (17,9 + 14,4 + 14,4 = 46,7 dari 50 ms).

### 4. Validasi perception saat berdampingan belum terkendali
`check_estimation.py` menyapu 55 → 9 m tetapi seluruhnya di lajur ego dengan ego
berjalan lurus. Angka untuk kasus berdampingan (bias +0,39 m, maks +2,41 m)
diambil dari log run loop tertutup — bukan sapuan yang dirancang. Padahal di
situlah `perception.koreksi_muka` bekerja paling keras, dan asumsi "ego dan
target sehadap" melemah saat yaw ego mencapai 10,8°.

### 5. 40 tick tanpa kandidat planner (vision) versus 4 (ground truth)
Bertahan 38-44 di **seluruh** sapuan parameter, jadi ini bukan soal tuning
melainkan geometri zona aman (`WRITING_SUMMARY.md` 19.13). Tidak menurunkan
keselamatan — jarak bodi vision justru 2,10 m versus 1,42 m milik GT — karena
sejak planner berkomitmen pada rencana terakhirnya, replan yang gagal bukan lagi
kehilangan arah. Menyelesaikannya menuntut perubahan rancangan.

### 6. Skrip rekam lama masih playback dan hardcode
`main.py --perception vision --rekam` sudah merekam **hasil kendali sungguhan**
dengan overlay deteksi dan kandidat planner, jadi kebutuhan utamanya tertutupi.
Yang tersisa: `record_maneuver.py` masih playback (physics mati, ego ditempel ke
lintasan planner) dan hardcode 13,9 / 7,0 / 50 m, serta menuliskan offset sumbu
belakang `-1.4329...` alih-alih membaca `out/vehicle_params.json`.

**Keduanya juga tidak bisa dijalankan di mesin ini**: `record_path.py` dan
`record_maneuver.py` memanggil ffmpeg, yang tidak terpasang. `overlay.py` sudah
memakai `cv2.VideoWriter` dan tidak butuh ffmpeg.

### 7. Sitasi
- **ByteTrack** — strateginya dipakai `tracking.py`, tapi sumbernya belum
  dibuka, jadi sengaja tidak ditulis sebagai entri pustaka. Terbit 2022, tepat
  di batas aturan empat tahun.
- **Flash & Hogan (1985)** — dasar quintic minimum-jerk, belum diverifikasi ke
  sumber primer.
- **Kebijakan aturan 4 tahun** untuk sumber asal konsep (Werling 2010,
  Hayward 1972, Flash & Hogan 1985, KITTI 2013) belum diputuskan. Lihat
  `WRITING_SUMMARY.md` bagian 16.

### 8. Lain-lain
- Klaim real-time sudah diukur (`WRITING_SUMMARY.md` 21.3), tetapi mesin **tidak
  benar-benar senggang** saat pengukuran (load ~3-5). Angka di mesin sepi
  kemungkinan sedikit lebih baik, bukan lebih buruk.
- Kasus terburuk anggaran tick praktis menyentuh batas: 14,4 + 34,5 = 48,9 ms
  dari 50 ms. Harus ditulis apa adanya, bukan dilaporkan sebagai "17,9 dari 50".
- `Q[v]` dan `R[a]` sengaja tidak dituning; `ThrottlePI` sudah menangani
  kecepatan (`TUNING_MPC.md` 6.4).
- Ego melebar sampai −4,71 m saat menyalip dengan vision (tepi lajur −5,25 m).
  Di dalam lajur, tapi marginnya 0,54 m.

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
