# Skripsi — MPC untuk Manuver Overtaking di CARLA

Implementasi Model Predictive Control untuk skenario manuver menyalip pada
kendaraan otonom, di CARLA Simulator 0.9.16.

Dokumen pendamping:

| File | Isi |
|---|---|
| `RANGKUMAN_PENULISAN.md` | angka, sitasi, dan keputusan terverifikasi — untuk menulis skripsi |
| `TUNING_MPC.md` | seluruh proses tuning bobot: teori, data percobaan, alasan berhenti |
| `CATATAN.md` | catatan kerja kronologis lengkap, termasuk bug dan jalan buntu |

---

## Setup

```bash
# 1. dependensi sistem
sudo apt install ffmpeg

# 2. dependensi Python (3.10)
pip install -r requirements.txt

# 3. jalankan server CARLA 0.9.16 terpisah, lalu:
python extract_params.py      # hanya kalau out/vehicle_params.json hilang
```

Versi paket `carla` **wajib sama persis** dengan versi server. Lihat catatan di
`requirements.txt`.

## Menjalankan

**Uji otomatis — tidak butuh server CARLA.** 57 uji, semuanya lolos.

```bash
for f in tests/*.py; do python "$f"; done
```

Uji dijalankan sebagai skrip, bukan lewat pytest. `tests/test_mpc.py` butuh
`out/vehicle_params.json` (sudah ikut di repo).

**Butuh server CARLA:**

| Perintah | Fungsi |
|---|---|
| `python main.py` | skenario S1 lengkap, loop tertutup, vonis berhasil/gagal |
| `python tuning.py --sweep Q_PSI 300,450,600` | harness tuning step response |
| `python validate_model.py` | validasi bicycle model terhadap plant |
| `python validate_model.py --steer` | verifikasi konversi kemudi |
| `python validate_model.py --scan` | cari spawn point ruas lurus |
| `python record_maneuver.py --kamera atas` | video dengan overlay kandidat |
| `python show_lanes.py` | gambar lingkungan uji dan kandidat planner |

## Arsitektur

```
localization.py  ground truth CARLA -> frame right-handed, titik sumbu belakang
perception.py    GroundTruthPerception -> halangan dalam FRAME EGO
planning.py      quintic/quartic, local planner, BehaviorFSM      [tanpa carla]
control.py       MPC CasADi + IPOPT, ThrottlePI, konversi kemudi  [tanpa carla]
evaluation.py    sensor tabrakan + kriteria keberhasilan 11.2
simulation.py    koneksi, mode sinkron, spawn, reference path
main.py          main loop: localization/perception 20 Hz, planner 10 Hz, MPC 20 Hz
config.py        semua konstanta
```

`planning.py` dan `control.py` **tidak boleh** mengimpor `carla` (aturan 2.4
rencana kerja). Ditegakkan oleh `tests/test_arsitektur.py`.

---

## Status

| Tahap | Isi | Status |
|---|---|---|
| 1 | Parameter kendaraan + validasi bicycle model | selesai |
| 2 | Localization + global planner | selesai |
| 3 | Local planner (quintic + quartic) | selesai |
| 4 | Behavior FSM | selesai |
| 5 | MPC + tuning bobot | selesai |
| 6 | Integrasi end-to-end | **tercapai**, verifikasi akhir belum (lihat #1 di bawah) |
| 7 | Baseline Pure Pursuit/Stanley | **dibatalkan** (keputusan penulis) |
| 8 | Perception lengkap (YOLOPX) | belum |
| 9 | Eksperimen penuh | belum |

Hasil terakhir skenario S1 (MPC + GT perception): deviasi tengah lajur 0,120 m
rata-rata saat `LANE_KEEPING`, 0 solver gagal, 0 tabrakan, urutan state FSM
lengkap.

---

## Pekerjaan yang Belum Selesai

Diurutkan dari yang paling mendesak.

### 1. `main.py` dengan kriteria keberhasilan belum pernah dijalankan end-to-end
Kriteria bagian 11.2 selesai ditulis dan lolos 7 uji sintetis, tapi server CARLA
mati sebelum sempat dijalankan penuh. **Jalankan `python main.py` lebih dulu**
dan pastikan vonisnya `BERHASIL — berhasil`.

### 2. Video hasil kendali MPC
`record_maneuver.py` masih **playback**: physics dimatikan, posisi ego ditempel
ke lintasan planner. Ganti sumber gerakannya jadi `apply_control` dari MPC;
bagian rekam (`record_path.capture`) dan overlay kandidat tidak perlu diubah.

### 3. Skenario S2–S5 belum ada, dan parameter skenario tidak konsisten
Bagian 11.1 minta definisi skenario sebagai konstanta di `config.py`, ditetapkan
sekali. Sekarang tersebar dan **saling bertentangan**:

| Berkas | Kecepatan ego | Kecepatan target | Jarak awal |
|---|---|---|---|
| `main.py` | `config.V_REF` = 13,4 m/s | 7,0 (hardcode) | 60 m (hardcode) |
| `record_maneuver.py` | **13,9 (hardcode)** | 7,0 (hardcode) | **50 m (hardcode)** |
| `record_path.py` | **13,9 (hardcode)** | — | — |

Selain itu, `record_maneuver.py` baris ~181 menuliskan langsung offset sumbu belakang
`-1.4329...` alih-alih membaca `out/vehicle_params.json`. S5 (kendaraan depan
mengerem mendadak) butuh profil kecepatan terjadwal.

### 4. Tahap 8 — `VisionPerception`
YOLOPX + ByteTrack + depth camera + Kalman filter, bagian 10 rencana kerja.
Belum dimulai. Termasuk **perbaikan data leakage** YOLOPX (split per-frame;
akurasi 96–98% sekarang tidak valid).

Antarmukanya sudah siap: keluarkan `ndarray (M, 4) = [x, y, vx, vy]` dalam
**frame ego**, posisi dan kecepatan **relatif** terhadap ego — sama persis
dengan `GroundTruthPerception`. Tidak perlu tahu soal frame jalan.

### 5. Tuning ulang setelah Tahap 8
Bagian 10.6: deteksi vision lebih berisik, bobot MPC dan parameter FSM **wajib**
dituning ulang. Bobot sekarang dituning di atas ground truth. Harness-nya siap
(`tuning.py`).

### 6. Tahap 9 — eksperimen penuh
Matriks bagian 11.4 tinggal dua baris: MPC + GT dan MPC + vision. Masing-masing
skenario diulang 10–20 kali.

### 7. Lain-lain
- Sitasi Flash & Hogan (1985) belum diverifikasi ke sumber primer.
- Klaim real-time: waktu solve **harus diukur di mesin senggang** — lihat
  "Perlu diperhatikan".
- `Q[v]` dan `R[a]` sengaja tidak dituning; `ThrottlePI` sudah menangani
  kecepatan (lihat `TUNING_MPC.md` bagian 6.4).

---

## Hal yang Perlu Diperhatikan

**Lingkungan**

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
  masukan kendali. Ditegakkan `test_arsitektur.py`.
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
