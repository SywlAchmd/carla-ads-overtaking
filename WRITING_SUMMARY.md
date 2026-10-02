# Rangkuman untuk Penulisan Skripsi

Dokumen ini dibuat untuk ditempel ke percakapan baru saat mulai menulis. Isinya
seluruh angka, sitasi, dan keputusan yang sudah terverifikasi sampai Tahap 4.
Catatan kerja lengkap ada di `NOTES.md` (1.088 baris); yang ini ringkasannya.

**Judul:** Implementasi Model Predictive Control untuk Skenario Manuver
Overtaking pada Sistem Autonomous Car Menggunakan CARLA Simulator
**Lingkungan:** CARLA 0.9.16, Ubuntu, Python 3.10. Kode di `PythonAPI/skripsi/`.

---

## 1. Status pengerjaan

| Tahap | Isi | Status |
|---|---|---|
| 1 | Parameter kendaraan + validasi bicycle model | **selesai** |
| 2 | Localization + global planner | **selesai** |
| 3 | Local planner (quintic + quartic) | **selesai** |
| 4 | Behavior planner (FSM) | **selesai** |
| 5 | MPC (CasADi + IPOPT) + tuning bobot | **selesai** |
| 6 | Integrasi end-to-end (S1 dan S3) | **selesai** |
| 7 | Baseline Pure Pursuit/Stanley | **DIBATALKAN** — lihat bagian 13 |
| 8 | Perception lengkap (YOLOPX + depth + tracking) | **selesai untuk S1** |
| 9 | Eksperimen penuh | **selesai untuk S1**; S2-S5 belum |

Kode: 8.466 baris, 133 uji otomatis semuanya lolos tanpa perlu menyalakan CARLA.
Terakhir diperbarui 2 Oktober 2026: checkpoint YOLOPX epoch 92 dan drivable area
di jalur kendali (bagian 31). Angka perception bagian 18 dan 29 memakai
checkpoint lama; pembandingnya di bagian 31.1-31.2.

**Yang boleh ditulis sekarang:** seluruh bab 3 (metodologi), dan bab 4 untuk
skenario S1 secara penuh — success rate dari ulangan yang sah (bagian 21),
metrik per layer dan per fase (bagian 23), perbandingan MPC+GT versus MPC+vision,
tuning bobot (`TUNING_MPC.md` bagian 10-11 dan bagian 20 di sini), serta seluruh
temuan metodologis (bagian 15, 19, 20).

**Belum boleh ditulis:** skenario S2/S4/S5 (definisinya tidak ada), hasil S3
dengan vision (butuh kamera belakang), dan **angka pelatihan YOLOPX** di bagian
18.1 — split-nya masih bocor. Yang sah dari perception adalah pengukuran
terhadap simulator: bagian 18.2-18.4 dan 19.2.

### Angka mana yang berlaku?

| Bagian | Kendaraan target | Checkpoint YOLOPX | Status angka |
|---|---|---|---|
| **31** | **Lincoln MKZ 2020** | **epoch 92** | **BERLAKU untuk bab 4** |
| 28-30 | Lincoln MKZ 2020 | epoch 263 (lama) | riwayat checkpoint lama; metodenya berlaku |
| 14-27 | **Nissan Patrol (lama)** | epoch 263 (lama) | riwayat; temuannya berlaku, angkanya TIDAK |
| 1-13 | -- | -- | parameter ego, lingkungan, sitasi: berlaku |

Bagian era Nissan Patrol diberi spanduk di judulnya. Kalau ragu: angka yang
dikutip di skripsi harus dari bagian 31 (atau 29 untuk metode tanpa angka baru).

---

## 2. Parameter kendaraan uji (Tahap 1)

Sumber: CARLA Python API `get_physics_control()`, diekstrak oleh
`extract_params.py`. **Bukan katalog kendaraan.** Cantumkan versi CARLA dan nama
blueprint; skripnya sendiri yang jadi bukti reproducibility.

Blueprint: `vehicle.dodge.charger_2020`

| Parameter | Nilai | Dipakai untuk |
|---|---|---|
| Wheelbase `L` | 3,044 m | prediction model (persamaan yaw rate) |
| Offset sumbu belakang | −1,433 m | konversi titik referensi |
| `delta_max` fisik | 1,222 rad (70,0°) | batas fisik roda |
| `delta_max` dipakai MPC | 0,5 rad | constraint kemudi |
| Panjang × lebar | 5,008 × 1,882 m | elips penghindaran tabrakan |
| Massa | 1.920 kg | kalibrasi throttle map |
| Drag coefficient | 0,30 | pelaporan |
| `steering_curve` | (0; 1,0), (20; 0,9), (60; 0,8), (120; 0,7) | batas kemudi vs kecepatan |

**Satuan sumbu-x `steering_curve` adalah km/jam** — ditentukan empiris, bukan dari
dokumentasi (CARLA tidak mendokumentasikannya). MAE terhadap sudut roda terukur
0,00083 rad bila ditafsirkan km/jam, versus 0,00387 rad bila m/s — 4,7× lebih
buruk. Sanity check: titik terakhir kurva di 120, dan 120 m/s = 432 km/jam.

**Offset sumbu belakang diukur, bukan diasumsikan `L/2`.** Nilai terukur
−1,433 m, sedangkan `L/2` = 1,522 m. Melewatkan konversi sama sekali memberi bias
XTE tetap **1,43 m** yang tidak hilang berapa pun bobot `Q` dinaikkan.

---

## 3. Justifikasi kendaraan uji terhadap standar Indonesia

CARLA tidak menyediakan model pasar Indonesia (41 blueprint, semuanya
Amerika/Eropa/Jepang-global). Kesesuaian dinyatakan lewat **dimensi terhadap
standar**, bukan lewat merek.

| Kendaraan | Panjang | Lebar | Jarak antar sumbu |
|---|---|---|---|
| Toyota Avanza (PDGJ 2021 Tabel 5-9) | 4,19 m | 1,66 m | 2,65 m |
| **Toyota Hiace (PDGJ 2021 Tabel 5-9)** | **5,38 m** | **1,88 m** | **3,11 m** |
| **Charger 2020 (CARLA, terukur)** | **5,01 m** | **1,88 m** | **3,04 m** |
| Isuzu ELF NLR 55 BLX (PDGJ 2021) | 6,17 m | 1,84 m | 3,36 m |

Kalimat siap pakai untuk bab 3:

> Dimensi kendaraan uji (5,01 × 1,88 m, jarak antar sumbu 3,04 m) berpadanan
> dengan Toyota Hiace (5,38 × 1,88 m, jarak antar sumbu 3,11 m), salah satu
> kendaraan desain untuk jalan kelas 1, 2, dan 3 menurut Pedoman Desain
> Geometrik Jalan No. 13/P/BM/2021 Tabel 5-9, dan memenuhi batas lebar kendaraan
> bermotor menurut PP No. 55 Tahun 2012 (maksimum 2.100 mm).

---

## 4. Lingkungan uji (Tahap 2)

Peta Town04, spawn point **indeks 75**: lajur paling kiri, 3 lajur searah di
kanan, lurus terverifikasi 400 m (simpangan arah 0,06°).

| Parameter | Nilai | Sumber |
|---|---|---|
| Lebar lajur | **3,50 m** | terukur dari peta (`waypoint.lane_width`), konstan |
| Jumlah lajur searah | 4 | terukur |
| Panjang ruas lurus | 400 m | terverifikasi (600 m ditolak, simpangan 55,3°) |
| Cuaca | ClearNoon | ditetapkan eksplisit |
| Mode simulasi | sinkron, `fixed_delta_seconds = 0,05` | 20 Hz |

**Lebar lajur 3,50 m cocok persis dengan standar yang berlaku.** PDGJ 2021
Tabel 5-58 menetapkan lebar lajur paling kecil menurut kecepatan desain:

| Kecepatan desain `V_D` | Lebar lajur paling kecil |
|---|---|
| tinggi: ≥ 80 km/jam | 3,60 m |
| **sedang: 40 ≤ V_D < 80 km/jam** | **3,50 m** |
| rendah: < 40 km/jam | 2,75 m |

Kecepatan desain skripsi 50 km/jam masuk pita "sedang" → 3,50 m. Dasar hukum
yang dirujuk tabel itu: Permen PU No. 19/PRT/M/2011.

**Catatan yang harus masuk batasan masalah:** 40% jalur dari spawn 75 bertanda
`is_junction = True`, tapi itu area pertemuan ramp di highway Town04, bukan
perempatan berlampu. Lajur kanan tetap terdefinisi 100% di sepanjang jalur, dan
geometrinya tetap lurus.

---

## 5. Validasi prediction model (Tahap 1)

**Apa yang dibandingkan:** mesin fisika CARLA (*plant*, kotak hitam) versus
kinematic bicycle model (*prediction model*, yang hidup di dalam solver MPC).
Kecepatan dan sudut roda yang benar-benar terjadi di CARLA disuapkan ke rumus,
lalu lintasannya diintegrasikan dari titik awal yang sama. Rumus tidak pernah
diberi tahu ke mana mobilnya pergi.

**Yang sengaja tidak divalidasi** (jangan salah klaim): bukan terhadap Dodge
Charger dunia nyata, bukan peta throttle → akselerasi, bukan pemetaan perintah
`steer` → sudut roda. Ketiganya dilepas supaya yang tersisa hanya geometrinya.

| | Run kecepatan rendah | Run kecepatan operasi |
|---|---|---|
| Rentang kecepatan | 7–18 km/jam | 34–50 km/jam |
| Percepatan lateral maks | ~0,3 m/s² | 2,05 m/s² |
| **Error posisi @ 2 detik** | 0,052 m | **0,309 m** |
| Error posisi akhir | 0,132 m @ 5 s | 0,538 m @ 3 s |

**Angka kunci: 0,309 m.** Horizon MPC `N=20 × dt=0,1` = 2 detik, jadi itulah
sejauh mana model dipercaya sebelum feedback mengoreksi — dan koreksinya datang
tiap 50 ms, 40× lebih sering. Ini jawaban untuk "kenapa model sesederhana ini
cukup".

Mismatch naik ~6× dari 18 ke 50 km/jam. Itu bukan kegagalan, melainkan bukti
kuantitatif untuk klaim bahwa kinematic bicycle paling akurat di kecepatan
rendah.

---

## 6. Parameter perencanaan dan perilaku (Tahap 3 & 4)

| Parameter | Nilai | Catatan |
|---|---|---|
| `v_max` | 13,9 m/s (50 km/jam) | UU 22/2009 Ps. 21; PP 79/2013 Ps. 23(4) |
| Arah menyalip | kanan | UU 22/2009 Ps. 109 |
| `SIDE_SIGN` | −1 (kanan = y negatif) | diverifikasi empiris, konsisten 3 run |
| Offset lateral kandidat | 3,0 / 3,5 / 4,0 m | |
| Durasi manuver kandidat | 3,0 / 3,5 / 4,0 s | |
| Batas percepatan lateral | 3,0 m/s² | kenyamanan |
| Zona aman: pangkat `p` | 4 | elips-super, bagian 14.3 |
| Zona aman `A`, `B` | 7,709 m, 3,204 m | diturunkan dari dimensi + `SAFE_DISTANCE` |
| Geser sumbu belakang -> pusat bodi | 1,433 m | zona diukur antar pusat bodi |
| Jarak ikut `FOLLOW_TIME` | 2,0 s | jarak waktu-tetap, bagian 14.4 |
| Laju menjauh maks untuk kembali | 0,1 m/s | gerbang `DD_RETURN` |
| Frekuensi local planner | 10 Hz | |
| Frekuensi MPC | 20 Hz | |
| `TTC_TRIGGER` | 5,0 s | mulai mempertimbangkan menyalip |
| `TTC_EXIT` | 7,0 s | histeresis |
| Gerbang eksekusi | 3,0 s | = durasi manuver tercepat |
| `FSM_DWELL` | 0,3 s | |

Percepatan lateral puncak quintic: `|y''|max = (10/√3)·|Δy|/T²`. Untuk
Δy = 3,5 m dan T = 3 s → **2,245 m/s²**, cocok dengan perkiraan proposal (~2,24).

---

## 7. Tiga penyimpangan dari proposal — ini yang layak jadi pembahasan

### 7.1 Ambang pemicu diubah dari jarak ke waktu (TTC)

Proposal menetapkan ambang jarak 20–25 m. Implementasi awal memakai itu dan
**gagal**: FSM masuk `CHECK_OVERTAKE` lalu terjebak, kendaraan depan terlewati
tanpa pernah menyalip.

Penelusuran (Δv = 6,9 m/s, ego 50 vs target 25 km/jam):

| t | celah | TTC | syarat waktu |
|---|---|---|---|
| 2,15 s | 25,2 m | 3,65 s | terpenuhi |
| 2,55 s | 22,4 m | 3,25 s | terpenuhi |
| **2,85 s** | **20,3 m** | **2,95 s** | **gugur** |

Manuver tercepat 3,0 s butuh celah minimal 3,0 × 6,9 = 20,7 m. Pemicu di 25 m
hanya menyisakan margin 0,62 detik, sedangkan **dua** dwell menghabiskan 0,6
detik. Syarat gugur tepat sebelum dwell kedua selesai.

Akar masalahnya bukan nilainya, tapi **satuannya**: jarak yang aman bergantung
selisih kecepatan. Diganti dengan ambang TTC:

```
TTC_TRIGGER ≥ T_tercepat + 2 × FSM_DWELL + margin = 3,0 + 0,6 + 1,4 = 5,0 s
```

Celah pemicu jadi menyesuaikan sendiri:

| Kecepatan target | Δv | Celah pemicu | TTC |
|---|---|---|---|
| 35 km/jam | 4,2 m/s | 20 m | 4,79 s |
| 30 km/jam | 5,6 m/s | 27 m | 4,85 s |
| 25 km/jam | 7,0 m/s | 34 m | 4,89 s |
| 20 km/jam | 8,3 m/s | 41 m | 4,91 s |

**Poin terkuat untuk sidang:** baris pertama menghasilkan 20 m — persis batas
bawah rentang 20–25 m di proposal. Jadi angka proposal **tidak salah**, dia benar
untuk selisih kecepatan kecil lalu tidak memadai saat selisihnya membesar.

Gerbang eksekusi 3,0 s **sama persis dengan baseline Bai dkk. (2025)** dan dekat
dengan 2,7 s Lin dkk. (2023). Angka itu tidak dipilih — datang dari durasi
manuver tercepat sistem ini.

### 7.2 Abort tidak menunggu dwell time

Proposal menulis dwell 0,3 s untuk "setiap transisi". Jalur abort dikecualikan:
menunda pembatalan justru menambah risiko. Chattering *masuk* ke abort lebih
aman daripada chattering *keluar* darinya.

### 7.3 `EgoState` di `localization.py`, bukan `types.py`

Struktur file proposal merancang `types.py` sebagai tempat dataclass bersama.
Ditunda sampai modul kedua benar-benar membacanya — saat ini `EgoState` punya
satu produsen dan nol konsumen.

---

## 8. Temuan teknis yang layak masuk pembahasan

### 8.1 Lompatan sudut 2π di data peta

`waypoint.transform.rotation.yaw` di ruas yang sama melaporkan **+89,78°** dan
**−270,22°** bergantian — selisihnya tepat 360°, arah yang sama.

Akar penyebab, dilacak sampai isi berkas OpenDRIVE (`map.to_opendrive()`):

| road_id | `hdg` tertulis | junction |
|---|---|---|
| 48 | 90,22° | −1 |
| 49 | 90,22° | −1 |
| **902** | **450,22°** | **890** |

Ruas 902 tertulis 450,22° = 90,22 + 360. CARLA menghitung `yaw = 180 − hdg`
tanpa menormalkan hasilnya, jadi `180 − 450,22 = −270,22`. Ruas 902 adalah
penghubung persimpangan yang **dibangkitkan otomatis**; program pembuat peta
menjumlahkan rotasi dan tidak memutarnya kembali saat melewati 360°.

Survei seluruh Town04: **140 dari 276 ruas** punya `hdg` di luar rentang normal,
dan ruas persimpangan terkena 57% versus 23% untuk jalan biasa.

**Akibat bila lolos:** MPC menghitung `yaw − psi_ref` dan mendapat error 359°
padahal 0,8°. Dengan `Q[psi] = 10` dan biaya kuadrat, itu ~200.000× lebih besar
dari seharusnya — solver membanting setir di satu titik tetap sepanjang jalur.
Gejalanya menyerupai bobot salah tuning.

**Solusi:** `np.unwrap()` pada reference path, dan normalisasi setiap pengurangan
sudut ke [−π, π] dengan `(a − b + π) mod 2π − π`.

### 8.2 Planner buta terhadap halangan (bug yang tertangkap lewat visualisasi)

Saat pertama divisualisasikan di dunia 3D, ego melewati kendaraan target dengan
jarak minimum 2,26 m dan constraint elips `g = 0,709` — dilanggar. Penyebabnya
`plan_lane_change` dipanggil tanpa argumen `obstacles`.

**Pelajarannya:** cacat ini tidak terlihat di uji unit maupun grafik kandidat.
Baru kelihatan saat lintasan ditaruh di dunia 3D bersama kendaraan lain.

---

## 9. Daftar pustaka terverifikasi

Semua entri di bawah sudah saya buka isinya, kecuali yang ditandai.

**Quintic polynomial — asal konsep:**

> Werling, M., Ziegler, J., Kammel, S. & Thrun, S. (2010). *Optimal Trajectory
> Generation for Dynamic Street Scenarios in a Frenét Frame.* IEEE International
> Conference on Robotics and Automation (ICRA), hlm. 987–993.

> Flash, T. & Hogan, N. (1985). Model minimum-jerk gerak lengan manusia.
> **Sitasi lengkap belum diverifikasi — cek sebelum dipakai.**

**Quintic polynomial — praktik terkini:**

> Bai, R., Xu, R., Rui, T., Liu, J., Oung, Q.W., Lee, H.L., Tian, Z. & Yuan, F.
> (2025). *Safe and Efficient Lane-Changing for Autonomous Vehicles: An Improved
> Double Quintic Polynomial Approach with Time-to-Collision Evaluation.* Journal
> of King Saud University — Computer and Information Sciences. arXiv:2509.00582.

> Jin, M., Qu, M., Gao, Q., Huang, Z., Su, T. & Liang, Z. (2024). *Advanced
> Trajectory Planning and Control for Autonomous Vehicles with Quintic
> Polynomials.* Sensors, 24(24), 7928. DOI: 10.3390/s24247928.

**Time-to-Collision:**

> Hayward, J.C. (1972). *Near-Miss Determination Through Use of a Scale of
> Danger.* Highway Research Record, 384, 24–35. Washington, D.C.: Highway
> Research Board.

> Lin, P., Javanmardi, E., Tao, Y., Chauhan, V., Nakazato, J. & Tsukada, M.
> (2023). *Time-to-Collision-Aware Lane-Change Strategy Based on Potential Field
> and Cubic Polynomial for Autonomous Vehicles.* arXiv:2306.06981.

> Singh, D., Das, P. & Ghosh, I. (2024). *Conflict-Based safety evaluations at
> unsignalized intersections using surrogate safety measures.* Heliyon.
> DOI: 10.1016/j.heliyon.2024.e27665. (untuk format "dikutip dalam")

**Standar dan regulasi Indonesia:**

> Direktorat Jenderal Bina Marga (2021). *Pedoman Desain Geometrik Jalan.*
> Pedoman No. 13/P/BM/2021, ditetapkan lewat Surat Edaran Direktur Jenderal Bina
> Marga No. 20/SE/Db/2021, 27 Oktober 2021.

> Peraturan Pemerintah Republik Indonesia No. 55 Tahun 2012 tentang Kendaraan.

> Undang-Undang No. 22 Tahun 2009 tentang Lalu Lintas dan Angkutan Jalan,
> Pasal 21 dan Pasal 109.

> Peraturan Pemerintah No. 79 Tahun 2013, Pasal 23 ayat (4).

> Peraturan Menteri Pekerjaan Umum No. 19/PRT/M/2011. (dirujuk PDGJ 2021 Tabel 5-58)

**Peringatan sitasi Hayward:** sumber sekunder tidak konsisten — banyak menulis
1971 (mengacu laporan teknis Penn State TTSC 7115), sebagian 1972 (mengacu
Highway Research Record). Arsip TRB menempatkannya di HRR 384 tahun 1972,
halaman 24–35. Sudah diverifikasi ke PDF aslinya.

**Catatan:** TPGJAK No. 038/TBM/1997 **dicabut** oleh PDGJ 2021 — jangan dikutip
sebagai standar berlaku. Status RSNI T-14-2004 tidak jelas; tidak perlu dikutip
karena PDGJ 2021 memuat semua yang dibutuhkan.

---

## 10. Gambar yang tersedia

| Berkas | Isi | Usulan penempatan |
|---|---|---|
| `lanes_camera.png` | Tangkapan kamera lingkungan uji, ego di lajur kiri, 3 lajur kosong di kanan | Bab 3 — lingkungan uji |
| `lanes_topdown.png` | Peta lajur koordinat `s`–`d`, posisi lateral tiap lajur, area junction | Bab 3 — lingkungan uji |
| `model_validation.png` | Validasi bicycle model 34–50 km/jam: lintasan, error, input | Bab 3 — validasi model |
| `model_validation_18kmh.png` | Validasi yang sama di 7–18 km/jam | Bab 3 — validasi model |
| `planner_candidates.png` | 9 kandidat lintasan + yang terpilih + profil percepatan lateral | Bab 3 — local planner |
| `ttc_threshold.png` | Kenapa ambang memakai waktu bukan jarak | Bab 3 atau bab 4 — pembahasan FSM |
| `why_q_psi_450.png` | Ilustrasi lompatan sudut 2π | Bab 4 — pembahasan temuan |
| `overtake_topdown.mp4` | Video manuver tampak atas dengan overlay kandidat | Sidang |
| `overtake_planner.mp4` | Video manuver kamera kejar dengan overlay kandidat | Sidang |
| `reference_path.mp4` | Video jalur acuan global planner 300 m | Sidang |

**Label video wajib:** semua video saat ini adalah **playback lintasan planner**,
bukan hasil kendali MPC. Physics dimatikan dan posisi ego ditempelkan ke lintasan.
Beri label "lintasan hasil local planner". Video hasil kendali sudah bisa dibuat
(Tahap 5 selesai), tapi `record_maneuver.py` belum diubah.

**Grafik hasil kendali sudah diperbarui** dan cocok dengan angka bagian 14:
`run_s1_mpc_gt.png` dan `run_s3_mpc_gt.png`, dibuat `plot_run.py` langsung dari log
(`out/run_s1_mpc_gt.npz`, `out/run_s3_mpc_gt.npz`) tanpa menjalankan simulasi
ulang. Empat panel: simpangan lateral, kecepatan, jarak antar bodi ke tiap
kendaraan, dan sudut kemudi + waktu solve; latar tiap panel diwarnai menurut
state FSM.

Data mentah: `vehicle_params.json`, `model_validation.csv`,
`model_validation_18kmh.csv`.

---

## 11. Aturan penulisan yang sudah disepakati

1. **Sumber parameter kendaraan = CARLA API**, bukan katalog. Spesifikasi Dodge
   Charger dunia nyata tidak dibutuhkan dan tidak boleh masuk skripsi — plant-nya
   CARLA, bukan mobil asli.
2. **Localization memakai ground truth CARLA**, nyatakan eksplisit di batasan
   masalah bahwa lokalisasi diasumsikan ideal. Tidak dievaluasi karena errornya
   nol per definisi.
3. **Depth camera adalah sensor ideal** tanpa padanan langsung di dunia nyata;
   sebutkan di batasan masalah bahwa implementasi nyata menggantinya dengan
   stereo camera atau LiDAR.
4. **Konvensi lajur CARLA berkebalikan dengan Indonesia.** Seluruh peta bawaan
   memakai lalu lintas kanan. Yang direplikasi adalah **geometri manuvernya**
   (bergeser ke kanan untuk melewati), bukan hierarki lajur menurut aturan
   Indonesia. Wajib masuk batasan masalah.
5. **Sebut Town04 sebagai "ruas jalan lurus dengan beberapa lajur searah"**,
   jangan mengklaim kategori jalan tertentu.
6. **Jangan sebut ROS** kalau tidak benar-benar dipakai.
7. Kutip **keduanya** untuk quintic: Werling 2010 untuk *kenapa* derajat lima,
   paper 2023–2025 untuk *bahwa* metode ini masih praktik terkini.

---

## 12. Yang masih terbuka

- `delta_max` (0,5 rad, constraint MPC) versus `delta_max_phys` (1,222 rad,
  penyebut normalisasi `steer`). Proposal bagian 7.5 menulis
  `steer = delta/delta_max` — itu salah dan menyebabkan understeer ~2,4×.
  Penyebutnya harus `delta_max_phys × curve(v)`, dengan `v` dalam km/jam.
  **Sudah diperbaiki di kode dan diverifikasi di CARLA** (meleset <= 4%), tapi
  tetap perlu ditulis sebagai penyimpangan dari proposal.
- Sitasi Flash & Hogan (1985) belum diverifikasi.
- **Constraint MPC memakai kotak sejajar sumbu, penilai tidak lagi.** Penilai kini
  memutar kotak menurut sudut hadap masing-masing kendaraan, jadi angka jarak yang
  dilaporkan sudah eksak. Constraint tetap memakai sudut nol; itu pendekatan desain,
  dan kecukupannya ditunjukkan hasil ukur: saat kedua bodi berdampingan sudut hadap
  ego hanya <= 5,1° dan jarak yang tercapai 1,42-1,47 m terhadap syarat 1,0 m.
  Nyatakan sebagai asumsi perancangan, bukan sebagai celah.
- **Dimensi kendaraan lain dianggap tetap** (Nissan Patrol, yang terbesar di
  skenario); perception tidak mengukur dimensi. Masuk batasan masalah.
- Skenario S2, S4, S5 **tidak punya definisi di repo ini sama sekali**. Definisi
  S1 dan S3 pun rekonstruksi, bukan salinan bagian 11.3 rencana kerja.
- `D_SAFE_FRONT` dan `D_SAFE_REAR` belum dituning: keduanya tidak mengikat di
  S1 (lajur tujuan kosong). `PASS_MARGIN` sudah disapu di Tahap 8 — 4/8/14 m
  nyaris tak berbeda, jadi 8,0 dipertahankan (bagian 20.2).

**Ditutup sejak draf sebelumnya:**

- ~~4 tick tanpa kandidat planner di S1 belum ditelusuri~~ → sudah: geometri zona
  aman, bukan tuning. Vision memberi 40 tick, dan angka itu bertahan 38-44 di
  seluruh sapuan (bagian 19.13, 20.5).
- ~~Tuning ulang setelah Tahap 8~~ → selesai, hanya `Q[y]`/`Q[psi]` yang berubah
  (bagian 20).
- ~~Klaim real-time belum diukur di mesin senggang~~ → diukur setelah server
  direstart (bagian 21.3), meski mesinnya tidak benar-benar sepi (load ~3-5).

**Terbuka sejak Tahap 8:**

- **Data leakage YOLOPX** — angka pelatihan bagian 18.1 tidak sah. Keputusan
  penulis: latih ulang dengan split per-sesi, atau nyatakan eksplisit dan
  bersandar pada pengukuran terhadap simulator saja.
- **Validasi perception saat berdampingan** belum terkendali; angkanya diambil
  dari log loop tertutup, bukan sapuan terancang (bagian 19.8).
- **S3 dengan vision** butuh kamera belakang (bagian 19.6).
- **Sitasi ByteTrack** belum dibuka sumbernya, jadi sengaja belum masuk daftar
  pustaka meski strateginya dipakai `tracking.py`. Terbit 2022 — tepat di batas
  aturan empat tahun.
- **Anggaran tick** kasus terburuk 48,9 ms dari 50 ms (bagian 21.3). Harus
  ditulis apa adanya.


---

## 13. Keputusan lingkup: baseline pembanding dibatalkan

Tahap 7 (Pure Pursuit + PID / Stanley) **tidak dikerjakan**. Keputusan penulis,
10 September 2026.

**Yang hilang:** kemampuan menyatakan "MPC lebih baik daripada pendekatan klasik
untuk tugas ini" dengan angka. Pertanyaan "kenapa MPC, bukan yang lebih
sederhana?" dijawab secara konseptual, bukan terukur:

- Batas kemudi, percepatan, kecepatan, dan penghindaran tabrakan dinyatakan
  sebagai **constraint eksplisit**, bukan disetel lewat gain
- Horizon 2 detik membaca acuan masa depan; Pure Pursuit hanya melihat satu
  titik lookahead
- Kemudi dan gas diselesaikan **bersamaan** dalam satu optimasi, bukan dua loop
  terpisah yang harus dikoordinasikan manual
- Tidak butuh anti-windup maupun penjadwalan gain per kecepatan

**Yang tetap ada.** Bagian 11.4 punya tiga baris pembanding; yang dibatalkan
hanya satu. Dua sisanya tetap dikerjakan:

| Konfigurasi | Tujuan | Status |
|---|---|---|
| MPC + GT perception | batas atas performa kendali | **sudah berjalan** |
| MPC + vision perception | sistem lengkap | Tahap 8 |
| ~~Pure Pursuit/Stanley + GT~~ | ~~baseline kendali~~ | dibatalkan |

Selisih antara dua baris pertama menunjukkan seberapa jauh error perception
merambat ke performa kendali -- dan untuk skripsi yang punya pipeline YOLOPX,
itu sumbu perbandingan yang lebih pusat daripada baseline kendali.

**Temuan yang tetap dimiliki skripsi tanpa baseline:**

1. Validasi prediction model di dua titik operasi (error 0,309 m pada horizon
   2 detik; degradasi ~6x dari 18 ke 50 km/jam)
2. Ambang pemicu FSM harus berbasis TTC, bukan jarak -- lengkap dengan
   pertidaksamaan yang mengikat ambang, durasi manuver, dan dwell time
3. Hasil tuning bobot: `Q[psi]` proposal meleset 45x dan `kp` throttle meleset
   ~1,8x, sedangkan `Q[Y]`, `Rd[delta]`, dan `ki` sudah benar sejak awal — dan
   ketiganya dibuktikan lewat sapuan, bukan didiamkan (lihat `TUNING_MPC.md`)
4. Perbandingan GT versus vision perception (Tahap 8)


---

## 14. Hasil kendali dan parameter final (Tahap 5 & 6)

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

Diukur ulang 16 September 2026 (setelah perbaikan jangkar halangan, bagian 19),
CARLA 0.9.16 kualitas Low, satu run per skenario.
Simulasinya deterministik: dua run berkonfigurasi sama memberi log **identik
bit-per-bit**, jadi satu run menggambarkan konfigurasi itu (ulangan tetap
dibutuhkan di Tahap 9 untuk success rate, bukan untuk ketepatan satu run).

### 14.1 Hasil per skenario

| | S1 (flying overtaking) | S3 (accelerative overtaking) |
|---|---|---|
| Susunan | target 7,0 m/s, 60 m di depan, lajur kanan kosong | idem + kendaraan 13,9 m/s di lajur kanan, mulai 10 m di belakang ego |
| Vonis bagian 11.2 | **BERHASIL** | **BERHASIL** |
| Jarak minimum antar bodi | 1,44 m | 1,47 m |
| Deviasi dari tengah lajur saat `LANE_KEEPING` | 0,009 m rata-rata, 0,113 m maks | 0,011 m rata-rata, 0,128 m maks |
| Durasi manuver | 11,6 s | 18,9 s |
| Kecepatan terendah | 48 km/jam | 19 km/jam (saat mengikuti; acuan `v_goal` turun sampai 14,7) |
| Perlambatan terdalam | −0,14 m/s² | −2,00 m/s² |
| Waktu solve MPC | 12,3 ms rata-rata, 16,9 ms maks | 12,1 ms rata-rata, 17,1 ms maks |
| Solver gagal | 0 dari 400 | 0 dari 500 |
| Urutan state FSM | lengkap | lengkap |

Anggaran waktu satu tick 50 ms; solve terukur di mesin senggang (lihat catatan
waktu solve di `TUNING_MPC.md` bagian 10).

### 14.2 Bobot dan gain final

| Parameter | Proposal 7.3 | Setelah Tahap 5 | **Final (setelah Tahap 8)** | Dasar |
|---|---|---|---|---|
| `Q` = (X, Y, psi, v) | (1; 20; 10; 2) | (1; 20; 450; 2) | **(1; 150; 3400; 2)** | `Q[y]` dari sapuan skenario penuh; `Q[psi]` menjaga RASIO redaman (bagian 20.3) |
| `Qf` | 5·Q | 5·Q | 5·Q | tidak disentuh |
| `R` = (a, delta) | (0,1; 1,0) | (0,1; 1,0) | (0,1; 1,0) | tidak disentuh |
| `Rd` = (a, delta) | (1,0; 20,0) | (1,0; 20,0) | (1,0; 20,0) | disapu, terbukti optimal |
| `rho` (slack zona aman) | 1000 | 1000 | 1000 | tidak disentuh |
| `rho_lat` (slack batas kenyamanan) | — | — | **50** | 20× di bawah `rho`: keselamatan menang, kenyamanan mengalah (bagian 19.12) |
| `kp` throttle | 0,08 | **0,14** | 0,14 | = 1/gain plant terukur |
| `ki` throttle | 0,25 | 0,25 | 0,25 | tengah geometrik plateau |

**Kolom "Setelah Tahap 5" dipertahankan dengan sengaja.** Nilai itu yang dipakai
seluruh hasil sebelum vision, dan `TUNING_MPC.md` bagian 7 dan 11 menurunkannya
panjang lebar. Yang berubah di Tahap 8 hanya pasangan `Q[y]`/`Q[psi]`, dan
alasannya bukan "nilai lama salah" melainkan bahwa **sapuan skenario penuh baru
menjadi sah setelah run vision terulang** (bagian 20.1).

Alasan tiap angka, deret sapuan, dan tafsiran fisiknya ada di `TUNING_MPC.md`
bagian 11 (Tahap 5) dan bagian 20 di dokumen ini (Tahap 8) — keduanya bahan
langsung untuk sub-bab tuning di bab 3.

### 14.3 Zona aman: constraint yang menyiratkan kriteria penilaian

Syarat lulus bagian 11.2 menuntut jarak antar bodi >= 1,0 m. Agar constraint
benar-benar menjaminnya, zona aman harus memuat seluruh "persegi terlarang"
antar pusat bodi: setengah sisi `(L_ego + L_lain)/2 + 1,0` = 5,81 m dan
`(W_ego + W_lain)/2 + 1,0` = 2,91 m.

Elips biasa tidak bisa: dengan `B` di bawah lebar lajur (syarat agar berpapasan
tetap layak), memuat sudut persegi menuntut `A` 11-14 m. Dipakai **elips-super
pangkat 4**: `g = ((dx/A)^4 + (dy/B)^4)^(1/4) >= 1` dengan `A = 7,709 m` dan
`B = 3,204 m`, keduanya diturunkan dari dimensi terukur.

Kalimat siap pakai untuk bab 3:

> Batas aman dinyatakan sebagai zona elips-super berpangkat empat antara pusat
> bodi kendaraan. Parameternya diturunkan agar zona memuat persegi terlarang
> yang dibentuk dimensi kedua kendaraan ditambah jarak aman minimum, sehingga
> constraint pada optimasi menyiratkan kriteria keberhasilan yang dinilai.

### 14.4 Perilaku mengikuti dan menyalip ulang

Bila menyalip tidak mungkin (kendaraan depan tidak cukup lambat, waktu tidak
cukup, atau lajur tujuan terisi), ego mengikuti kendaraan depan pada jarak
`d* = A + 1,433 + 2,0·v_front` dengan kecepatan acuan `v_front + 2e/T`,
`e = gap − d*`, `T` = durasi manuver terpanjang. Begitu lajur tujuan aman,
ego menyalip dari posisi mengikuti — *accelerative overtaking* menurut
klasifikasi Fabricius dkk. (2022), berbeda dari S1 yang *flying*.

Pemicu memakai `max(v_ego, V_REF)`, bukan `v_ego`: alasan menyalip adalah
kendaraan depan lebih lambat daripada kecepatan yang **diinginkan**. Tanpa itu,
ego yang sudah melambat mengikuti punya TTC tak hingga dan tidak akan pernah
mencoba menyalip lagi.

---

## 15. Temuan metodologis (12 September 2026)

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

Lima temuan berikut layak masuk bab pembahasan. Semuanya punya pola sama:
**angka yang terlihat baik karena alasan yang salah.**

### 15.1 Determinisme harus ditegakkan kode, bukan diasumsikan

Lima run `main.py` berkonfigurasi identik memberi lima hasil berbeda sejak tick
pertama, satu di antaranya gagal. Fisika CARLA sendiri terbukti deterministik.
Penyebabnya perintah aktor (`apply_control`, `set_target_velocity`) dikirim
asinkron sementara `world.tick()` menunggu, sehingga perintah kadang berlaku
satu frame terlambat. Setelah perintah dikirim lewat `apply_batch_sync` yang
menunggu, run kembali identik bit-per-bit.

### 15.2 "Optimum" yang ternyata artefak

Sapuan `kp` throttle menunjukkan optimum tajam di 0,3 (error 0,021 m/s versus
~0,105 di sekitarnya). Penyebabnya bukan kualitas pengendali: `ThrottlePI`
me-reset integrator setiap `a_ref < 0`, dan pada `kp = 0,3` permintaan negatif
kebetulan tidak pernah muncul. Setelah logika rem diperbaiki, `kp` datar di
seluruh rentang 0,035-0,3 dan nilainya bisa dipilih dari teori (1/gain plant).

### 15.3 Lup umpan balik planner-MPC

Planner memakai percepatan lateral **terukur** sebagai syarat awal. MPC
mengikuti kelengkungan awal lintasan, percepatan itu terukur lagi, lalu jadi
syarat awal rencana berikutnya. Di S3 satu gangguan kecil tumbuh menjadi
simpangan 2,3 m keluar lajur, padahal planner terus menargetkan tengah lajur.
Perbaikannya memakai percepatan dari rencana sebelumnya; deviasi lajur S1 turun
dari 0,122 ke 0,011 m.

### 15.4 Constraint yang tidak menyiratkan kriteria penilaian

Elips lama hanya menjamin jarak bodi 0,29 m, sementara syarat lulus 1,0 m.
Selama jalur uji lebar, angkanya tetap lolos — kecacatan baru muncul pada
manuver dari posisi mengikuti. Pelajaran: **kriteria penilaian harus bisa
diturunkan dari constraint**, bukan sekadar terpenuhi secara kebetulan.

### 15.5 Metrik yang mengukur hal lain

"Deviasi dari tengah lajur saat `LANE_KEEPING`" hampir seluruhnya berisi ekor
setelah kembali ke lajur; sebelum manuver angkanya 0,001 m. Pisahkan per fase
saat mendefinisikan metrik bab 4.

**Dua perbaikan yang sempat ditempuh sebelum 15.3 ketemu** (margin zona dan
gerbang laju lateral) keduanya masuk akal dan didukung korelasi data, tapi hanya
mengobati gejala. Yang membedakan perbaikan yang benar: mekanismenya
direkonstruksi tick demi tick, bukan disimpulkan dari korelasi.

---

## 16. Sitasi tambahan — terbit <= 4 tahun, ber-URL, isi sudah dibuka

Aturan penulis (11 September 2026): sitasi baru harus nyata, ber-URL, terbit
maksimal empat tahun terakhir, dan isinya dibaca langsung — bukan dari cuplikan
hasil pencarian.

**Arsitektur kendali bertingkat (MPC sebagai HLC, PI sebagai LLC):**

> Yuan, T., & Zhao, R. (2022). LQR-MPC-Based Trajectory-Tracking Controller of
> Autonomous Vehicle Subject to Coupling Effects and Driving State Uncertainties.
> *Sensors*, 22(15), 5556. https://doi.org/10.3390/s22155556

> Pitschi, P., Sagmeister, S., Goblirsch, S., Lienkamp, M., & Lohmann, B. (2025).
> Longitudinal Control for Autonomous Racing with Combustion Engine Vehicles.
> arXiv:2504.17418. https://arxiv.org/abs/2504.17418

**Zona aman berbentuk elips-super:**

> Moran, R., Bagley, S., Kasmann, S., Martin, R., Pasley, D., Trimble, S.,
> Dianics, J., & Sopasakis, P. (2024). NMPC for Collision Avoidance by
> Superellipsoid Separation. *Modeling, Estimation, and Control Conference
> (MECC 2024)*. arXiv:2404.14257. https://arxiv.org/abs/2404.14257

**Kebijakan jarak waktu-tetap dan klasifikasi manuver menyalip:**

> El-Baklish, S. K., Kouvelas, A., & Makridis, M. A. (2025). Driving Towards
> Stability and Efficiency: A Variable Time Gap Strategy for Adaptive Cruise
> Control. arXiv:2402.14110. https://arxiv.org/abs/2402.14110

> Lee, K., & Lee, C. (2025). String Stability Analysis and Design Guidelines for
> PD Controllers in Adaptive Cruise Control Systems. *Sensors*, 25(11), 3518.
> https://doi.org/10.3390/s25113518

> Fabricius, V., Habibovic, A., Rizgary, D., Andersson, J., & Wärnestål, P.
> (2022). Interactions between heavy trucks and vulnerable road users—A
> systematic review to inform the interactive capabilities of highly automated
> trucks. *Frontiers in Robotics and AI*, 9, 818019.
> https://doi.org/10.3389/frobt.2022.818019

**Yang sengaja TIDAK dikutip:** nilai time gap ISO 15622 (standarnya 2018, dan
sumber <= 4 tahun yang memuat angkanya tidak ditemukan), Rajamani (2012), serta
buku teks kendali klasik. Nilai `FOLLOW_TIME` karena itu bersandar pada sapuan
eksperimen sendiri, bukan pada standar.

**Perlu keputusan penulis:** apakah aturan 4 tahun berlaku juga untuk sumber
asal konsep di bab 2 — Werling dkk. (2010), Hayward (1972), Flash & Hogan (1985).
Mengganti ketiganya akan melemahkan landasan teori, karena justru merekalah
sumber pertamanya.


---

## 17. Konfigurasi sensor (Tahap 8)

Penempatan kamera mengikuti rig KITTI, keputusan penulis 15 September 2026.
Angka KITTI diambil dari Gambar 3 makalah datasetnya (Geiger dkk., 2013):

| Besaran | KITTI | Skripsi ini |
|---|---|---|
| Tinggi kamera di atas jalan | 1,65 m | 1,65 m (terukur di simulator 1,652 m) |
| Jarak di depan sumbu roda belakang | 1,68 m | 1,68 m (terukur 1,680 m) |
| Baseline stereo kamera warna | 0,54 m | 0,54 m (tersedia, belum dipakai) |
| Sudut buka lensa | ~90 derajat (lensa 4 mm) | 90 derajat |
| Resolusi | 1392 x 512 | **1280 x 720** (keputusan penulis) |
| Tinggi Velodyne | 1,73 m | tidak dipakai |

Depth camera ditaruh satu titik dengan kamera warna kiri, sehingga piksel hasil
deteksi bisa langsung dibaca kedalamannya tanpa kalibrasi antar sensor. Depth di
CARLA adalah sensor ideal -- sudah masuk aturan penulisan nomor 3.

**Yang perlu masuk batasan masalah:** kap mesin Dodge Charger memenuhi sekitar 15%
bagian bawah citra, karena rig KITTI dipasang di atap station wagon sedangkan di
sini kamera berada di atas kabin sedan. Bagian itu tidak membawa informasi jalan.

Verifikasi penempatan dilakukan terhadap simulator, bukan terhadap nilai yang
diminta: `check_sensors.py` membaca transform sensor yang benar-benar terjadi dan
membandingkannya dengan titik terbawah bodi sebagai permukaan jalan.

**Catatan sitasi:** makalah KITTI terbit 2013, di luar aturan empat tahun. Ia
dipakai sebagai spesifikasi rig yang ditiru, bukan sebagai klaim state of the art,
jadi perlakuannya sama dengan sumber asal konsep di bab 2 -- putuskan bersama
butir di bagian 16.

> Geiger, A., Lenz, P., Stiller, C., & Urtasun, R. (2013). Vision meets Robotics:
> The KITTI Dataset. *International Journal of Robotics Research*, 32(11),
> 1231-1237. https://www.cvlibs.net/publications/Geiger2013IJRR.pdf


---

## 18. Kalibrasi deteksi YOLOPX terhadap ground truth (Tahap 8)

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

15 September 2026, `check_detection.py`. Nissan Patrol ditaruh pada sembilan jarak di
depan ego, di lajur ego dan lajur menyalip; tiap frame dibandingkan dengan kotak
2D hasil proyeksi bounding box 3D-nya. Ini pengukuran terhadap **simulator**, di
lingkungan uji yang sama dengan eksperimen kendali -- bukan angka pelatihan.

Dua model dibandingkan:

| | `best.pth` | `epoch-195.pth` |
|---|---|---|
| Asal | fine-tuning penulis ke domain CARLA, epoch 263 | weight resmi YOLOPX, latihan BDD100K |
| Peran di skripsi | **model yang dipakai** | pembanding jarak domain, sekaligus titik awal fine-tuning |

Masukan 1280x720 di-letterbox ke 384x640; inferensi **14,4 ms** per frame di
RTX 5060 (anggaran tick 50 ms, MPC memakai ~12 ms).

### 18.1 Hasil fine-tuning (dari `out/yolopx_finetune_results.csv`)

363 epoch, terbaik di epoch 263. Diukur pada split validasi penulis:

| Metrik | Epoch 1 (masih BDD) | Epoch 263 |
|---|---|---|
| mAP50 | 0,945 | **0,991** |
| mAP50-95 | 0,610 | **0,976** |
| IoU area jalan | 0,554 | **0,990** |
| Akurasi garis lajur | 0,000 | **0,988** |
| IoU garis lajur | 0,000 | **0,582** |

**Angka ini terlalu optimistis dan tidak boleh diklaim apa adanya**: split-nya masih
bocor (frame berurutan dari sesi rekaman yang sama masuk ke train dan val
sekaligus). Yang sah disimpulkan darinya hanyalah bahwa adaptasi domain bekerja.

### 18.2 Laju deteksi terhadap jarak, di Town04

| Jarak | Lebar target | `best.pth` lajur ego | `best.pth` lajur menyalip | `epoch-195.pth` |
|---|---|---|---|---|
| 10 m | 207-326 px | 0,99 | 0,99 | 0,91 |
| 20 m | 77-100 px | 0,98 | 0,98 | 0,91 |
| 30 m | 48-57 px | 0,99 | 0,99 | 0,91 |
| 40 m | 34-39 px | 0,97 | 0,98 | 0,84 |
| 50 m | 27-30 px | **tidak terdeteksi** | 0,62 | 0,84 |
| 60 m | 22-24 px | **tidak terdeteksi** | **tidak terdeteksi** | 0,85 |
| 80 m | 16-17 px | **tidak terdeteksi** | **tidak terdeteksi** | 0,84 |

**Pertukaran yang harus ditulis di pembahasan:** fine-tuning menaikkan keyakinan di
jarak dekat (0,84-0,92 menjadi 0,97-0,99) dan menghapus seluruh positif palsu,
tetapi **memotong jangkauan dari 80 m menjadi 40-50 m**. Penyebab yang paling
masuk akal: data latih penulis diambil di jalan kota, sehingga kendaraan sejauh
50-80 m (lebar 16-30 piksel) nyaris tidak terwakili.

Jangkauan 40-50 m masih memenuhi kebutuhan kendali, tapi marginnya tipis: pemicu
menyalip bekerja pada celah ~32 m dan horizon MPC 2 detik setara 27 m.

### 18.3 Positif palsu

| Ambang | `best.pth` | `epoch-195.pth` |
|---|---|---|
| 0,3 | **0** | 4-8 |
| 0,4 | **0** | 0-3 |
| 0,5 | 0 | 0 |

`DETECTION_CONF = 0,5` dipertahankan: pada model fine-tuned ia bahkan tidak lagi
diperlukan untuk menekan positif palsu, tetapi tetap memberi margin terhadap
deteksi terlemah (0,62 pada 50 m). Positif palsu model BDD seluruhnya batu, semak,
dan pagar di garis horizon -- bukan benda menyerupai kendaraan.

### 18.4 Depth membaca MUKA kendaraan, bukan pusatnya

Galat jarak dari depth camera konsisten **-2,3 m** terhadap pusat bodi target, di
semua jarak dan pada kedua model. Itu tepat setengah panjang Nissan Patrol
(4,605/2 = 2,30 m): depth mengukur permukaan yang terlihat. Diukur ulang terhadap
muka kendaraan, galatnya tinggal **-0,48 sampai +0,19 m**.

**Konsekuensi untuk `VisionPerception`:** keluarannya harus posisi PUSAT bodi, sama
seperti `GroundTruthPerception`, supaya perbandingan bagian 11.4 membandingkan
besaran yang sama. Jarak depth perlu ditambah setengah panjang kendaraan yang
diasumsikan (`OTHER_LENGTH`), dan asumsi itu masuk batasan masalah.

### 18.5 Segmentasi

Dengan `best.pth`, area jalan bersih dan garis lajur terdeteksi sebagai marka
putus-putus yang mengikuti marka sebenarnya. Dengan `epoch-195.pth`, keluaran
lajur pecah menjadi 7-8 komponen dan mengecat bahu jalan kanan beserta pagar
(3 komponen rapi pada citra BDD asli sebagai pembanding).

Perlu dikonfirmasi ke anotasi: pada keluaran fine-tuned, area jalan menutupi lajur
di kanan ego tetapi **tidak menutupi lajur yang sedang ditempati ego**. Itu
mengikuti konvensi anotasi data latih, bukan kekeliruan model.

Karena kendali skripsi ini memakai geometri lajur dari peta, dua keluaran
segmentasi itu berperan sebagai bahan pembahasan, bukan masukan kendali.
*(Usang: sejak bagian 28 garis lajur, dan sejak bagian 31 area jalan, menjadi
masukan kendali.)*

Gambar: `out/detection_30m_lane0.png` (fine-tuned), `out/detection_30m_lane0_bdd.png`
(BDD, pembanding), `out/detection_15m_lane1.png` (lajur menyalip).


---

## 19. VisionPerception dan perbaikan jangkar halangan (Tahap 8)

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

16 September 2026.

### 19.1 Depth CARLA adalah jarak planar

Diverifikasi terhadap permukaan jalan: kedalaman tetap sepanjang satu baris
citra (rasio tepi/tengah **1,000x** terukur, versus 1,28x yang diprediksi
radial). Balik-proyeksinya karena itu langsung, tanpa faktor sinar:
`Z = depth`, `y = -Z·du/f`, dengan `f = 640 px`.

Kalimat siap pakai untuk bab 3:

> Kedalaman yang dikeluarkan depth camera diverifikasi sebagai jarak planar
> sepanjang sumbu optik, bukan jarak radial sepanjang sinar. Verifikasi dilakukan
> terhadap permukaan jalan, yang profil kedalamannya dapat diturunkan secara
> analitik dari tinggi kamera, sehingga kedua hipotesis memberi prediksi yang
> terpisah tanpa memerlukan objek acuan.

Kenapa ini layak ditulis: selisih kedua tafsiran **nol di tengah citra** dan
membesar ke tepi (0,64 m pada lajur sebelah 10 m; 1,10 m pada 5 m). Kalibrasi
bagian 18 seluruhnya di tengah citra, jadi tafsiran yang salah tidak akan
terdeteksi di sana -- baru muncul di lajur sebelah pada jarak dekat, tempat
keputusan menyalip diambil.

Planar sekaligus yang paling sesuai dengan koreksi 18.4: muka belakang kendaraan
adalah bidang tegak lurus sumbu jalan, sehingga saat ego dan target sehadap,
koreksi muka -> pusat bodi menjadi penambahan satu konstanta `OTHER_LENGTH/2`
pada satu sumbu. **Asumsi sehadap** melemah saat yaw ego mencapai 13,6 derajat
waktu pindah lajur -- masuk batasan masalah.

### 19.2 Ketelitian estimasi terhadap ground truth simulator

`check_estimation.py`, geometri S1 dengan kecepatan dipaksa tetap (ego 13,4 m/s,
target 7,0 m/s), jarak menyapu 55 -> 9 m, 140 tick di 20 Hz.

| Besaran | Bias | RMS | Maks |
|---|---|---|---|
| x memanjang | -0,011 m | **0,046 m** | 0,205 m |
| y melintang | +0,017 m | 0,019 m | 0,030 m |
| vx | -0,007 m/s | **0,020 m/s** | 0,173 m/s |
| vy | -0,001 m/s | 0,015 m/s | 0,035 m/s |

Terdeteksi 116 dari 140 tick; **deteksi pertama pada 45,6 m**, mereproduksi
batas jangkauan 40-50 m bagian 18.2 secara independen. Kecepatan konvergen ke
galat < 0,1 m/s dalam **0,60 s (12 frame)** sejak track lahir -- angka ini yang
menentukan seberapa cepat FSM boleh mempercayai TTC dari kendaraan yang baru
masuk pandangan.

**Batasan pengukuran ini:** seluruhnya di lajur ego, ego berjalan lurus. Target
di lajur sebelah pada sudut besar, dan ego yang sedang menyudut, belum diukur.

### 19.3 Acuan kecepatan: pergeseran posisi, bukan `get_velocity()`

| Sumber kecepatan relatif | Rata-rata | Simpangan baku |
|---|---|---|
| `get_velocity()` simulator | -6,394 | **0,283** |
| pergeseran posisi GT / dt | -6,396 | 0,018 |
| VisionPerception | -6,402 | **0,006** |

`get_velocity()` berderau 0,28 m/s per tick ketika kecepatan aktor dipaksa tiap
tick, jadi memakainya sebagai acuan berarti menghukum estimator dengan derau
milik alat ukur (RMS terbaca 0,589 m/s; terhadap acuan yang benar 0,020 m/s).
Keluaran Kalman filter bahkan **lebih halus daripada pembacaan kecepatan
simulator sendiri**. Temuan ini satu keluarga dengan bagian 15.

### 19.4 Bias titik acuan yang ketiga: jangkar halangan 1,433 m

Terukur `ego.bounding_box.location.x = -0,005 m`, jadi titik asal aktor berimpit
dengan pusat bodi. `GroundTruthPerception` melaporkan relatif pusat bodi ego,
sedangkan `localization.obstacles_ego_to_road` menambahkan `ego.x` yang merupakan
sumbu belakang -- halangan di frame jalan meleset **1,433 m terlalu dekat**.

Arahnya konservatif, sehingga tidak pernah muncul sebagai kegagalan: zona aman
yang benar-benar berlaku 7,71 + 1,43 = **9,14 m** ke depan, bukan `ELLIPSE_A`
yang diturunkan di bagian 14.3.

Ini bias titik acuan ketiga di proyek ini (setelah bias XTE Tahap 1 dan kotak
penilai jarak, bagian 15). Polanya sama: **besaran benar, titik acuannya yang
salah, dan hasilnya tetap lolos karena kebetulan berada di sisi aman.**

Satu perbaikan membenarkan kedua pemakainya sekaligus, karena masing-masing
sudah menuliskan acuannya sendiri -- zona planner/MPC menggeser ego ke pusat
bodi, `main.py` mengurangkan sumbu belakang untuk FSM sesuai dokumentasi
`_v_follow`. Dikunci `tests/test_localization.py`.

### 19.5 Hasil kendali sesudah perbaikan

| | S1 sebelum | S1 sesudah | S3 sebelum | S3 sesudah |
|---|---|---|---|---|
| Vonis | BERHASIL | BERHASIL | BERHASIL | BERHASIL |
| Jarak min antar bodi | 1,43 m | **1,44 m** | 1,51 m | **1,47 m** |
| Deviasi lajur | 0,011 m | 0,009 m | 0,012 m | 0,011 m |
| Durasi manuver | 11,7 s | 11,6 s | 19,2 s | 18,9 s |

S3 turun karena zona aman tidak lagi kelebihan 1,43 m: ego boleh mendekat sampai
batas rancangannya. Keduanya masih jauh di atas syarat 1,0 m, dan sekarang
angkanya **dapat diturunkan dari constraint**, bukan kebetulan lolos.

### 19.6 Loop tertutup S1 dengan vision perception

`python main.py --scenario S1 --perception vision`. Vonis bagian 11.2
**BERHASIL**, tetapi perilakunya jelas lebih buruk daripada ground truth:

| | MPC + GT | MPC + vision |
|---|---|---|
| Vonis 11.2 | BERHASIL | **BERHASIL** |
| Jarak min antar bodi | 1,44 m | 1,79 m |
| Deviasi lajur | 0,009 m | 0,013 m |
| Durasi manuver | 11,6 s | 11,3 s |
| Waktu solve | 12,3 / 16,9 ms | 13,7 / 31,3 ms |
| Solver gagal | 0 dari 400 | 0 dari 400 |
| **Perlambatan terdalam** | −0,14 m/s² | **−4,43 m/s²** |
| **Tick tanpa kandidat planner** | 4 | **42** |
| **Simpangan lateral terjauh** | −3,82 m | **−5,46 m** |

Jarak bodi 1,79 m **bukan tanda lebih aman**: ego mencapainya dengan membanting
keluar jalur, bukan dengan menjaga jarak. Lajur menyalip membentang −1,75 sampai
−5,25 m, jadi −5,46 m berarti ego **keluar dari lajur tujuan selama 23 tick**
(1,15 detik) dan masuk lajur ketiga. Kriteria 11.2 tidak memeriksa hal itu --
kriteria itu ditulis untuk kembali ke lajur asal, bukan untuk menjaga lajur
selama menyalip. **Ini kekurangan definisi kriteria, dan perlu ditulis.**

Kandidat nol terjadi pada t = 5,00-8,75 s dan menyebar sampai state OVERTAKING
(pada GT hanya 4 tick, seluruhnya di CHECK_OVERTAKE). Rem terdalam pada t = 6,90 s
jatuh di tengah rentang itu.

**Acuan cadangan adalah tangga, dan itu yang membuat manuvernya kasar.** Saat
planner tidak menghasilkan kandidat, `main.xref_hold` memberi MPC garis lurus
pada `y_goal`. Di t = 6,30 s ego masih di y = -0,27 m sementara `y_goal` sudah
-3,50 m: acuannya melompat 3,23 m sekaligus. MPC mengejarnya -- `delta` -0,078 ->
-0,159 rad, `a_cmd` +1,25 -> -2,92 -> -4,43 m/s^2. Laju lateral yang terkumpul di
0,9 detik itu yang kemudian membawa ego sampai -5,46 m, jauh setelah kandidat
kembali tersedia: dari t = 8,90 s planner sudah 9/9 dan `delta` sudah positif
(membelok kembali), tetapi ego baru berhenti melebar di t = 10,30 s. Jadi
lambungan itu **sisa momentum, bukan percobaan keluar jalur yang kedua.**

Perbaikan yang masuk akal (belum dikerjakan): acuan cadangan menahan `y` yang
SEDANG berlaku, bukan `y_goal`. Itu membuat kehilangan kandidat menjadi "lanjutkan
lurus" alih-alih "lompat ke lajur tujuan sekarang".

**Terukur** (kolom `x_est`, `y_est` ditambahkan ke log, run diulang):

| Fase | n | Galat x: bias | RMS | Maks | Galat y: RMS |
|---|---|---|---|---|---|
| target di depan (> 10 m) | 117 | -0,04 m | **0,12 m** | 0,39 m | 0,37 m |
| berdampingan (<= 10 m) | 23 | **+1,31 m** | **1,86 m** | **3,82 m** | 0,69 m |

Galatnya **positif**: target dilaporkan lebih JAUH KE DEPAN daripada kenyataan,
memuncak +3,44 m saat jarak sebenarnya tinggal 3,1 m -- praktis seolah kendaraan
itu berada tepat di depan ego. Sebabnya koreksi muka -> pusat (bagian 18.4)
mengasumsikan yang terlihat muka BELAKANG; begitu berdampingan kamera melihat
SISI, dan penambahan `OTHER_LENGTH/2` di sumbu memanjang tidak lagi sah.

Perhatikan bahwa episode kandidat-nol yang PERTAMA (t = 6,4-7,15 s) bukan karena
galat memanjang -- di situ galat x masih <= 0,38 m. Yang meleset galat
**melintang**, tumbuh sampai -0,97 m ke arah lajur tujuan, yaitu persis ke tempat
ego sedang menuju. Jadi ada dua kegagalan berbeda, bukan satu.

Ini membenarkan butir 10.6 rencana kerja: bobot MPC dan parameter FSM **wajib**
dituning ulang di atas vision, bukan diwarisi dari tuning ground truth.

### 19.7 Video

`out/vision_s1.mp4` (400 frame, 20 detik, dari kamera yang sama yang dipakai
kendali). Isinya kotak deteksi berikut jarak dan kecepatan hasil Kalman filter,
seluruh kandidat lintasan planner yang lolos beserta yang sedang dieksekusi,
state FSM, laju ego, dan waktu solve. Dibangkitkan dengan
`main.py --perception vision --record`.

Kap mesin Dodge Charger menutup sekitar 15% bagian bawah citra, sesuai catatan
bagian 17 -- terlihat jelas di video dan layak dipakai sebagai gambar pendukung
batasan masalah.

### 19.8 Dua perbaikan: satu berhasil, satu salah sasaran

**Perbaikan A -- koreksi permukaan sadar-sudut-pandang (`perception.face_correction`).**
Rasio lebar/tinggi kotak deteksi membedakan tampak belakang (terukur 1,04) dari
tampak samping (2,48); geseran ke pusat bodi dicampur linier di antaranya.
Hasilnya terukur, bukan satu angka run tunggal:

| | Sebelum | Sesudah |
|---|---|---|
| galat x berdampingan, bias | +1,31 m | **+0,39 m** |
| galat x berdampingan, RMS | 1,86 m | **1,10 m** |
| galat x berdampingan, maks | +3,82 m | **+2,41 m** |
| galat y saat target di depan, RMS | 0,37 m | **0,07 m** |

**Perbaikan B -- acuan cadangan menahan `y` berjalan, bukan `y_goal`.** Tidak
memperbaiki apa pun: laju lateral puncak tetap 4,10 m/s (sebelumnya 4,12).
Sasarannya keliru. Yang mendorong banting setir bukan acuan planner, melainkan
**constraint elips MPC sendiri**: begitu halangan masuk zona aman, suku slack
berbobot `rho` = 1000 mendominasi biaya pelacakan, dan MPC menghindar tanpa
peduli acuannya berkata "tahan lajur". Perbaikan B tetap dipertahankan karena
alasannya berdiri sendiri, tetapi **belum terbukti memberi manfaat**.

### 19.9 TEMUAN: planner dan MPC tidak berbagi ruang kelayakan

Ini penjelasan sebenarnya kenapa ego "kehilangan arah" begitu kandidat habis.

Saringan penolak kandidat, dihitung ulang dari log (`planning.py` murni numerik,
jadi planner bisa diputar ulang tanpa simulator):

| Saringan | Tick nol kandidat |
|---|---|
| zona aman elips | 377 |
| percepatan lateral | 73 |
| kelengkungan | 0 |

**Uji pemisah yang menentukan:** pada lintasan ego yang SAMA, kandidat dihitung
dua kali -- sekali dengan estimasi vision, sekali dengan posisi halangan ground
truth. Hasilnya **86 tick nol kandidat pada keduanya, identik.** Jadi begitu
ayunan dimulai, ketelitian perception tidak lagi relevan: posisi halangan yang
sempurna pun tidak menyelamatkannya.

Yang membedakan adalah keadaan ego itu sendiri:

| | MPC + GT | MPC + vision |
|---|---|---|
| Laju lateral terukur, puncak | 1,61 m/s | **4,10 m/s** |
| Sudut hadap terhadap jalan, puncak | 6,7° | **17,1°** |

Penyebab strukturalnya: **planner membatasi kandidat pada `MAX_LATERAL_ACCEL`
3,0 m/s², sedangkan MPC tidak punya batas percepatan lateral sama sekali** --
satu-satunya batas kemudinya `DDELTA_MAX` per langkah. MPC karena itu boleh
membawa kendaraan ke keadaan yang planner tidak akan pernah rencanakan, dan
setelah itu planner tidak bisa melanjutkan dari keadaan tersebut karena setiap
quintic yang berangkat dengan laju lateral 4 m/s melanggar batasnya sendiri.
Lupnya menutup: kandidat habis -> MPC menghindar sendirian -> laju lateral naik
-> kandidat makin habis.

Ground truth tidak pernah menyingkapnya karena kandidat nyaris tak pernah habis
(4 tick). Vision menyingkapnya. **Ini bukan kekurangan perception, melainkan
kekurangan arsitektur yang selama ini tertutupi oleh perception yang sempurna.**
Satu keluarga dengan bagian 15.3 (lup planner-MPC lewat percepatan lateral
terukur), dan menunjukkan perbaikan 15.3 baru separuh: `ddy0` sudah diambil dari
rencana, tetapi `dy0` masih hasil ukur.

### 19.10 Run vision TIDAK terulang

Dua run S1 vision dengan kode identik:

| | Run 1 | Run 2 |
|---|---|---|
| Vonis | BERHASIL | BERHASIL |
| Jarak min antar bodi | 1,30 m | 1,31 m |
| Durasi manuver | 11,5 s | 10,9 s |
| Tick nol kandidat | 50 | 52 |
| **Simpangan lateral terjauh** | **−5,58 m** | **−4,55 m** |

Simpangan lateral berselisih **1,03 m antar run berkonfigurasi sama**. Render
kamera tidak deterministik (sudah tercatat di Tahap 8 Langkah 1), dan lup tidak
stabil di atas memperbesarnya. Konsekuensi untuk bagian 11.4: **perbandingan
MPC+GT versus MPC+vision tidak boleh memakai satu run per konfigurasi.** GT boleh
(deterministik bit-per-bit); vision menuntut ulangan, dan selisih di bawah ~1 m
pada metrik lateral tidak berarti apa-apa tanpa ulangan itu.

### 19.11 Rencana berkedip: planner tidak punya komitmen

Yang terlihat di video sebagai "beberapa kali mau menyalip lalu batal" bisa
dihitung dari log. Rencana muncul/hilang sepanjang run:

| | MPC + GT | MPC + vision |
|---|---|---|
| Rencana muncul/hilang | 2 kali | **10 kali** |

Panjang tiap kepingan rencana selama manuver vision, pada replan 10 Hz:
**2, 11, 2, 2, 1, 53 replan** = 0,2 s / 1,1 s / 0,2 s / 0,2 s / 0,1 s / 5,3 s.
Lima kepingan pertama jauh lebih pendek daripada durasi manuver yang
direncanakannya sendiri (`MANEUVER_TIMES` 3,0-4,0 s). Jadi ego berkali-kali
memulai quintic menuju lajur tujuan, lalu meninggalkannya setelah satu-dua
replan.

Sebabnya struktural: **planner tidak punya histeresis maupun komitmen.** Setiap
replan berdiri sendiri dan bersifat semua-atau-tidak-sama-sekali. Bandingkan
dengan FSM, yang justru diberi `FSM_DWELL` 0,3 detik dan ambang histeresis
berpasangan dengan alasan yang ditulis eksplisit di `config.py`: "tanpa keduanya,
noise perception membuat state bolak-balik dan MPC menerima referensi yang
berubah tiap frame". Perlindungan itu diberikan ke FSM dan **tidak pernah
diberikan ke planner**. Ditambah lagi, saat plan gagal `main.py` menyetel
`traj = None`, sehingga rencana sebelumnya dibuang seluruhnya dan replan
berikutnya memulai quintic baru dari nol dengan `ddy0 = 0`.

### 19.12 Batas lateral MPC disamakan dengan planner: bekerja, tapi tidak cukup

Constraint `|v^2 tan(delta)/L| <= MAX_LATERAL_ACCEL` ditambahkan ke MPC, memakai
konstanta yang SAMA dengan planner. `dy0` planner juga diambil dari rencana,
bukan hasil ukur (melengkapi perbaikan bagian 15.3 yang baru menyentuh `ddy0`).

Keduanya bekerja persis seperti maksudnya:

| | Sebelum | Sesudah |
|---|---|---|
| Percepatan lateral diperintahkan, puncak | tak dibatasi | **3,00 m/s² (tersaturasi)** |
| Laju lateral terukur, puncak | 4,10 m/s | **2,95 m/s** |
| Sudut hadap, puncak | 17,1° | **12,3°** |
| Kandidat ditolak saringan percepatan lateral | 73 | **32** |
| Solver gagal | 0 dari 400 | **0 dari 400** |

Dan 2,95 m/s memang sudah masuk ke dalam kemampuan planner: disapu di keadaan
tengah-manuver (y0 = −1,5 m, y_goal = −3,5 m), planner masih memberi 9 kandidat
sampai `dy0` = 3,0 m/s, tinggal 4 di 3,5, dan nol di 4,0.

**Tetapi perilaku loop tertutupnya tidak membaik.** Tick nol kandidat tetap 50,
dan penolak yang tersisa hampir seluruhnya elips (410 dari 450). Uji tukar
halangan pada lintasan yang sama memberi 49 (vision) vs 42 (GT) -- jadi
perception hanya menyumbang sekitar seperlima sisa masalah.

**Ongkos yang harus dicatat:** jarak bodi minimum turun 1,79 -> 1,30 -> **1,13 m**
seiring perbaikan-perbaikan ini, mendekati syarat lulus 1,0 m. Masuk akal secara
fisik: membatasi percepatan lateral mengurangi kemampuan MPC melebarkan jarak
saat berpapasan. `MAX_LATERAL_ACCEL` adalah **batas kenyamanan**, dan
memberlakukannya sebagai constraint keras berarti kenyamanan mengalahkan
pelebaran jarak. Itu keputusan perancangan yang harus dinyatakan, bukan efek
samping yang didiamkan.

### 19.13 KOREKSI: pemicu ternyata sudah cukup; yang mengikat adalah titik tak-bisa-balik

Draf sebelumnya menyimpulkan `TTC_TRIGGER` terlalu lambat. **Itu keliru**, dan
sapuan terhadap planner yang sebenarnya membantahnya. Celah minimum agar ada
kandidat, dihitung dengan memanggil saringan `plan_lane_change` sendiri:

| dv | Celah saat pemicu (`TTC_TRIGGER`·dv) | Celah minimum | Margin |
|---|---|---|---|
| 3,0 (= `DV_TRIGGER`) | 15,0 m | 13,5 m | +1,5 m |
| 6,4 (S1) | 32,0 m | 19,0 m | +13,0 m |
| 10,0 | 50,0 m | 25,0 m | +25,0 m |

Cukup di seluruh rentang. Kasus terketat adalah `dv` terkecil yang masih memicu,
dan di situ pun masih +1,5 m. Nilainya **tidak diubah**; yang ditambahkan uji
`test_ttc_trigger_sufficient_for_safety_zone` supaya kecocokan dua turunan terpisah
ini tidak lagi bersandar pada kebetulan (pelajaran bagian 15.4).

**Yang sebenarnya mengikat** adalah celah minimum sebagai fungsi seberapa jauh
ego sudah menyeberang:

| y ego | dy0 = 0 | dy0 = −1 m/s | dy0 = −2 m/s |
|---|---|---|---|
| 0,0 m | 19,0 m | 18,0 m | 16,0 m |
| −1,0 m | 18,0 m | 16,0 m | 13,5 m |
| −1,5 m | 17,0 m | 14,5 m | 11,5 m |
| −2,5 m | 14,0 m | 9,5 m | 8,0 m |
| −3,0 m | 10,5 m | 6,0 m | 4,0 m |

Celah yang dibutuhkan **mengecil** seiring ego menyeberang, dan mengecil lagi
kalau ego sedang bergerak lateral. Menyeberang karena itu adalah **balapan**
antara kemajuan lateral dan celah yang menutup pada 6,4 m/s. Konsekuensinya
tajam: **berhenti di tengah penyeberangan adalah tindakan terburuk yang mungkin**
-- ia menaikkan kembali celah yang dibutuhkan (kolom dy0 = 0) tepat ketika celah
yang tersedia sedang menyusut. Itulah yang dilakukan acuan cadangan, dan itulah
kenapa rencana yang berkedip berakibat fatal.

### 19.14 Tiga perbaikan terakhir dan hasilnya

1. **Uji geometri pemicu** (bukan perubahan nilai) -- mengunci `TTC_TRIGGER`
   terhadap syarat zona aman di seluruh rentang `dv`.
2. **Komitmen planner**: replan yang gagal tidak lagi membuang rencana berjalan;
   rencana dipegang sampai durasinya habis. `Trajectory.lateral_at` menjepit `t`
   ke durasi rencana -- polinomial quintic meledak di luar selangnya, dan sejak
   rencana dipertahankan `t` memang bisa melewatinya.
3. **Batas percepatan lateral MPC dilunakkan** dengan slack berbobot
   `MPC_RHO_LAT` = 50, dua puluh kali di bawah `rho` zona aman. Urutan
   prioritasnya jadi tegas: keselamatan menang, kenyamanan mengalah.

**S1, MPC + vision:**

| | Sebelum ketiganya | Sesudah |
|---|---|---|
| Jarak min antar bodi | 1,13 m | **2,12 m** |
| Perlambatan terdalam | −5,80 m/s² | **−2,71 m/s²** |
| Tick nol kandidat | 50 | **38** |
| Rencana muncul/hilang | 10 kali | **6 kali** |
| Kepingan rencana saat manuver | 2, 11, 2, 2, 1, 53 | **2, 11, 5, 55** |
| Laju lateral puncak | 4,10 m/s | **2,58 m/s** |
| Sudut hadap puncak | 17,1° | **10,8°** |
| Durasi manuver | 11,9 s | **10,5 s** |
| Simpangan lateral terjauh | −5,60 m | −5,38 m |

Tiga kepingan rencana terpendek (0,2 / 0,2 / 0,1 detik) hilang seluruhnya.

**Dan run vision menjadi terulang.** Dua run kode identik:

| | Run 1 | Run 2 |
|---|---|---|
| Jarak min antar bodi | 2,12 m | 2,12 m |
| Tick nol kandidat | 38 | 38 |
| Durasi manuver | 10,5 s | 10,5 s |
| Simpangan lateral terjauh | −5,38 m | −5,39 m |

Sebaran simpangan lateral antar-run runtuh dari **1,03 m menjadi 0,01 m**. Ini
bukti paling kuat bahwa yang diperbaiki memang ketidakstabilannya: nondeterminisme
render kamera tetap ada, tetapi tidak lagi diperbesar oleh lup yang tidak stabil.
Catatan bagian 19.10 -- bahwa vision menuntut ulangan -- tetap berlaku sebagai
kehati-hatian, tetapi alasan aslinya sudah hilang.

**Ground truth tidak mengalami regresi** (perubahan menyentuh MPC, jadi wajib
diperiksa ulang):

| | S1 sebelum | S1 sesudah | S3 sebelum | S3 sesudah |
|---|---|---|---|---|
| Vonis | BERHASIL | BERHASIL | BERHASIL | BERHASIL |
| Jarak min antar bodi | 1,44 m | 1,42 m | 1,47 m | 1,47 m |
| Durasi manuver | 11,6 s | 11,6 s | 18,9 s | 19,0 s |
| Tick nol kandidat | 4 | 4 | 0 | 0 |

**Ongkos yang harus dicatat:** waktu solve naik dari 12,3 ke 16,9 ms rata-rata
(GT) karena `N` variabel slack tambahan -- masih jauh di bawah anggaran tick
50 ms, tetapi bukan gratis. Deviasi lajur GT saat `LANE_KEEPING` juga naik
sedikit, 0,009 -> 0,015 m.

**Yang masih terbuka:** ego tetap melebar sampai −5,38 m, yaitu keluar dari lajur
tujuan (tepi −5,25 m). Perbaikan ini menyembuhkan ketidakstabilannya, bukan
lebarnya. Dan 38 tick nol kandidat masih jauh di atas 4 tick milik GT.


---

## 20. Tuning ulang di atas vision (Tahap 8, bagian 10.6)

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

16 September 2026, `tune_vision.py`.

### 20.1 Sapuan skenario penuh kini sah

`tuning.py` menyapu lewat step response garis lurus tanpa halangan dan tanpa FSM,
dan itu memang satu-satunya cara yang sah sebelumnya: skenario penuh belum
terulang, sehingga selisih antar konfigurasi tidak bisa dibedakan dari derau.
Setelah lup planner-MPC distabilkan (bagian 19.14), dua run S1 vision berkode
identik memberi jarak bodi 2,12 / 2,12 m dan tick nol kandidat 38 / 38. **Satu
run kini menggambarkan satu konfigurasi**, dan `tune_vision.py` menyapu justru
parameter yang tidak bisa disentuh step response: ambang FSM, bobot pemilihan
kandidat, dan slack yang menengahi kenyamanan versus jarak aman.

Satu jebakan yang sempat memberi vonis palsu: durasi run 15 detik membuat
SELURUH konfigurasi divonis `lane_departure`, karena manuver selesai ~15 s dan
run terpotong sebelum `PASS_HOLD` 2,0 detik terpenuhi. Durasi disamakan dengan
`main.py` (20 s).

### 20.2 Hanya satu pasangan parameter yang berubah

| Parameter | Nilai | Hasil sapuan | Keputusan |
|---|---|---|---|
| `MPC_Q[y]` | 20 -> **150** | lambungan −5,39 -> −4,75 m | **diubah** |
| `MPC_Q[psi]` | 450 -> **3400** | wajib mengikuti, lihat 20.3 | **diubah** |
| `K_DEV` | 20 | 10/20/40/80 memberi hasil sama | tetap |
| `FSM_DWELL` | 0,3 s | 0,5 dan 0,8 jauh lebih buruk | tetap |
| `PASS_MARGIN` | 8,0 m | 4/8/14 nyaris tak berbeda | tetap |

Sapuan `MPC_Q[y]` di skenario penuh:

| `Q_y` | Jarak min | Lambungan lateral | Nol kandidat | Rem | Deviasi |
|---|---|---|---|---|---|
| 20 | 2,12 m | −5,39 m | 38 | −2,76 | 0,029 |
| 60 | 2,13 m | −4,99 m | 38 | −1,79 | 0,025 |
| **150** | **2,12 m** | **−4,75 m** | **38** | **−0,46** | **0,019** |
| 300 | 2,08 m | −4,52 m | 40 | −0,43 | 0,015 |
| 600 | 2,01 m | −4,30 m | 42 | −0,70 | 0,015 |
| 1200 | 1,95 m | −4,24 m | 44 | −0,69 | 0,014 |

Kriteria pemilihan sama dengan yang dipakai untuk `kp` throttle dan `FOLLOW_TIME`:
**nilai terakhir sebelum ada metrik yang mulai memburuk.** Jarak bodi dan jumlah
kandidat datar sampai 150 lalu menurun terus; lambungan lateral membaik terus
tetapi dengan hasil yang makin mengecil. 150 juga yang mengembalikan ego ke
**dalam** lajur tujuan -- tepi lajur ada di −5,25 m.

### 20.3 TEMUAN: redaman ditentukan RASIO `Q_psi`/`Q_y`, bukan `Q_psi`

Sapuan skenario dan sapuan step response memberi jawaban berlawanan. Pada
`Q_y` = 150 dengan `Q_psi` tetap 450, step response justru rusak:

| | `Q_y`=20 | `Q_y`=150, `Q_psi`=450 | `Q_y`=150, `Q_psi`=3400 |
|---|---|---|---|
| Overshoot | 7,4 % | **28,9 %** | **7,5 %** |
| Settling | 1,60 s | 2,50 s | 1,60 s |
| Chatter | 0,0008 mrad | **0,0409 mrad** | **0,0003 mrad** |
| Jitter total | 1,560 | 3,717 | 1,710 |

Penyelesaiannya bukan memilih salah satu, melainkan menyadari bahwa `Q_psi` = 450
dulu dipilih sebagai titik redaman kritis **pada `Q_y` = 20**. Yang menentukan
redaman adalah rasionya: 450/20 = 22,5, dan 22,5 x 150 = 3375 -- sangat dekat
dengan 3400 yang terpilih dari sapuan step response secara independen. Prediksi
dan pengukuran bertemu.

Batas atasnya juga terukur: pada `Q_psi` = 6000 overshoot mencapai 0,0 % tetapi
chatter meledak ke 0,1929 mrad, yaitu 600x nilai di 3400.

**Pelajaran untuk pembahasan:** bobot MPC tidak boleh dituning satu per satu
kalau sepasang di antaranya bersama-sama menentukan satu sifat fisik. Sapuan
satu-dimensi pada `Q_y` akan selalu tampak buruk di step response, dan sapuan
satu-dimensi pada `Q_psi` tidak akan pernah menemukan 3400.

### 20.4 FSM tidak butuh peredaman tambahan

| `FSM_DWELL` | Jarak min | Lambungan | Nol kandidat | Rem |
|---|---|---|---|---|
| **0,3 s** | **2,11 m** | −4,73 m | **38** | **−0,42** |
| 0,5 s | 2,02 m | −4,71 m | 44 | −1,17 |
| 0,8 s | 2,05 m | −4,62 m | 68 | **−6,00 (tersaturasi)** |

Dugaan awal -- deteksi vision lebih berisik, jadi FSM perlu dwell lebih panjang
-- **terbantah**. Sebabnya peredaman sudah dipindahkan ke tempat yang lebih tepat:
`tracking.Tracker` menuntut `TRACK_N_INIT` = 3 frame berturut sebelum sebuah track
dilaporkan, dan menahannya melayang sampai 5 frame. FSM sudah menerima masukan
yang bersih, dan dwell tambahan hanya menunda keputusan sampai celahnya keburu
menyusut.

### 20.5 Hasil akhir

| | MPC + GT | MPC + vision |
|---|---|---|
| Vonis 11.2 | BERHASIL | **BERHASIL** |
| Jarak min antar bodi | 1,42 m | **2,10 m** |
| Deviasi lajur | 0,015 m | 0,018 m |
| Durasi manuver | 11,6 s | **10,4 s** |
| Perlambatan terdalam | −0,14 m/s² | **−0,95 m/s²** |
| Tick nol kandidat | 4 | 40 |
| Lambungan lateral | −3,89 m | −4,71 m (tepi lajur −5,25 m) |
| Waktu solve | 17,2 / 30,8 ms | 17,5 / 37,2 ms |
| Solver gagal | 0 dari 400 | 0 dari 400 |

S3 GT juga tidak berubah: BERHASIL, 1,47 m, 19,0 s, nol tick tanpa kandidat.
**Bobot baru tidak menyentuh hasil ground truth sama sekali** -- masuk akal,
karena redamannya dijaga sama dan acuan GT selalu mulus sehingga penguatan
lateral yang lebih tinggi tidak punya galat besar untuk dikejar.

**Yang masih terbuka:** 40 tick nol kandidat versus 4 milik GT. Angka itu
bertahan di SELURUH sapuan (38-44 di semua parameter), jadi ia **bukan soal
tuning** melainkan geometri zona aman -- lihat bagian 19.13. Menyelesaikannya
menuntut perubahan rancangan, bukan bobot.


---

## 21. Tahap 9 — eksperimen penuh S1 (matriks bagian 11.4)

> **DIGANTIKAN. Angka di bagian ini diukur dengan kendaraan target lama (Nissan
> Patrol) dan jalur perception lama.** Dipertahankan sebagai riwayat, bukan
> sebagai hasil. Yang berlaku untuk skripsi ada di **bagian 31**.


16 September 2026, `experiment.py`. Server CARLA **direstart tepat sebelum
pengukuran** -- README mencatat waktu solve 26-31 ms saat senggang versus 70 ms
setelah server berjalan berjam-jam, jadi tanpa restart seluruh klaim real-time
tidak sah.

### 21.1 Hasil

| | MPC + GT | MPC + vision |
|---|---|---|
| Ulangan | 5 | 10 |
| **Success rate** | **5/5 = 100 %** | **10/10 = 100 %** |
| Jarak min antar bodi | 1,423 m (sd 0,000) | **2,105 m (sd 0,004)** |
| Durasi manuver | 11,65 s (sd 0,00) | 10,43 s (sd 0,02) |
| Simpangan lateral terjauh | −3,892 m (sd 0,000) | −4,694 m (sd 0,026) |
| Deviasi lajur saat `LANE_KEEPING` | 0,0149 m | 0,0182 m (sd 0,0010) |
| Perlambatan terdalam | −0,14 m/s² | −0,68 m/s² (sd 0,18) |
| Tick tanpa kandidat planner | 4,0 (sd 0,0) | 39,4 (sd 0,9) |
| Waktu solve rata-rata | 17,36 ms (sd 0,15) | 17,92 ms (sd 0,08) |
| Waktu solve maksimum | 27,69 ms | 34,47 ms |
| **Solver gagal** | **0 dari 2.000** | **0 dari 4.000** |

Kedua mode lulus seluruh syarat bagian 11.2 di setiap ulangan. Jarak minimum
terhadap syarat 1,0 m: GT unggul 42 %, vision unggul 111 %.

### 21.2 Jumlah ulangan berbeda, dan alasannya berbeda

**GT diulang 5 kali bukan untuk success rate**, melainkan untuk membuktikan
determinisme masih berlaku setelah seluruh perubahan Tahap 8. Terverifikasi:
**log identik bit-per-bit di kelima ulangan**, sehingga sd = 0,000 pada setiap
metrik kendali. Satu run memang menggambarkan konfigurasi itu.

**Vision diulang 10 kali karena render kamera tidak deterministik.** Sebarannya
ternyata sangat rapat -- jarak bodi sd 0,004 m, durasi sd 0,02 s. Bandingkan
dengan sebelum lup distabilkan, ketika dua run berkode identik berselisih
**1,03 m** pada simpangan lateral (bagian 19.10). Turun menjadi sd 0,026 m.

**Jebakan di alat ukur, lagi.** Pemeriksaan determinisme mula-mula melaporkan
"TIDAK" padahal seluruh metrik kendali identik sampai digit terakhir. Sebabnya
dua: kolom `solve_ms` adalah jam dinding dan memang tidak deterministik, dan
`np.array_equal` mengembalikan False untuk NaN (kolom estimasi berisi NaN saat
tidak ada deteksi). Setelah kolom waktu dikecualikan dan `equal_nan` dipakai,
determinisme terkonfirmasi. Ini kejadian keempat dengan pola yang sama: **yang
rusak alat ukurnya, bukan yang diukur.**

### 21.3 Anggaran waktu satu tick

Anggaran `FIXED_DELTA_SECONDS` = 50 ms.

| | Rata-rata | Terburuk terukur |
|---|---|---|
| Inferensi YOLOPX (bagian 18) | 14,4 ms | 14,4 ms |
| Solve MPC | 17,9 ms | 34,5 ms |
| **Total jalur vision** | **32,3 ms** | **48,9 ms** |

Rata-ratanya nyaman, **tetapi kasus terburuknya praktis menyentuh anggaran.**
Ini harus ditulis apa adanya, bukan dilaporkan sebagai "17,9 ms dari 50 ms".
Waktu solve naik dari 12,3 ms sebelum Tahap 8 karena `N` variabel slack batas
percepatan lateral (bagian 19.12) -- jadi kelonggaran itu memang dibeli.

Catatan kejujuran: mesin tidak benar-benar senggang saat pengukuran (load
average ~3), meskipun server CARLA baru direstart. Angka di mesin yang
benar-benar sepi kemungkinan sedikit lebih baik, bukan lebih buruk.

### 21.4 Satu-satunya metrik yang jelas lebih buruk

Tick tanpa kandidat planner: **39,4 (vision) versus 4,0 (GT)**, dan sd-nya hanya
0,9 -- jadi ini sifat sistematis, bukan kebetulan satu run. Sudah ditelusuri di
bagian 19.13: penyebabnya geometri zona aman, bukan tuning (angkanya bertahan
38-44 di seluruh sapuan bagian 20) dan bukan ketelitian perception (uji tukar
halangan memberi 49 versus 42 pada lintasan yang sama).

**Yang penting untuk pembahasan:** kehilangan kandidat sesering itu TIDAK
menurunkan keselamatan -- jarak bodi vision justru 2,105 m versus 1,423 m milik
GT. Sejak planner berkomitmen pada rencana terakhirnya (bagian 19.14), replan
yang gagal tidak lagi berarti kehilangan arah. Angka 39,4 itu kini menandai
"planner tidak menemukan rencana BARU", bukan "kendaraan tidak punya rencana".


---

## 22. Metrik evaluasi per layer — definisi untuk slide

Disusun mengikuti pola slide perception yang sudah ada (rumus dulu, lalu tabel
hasil). Perception dikecualikan: angkanya datang dari test set, bukan dari run
skenario.

### 22.1 Localization Layer — apa yang sebenarnya bisa dievaluasi

**Peringatan lingkup.** Localization di skripsi ini memakai **transform ground
truth dari server CARLA**, bukan fusi IMU + GNSS dengan EKF. Modul
`localization.py` hanya mengubah frame: CARLA left-handed -> right-handed, dan
titik asal aktor -> titik sumbu belakang. Karena masukannya sudah kebenaran itu
sendiri, **layer ini tidak punya galat terhadap dirinya sendiri** -- tidak ada
RMSE posisi yang jujur bisa dilaporkan. Ini wajib masuk batasan masalah.

Yang sah dievaluasi untuk layer ini adalah **prediction model** yang hidup di
dalam solver MPC, yaitu kinematic bicycle. Itulah yang menentukan seberapa jauh
ke depan estimasi keadaan masih bisa dipercaya.

| Metrik | Rumus | Keterangan |
|---|---|---|
| Galat posisi pada horizon | `e_pos(tau) = || p_model(tau) - p_plant(tau) ||` | `p_model` = lintasan hasil integrasi bicycle model; `p_plant` = lintasan mesin fisika CARLA |
| Galat posisi akhir | `e_pos(T_uji)` | `T_uji` = 3 s (kecepatan operasi), 5 s (kecepatan rendah) |

Dievaluasi pada `tau` = horizon MPC = `MPC_N x MPC_DT` = **2,0 detik**.

Hasil (bagian 5, `validate_model.py`):

| | Kecepatan rendah (7-18 km/jam) | **Kecepatan operasi (34-50 km/jam)** |
|---|---|---|
| Percepatan lateral maks | ~0,3 m/s² | 2,05 m/s² |
| **Galat posisi @ 2 detik** | 0,052 m | **0,309 m** |
| Galat posisi akhir | 0,132 m @ 5 s | 0,538 m @ 3 s |

Kalimat siap pakai: *0,309 m adalah sejauh mana model dipercaya sebelum umpan
balik mengoreksi, dan koreksinya datang tiap 50 ms -- 40 kali lebih sering
daripada horizonnya.*

### 22.2 Planner Layer

| Metrik | Rumus | Keterangan |
|---|---|---|
| Laju kandidat layak | `rho = (1/K) * sum_k (n_feasible,k / 9)` | 9 = 3 `LATERAL_OFFSETS` x 3 `MANEUVER_TIMES`; K = jumlah replan |
| Replan tanpa kandidat | `(1/K) * sum_k 1[n_feasible,k = 0] x 100 %` | planner gagal memberi rencana baru |
| Jerk lateral RMS | `J = sqrt( (1/T) * integral (d a_lat / dt)^2 dt )` | `a_lat = v^2 tan(delta) / L`; quintic memang meminimalkan jerk |
| Zona aman tercapai | `g = ( (dx/A)^4 + (dy/B)^4 )^(1/4)`, aman bila `g >= 1` | `A` = 7,709 m, `B` = 3,204 m, antar PUSAT bodi |
| Durasi manuver terpilih | `T` dari kandidat termurah | 3,0 / 3,5 / 4,0 s |
| Offset lateral terpilih | `offset` dari kandidat termurah | 3,0 / 3,5 / 4,0 m |

### 22.3 Controller Layer (MPC)

| Metrik | Rumus | Keterangan |
|---|---|---|
| **IAE lateral** | `integral \|y - y_lane\| dt` | dihitung saat `LANE_KEEPING`; tengah lajur TIDAK ikut bergerak bersama ego, jadi galatnya sah |
| **IAE kecepatan** | `integral \|v - v_goal\| dt` | `v_goal` dari FSM, juga tidak menempel ke ego |
| ~~Galat lacak lateral RMS~~ | `sqrt( (1/K) * sum_k (y_k - y_ref,k)^2 )` | **JANGAN dipakai sebagai galat pelacakan** -- lihat 22.5 |
| Galat kecepatan RMS | `sqrt( (1/K) * sum_k (v_k - v_goal,k)^2 )` | |
| Sudut hadap maksimum | `max |psi|` | terhadap arah jalan |
| Waktu solve | rata-rata dan maksimum, ms | anggaran satu tick 50 ms |
| Iterasi solver | rata-rata dan maksimum | **tidak terpengaruh beban mesin** -- lebih jujur daripada ms |
| Laju keberhasilan solver | `(jumlah solve sukses / total) x 100 %` | |
| Kepatuhan zona aman | `max epsilon` | slack constraint elips; **0 = tidak pernah dilanggar** |
| Kepatuhan batas kenyamanan | `max epsilon_lat` | slack batas percepatan lateral; 0 = dihormati penuh |
| Percepatan lateral maksimum | `max \|v^2 tan(delta)/L\|` | terhadap `MAX_LATERAL_ACCEL` 3,0 m/s² |
| Jitter kemudi | `sum_k |steer_k - steer_(k-1)|` | ukuran kehalusan kemudi |
| Usaha kendali | `(1/K) * sum_k |a_cmd,k|` | |

### 22.5 KOREKSI: `y_ref` tidak bisa dipakai mengukur galat pelacakan

Draf sebelumnya melaporkan "galat lacak lateral RMS 0,0017 m" sebagai **galat
pengendali**. Itu keliru, dan keliru dengan cara yang sudah tercatat sendiri di
`TUNING_MPC.md` bagian 9 sebagai jebakan nomor 1: *"XTE terhadap acuan yang
di-anchor di posisi ego -> acuan bergerak bersama mobil; metriknya mengukur
konstruksi, bukan kinerja."*

Terukur ulang pada log S1:

| | \|y - y_ref\| rata-rata | maksimum |
|---|---|---|
| Tick replan (10 Hz) | **1,8 x 10^-8 m** | 2,1 x 10^-6 m |
| Tick antara dua replan | 1,6 x 10^-3 m | 6,6 x 10^-3 m |

Nol pada tick replan itu **definisi, bukan prestasi**: planner membangun rencana
BERANGKAT dari posisi ego saat itu, jadi `y_ref` sama dengan `y` per konstruksi.
Yang tersisa hanya galat lookahead 50 ms.

**Yang sah dilaporkan** adalah galat terhadap acuan yang tidak menempel ke ego:
tengah lajur saat `LANE_KEEPING`, dan `v_goal` untuk kecepatan. Keduanya sudah
masuk tabel di atas sebagai IAE.

**Yang belum bisa dihitung** adalah galat pelacakan lintasan yang sesungguhnya.
Itu menuntut pencatatan rencana pada lookahead TETAP -- misalnya apa yang
direncanakan untuk `t + 0,5 s` -- lalu dibandingkan dengan posisi sebenarnya
setelah selang itu. Satu kolom log tambahan, dan eksperimen harus diulang.

**Kenapa slack adalah metrik, bukan diagnostik internal:** constraint elips dan
batas percepatan lateral keduanya LUNAK -- solver boleh melanggarnya dengan
membayar penalti. Nilai slack terpakai karena itu adalah ukuran langsung
"seberapa jauh batas dilanggar", dan nol berarti seluruh horizon patuh. Ini
yang membedakan "constraint terpenuhi" dari "constraint kebetulan tidak aktif".

### 22.4 Skenario — kriteria keberhasilan (bagian 11.2)

Ditetapkan **sebelum** eksperimen dijalankan supaya success rate tidak subjektif.

| Syarat | Ambang | Konstanta |
|---|---|---|
| 1. Menyelesaikan urutan state FSM | lengkap | — |
| 2. Kembali ke lajur asal | simpangan <= 0,5 m, bertahan >= 2,0 s | `PASS_LATERAL`, `PASS_HOLD` |
| 3. Durasi manuver | <= 20 s sejak keluar `LANE_KEEPING` | `MANEUVER_LIMIT` |
| 4. Tidak ada tabrakan | sensor tabrakan CARLA | — |
| 5. Jarak antar bodi | > 1,0 m sepanjang run | `SAFE_DISTANCE` |

`success rate = (jumlah run lulus kelima syarat / jumlah run) x 100 %`.

Jarak antar bodi diukur dari **ground truth**, antar **pusat bodi**, dengan kotak
yang **diputar menurut sudut hadap** masing-masing kendaraan -- bukan dari
perception, dan bukan kotak sejajar sumbu. Yang dinilai harus benar; yang dipakai
mobil boleh berisik.


---

## 23. Hasil Tahap 9 lengkap, per layer dan per fase (S1)

> **DIGANTIKAN. Angka di bagian ini diukur dengan kendaraan target lama (Nissan
> Patrol) dan jalur perception lama.** Dipertahankan sebagai riwayat, bukan
> sebagai hasil. Yang berlaku untuk skripsi ada di **bagian 31**.


16 September 2026, `experiment.py` + `metrics.py --layer --experiment`.
GT 5 ulangan (log identik bit-per-bit), vision 10 ulangan. Server direstart
sebelum pengukuran. Nilai ditulis rata-rata ± sd lintas ulangan; tanpa ± berarti
sd di bawah resolusi yang dicetak.

### 23.1 Skenario — kriteria bagian 11.2 (slide 23)

| | MPC + GT | MPC + vision |
|---|---|---|
| Ulangan | 5 | 10 |
| **Success rate** | **5/5 = 100 %** | **10/10 = 100 %** |
| Jarak min antar bodi (syarat > 1,0 m) | **1,423 m** (sd 0,000) | **2,103 ± 0,004 m** |
| Durasi manuver (syarat <= 20 s) | 11,65 s (sd 0,00) | 10,43 ± 0,02 s |
| Tabrakan | 0 | 0 |
| Urutan state FSM | lengkap | lengkap |
| Kembali ke lajur <= 0,5 m, tahan 2,0 s | terpenuhi | terpenuhi |

### 23.2 Planner Layer (slide 21)

| Metrik | MPC + GT | MPC + vision |
|---|---|---|
| Kandidat lolos per replan (dari 9) | 8,01 | 7,57 ± 0,01 |
| **Replan tanpa kandidat** | **1 %** | **10 %** |
| Durasi manuver terpilih `T` | 3,32 s | 3,29 ± 0,01 s |
| Offset lateral terpilih | 3,55 m | 3,52 ± 0,00 m |
| **Jerk lateral RMS** | **1,33 m/s³** | **5,64 ± 0,06 m/s³** |
| **Zona aman `g` minimum** (>= 1 aman) | **1,022** | **1,128 ± 0,002** |

`g` minimum > 1 pada keduanya: zona aman **tidak pernah dilanggar sepanjang run**,
diukur dari ground truth antar pusat bodi. Vision justru lebih longgar (1,128
versus 1,022) -- konsisten dengan jarak bodinya yang lebih besar.

Jerk lateral vision **4,2x** lipat GT. Inilah harga yang dibayar untuk 10 %
replan tanpa kandidat: setiap rencana baru berangkat dari keadaan yang sedikit
berbeda, dan sambungannya terasa sebagai jerk. Ini metrik kenyamanan, bukan
keselamatan.

### 23.3 Controller Layer / MPC (slide 22)

| Metrik | MPC + GT | MPC + vision |
|---|---|---|
| **IAE kecepatan** \|v − v_goal\| | **4,556 m** | **3,670 ± 0,070 m** |
| **IAE lateral saat `LANE_KEEPING`** | **0,143 m·s** | **0,197 ± 0,013 m·s** |
| IAE lateral seluruh run | 17,168 m·s | 18,744 ± 0,064 m·s |
| **Galat prediksi @ 0,5 s, RMS** | **0,0204 m** | **0,0754 ± 0,0008 m** |
| **Galat prediksi @ 2,0 s, RMS** | **0,287 m** | **0,351 ± 0,002 m** |
| **XTE ke lajur terdekat, maks** | 1,733 m | 1,742 ± 0,003 m |
| ~~Galat lacak lateral RMS~~ | ~~0,0017 m~~ | ~~0,0270 m~~ (tidak sah, bagian 22.5) |
| Galat kecepatan RMS | 0,293 m/s | 0,252 ± 0,003 m/s |
| Sudut hadap maksimum | 6,47° | 8,63 ± 0,01° |
| Waktu solve rata-rata | 18,00 ± 0,34 ms | 18,07 ± 0,24 ms |
| Waktu solve maksimum | 32,57 ± 5,58 ms | 33,88 ± 3,48 ms |
| **Iterasi solver rata-rata** | **8,28** | **8,55 ± 0,01** |
| **Solver berhasil** | **100 %** (2.000 solve) | **100 %** (4.000 solve) |
| Slack zona aman maksimum | 0,00016 | 0,645 ± 0,027 |
| Slack batas kenyamanan maksimum | 0,00100 | 0,302 ± 0,024 |
| Percepatan lateral maksimum | 1,29 m/s² | 3,30 ± 0,02 m/s² |
| Jitter kemudi total | 0,272 | 0,506 ± 0,006 |
| Usaha kendali \|a\| rata-rata | 0,082 m/s² | 0,110 ± 0,005 m/s² |

**Iterasi solver 8,3-8,6 dari batas 100.** Angka ini yang layak dipakai untuk
klaim real-time, bukan milidetik: ia tidak terpengaruh beban mesin, dan uji
`test_mpc.py` memang meng-assert iterasi, bukan waktu.

**Slack: keduanya praktis nol di GT, kecil tapi nyata di vision.** Slack zona
aman 0,645 berarti solver pernah menerima pelanggaran elips **di dalam
horizon prediksinya** -- tetapi `g` minimum yang benar-benar TERJADI tetap 1,128
(bagian 23.2). Dua hal berbeda: MPC merencanakan lewat titik yang sedikit
melanggar, lalu kenyataan tidak pernah sampai ke sana karena replan 20 Hz
mengoreksinya lebih dulu. Ini justru argumen untuk receding horizon, dan layak
ditulis begitu -- bukan disembunyikan.

Percepatan lateral vision 3,30 m/s² melampaui batas kenyamanan 3,0 m/s² sebesar
10 %. Batas itu memang **lunak** sejak bagian 19.12, dan pelunakannya disengaja:
sebagai constraint keras ia menurunkan jarak bodi ke 1,13 m.

### 23.4 Per fase manuver

Bagian 15.5 menuntut metrik dipisah per fase. `LANE_KEEPING` juga dipisah
sebelum dan sesudah manuver -- dua hal berbeda meski namanya sama.

**MPC + GT** (5 ulangan; sd = 0 pada semua kolom, determinisme)

| Fase | tick | lacak rata | lacak maks | yaw maks | v err | a_lat maks | jitter |
|---|---|---|---|---|---|---|---|
| LANE_KEEPING (sebelum) | 92 | 0,00000 | 0,00000 | 0,00° | 0,042 | 0,00 | 0,000 |
| CHECK_OVERTAKE | 10 | 0,00000 | 0,00000 | 0,00° | 0,010 | 0,00 | 0,000 |
| LANE_CHANGE_OVERTAKE | 72 | 0,00189 | 0,00660 | 6,47° | 0,331 | 1,29 | 0,117 |
| OVERTAKING | 40 | 0,00120 | 0,00429 | 2,62° | 0,368 | 0,77 | 0,033 |
| LANE_CHANGE_RETURN | 86 | 0,00135 | 0,00513 | 5,39° | 0,410 | 1,00 | 0,098 |
| LANE_KEEPING (sesudah) | 100 | 0,00024 | 0,00236 | 1,32° | 0,134 | 0,41 | 0,016 |

**MPC + vision** (10 ulangan, rata-rata ± sd)

| Fase | tick | lacak rata | lacak maks | yaw maks | v err | a_lat maks | jitter |
|---|---|---|---|---|---|---|---|
| LANE_KEEPING (sebelum) | 96 | 0,00000 | 0,00000 | 0,00° | 0,040 | 0,00 | 0,000 |
| CHECK_OVERTAKE | 10 | 0,00000 | 0,00000 | 0,00° | 0,012 | 0,00 | 0,000 |
| LANE_CHANGE_OVERTAKE | 62 | 0,02141±0,00006 | 0,13010±0,00156 | 8,63±0,01° | 0,235±0,003 | 3,02 | 0,285±0,007 |
| OVERTAKING | 25 | 0,04124±0,00107 | 0,29048±0,00593 | 7,83±0,15° | 0,199±0,029 | 3,30±0,02 | 0,078±0,002 |
| LANE_CHANGE_RETURN | 85 | 0,00174±0,00003 | 0,00690±0,00017 | 6,74±0,02° | 0,393±0,007 | 1,37±0,03 | 0,118 |
| LANE_KEEPING (sesudah) | 122 | 0,00023±0,00001 | 0,00280±0,00009 | 1,50±0,12° | 0,135±0,005 | 0,49±0,02 | 0,020±0,001 |

**Yang harus dibaca dari tabel ini:** galat lacak terpusat seluruhnya di dua fase
ketika halangan berada di dalam zona aman -- `LANE_CHANGE_OVERTAKE` dan
`OVERTAKING`. Di luar itu, vision dan GT sama-sama melacak sampai 2 milimeter.
Jadi selisih GT versus vision **bukan** "pengendali lebih buruk", melainkan
"constraint keselamatan lebih sering aktif karena halangannya berisik".

Dan angka `LANE_KEEPING (before)` = 0,00000 m menutup catatan 15.5 secara
kuantitatif: deviasi lajur yang selama ini dilaporkan sebagai ~0,015 m
seluruhnya berasal dari ekor SESUDAH manuver, bukan dari kemampuan menjaga lajur.


---

## 24. Daftar gambar dan video untuk slide

Seluruhnya di `out/`, dibangkitkan ulang dari log tanpa menjalankan simulasi
(kecuali video, yang butuh server).

| Berkas | Isi | Perintah |
|---|---|---|
| `compare_s1.png` | **GT versus vision berdampingan** — simpangan lateral, kecepatan, jarak antar bodi, kandidat planner. Gambar paling padat informasi untuk slide hasil | `python plot_compare.py` |
| `run_s1_mpc_gt.png` | S1 ground truth, 4 panel, latar diwarnai state FSM | `python plot_run.py` |
| `run_s1_mpc_vision.png` | S1 vision, format sama | `python plot_run.py --perception vision` |
| `run_s3_mpc_gt.png` | S3 ground truth (mengikuti lalu menyalip ulang) | `python plot_run.py --scenario S3` |
| `vision_s1.mp4` | Video kamera dengan kotak deteksi berisi jarak dan kecepatan, kandidat planner, dan yang dieksekusi | `python main.py --perception vision --record` |
| `detection_30m_lane0.png` | Deteksi + segmentasi, model fine-tuned | `python check_detection.py` |
| `detection_30m_lane0_bdd.png` | Pembanding: weight BDD100K asli | `python check_detection.py --weight ...` |
| `detection_15m_lane1.png` | Target di lajur menyalip | `python check_detection.py --lane 1` |
| `sensor_rig_photo.png` | **Foto ego di CARLA** dengan sensor dan sumbunya ditimpakan, ala KITTI Gambar 2a | `python show_rig.py` |
| `sensor_rig_topdown.png` | **Skema berdimensi tampak atas**, ala KITTI Gambar 2b | `python show_rig.py` |
| `sensor_rig_topdown_render.png` | Tampak atas **hasil render CARLA**, berdimensi | `python show_rig.py` |
| `sensor_rig.png` | Penempatan kamera: tampak samping + bidang pandang | `python plot_concepts.py` |
| `localization_frames.png` | Tiga titik acuan + rantai konversi frame | `python plot_concepts.py` |
| `scenario_s1.png` | Susunan S1 + tabel parameter kedua kendaraan | `python plot_concepts.py` |
| `pipeline.png` | Rantai end-to-end berikut laju tiap blok | `python plot_concepts.py` |
| `mpc_concept.png` | Receding horizon, dengan batang waktu horizon | `python plot_concepts.py` |
| `sensor_rgb.png`, `sensor_depth.png` | Contoh keluaran rig kamera | `python check_sensors.py` |
| `model_validation.png` | Validasi bicycle model (Tahap 1) | `python validate_model.py` |
| `planner_candidates.png` | 9 kandidat lintasan planner | `python show_lanes.py` |
| `ttc_threshold.png`, `why_q_psi_450.png`, `mpc_concept.png` | Gambar konsep untuk bab 2-3 | — |

**Yang belum ada gambarnya:** tidak ada visual untuk S3 + vision, karena S3 tidak
bisa dijalankan dengan rig satu kamera depan (kendaraan lajur tujuan mulai di
belakang ego). Itu keterbatasan yang dinyatakan, bukan gambar yang tertinggal.


---

## 25. Metrik galat: XTE, IAE, dan galat prediksi (S1)

> **DIGANTIKAN. Angka di bagian ini diukur dengan kendaraan target lama (Nissan
> Patrol) dan jalur perception lama.** Dipertahankan sebagai riwayat, bukan
> sebagai hasil. Yang berlaku untuk skripsi ada di **bagian 31**.


17 September 2026. Menjawab janji proposal soal XTE dan IAE, dengan definisi
yang diperbaiki di bagian 22.5. GT 5 ulangan, vision 10 ulangan.

### 25.1 XTE terhadap tengah lajur terdekat

| Metrik | MPC + GT | MPC + vision |
|---|---|---|
| XTE RMS | 0,535 m | 0,585 ± 0,006 m |
| **XTE maksimum** | **1,733 m** | **1,742 ± 0,003 m** |
| IAE lateral | 5,890 m·s | 6,576 ± 0,096 m·s |
| ISE lateral | 5,726 m²·s | 6,768 ± 0,152 m²·s |
| ITAE lateral | 35,37 m·s² | 34,88 ± 0,621 m·s² |

**XTE maksimum 1,733 m terhadap lebar lajur 3,50 m** — setengahnya 1,75 m. Itu
bukan kebetulan melainkan bukti definisinya benar: nilai terbesar terjadi tepat
saat ego berada di tengah antara dua lajur, dan secara geometris tidak mungkin
lebih besar selama ego menyeberang tanpa melewatinya.

Dipisah per fase, XTE saat `LANE_KEEPING` **sebelum** manuver = 0,000 m dan
**sesudah** = 0,029 m. Itulah angka lane-keeping yang sah untuk dikutip.

### 25.2 Galat prediksi pada lookahead tetap — metrik MPC yang sah

Rencana untuk `t + 0,5 s` dan `t + 2,0 s` dicatat, lalu dibandingkan dengan
posisi yang **benar-benar terjadi** setelah selang itu. Acuannya tidak menempel
ke ego, jadi bebas dari masalah bagian 22.5.

| | MPC + GT | MPC + vision |
|---|---|---|
| Galat prediksi @ 0,5 s, RMS | **0,0204 m** | **0,0754 ± 0,0008 m** |
| Galat prediksi @ 0,5 s, maks | 0,059 m | 0,478 ± 0,014 m |
| Galat prediksi @ 2,0 s, RMS | **0,287 m** | **0,351 ± 0,002 m** |
| Galat prediksi @ 2,0 s, maks | 1,419 m | 1,615 ± 0,015 m |

**Pada 0,5 detik, vision 3,7 kali lebih buruk daripada GT** (0,0754 versus
0,0204 m). Inilah penurunan kinerja pengendali yang sesungguhnya akibat
perception berisik — bukan angka 0,0017 versus 0,0270 m yang sempat dilaporkan
dan ternyata artefak.

### 25.3 TEMUAN: pada horizon penuh, yang membatasi adalah MODEL

Bandingkan galat prediksi 2 detik dengan validasi bicycle model Tahap 1
(bagian 5), yang diukur **lup terbuka, tanpa pengendali, tanpa gangguan**:

| | Nilai |
|---|---|
| Mismatch bicycle model @ 2 s (Tahap 1, lup terbuka) | **0,309 m** |
| Galat prediksi MPC @ 2 s (Tahap 9, lup tertutup, GT) | **0,287 m** |
| Galat prediksi MPC @ 2 s (lup tertutup, vision) | 0,351 m |

Ketiganya praktis sama. Artinya **seluruh horizon 2 detik dibatasi oleh
ketidaksesuaian model, bukan oleh pengendali maupun oleh derau perception.**
MPC melacak rencananya sampai 2 cm pada 0,5 detik; yang membuat prediksi 2 detik
meleset 29 cm adalah kinematic bicycle yang memang tidak menangkap fisika CARLA
sepenuhnya.

Dua konsekuensi yang layak ditulis:

1. **Memperpanjang horizon tidak akan menolong** selama modelnya tetap. Yang
   membatasi bukan berapa jauh solver melihat, melainkan seberapa jauh modelnya
   masih benar.
2. **Angka Tahap 1 terpakai kembali di Tahap 9.** Validasi model yang tampak
   seperti pekerjaan persiapan ternyata memberi batas bawah untuk kinerja
   pengendali di akhir — dan keduanya diukur dengan cara yang sepenuhnya
   berbeda, lalu bertemu di angka yang sama.

### 25.4 IAE kecepatan

| | MPC + GT | MPC + vision |
|---|---|---|
| IAE `\|v - v_goal\|` | 4,556 m | 3,879 ± 0,185 m |

Setara galat kecepatan rata-rata 0,23 m/s (GT) dan 0,19 m/s (vision) sepanjang
run 20 detik. Vision **lebih kecil** karena manuvernya selesai lebih cepat,
bukan karena pengendalinya lebih baik — jangan diklaim terbalik.

---

## 26. Dua cacat jalur vision, ditemukan 17 September 2026

> **ERA NISSAN PATROL (sebelum 18 Sep 2026).** Angka di bagian ini diukur dengan
> kendaraan target lama dan checkpoint lama. Temuannya tetap berlaku; angka yang
> berlaku ada di **bagian 31**.

> **KEDUANYA SUDAH DIPERBAIKI** pada hari yang sama -- lihat **bagian 27** untuk
> perbaikannya dan hasil ulangnya. Bagian ini sengaja dipertahankan utuh sebagai
> diagnosisnya: bagaimana kedua cacat ditemukan, dan bukti apa yang menunjukkan
> sebabnya. Itu bahan bab pembahasan, bukan sekadar catatan yang sudah basi.


Keduanya muncul dari satu sebab fisik yang sama: **rig satu kamera depan
(fov 90°) kehilangan target tepat saat ego berdampingan dengannya.** Yang
berbeda adalah akibatnya — satu membuat FSM memutuskan tanpa bukti, satu
membuat MPC menghindari halangan yang tidak ada.

Sumber angka: `out/experiment_s1_vision.npz` dan `out/experiment_s1_gt.npz`
(Tahap 9, 10 run vision + 5 run GT). Ketiga run vision pertama identik sampai
0,01 m, jadi angka di bawah memakai run 0 dan berlaku untuk seluruhnya.

### 26.1 Keputusan kembali ke lajur diambil saat ego buta

Gerbang kembali di `planning.BehaviorFSM`:

```python
belum_lewat = asal[asal[:, 0] > -config.PASS_MARGIN]
if len(belum_lewat) == 0 and not menjauh:
```

`len(...) == 0` **ambigu**. Daftar kosong bisa berarti dua hal yang berlawanan:

- "target sudah saya lewati sejauh `PASS_MARGIN` = 8 m" — aman
- "saya tidak melihat apa-apa" — tidak tahu apa-apa

Dengan `GroundTruthPerception` ambiguitas ini tidak pernah terwujud, karena
target selalu terlihat. Dengan vision ia terwujud **di setiap run**.

Runtutan kejadiannya, run 0:

| t (s) | Jarak memanjang sesungguhnya | Keadaan perception |
|---|---|---|
| 8,60 | +4,23 m (target masih di depan) | pengukuran asli terakhir |
| 8,65-8,85 | +3,89 → +2,53 m | **melayang** — Kalman menebak |
| 8,90 | +2,18 m | `lost` > `TRACK_MAX_LOST`, track dihapus |
| 9,70 | **-3,17 m** | daftar halangan masih kosong → **FSM memutuskan kembali** |

Bukti bahwa 8,65-8,85 adalah lamunan, bukan pengukuran: `x_est` naik persis
0,505 m tiap tick dan `y_est` persis 0,172 m tiap tick, konstan sampai tiga
angka di belakang koma. Itu tanda tangan prediksi kecepatan tetap.

**Keputusan kembali diambil setelah 1,10 detik tanpa satu pun pengukuran.**

Ada tiga mekanisme yang menunda, tetapi **tidak satu pun memverifikasi bahwa
target sudah terlewati**:

| Mekanisme | Nilai | Yang sebenarnya dijaga |
|---|---|---|
| `TRACK_MAX_LOST` | 5 frame = 0,25 s | track tidak mati karena satu frame gagal |
| gerbang `receding` (`DD_RETURN`) | 0,1 m/s | quintic kembali tidak berangkat sambil menjauh |
| `FSM_DWELL` | 0,3 s | derau tidak mengubah state |

Ketiganya kebetulan menunda. Tidak ada yang bertanya "apakah saya benar-benar
sudah melewatinya?"

Akibatnya terukur:

| | Jarak memanjang saat memutuskan kembali | Target terdeteksi? |
|---|---|---|
| MPC + GT (3 run) | **-10,40 m** (lewat 8 m, syarat terpenuhi) | ya |
| MPC + vision (run 0, 1) | **-3,17 m** | **tidak** |
| MPC + vision (run 2) | -2,34 m | **tidak** |

`PASS_MARGIN = 8,0 m` **tidak pernah terpenuhi oleh bukti** di jalur vision.
Ego memotong balik 7,2 m lebih awal daripada aturannya sendiri.

**Kenapa tetap tidak menabrak.** Manuver kembali memakan 3,6 s, dan selama itu
ego melaju ~6,4 m/s lebih cepat. Saat bodi ego benar-benar masuk kembali ke
lajur asal, jaraknya sudah -27,6 m. Jarak bodi minimum vision 2,11 ± 0,01 m,
justru lebih longgar daripada GT 1,42 m.

Itu keselamatan yang datang dari **durasi manuver**, bukan dari gerbangnya.
Harus ditulis begitu. Kalau target lebih cepat, atau manuver kembali lebih
agresif, marginnya habis dan gerbangnya tidak akan menahan apa pun.

### 26.2 TEMUAN: MPC menghindari halangan yang tidak pernah ada

Saat ego mulai berdampingan, kotak deteksi berubah dari tampak belakang menjadi
tampak samping. Di masa peralihan itu `perception.face_correction` **gagal
mengenali peralihannya**, dan estimasi target bergeser mendekat ke ego.

Galat pada t = 8,15 s: **-0,43 m memanjang dan -1,06 m melintang**. Keduanya
menunjuk ke arah ego.

Angka -1,06 m itu bukan sembarang: `OTHER_WIDTH / 2 = 0,966 m`. Artinya koreksi
melintang praktis **tidak diterapkan sama sekali** (`f ≈ 0`) justru pada saat ia
paling dibutuhkan — estimasi berhenti di permukaan sisi yang terlihat, tidak
diteruskan ke pusat bodi di baliknya.

Akibatnya pada zona aman superelips `g = ((dx/A)^4 + (dy/B)^4)^(1/4)`:

| t (s) | `g` dari estimasi | `g` dari ground truth | kandidat lolos | slack elips `eps` |
|---|---|---|---|---|
| 7,90 | 1,139 | 1,249 | 9 | 0,000 |
| 8,00 | 1,061 | 1,207 | **0** | 0,301 |
| 8,15 | 0,970 | 1,172 | **0** | **0,675** |
| 8,45 | **0,921** | 1,233 | **0** | 0,495 |
| 8,50 | 1,034 | 1,254 | 9 | 0,000 |

**Geometri sesungguhnya tidak pernah melanggar.** `g` sebenarnya tidak pernah
turun di bawah **1,169** — margin 17% sepanjang manuver. Yang dilihat MPC turun
sampai **0,921**, yaitu pelanggaran 8%.

Reaksinya persis seperti yang seharusnya untuk halangan sungguhan:

- kandidat planner runtuh **9 → 0**
- slack elips naik 0 → 0,675; dengan `rho = 1000` suku itu mendominasi biaya
- kemudi berbalik dari -0,016 ke **+0,047** (membanting menjauhi lajur asal)
- laju yaw berbalik dari **+4,0 °/s menjadi -9,3 °/s** — pembalikan 13 °/s
- ego melebar sampai **-4,71 m**, dari tepi lajur -5,25 m

Lalu pada t = 8,50 deteksi pulih (kotak sudah benar-benar tampak samping, rasio
lebar/tinggi melewati ambang), `g` estimasi kembali di atas 1, 9 kandidat
kembali lolos, dan ego berbelok pulih pada +12 °/s. Dari kursi pengemudi:
membanting ke satu sisi lalu balik lagi, tanpa sebab yang terlihat.

**Inilah "ketidakstabilan saat melambung" yang dilaporkan sejak bagian 19.**
Sebabnya bukan tuning MPC, bukan acuan planner, melainkan **halangan hantu**
yang dilahirkan galat estimasi melintang selama peralihan sudut pandang.

### 26.3 38 tick tanpa kandidat terurai jadi tiga sebab berbeda

Angka "40 tick" yang dipakai sejak bagian 19.13 ternyata gabungan tiga kejadian
yang tidak berhubungan. Run 0, 38 tick:

| Selang (s) | Tick | Sebab | Ada juga di GT? |
|---|---|---|---|
| 5,00-5,25 | 6 | jepitan awal pindah lajur | ya (4,90-5,05, 4 tick) |
| 6,40-7,45 | 22 | **asimetri planner-MPC** (bagian 19.9): planner menolak keras di sepanjang horizon, `g` estimasi 2,38 → 1,51 — tidak ada pelanggaran sama sekali saat itu | tidak |
| 8,00-8,45 | 10 | **halangan hantu** (bagian 26.2) | tidak |

Perbedaannya penting untuk ditulis:

- Di selang 6,40-7,45 galat perception **dapat diabaikan** — `g` estimasi dan
  `g` sebenarnya hanya berbeda 0,02. Planner menolak karena melihat 2-4 detik ke
  depan dan memproyeksikan pelanggaran di masa depan, sementara MPC menerimanya
  secara lunak. Itu cacat **rancangan**, bukan cacat perception.
- Di selang 8,00-8,45 galat perception **adalah** sebabnya.

Jadi klaim lama "38-44 tick bertahan di seluruh sapuan parameter, berarti ini
geometri bukan tuning" tetap benar untuk 22 tick, tetapi **tidak benar untuk 10
tick berikutnya** — yang itu bisa hilang kalau `face_correction` diperbaiki.

### 26.4 Pilihan penanganan

| | Ongkos | Yang bisa dipertahankan di sidang |
|---|---|---|
| **A. Tulis sebagai batasan** | nol | "rig satu kamera depan tidak dapat memverifikasi `PASS_MARGIN`; keselamatan bersandar pada durasi manuver". Jujur, tetapi mengakui gerbangnya tidak bekerja |
| **B. Dead reckoning eksplisit** | ~10 baris | Saat track hilang sementara state masih `OVERTAKING`, teruskan posisi terakhirnya dengan kecepatan terakhir sampai jelas terlewat 8 m. Tetap tebakan, tetapi **dinyatakan** sebagai tebakan alih-alih disamarkan menjadi "kosong" |
| **C. Kamera belakang** | 14,4 ms/tick | Perbaikan sebenarnya; menutup 26.1 dan 26.2 sekaligus. Lihat `README.md` pekerjaan belum selesai nomor 3 |

Untuk 26.2 perbaikan yang setara adalah membuat `face_correction` tidak bergantung
pada rasio kotak saja — misalnya memakai sudut pandang geometris ke pusat kotak,
yang diketahui pasti dari kalibrasi kamera, sebagai pembobot kedua. Belum
dikerjakan.

**Status: sudah diperbaiki, lihat bagian 27.** Pilihan yang diambil bukan A
melainkan B untuk kedua cacat: dihitung, bukan ditebak (26.2) dan
diekstrapolasi, bukan dikosongkan (26.1). Opsi C (kamera belakang) tetap
terbuka dan akan menutup sisa yang dicatat di bagian 27.7.

---

## 27. Perbaikan kedua cacat bagian 26, dan hasil ulang (17 September 2026)

> **DIGANTIKAN untuk ANGKANYA. Kendaraan target masih Nissan Patrol di sini, dan
> peta HD masih dipakai.** Metodenya tetap berlaku dan penting; angkanya sudah
> digantikan **bagian 31**.

**Bagian 26 mendiagnosis, bagian ini memperbaiki.** Keduanya dikerjakan, lalu
seluruh jalur vision diukur ulang: 10 run Tahap 9 plus satu run rekaman video.
Jalur ground truth diukur ulang juga (5 run) dan hasilnya **identik bit-per-bit**
dengan sebelumnya -- yang memang seharusnya, dan menjadi bukti bahwa kedua
perbaikan hanya menyentuh jalur vision.

Berkas: `out/experiment_s1_vision_before.npz` versus
`out/experiment_s1_vision.npz`; video `out/vision_s1_before.mp4` versus
`out/vision_s1_after.mp4`. Bandingkan sendiri dengan
`python metrics.py --layer --experiment [--suffix _before]`.

### 27.1 Perbaikan 1 -- sudut pandang dihitung, bukan ditebak

`perception.face_correction` dulu menebak sudut pandang dari **rasio lebar/tinggi
kotak deteksi**. Rasio itu runtuh justru saat ego berdampingan, karena kotaknya
terpotong tepi citra.

Sudut pandangnya sendiri **sudah diketahui pasti** dari kalibrasi kamera: pada
titik `(d, y)` hasil balik-proyeksi, `theta = atan2(y, d)`. Pada sudut itu lebar
siluet target adalah `LENGTH*sin(theta)` dari sisi ditambah
`WIDTH*cos(theta)` dari buritan, dan **porsi sisi itulah bobot campurannya**:

```python
sisi = config.OTHER_LENGTH * abs(math.sin(theta))
belakang = config.OTHER_WIDTH * abs(math.cos(theta))
f = sisi / max(sisi + belakang, 1e-9)
return (1.0 - f) * config.OTHER_LENGTH / 2.0, f * config.OTHER_WIDTH / 2.0
```

Enam baris. Batas-batasnya benar dengan sendirinya: `theta = 0` memberi koreksi
memanjang penuh, `theta = 90 deg` memberi koreksi melintang penuh, dan
`f` terkurung di [0, 1] tanpa perlu dijepit.

Kalibrasi kamera tidak pernah terpotong tepi citra. Itulah seluruh alasan
perbaikan ini bekerja.

Hasilnya, galat estimasi saat berdampingan (10 run, seluruh tick terdeteksi
dengan |dx| < 8 m):

| | Sebelum | Sesudah | Perbaikan |
|---|---|---|---|
| galat memanjang, bias | +1,134 m | **+0,011 m** | 103x |
| galat memanjang, RMS | 1,681 m | **1,044 m** | 1,6x |
| galat memanjang, maks | 3,290 m | **1,830 m** | 1,8x |
| galat melintang, bias | -0,828 m | **-0,203 m** | 4,1x |
| galat melintang, RMS | 0,934 m | **0,214 m** | 4,4x |
| galat melintang, maks | 1,344 m | **0,326 m** | 4,1x |

Bias memanjang praktis lenyap. Itu angka yang paling layak dikutip: estimasi
yang dulu sistematis meleset satu meter ke depan kini tidak bias sama sekali.

### 27.2 Perbaikan 2 -- daftar kosong bukan lagi bukti

`planning.BehaviorFSM` dulu membaca `len(belum_lewat) == 0` sebagai "sudah
terlewat". Sekarang yang terakhir terlihat diteruskan dengan kecepatan
relatifnya sampai ekstrapolasi menyimpulkan sudah unggul `PASS_MARGIN`.

Satu detail menentukan, dan baru ketahuan setelah percobaan pertama: **laju
hanya dibekukan selagi target masih di depan.** Percobaan pertama memakai laju
yang terukur terakhir, yaitu laju saat berdampingan -- terukur -3,2 m/s padahal
sesungguhnya -6,4 m/s, karena kotaknya terpotong. Akibatnya kembali tertunda
sampai -21,5 m. Saat masih di depan, target terlihat utuh dan lajunya benar.

| | Sebelum | Sesudah | GT (acuan) |
|---|---|---|---|
| jarak saat memutuskan kembali | **-2,73 ± 0,39 m** | **-18,02 ± 0,40 m** | -10,40 m |
| buta sebelum memutuskan | 0,80 ± 0,05 s | 3,28 ± 0,05 s | 0,05 s |

Sekarang keputusannya **melewati** syarat 8 m, bukan melanggarnya. Tetapi
lewatnya jauh: -18,0 m versus -10,4 m milik GT. Penyebabnya ekstrapolasi
berangkat dari posisi terakhir yang **sudah melayang** (karena itu melebih-lebih)
lalu memakai laju yang konservatif. **Biasnya sengaja ke arah menunggu** --
untuk sensor yang buta, terlalu lama di lajur salip adalah kesalahan yang jauh
lebih murah daripada memotong terlalu cepat.

Ongkosnya nyata dan harus ditulis: manuver memanjang dari 10,43 s menjadi
13,03 s, dan ITAE ikut naik karena metrik itu menimbang galat akhir dengan waktu.

### 27.3 Hasil Tahap 9 -- ini angka yang berlaku untuk skripsi

10 run vision, 5 run GT. **10/10 dan 5/5 berhasil** di ketiga kondisi; tidak ada
tabrakan, tidak ada kegagalan solver.

| Metrik | Vision SEBELUM | **Vision SESUDAH** | GT |
|---|---|---|---|
| Jarak min antar bodi (syarat > 1,0 m) | 2,106 ± 0,004 m | **1,644 ± 0,009 m** | 1,423 m |
| Durasi manuver (syarat <= 20 s) | 10,43 ± 0,03 s | **13,03 ± 0,05 s** | 11,65 s |
| Lambungan lateral (tepi lajur -5,25 m) | -4,692 ± 0,030 m | **-4,280 ± 0,008 m** | -3,892 m |
| Perlambatan terdalam | -0,594 ± 0,147 m/s² | **-2,103 ± 0,124 m/s²** | -0,145 m/s² |
| Tick tanpa kandidat planner | 39,2 ± 1,0 | **33,8 ± 0,6** | 4,0 |
| Deviasi lajur saat LANE_KEEPING | 0,018 m | **0,017 m** | 0,015 m |

Perhatikan dua baris yang **memburuk**, dan keduanya punya penjelasan:

- **Jarak min turun 2,106 -> 1,644 m.** Itu bukan penurunan keselamatan
  melainkan lenyapnya lambungan berlebih: jarak 2,1 m dulu diperoleh karena ego
  menghindari hantu, bukan karena rancangan. Angka baru mendekati 1,423 m milik
  GT dari arah yang aman, dan tetap 64% di atas syarat 1,0 m.
- **Perlambatan dalam dari -0,59 menjadi -2,10 m/s².** Muncul hanya 8 tick
  (0,4 detik, t = 8,25-8,60 s) saat peralihan sudut pandang. Sebabnya justru
  estimasi yang kini benar: dulu target tampak +1,13 m lebih jauh ke depan
  sehingga celah memanjang tampak longgar dan MPC memilih menghindar ke samping;
  kini celahnya terlihat apa adanya, dan MPC memakai derajat kebebasan
  memanjang -- **mengerem alih-alih membanting setir**. Untuk penumpang itu
  pertukaran yang lebih baik, dan -2,10 m/s² masih di bawah ambang kenyamanan
  yang lazim dikutip (3 m/s²).

### 27.4 Metrik per layer

| Metrik | Vision SEBELUM | **Vision SESUDAH** | GT |
|---|---|---|---|
| **Galat prediksi @ 0,5 s, RMS** | 0,0754 ± 0,0008 m | **0,0361 ± 0,0001 m** | 0,0204 m |
| Galat prediksi @ 2,0 s, RMS | 0,3513 ± 0,0022 m | **0,3136 ± 0,0010 m** | 0,2873 m |
| Slack zona aman maks (0 = patuh) | 0,6445 ± 0,0254 | **0,2525 ± 0,0005** | 0,00016 |
| Slack batas lateral maks | 0,2067 ± 0,0900 | **0,0328 ± 0,0006** | 0,00100 |
| Galat lacak lateral RMS | 0,0238 ± 0,0036 m | **0,0101 ± 0,0001 m** | 0,0017 m |
| Sudut hadap maks | 8,63° | **7,25°** | 6,47° |
| Jerk lateral RMS | 5,653 ± 0,051 m/s³ | **4,688 ± 0,039 m/s³** | 1,331 m/s³ |
| Jitter kemudi total | 0,503 ± 0,010 | **0,461 ± 0,001** | 0,272 |
| Percepatan lateral maks | 3,207 m/s² | **3,033 m/s²** | 1,285 m/s² |
| Replan tanpa kandidat | 9,80% | **8,45%** | 1,00% |
| Kandidat lolos per replan (dari 9) | 7,577 | 7,579 | 8,010 |

**Galat prediksi 0,5 detik turun separuh: 0,0754 -> 0,0361 m.** Dulu vision 3,7
kali lebih buruk daripada GT; sekarang 1,8 kali. Itu ukuran paling langsung dari
"seberapa besar derau perception merusak pengendali", dan separuhnya ternyata
bukan derau melainkan **bias yang bisa dihilangkan**.

Slack zona aman turun 2,6 kali dan slack batas lateral 6,3 kali. Artinya MPC
kini jauh lebih jarang terpaksa melanggar batasnya sendiri -- persis yang
diharapkan bila halangan hantu hilang.

### 27.5 XTE dan IAE -- perbarui bagian 25 dengan angka ini

| Metrik | Vision SEBELUM | **Vision SESUDAH** | GT |
|---|---|---|---|
| XTE ke lajur terdekat, RMS | 0,5817 ± 0,0065 m | **0,5752 ± 0,0011 m** | 0,5351 m |
| XTE ke lajur terdekat, maks | 1,725 m | **1,748 m** | 1,733 m |
| IAE lateral | 6,576 ± 0,096 m·s | **7,026 ± 0,022 m·s** | 5,890 m·s |
| ISE lateral | 6,768 ± 0,152 m²·s | **6,617 ± 0,024 m²·s** | 5,726 m²·s |
| ITAE lateral | 34,88 ± 0,62 m·s² | **46,20 ± 0,22 m·s²** | 35,37 m·s² |
| IAE kecepatan | 3,879 ± 0,185 m | **4,300 ± 0,148 m** | 4,556 m |
| IAE lateral saat LANE_KEEPING | 0,1988 ± 0,0129 m·s | **0,1372 ± 0,0002 m·s** | 0,1432 m·s |

Dua catatan supaya tidak salah klaim:

- **ITAE naik 34,88 -> 46,20 m·s².** Itu **bukan** pengendalian yang memburuk.
  ITAE menimbang galat dengan waktu, dan manuvernya kini 2,6 detik lebih lama
  karena gerbang kembali menunggu bukti. Bandingkan ISE, yang tidak menimbang
  waktu: justru **turun** 6,768 -> 6,617 m²·s. Dua metrik bergerak berlawanan
  dari data yang sama, dan yang membedakan hanya bobot waktu.
- **IAE lateral saat LANE_KEEPING turun 0,1988 -> 0,1372 m·s**, kini **lebih
  baik daripada GT** (0,1432). Ekor setelah kembali ke lajur lebih tenang karena
  ego masuk dari jarak jauh, bukan memotong mepet.

Angka IAE kecepatan naik 3,879 -> 4,300 m karena manuvernya lebih panjang; lihat
peringatan bagian 25.4 yang berlaku sama.

### 27.6 Yang TIDAK berubah, dan kenapa itu penting

| | Sebelum | Sesudah |
|---|---|---|
| Ground truth, seluruh metrik | 1,423 m / 11,65 s / 4 tick | **identik bit-per-bit** |
| Waktu solve rata-rata | 19,35 ms | 19,41 ms |
| Kandidat lolos per replan | 7,577 | 7,579 |
| Uji otomatis | 91 lolos | **98 lolos** (7 uji regresi baru) |

Gerbang baru identik dengan gerbang lama selama halangan selalu terlihat:
`max(x) <= -PASS_MARGIN` adalah syarat yang sama dengan "tidak ada `x` di atas
`-PASS_MARGIN`". Itu bukan kebetulan melainkan rancangan, dikunci oleh uji
`test_ground_truth_behaviour_unchanged`, dan terbukti di data: lima run GT
menghasilkan log yang sama persis seperti sebelum perbaikan. **Karena itu
seluruh angka jalur ground truth di bagian 23 dan 25 tetap berlaku.**

### 27.7 Yang masih tersisa

- **33,8 tick tanpa kandidat** (dari 39,2). Sepuluh tick "halangan hantu" hilang
  seperti diramalkan bagian 26.3; sisanya adalah asimetri planner-MPC, yang
  memang perubahan rancangan dan bukan cacat perception.
- **`g` yang dilihat masih turun ke 0,931** sementara yang sesungguhnya 1,070.
  Selisihnya menyempit dari 0,253 menjadi 0,139, tetapi belum nol -- sisa galat
  memanjang RMS 1,044 m saat berdampingan masih ada. Menutupnya butuh kamera
  belakang, bukan tuning.
- **Kembali pada -18,0 m versus -10,4 m milik GT.** Konservatif karena buta;
  ongkosnya 2,6 detik lebih lama di lajur salip. Kamera belakang akan
  menyelesaikannya sekaligus.

Ketiganya bermuara pada satu hal yang sama, dan sudah tercatat sebagai pekerjaan
belum selesai nomor 3: **rig satu kamera depan**.

---

## 28. Perception tanpa peta HD (branch `perception-tanpa-gt`, 18 September 2026)

> **PERINGATAN ANGKA.** Bagian ini menjelaskan METODE dan hasil validasi
> per-modul. Angka Tahap 9 di bagian 21-27 diukur **sebelum** perubahan di sini
> dan **sebelum** kendaraan target diganti, jadi tidak lagi menggambarkan kode di
> branch ini. Eksperimen penuh harus diulang sebelum apa pun dibandingkan.

### 28.0 Persoalannya

Audit 17 September mendaftar enam tempat di jalur kendali vision yang masih
meminta data langsung ke simulator, di luar lokalisasi ego yang memang
dikecualikan. Tiga di antaranya diselesaikan di sini, satu dinyatakan sebagai
batasan, dan dua sisanya memang bukan ketergantungan.

| Sebelum | Sesudah |
|---|---|
| Dimensi kendaraan lain dari `bounding_box` simulator | diukur dari kotak deteksi |
| Zona aman dari dimensi itu | dari kendaraan desain PDGJ 2021 |
| Lebar & tengah lajur dari `LANE_WIDTH` dan peta | diukur dari kepala segmentasi |
| Arah jalan & jangkar frame dari `world.get_map()` | diukur, lalu dijejak |
| Kepala segmentasi lajur **dihitung lalu dibuang** (`box, _, _`) | dipakai |
| Depth camera eksak tanpa derau | **tetap** -- masuk batasan masalah |

Yang tersisa di jalur kendali: depth ideal, lokalisasi ego, dan `LANE_WIDTH`
sebagai nilai cadangan yang -- terukur -- tidak pernah aktif setelah frame
pertama. Peta HD masih dipakai untuk menempatkan kendaraan skenario dan sebagai
pembanding penilaian; keduanya alat ukur, bukan masukan kendali.

### 28.1 Geometri lajur: tiga lapis, dan yang tengah adalah kuncinya

**Lapis 1 -- piksel menjadi meter (IPM).** Kamera terpasang 1,65 m di atas jalan
dengan pitch nol, jadi baris `v` di bawah horizon memotong permukaan pada

    x = f * z / (v - c_v)        y = -x * (u - c_u) / f

Masukannya hanya tinggi pemasangan, intrinsik kamera, dan asumsi jalan datar.
Di kendaraan sungguhan ketiganya datang dari pemasangan dan kalibrasi papan
catur -- bukan dari API simulator. Balik-proyeksi lewat IPM dipilih, bukan lewat
depth, supaya lapisan ini tidak menambah ketergantungan pada sensor kedua.

**Lapis 2 -- bagaimana penggal marka menjadi SATU garis lajur.** Ini pertanyaan
yang paling sering muncul, dan jawabannya bukan di jaringan.

YOLOPX hanya mengeluarkan masker biner: tiap piksel dijawab "cat marka" atau
"bukan". **Tidak ada identitas, tidak ada nomor garis, tidak ada pengelompokan.**
Jaringan tidak tahu ada berapa garis lajur di depannya.

Penyatuan terjadi secara geometris. Di jalan lurus seluruh marka sejajar, jadi
mereka berbagi satu kemiringan `b`. Untuk tiap titik hitung

    c = y - b * x

`c` adalah **jarak melintang garis itu dari ego**, dan nilainya SAMA untuk
seluruh penggal milik satu garis lajur, sejauh apa pun penggal itu di depan.
Histogram `c` karena itu memuncak sekali per garis lajur -- dan **puncak itulah
identitas garisnya**. Penggalnya tidak pernah disambung; mereka hanya kebetulan
punya alamat melintang yang sama.

Kemiringan bersamanya sendiri dicari dengan menyapu calon `b` dan mengambil yang
histogramnya paling tajam (jumlah kuadrat cacah maksimum). Satu parameter dicari
dari ribuan piksel sekaligus, bukan satu garis dicocokkan dari serpihan.

**Lapis 3 -- kisi.** Marka lajur berjarak sama, seperti garis buku tulis, jadi
yang dicari cukup satu jarak `w` dan satu fase. Dicocokkan lewat rerata
melingkar: tiap offset dipetakan ke sudut `2*pi*c/w`, dan `w` terbaik
memaksimalkan panjang resultannya.

Lapis ini tidak ada di rancangan pertama, dan alasannya layak ditulis:

| Terhadap peta HD (320 frame) | Median jarak antar garis | **Kisi** |
|---|---|---|
| Lebar lajur, bias | +0,705 m | **-0,051 m** |
| Lebar lajur, RMS | 1,550 m | **0,074 m** |
| Lebar lajur, maks | 3,444 m | **0,500 m** |
| Simpangan ego, bias | +0,467 m | **+0,017 m** |
| Simpangan ego, RMS | 0,888 m | **0,051 m** |
| Simpangan ego, maks | 1,795 m | **0,101 m** |
| Sudut hadap, RMS | 0,259 deg | 0,257 deg |

**21 kali pada lebar lajur, 17 kali pada simpangan.** Median gagal karena satu
marka yang terlewat membuat celah ke tetangga berikutnya menjadi DUA KALI lebar
lajur, dan seluruh hitungan ikut melar. Pada kisi, marka yang hilang hanya
menyisakan lubang: jaraknya tetap ditentukan garis-garis lain.

Perhatikan sudut hadap **tidak berubah** -- ia sudah ditentukan di lapis 2, dan
lapis 3 memang tidak menyentuhnya. Itu memisahkan dengan bersih mana perbaikan
yang datang dari mana.

**Validasi.** `check_lanes.py`, 320 frame, pose ego DITETAPKAN dan disapu sengaja
(simpangan +-1,2 m, sudut hadap +-6 deg) supaya `lane_dev` yang selalu
mengembalikan nol pun tidak lolos. Terbaca 100% frame. Peta HD di situ alat ukur,
bukan masukan.

### 28.2 Dimensi kendaraan dari kotak deteksi

**Tinggi terukur langsung** dan tidak bergantung sudut pandang sama sekali:
`H = h_piksel * d / f`.

**Panjang dan lebar tidak bisa dipisahkan dari satu kotak.** Yang terukur lebar
siluet, dan pada sudut pandang `theta`

    W(theta) = L * |sin theta| + W * |cos theta|

Satu persamaan, dua anu. Diselesaikan kuadrat terkecil terbobot yang tumbuh tiap
frame selagi `theta` menyapu, dengan prior kendaraan desain sebagai regularisasi.

**Batas yang tidak bisa dilanggar, dan harus ditulis apa adanya.** Pada
`theta` = 0 target tepat di depan, `sin theta` = 0, dan panjang **tidak
menyumbang satu piksel pun**. Bukan kekurangan algoritma -- informasinya tidak
ada di citra, dan tidak ada metode apa pun yang bisa mengambilnya dari satu kotak
tampak-belakang. Padahal justru di situ `face_correction` memakai `length/2`
sepenuhnya.

Karena itu prior tidak bisa dihilangkan; yang bisa dipilih hanya **apa dasarnya**.
Dipakai kendaraan desain Toyota Hiace (PDGJ 2021 Tabel 5-9), sumber yang sama
yang menjustifikasi kendaraan uji di bagian 3 -- **bukan** bounding box
simulator. Itu pernyataan tentang jalan yang harus ditampung, bukan tentang mobil
yang kebetulan ada di depan.

Di manuver sungguhan target keluar dari fov 90 deg sebelum sudutnya menyapu jauh;
keteramatan tercatat **0,028**. Hasilnya jujur terbelah (Lincoln MKZ 2020,
~130 amatan per run):

| | Ditaksir | Sebenarnya | Galat |
|---|---|---|---|
| **Tinggi** | 1,507 m | 1,490 m | **+1,1%** |
| Lebar | 1,642 m | 1,837 m | -10,6% |
| Panjang | 4,046 m | 4,892 m | -17,3% |

**Tinggi sama sekali tidak punya prior**, dan itulah yang membuatnya jadi bukti:
kendaraan target diganti dari Nissan Patrol (SUV, 1,855 m) ke Lincoln MKZ 2020
(sedan, 1,490 m), dan penaksir melaporkan 1,507 m untuk mobil yang belum pernah
dilihatnya. Penaksir yang hanya mengembalikan priornya tidak akan bergerak.

**Pemisahan tugas yang lahir dari kegagalan.** Percobaan pertama memberi zona aman
dimensi hasil taksiran. Runnya **gagal lane_departure**: bias lebar 0,26 m sudah
cukup menggeser zona. Jadi keduanya dipisah menurut apa yang dituntut
masing-masing:

- **`face_correction` memakai dimensi terukur** -- yang dituntut KETELITIAN.
- **Zona aman memakai kendaraan desain** -- yang dituntut KESELAMATAN, dan margin
  keselamatan tidak boleh bisa MENYUSUT oleh galat penaksir.

Keduanya tetap bebas dari bounding box simulator.

Satu percobaan lain yang gagal dan sudah dikembalikan: mengukur dimensi pada
depth PUSAT bodi (`d + dx`) alih-alih depth permukaan, dengan alasan yang sama
yang melahirkan `face_correction`. **Keliru** -- tinggi terbentang di muka yang
terlihat, yang memang ada di depth `d`. Tinggi rusak dari +1,1% menjadi +14,3%,
lebar menabrak batas jepitnya, zona ikut berubah, dan run gagal. Yang ditaksir di
sini ukuran BENDA, bukan letak pusatnya.

### 28.3 Jangkar frame jalan: galat arah TUMBUH, galat posisi tidak

Titik asal dan arah frame jalan dulu dari `world.get_map()`. Sekarang keduanya
dari kepala segmentasi:

    arah jalan    = yaw ego - yaw ego terhadap lajur
    sumbu lajur   = posisi ego digeser sejauh simpangan terukur

`s = 0` tetap disamakan dengan frame peta. Itu **konvensi, bukan geometri** --
tidak ada besaran fisik yang bergantung pada di mana nol longitudinal
diletakkan -- dan menyamakannya membuat log kedua jalur bisa dibandingkan angka
per angka.

**TEMUAN: menjangkar sekali tidak cukup, dan alasannya berlaku umum.**

Percobaan pertama mengukur arah jalan sekali di akhir pemanasan lalu memakainya
seterusnya. Galat ukurnya 0,3 deg -- sekilas tak berarti. Tetapi arah yang
dibekukan membuat ego mengikuti sumbu yang sedikit salah, dan **galat arah tumbuh
linear terhadap jarak tempuh**: 250 m dikali tan(0,3 deg) = 1,3 m.

Dan ini bukan artefak frame yang miring. Ego **benar-benar** keluar dari tengah
lajur, karena ia dengan patuh melacak sumbu frame, bukan jalan:

| | Jangkar sekali | **Arah dijejak** |
|---|---|---|
| Simpangan lateral | -4,06 .. **+1,20 m** | -4,39 .. **-0,00 m** |
| Simpangan dari tengah lajur, maks | **0,871 m** | **0,297 m** |

Perbaikannya menjejak arah jalan dengan tapis tetapan waktu ~2,5 detik
(`config.ALPHA_ROAD_HEADING`). Satu detail tidak boleh dilewat: **frame tidak boleh
sekadar diputar.** Ego berada ~250 m dari titik asal, jadi rotasi 0,3 deg
melompatkan `y`-nya 1,3 m. Titik asalnya ikut digeser supaya `(x, y)` ego tetap;
yang dikoreksi hanya arah ke depan (`PathFrame.from_pose`).

Kalimat siap pakai untuk pembahasan:

> Galat posisi diam di tempat; galat arah tumbuh sebanding jarak tempuh. Sistem
> yang menjangkar arahnya sekali di awal karena itu akan selalu melenceng, berapa
> pun telitinya pengukuran awal tersebut.

### 28.4 Anotasi lajur menandai CAT, bukan batas lajur

`dataset_recorder.py` membangkitkan label lajur dari kamera segmentasi
semantik CARLA:

```python
ll = np.where(self.color_mask(seg, LL_COLOR), 255, 0)
```

`LL_COLOR` adalah kelas RoadLine, yang melabeli **cat yang benar-benar ada di
aspal**, piksel per piksel. Di antara dua penggal marka tidak ada cat, jadi tidak
ada label. BDD100K sebaliknya: anotator manusia menggambar **polyline menerus**
menyusuri batas lajur, termasuk di tempat yang tidak bercat -- itu tafsiran.

Jadi model tidak cacat. Ia mereproduksi persis apa yang diajarkan.

Dari situ lahir argumen yang paling kuat di bagian ini:

> **Menyambungkan marka adalah persoalan geometri, bukan persoalan segmentasi.**
> Jaringan hanya dapat melihat bukti di piksel, dan di celah antar-marka tidak
> ada bukti apa pun untuk dilihat. Yang mampu menjembatani celah itu adalah
> batasan geometris -- bahwa marka lajur lurus, sejajar, dan berjarak sama.

Bukti bahwa pembagian tugas itu benar: **sisa cocokan kisi 0,011 m** di ruas
lurus, ditampilkan langsung di HUD video supaya bisa diperiksa, bukan dipercaya.

Konsekuensi praktisnya tajam, dan menutup godaan yang wajar: melatih ulang dengan
label menerus justru **melemahkan** sistem secara metodologis. Jaringan akan
diajari mengarang cat yang tidak ada, lalu karangannya dipercaya -- padahal
besaran yang sama dapat dihitung secara tertutup dengan galat 11 mm.

### 28.5 Setelan render adalah parameter eksperimen

Diuji `-quality-level=Low` versus `Epic` dengan sapuan `check_lanes.py` yang
sama, 320 frame:

| Terhadap peta HD | Low | Epic |
|---|---|---|
| Lebar lajur, RMS | 0,074 m | 0,073 m |
| Simpangan ego, RMS | 0,051 m | **0,038 m** |
| Sudut hadap, RMS | 0,257 deg | **0,212 deg** |
| Sudut hadap, **maks** | **0,289 deg** | 0,573 deg |

Perbaikannya **sedang**, dan maksimum sudut hadapnya justru **memburuk**. Epic
tidak seragam lebih baik.

Yang layak ditulis: **setelan render memengaruhi ketelitian perception, jadi ia
bagian dari konfigurasi lingkungan uji** -- sederajat dengan resolusi kamera dan
fov, dan harus dinyatakan di bab 3. Seluruh angka wajib berasal dari satu setelan
yang sama. Dipilih **Low** (keputusan penulis, 18 September 2026).

**Koreksi.** Temuan ini sempat saya laporkan jauh lebih besar daripada
sebenarnya, berdasarkan nilai jangkar satu run: -0,012 deg di Epic versus
-0,300 deg di Low. Itu **keliru dan ditarik**. Keduanya satu sampel dari sebaran
ber-RMS 0,25 deg; -0,300 deg sama sekali tidak istimewa dan -0,012 deg kebetulan
beruntung. Yang berlaku adalah sapuan 320 frame di atas.

### 28.6 Hasil loop tertutup dan status

Konfigurasi baru (MKZ 2020, kualitas Low, jangkar dari kamera), tiga run berturut:

| | Vision | GT (acuan) |
|---|---|---|
| Vonis | **3/3 BERHASIL** | BERHASIL |
| Durasi manuver | 10,9 / 12,1 / 12,4 s | 11,6 s |
| Jarak min antar bodi | 1,80 / 1,81 / 1,81 m | 1,43 m |
| Simpangan lateral terjauh | -4,39 .. -4,51 m | -3,89 m |

Uji otomatis naik 91 -> **125**. Zona aman berubah menjadi 7,758 / 3,180 m
(dari 7,709 / 3,204) semata karena dimensi kendaraan target berganti.

**Yang belum, dan harus dikerjakan sebelum bab 4 ditulis:**

1. **Seluruh Tahap 9 harus diulang.** Kendaraan target berganti DAN jalur
   perception berubah; `experiment_s1_*.npz` yang ada diukur sebelum keduanya.
2. **Penilaian jalur vision masih memakai frame hasil perception**, jadi syarat
   `|y| < 0,5 m` menilai "kembali ke lajur yang DIYAKINI kamera". Selisih kedua
   frame ~0,05 m terhadap ambang 0,5 m sehingga vonisnya tidak berubah, tetapi
   pemisahannya ke frame peta menuntut dua kolom log tambahan dan harus dilakukan
   sebelum pengukuran ulang.
3. **Depth camera tetap ideal** -- tanpa derau, tanpa lubang, tanpa batas
   jangkauan. Satu-satunya ketergantungan sensor yang tersisa, dan harus
   dinyatakan di batasan masalah: yang diuji adalah perencanaan dan kendali di
   bawah persepsi berbasis citra, bukan sistem persepsi lengkap.

---

## 29. Hasil Tahap 9 setelah perception tanpa peta HD (28 September 2026)

> **CHECKPOINT LAMA (epoch 263).** Angka vision di bagian ini digantikan
> **bagian 31** (checkpoint epoch 92). Angka GT tidak bergantung checkpoint dan
> tetap sama. Bagian 21-27 lebih lama lagi: era Nissan Patrol.

Konfigurasi: Lincoln MKZ 2020 sebagai kendaraan yang disalip, render
`-quality-level=Low`, jangkar frame jalan dari kamera, arah jalan dijejak,
tengah lajur ditapis, dimensi kendaraan diukur, kisi kandidat planner berjangkar
pada lebar lajur nominal. 10 run vision, 5 run GT.

### 29.1 Vonis dan metrik end-to-end

| Metrik | MPC + vision (10 run) | MPC + GT (5 run) |
|---|---|---|
| **Vonis** | **10/10 BERHASIL** | **5/5 BERHASIL** |
| Jarak min antar bodi (syarat > 1,0 m) | **1,797 ± 0,017 m** | **1,432 m** (sd 0,000) |
| Durasi manuver (syarat <= 20 s) | 12,12 ± 0,05 s | 11,65 s |
| Simpangan lateral terjauh (tepi lajur -5,25 m) | -4,415 ± 0,008 m | -3,879 m |
| **Deviasi lajur, SEBELUM manuver** | **0,0176 ± 0,0016 m** | **0,0000 m** |
| Deviasi lajur, ekor SESUDAH manuver | 0,1303 ± 0,0012 m | 0,0286 m |
| Tick tanpa kandidat planner | 36,6 ± 0,9 | 4,0 |
| Solve rata-rata | 17,21 ± 0,09 ms | 18,21 ms |
| Kegagalan solver | 0 dari 4.000 | 0 dari 2.000 |
| Tabrakan | tidak ada | tidak ada |

**Jalur ground truth tetap deterministik penuh**: lima run identik bit-per-bit di
luar kolom waktu.

**Baris deviasi lajur DIPISAH sebelum/sesudah manuver, dan itu bukan kosmetik.**
Digabung, angkanya hampir seluruhnya berisi ekor transien setelah kembali ke
lajur -- jebakan yang sudah tercatat di bagian 15.5 untuk jalur GT, dan menggigit
jauh lebih keras di jalur vision. Diukur terhadap tengah lajur SEBENARNYA
(kolom `y_map`): **0,008 m sebelum manuver versus 0,271 m sesudahnya**.

Angka yang menjawab "seberapa baik sistem ini menjaga lajur" adalah yang
pertama. Yang kedua menjawab "berapa lama transien kembali mengendap", dan itu
pertanyaan lain.

**Catatan waktu solve maksimum.** Terukur 36,18 ± 9,97 ms dengan satu run
menyentuh 55,20 ms, melewati anggaran tick 50 ms. Pengukuran diambil saat mesin
menjalankan 13 langkah beruntun, jadi kemungkinan besar itu kontensi dan bukan
regresi solver -- rata-ratanya justru turun ke 17,21 ms. **Harus diukur ulang di
mesin senggang sebelum dikutip.**

### 29.2 Metrik per layer

| Metrik | Vision | GT |
|---|---|---|
| **Galat prediksi @ 0,5 s, RMS** | **0,0394 ± 0,0006 m** | **0,0203 m** |
| Galat prediksi @ 2,0 s, RMS | 0,3170 ± 0,0017 m | 0,2872 m |
| Galat lacak lateral RMS | 0,0142 ± 0,0003 m | 0,0017 m |
| Slack zona aman maks (0 = patuh) | 0,2046 ± 0,0041 | 0,00015 |
| Slack batas lateral maks | 0,0303 ± 0,0009 | 0,00100 |
| Sudut hadap maks | 7,50 ± 0,07 deg | 6,42 deg |
| Jerk lateral RMS | 4,064 ± 0,061 m/s³ | 1,326 m/s³ |
| Jitter kemudi total | 0,466 ± 0,003 | 0,270 |
| Zona aman `g` minimum (>=1 aman) | 1,069 ± 0,002 | 1,022 |
| Kandidat lolos per replan (dari 9) | 7,55 ± 0,01 | 8,02 |
| Replan tanpa kandidat | 9,15 ± 0,23 % | 1,00 % |

**Galat prediksi 0,5 detik: vision 1,9 kali GT.** Riwayatnya 3,7x sebelum
perbaikan bagian 26-27, 1,8x sesudahnya, dan tetap 1,9x setelah peta HD dibuang
seluruhnya dari jalur kendali. **Memangkas ground truth tidak memperburuk
pengendali** -- itu temuan utama bagian 28 dan 29.

`g` minimum 1,069 berarti zona aman **tidak pernah dilanggar** oleh geometri
sesungguhnya di seluruh 10 run.

### 29.3 Metrik galat: XTE, IAE, ISE, ITAE

**Diukur di frame PETA**, bukan di frame yang dijangkarkan kamera (bagian 28.3,
kolom `y_map`). Jadi yang dijawab "seberapa jauh ego dari lajur yang
SEBENARNYA", bukan "dari lajur yang diyakininya sendiri".

| Metrik | Vision | GT |
|---|---|---|
| XTE ke lajur terdekat, RMS | 0,5696 ± 0,0013 m | 0,5353 m |
| XTE ke lajur terdekat, maks | 1,746 ± 0,003 m | 1,735 m |
| IAE lateral | 7,598 ± 0,015 m·s | 5,877 m·s |
| ISE lateral | 6,490 ± 0,029 m²·s | 5,730 m²·s |
| ITAE lateral | 52,02 ± 0,18 m·s² | 35,28 m·s² |
| **IAE lateral saat LANE_KEEPING** | **0,662 ± 0,014 m·s** | 0,143 m·s |
| IAE kecepatan | 4,841 ± 0,234 m | 4,551 m |

XTE maksimum 1,746 m tetap mendekati setengah lebar lajur (1,75 m), yang
memvalidasi definisinya: nilai terbesar terjadi tepat saat ego di tengah antara
dua lajur, dan secara geometris tidak mungkin lebih besar.

**XTE dan IAE total praktis tidak bisa diperbaiki lagi lewat perception**, dan
alasannya struktural: keduanya **didominasi manuvernya sendiri**. Menyeberang
lajur adalah simpangan 3,5 m selama 12 detik; kedutan 0,1 m saat menjaga lajur
nyaris tidak menyumbang. Selisih terhadap GT (0,570 versus 0,535) sebagian besar
berasal dari ekor transien kembali, bukan dari derau perception.

Yang BISA diperbaiki, dan sudah: **IAE lateral saat LANE_KEEPING turun
1,036 -> 0,662 m·s** dalam satu hari, lewat tiga perbaikan yang berbeda
(bagian 29.5).

### 29.4 Ketelitian perception

**Geometri lajur** (`check_lanes.py`, 320 frame, pose disapu +-1,2 m dan +-6 deg,
terbaca 100%):

| | Bias | RMS | Maks |
|---|---|---|---|
| Sudut hadap | -0,062 deg | **0,086 deg** | 0,189 deg |
| Simpangan ego dari tengah | -0,004 m | **0,018 m** | 0,050 m |
| Lebar lajur | -0,053 m | 0,076 m | 0,500 m |

**Jarak dan kecepatan halangan** (`check_estimation.py`, sapuan 53 -> 9 m,
terdeteksi 111 dari 140 tick, deteksi pertama **44,0 m**):

| | Bias | RMS | Maks |
|---|---|---|---|
| x memanjang | +0,321 m | 0,323 m | 0,496 m |
| y melintang | +0,014 m | 0,018 m | 0,039 m |
| vx | +0,002 m/s | 0,021 m/s | 0,158 m/s |

**Dimensi kendaraan** (dari kotak deteksi): tinggi 1,507 m terhadap 1,490 m
sebenarnya (**+1,1%**), lebar 1,642 terhadap 1,837 m (-10,6%), panjang 4,046
terhadap 4,892 m (-17,3%), keteramatan 0,028. Batasnya diuraikan di bagian 28.2.

### 29.5 Tiga perbaikan yang membawa angka ini, dan empat dugaan yang gugur

Bagian 28 menutup jalur kendali dari peta HD. Bagian ini mencatat penyetelan
sesudahnya, yang seluruhnya lahir dari satu pertanyaan: **kenapa ego duduk 0,24 m
dari tengah lajur?**

**Perbaikan 1 -- tengah lajur ditapis.** FSM menyalin hasil ukur mentah tiap tick,
sehingga `y_goal` melompat sampai 0,147 m antar replan dan MPC mengejar acuan
yang bergerigi. Tapisnya harus berada di balik gerbang state yang SAMA dengan
latch-nya: ditapis di pemanggil, ia ikut berjalan selama manuver -- ketika
`lane_dev` mengacu ke lajur SALIP -- dan lompatannya justru naik ke 0,693 m.
Di tempat yang benar, lompatan turun ke **0,0094 m**.

**Perbaikan 2 -- kemiringan lajur diselesaikan, bukan dicari.**
`_shared_slope` memilih argmax histogram pada bin 0,10 m. Pada masker
SINTETIS -- garis lurus sempurna, kamera sempurna, tanpa derau -- galatnya tetap
0,210 deg RMS. Itu membuktikan galatnya lahir di dalam algoritma. Memperhalus
langkah pencarian tidak menolong (0,005 -> 0,0005 memberi 0,210 -> 0,261).
Yang menolong mengganti langkah terakhirnya dengan penyelesaian: regresi
dalam-kelompok terkumpul, satu kemiringan bersama dengan perpotongan bebas.
Terhadap peta HD: sudut hadap RMS **0,257 -> 0,086 deg**, simpangan ego RMS
**0,050 -> 0,018 m**.

**Perbaikan 3 -- kisi kandidat berjangkar pada lebar nominal.**
`LATERAL_OFFSETS` adalah magnitudo terhadap lajur nominal 3,5 m, tetapi
dikurangi lebar lajur HASIL UKUR. Seluruh kisi kandidat karena itu bergeser
sebesar galat ukur, dan **tidak ada satu pun kandidat yang jatuh di tengah lajur
tujuan**. Bidikan planner di 2 detik: **0,042 -> 0,011 m**. Regresi yang masuk
bersama bagian 28, saat lebar lajur berubah dari konstanta menjadi hasil ukur.

**Empat dugaan yang gugur, dan kenapa itu layak ditulis:**

| Dugaan | Bagaimana dibantah |
|---|---|
| `K_DEV` terlalu kecil | Disapu 20 -> 320, **16 kali**. Deviasi 0,087 / 0,090 / 0,089 -- tidak bergerak |
| Galat tunak pengendali | MPC melacak acuannya sampai 0,0001 m; yang meleset bidikan planner |
| Lup planner-MPC terbuka pada `dy0` | Sudah tertutup sejak lama; laju lateral rencana dan hasil ukur sepakat sampai **0,012 m/s** |
| Kuantisasi kisi pencarian sudut | Memperhalus langkah 10 kali justru MEMBURUKKAN hasilnya |

Dan satu kesalahan metodologis yang harus dicatat karena ironis: uraian galat
yang sempat saya susun dibaca dari kolom `y_ref`, yaitu acuan pada langkah
PERTAMA horizon, yang menurut konstruksi menempel di posisi ego. Angka itu hanya
mengulang galatnya sendiri. **Itu jebakan nomor 1 di `TUNING_MPC.md` bagian 9,
yang sudah ditulis di bagian 22.5 dokumen ini** -- dan tetap terjadi. Sasaran
planner yang sah adalah `y_plan_20`.

**Yang akhirnya menyelesaikannya bukan dugaan kelima**, melainkan berhenti
menebak dan menggambar bentuk galat terhadap waktu:

| t (s) | Simpangan dari tengah lajur sebenarnya |
|---|---|
| 13,75 | -0,222 m (masih di sisi lajur salip) |
| 14,50 | +0,166 m (menyeberang) |
| 15,25 | **+0,300 m** (puncak lampauan) |
| 16,75 | +0,252 m |
| 18,25 | +0,173 m |
| 19,75 | +0,153 m (masih turun saat run berakhir) |

**Melampaui lalu meluruh** -- bukan galat tunak, bukan ayunan. Jendela 20 detik
berakhir sebelum transiennya mengendap. Itu sebabnya metriknya harus dipisah per
fase, dan itu sebabnya angka gabungan menyembunyikan perbaikan 4,3 kali pada
fase yang benar-benar mengukur kualitas menjaga lajur.

### 29.6 Yang belum

1. **Perbandingan bagian 27 versus 29 tidak bersih** -- kendaraan target berganti
   bersamaan dengan perombakan perception, jadi selisihnya memuat dua sebab.
2. **Waktu solve maksimum harus diukur ulang di mesin senggang.**
3. **Depth camera tetap ideal** -- tanpa derau, tanpa lubang, tanpa batas
   jangkauan. Satu-satunya ketergantungan sensor yang tersisa, dan harus
   dinyatakan di batasan masalah.
4. **Transien kembali** memuncak 0,30 m dan butuh lebih dari 6 detik mengendap.
   Belum ditelusuri; menuntut percobaan yang dirancang untuk itu, bukan dugaan
   kelima.

---

## 30. Geometri lajur: pembuktian, dan pemilihan anotasi (30 September 2026)

Bagian 28 memakai `lanes.py` untuk membuang peta HD; bagian ini **membuktikan**
tahap-tahapnya dan **memilih** anotasi latih yang memberinya masukan terbaik.
Seluruhnya bisa dibangkitkan ulang: `check_ipm.py`, `plot_ipm.py`,
`plot_lane_fit.py`, `plot_lane_pipeline.py`.

### 30.1 Turunan IPM

Model lubang jarum. Titik di frame kamera `(Xc, Yc, Zc)` -- `Zc` ke depan
sepanjang sumbu optik, `Xc` ke kanan, `Yc` ke bawah -- jatuh di piksel

    u = c_u + f * Xc / Zc                                              (1)
    v = c_v + f * Yc / Zc                                              (2)

Panjang fokus dari sudut buka mendatar: setengah lebar sensor `W/2` memandang
sudut `fov/2`, jadi

    tan(fov / 2) = (W / 2) / f      ->      f = W / (2 tan(fov / 2))    (3)

Kamera setinggi `h` di atas jalan, menghadap lurus (pitch nol). Untuk titik **di
permukaan jalan** sejauh `x` di depan: `Zc = x`, `Yc = h`, `Xc = -y` (frame ego,
`y` positif ke kiri). Masukkan ke (2) lalu (1):

    x = f * h / (v - c_v)                                              (4)
    y = -x * (u - c_u) / f                                             (5)

(4) dan (5) itulah `lanes.ipm`. Baris `v = c_v` memberi pembagian nol: berkas
lewat titik hilang sejajar jalan dan tidak pernah memotongnya.

**Masukannya tiga, dan ketiganya dari pemasangan kamera, bukan dari simulator:**
tinggi pemasangan, intrinsik kamera, dan asumsi jalan datar. Di kendaraan
sungguhan ketiganya didapat saat memasang dan mengalibrasi.

### 30.2 Empat verifikasi numerik

`check_ipm.py`, berlapis dari yang paling ketat ke yang paling independen.

| # | Yang diuji | Hasil |
|---|---|---|
| 1 | `f` dari sudut buka, persamaan (3) | 640,000000 px, selisih **0** |
| 2 | Bolak-balik `ipm(to_pixel(x, y))` | galat **6,4 x 10⁻¹⁴ m** |
| 3 | Proyeksi (1)-(2) versus matriks kamera CARLA | selisih **0,0001 px** |
| 4 | Jarak (4) versus **kamera depth** | -0,22% sampai -1,95% |

Nomor 1 aritmetika: fov 90 deg membuat `tan 45 deg = 1` tepat, jadi `f = W/2`.
Nomor 2 batas presisi float, bukan galat model. Nomor 3 memastikan konvensi
frame saya sama dengan CARLA.

**Nomor 4 yang paling berarti**, karena kamera depth tidak berbagi satu pun
asumsi dengan IPM: ia membaca z-buffer GPU, tidak tahu tinggi kamera, tidak
memakai panjang fokus, dan tidak mengandaikan jalan datar.

| Baris `v` | `x` rumus (4) | depth ukur | galat |
|---|---|---|---|
| 580 | 4,800 m | 4,790 m | -0,22% |
| 520 | 6,600 m | 6,580 m | -0,30% |
| 470 | 9,600 m | 9,558 m | -0,44% |
| 430 | 15,086 m | 14,981 m | -0,69% |
| 400 | 26,400 m | 26,078 m | -1,22% |
| 385 | 42,240 m | 41,417 m | -1,95% |

**Uji yang lebih tajam: balik rumusnya.** Dari tiap bacaan depth hitung
`h = x * dv / f`. Kalau modelnya benar, hasilnya harus konstan.

    h efektif = 1,6368 +- 0,0101 m      (dikonfigurasi 1,6500 m)

Tetap dalam +-10 mm sepanjang 4,8 sampai 42 m. **Kalau model (4) salah** --
depth ternyata radial, misalnya, atau proyeksinya keliru -- **`h` efektif akan
melayang sistematis mengikuti jarak.** Ia tidak melayang.

### 30.3 Dua sisa yang diukur, bukan diasumsikan

**(a) Simpangan tetap 13,2 mm.** Nominal 1,650 m, efektif 1,637 m. Ego duduk di
suspensi, jadi kameranya memang sedikit lebih rendah. Bisa dikalibrasi keluar;
belum dilakukan, dan selisih 0,8% itu masuk ke seluruh jarak memanjang.

**(b) `h` efektif merayap +28,6 mm** dari 4,8 ke 42 m, menurun searah -- bukan
derau melainkan **kemiringan jalan ~0,076%**.

Poin (b) berharga karena **inilah ongkos asumsi jalan datar, dan besarnya kini
terukur: ~0,73 m galat jarak pada 42 m.** Selama ini "asumsi jalan datar" hanya
disebut sebagai batasan tanpa angka.

**Catatan tambahan yang ikut ketahuan:** tiga baris citra paling bawah meleset
-62%, dan itu bukan cacat. Kamera dipasang 1,68 m di depan sumbu belakang
sementara bodinya 5,01 m, jadi **moncong ego menutupi jalan lebih dekat dari
~4,8 m**. Yang terbaca kap mesin sendiri. `check_ipm.py` mengecualikannya dan
mencetak alasannya.

### 30.4 Kemiringan bersama adalah sebuah SHEAR

`c = y - b*x` bukan rumus abstrak: ia **pergeseran miring** pada bidang jalan.
Di jalan lurus semua marka sejajar, jadi ADA satu `b` yang menegakkan semuanya
sekaligus. Setelah digeser dengan `b` yang benar, tiap garis lajur jatuh pada
satu nilai `c`, dan meruntuhkan gambar ke sumbu mendatar memberi histogram yang
memuncak tajam.

Terukur pada frame ruas lurus (`plot_lane_fit.py`):

| | Shear salah (`b = -0,10`) | Shear tercocok (`b = +0,0008`) |
|---|---|---|
| Ketajaman histogram | 0,61 x 10⁵ | **1,53 x 10⁵** |
| Bentuk puncak | gundukan melebar | paku |

**2,5 kali lebih tajam.** Itu yang dicari `_shared_slope`: sapu calon `b`,
ambil yang paling memuncak. Satu parameter dicari dari 1.840 titik sekaligus --
bukan satu garis dicocokkan dari serpihan marka.

Langkah terakhirnya diganti penyelesaian kuadrat terkecil, bukan argmax; alasan
dan angkanya di bagian 28.1 (RMS 0,257 -> 0,086 deg).

### 30.5 Pencocokan kisi, dan kenapa median gagal

Puncak-puncak itu tidak sembarang letaknya: marka lajur **berjarak sama**. Jadi
yang dicari cukup satu jarak `w` dan satu fase.

Diperlihatkan dengan data frame ruas lurus:

```
Tiga garis terdeteksi:  c = -5,35 | -1,78 | +1,79
Jarak antar tetangga:      3,58      3,57      ->  lebar lajur 3,570 m   BENAR

Hapus marka tengah (tertutup kendaraan / tidak terdeteksi):
Dua garis tersisa:      c = -5,35 | +1,79
Jarak antar tetangga:      7,14                 <- DUA lebar lajur
  cara naif (median)  ->  lebar lajur 7,140 m   SALAH
  cara kisi           ->  lebar lajur 3,570 m   BENAR
```

Kisi tahu 7,14 = 2 x 3,57 -- **kelipatan bulat**, jadi celah itu terbaca sebagai
**lubang**, bukan sebagai lajur yang melebar. Analoginya buku tulis bergaris:
kalau satu garis pudar, Anda tetap tahu di mana ia seharusnya.

Itulah yang menjelaskan perbaikan 21 kali di bagian 28.1.

**Batasnya juga terukur, dan harus ditulis.** Satu frame dengan ego menyudut 6 deg
memberi puncak di `c` = -10,97 / -4,73 / +1,58, berjarak 6,23 dan 6,31 m -- dua
lebar lajur, karena marka tengahnya tidak terdeteksi dan yang terjauh kemungkinan
pagar pengaman. Kisi membaca celah itu sebagai dua langkah (benar), tetapi dengan
hanya tiga sisa yang jaraknya tidak teratur ia mendarat di **3,140 m terhadap
3,50 m sebenarnya**.

> **Pencocokan kisi bertahan terhadap SATU marka yang hilang. Ia tidak bertahan
> kalau sebagian besarnya hilang.**

### 30.6 Penyaringan yang bekerja

Pada frame ruas lurus, dari **1.840 titik** hasil IPM:

| | |
|---|---|
| Diangkat jadi garis lajur | **3 puncak**, bobot 569 / 289 / 199 piksel |
| **Ditolak** | **768 piksel** (42%), tersebar di `c` = -14 sampai +27 m |
| Ambang | 25% dari puncak tertinggi |

Yang ditolak: pagar pengaman, bahu jalan, serpihan tepi. Titik hijau yang tidak
menjadi garis merah **bukan kegagalan** -- itu penyaringan, dan porsinya hampir
separuh menunjukkan penyaringan itu mengerjakan sesuatu yang berat.

### 30.7 Anotasi marka versus menerus -- DIPILIH `marking`

Penulis melatih dua checkpoint dari data yang sama, berbeda anotasi lajurnya:
`yolopx-marking.pt` menandai tiap penggal marka, `yolopx-continuous.pt` menandai
batas lajur menerus. Keduanya diuji pada sapuan yang sama.

**Ketelitian geometri lajur (`check_lanes.py`, 320 frame):**

| Terhadap peta HD | **marking** | continuous |
|---|---|---|
| Lebar lajur, bias | **-0,050 m** | -0,166 m |
| Lebar lajur, RMS | **0,076 m** | 0,181 m |
| Simpangan ego, RMS | **0,020 m** | 0,039 m |
| Sudut hadap, RMS | **0,100 deg** | 0,195 deg |
| Garis terdeteksi (modus) | 4 (2-5) | **5 (4-7)** |
| Piksel masker per frame | 1.731 | 6.314 |

**Marking menang dua kali lipat di semua metrik -- dengan piksel 3,6 kali lebih
sedikit.** Baris terakhir menjelaskan sebabnya: `continuous` mendeteksi lebih
banyak garis daripada yang ada di jalan.

**Sebabnya terlihat langsung.** Pada satu frame, masker `continuous` menarik
garis lajur **di atas rel kereta** di bahu kanan -- bantalan rel berjarak teratur
dan bergaris, jadi bagi jaringan yang dilatih melihat "garis menerus" ia tampak
persis seperti marka. Garis palsu itu lalu mencemari kisi:

| Satu frame yang sama | marking | continuous |
|---|---|---|
| Garis terdeteksi | 3 | 5 |
| Lebar lajur | 3,570 m | 3,440 m |
| **Sisa kisi** | **0,003 m** | **0,166 m** |

Sisa kisi **55 kali lebih besar**.

**Loop tertutup, satu run masing-masing:**

| | marking | continuous |
|---|---|---|
| Vonis | BERHASIL | BERHASIL |
| Lebar lajur ditaksir | **3,530 m** (+0,030) | 3,370 m (-0,130) |
| Simpangan dari lajur SEBENARNYA, fase menjaga lajur | **0,0101 m** | 0,0685 m |

**6,8 kali lebih baik.** Galat lebar lajur 0,13 m itu bukan kosmetik: `y_goal`
saat menyalip dihitung sebagai tengah lajur ditambah lebar lajur, jadi sasarannya
ikut meleset 0,13 m.

**Yang harus disebut adil:** `continuous` lebih baik di DETEKSI -- jangkauan
46,5 m versus 42,0 m, dan 119 versus 105 tick terdeteksi dari 140. Tetapi itu
tidak mengubah perilaku: pemicu menyalip bekerja di celah ~32 m dan horizon MPC
27 m, keduanya jauh di dalam 42 m. Galat geometri lajur sebaliknya masuk
**langsung** ke kendali.

**Kesimpulan yang agak berlawanan dengan intuisi, dan itu justru nilainya:**

> Anotasi yang lebih "lengkap" menghasilkan pengukuran yang lebih buruk, karena
> ia mengajari jaringan menebak di tempat yang tidak ada buktinya -- dan tebakan
> itu mendarat di rel kereta. Kelebihan piksel bukan informasi tambahan
> melainkan **kebisingan berstruktur**, jenis yang paling merusak karena tampak
> seperti garis sungguhan.

Ini sekaligus menutup godaan melatih ulang dengan label menerus yang dibahas di
bagian 28.4: pilihan itu bukan hanya lemah secara metodologis, ia **terukur lebih
buruk**.

### 30.8 Gambar yang tersedia

| Berkas | Menjelaskan |
|---|---|
| `out/ipm_explained.png` | kisi meter di citra, horizon, kurva meter-per-piksel |
| `out/ipm_birdseye.png` | warp IPM penuh -- bentuk yang lazim ditampilkan |
| `out/lane_fit_explained.png` | shear salah versus tercocok, plus kisi |
| `out/lane_pipeline.png` | tiga tahap, dua model anotasi berdampingan |
| `out/lane_annotation_comparison.png` | dua model pada satu frame, dengan sisa kisi |
| `out/town04_test_section.png` | letak ruas uji di peta Town04 |

**Panel ketiga `ipm_explained.png` layak dikutip sendiri.** Ia memuat kurva
meter-per-piksel: 0,09 m/px di 10 m, **1,84 m/px di 44 m**. Garis putus
vertikalnya adalah 44,0 m, yaitu jarak deteksi pertama yang diukur
`check_estimation.py`.

Selama ini batas 44 m dijelaskan sebagai keterbatasan detektor (data latih kurang
kendaraan jauh, bagian 18). Kurva itu menunjukkan **geometrinya juga habis di
tempat yang sama**: pada 44 m satu piksel sudah bernilai 1,84 m, jadi deteksi
yang sempurna pun jaraknya tidak akan teliti. Dua sebab yang kebetulan bertemu di
angka yang sama -- dan itu lebih jujur daripada menyalahkan detektornya saja.

---

## 31. Checkpoint baru dan drivable area (2 Oktober 2026)

Dua perubahan di hari yang sama, dicatat terpisah supaya sebab tiap selisih
angka jelas: **checkpoint YOLOPX diganti** (31.1-31.2) dan **drivable area masuk
jalur kendali** (31.3-31.4). Perbandingan 31.4 memakai checkpoint yang SAMA di
kedua sisi, jadi selisih bagian 29 -> 31.2 adalah efek checkpoint, bukan efek
drivable area.

Seluruh angka bisa dibangkitkan ulang: `experiment.py`, `metrics.py`,
`check_detection.py`, `check_estimation.py`, `check_lanes.py`. Seluruh gambar
di `out/` yang punya skrip pembuat sudah dibangkitkan ulang dengan keadaan ini.

### 31.1 Checkpoint YOLOPX epoch 92

`weights/best.pth` diganti penulis: epoch 263 -> **epoch 92**. Angka bagian 18
dan 29 memakai checkpoint lama.

**Ketelitian perception, terhadap bagian 29.4** (target sama, Lincoln MKZ 2020):

| Besaran | epoch 263 (bagian 29.4) | **epoch 92** |
|---|---|---|
| x memanjang, bias / RMS | +0,321 / 0,323 m | +0,313 / 0,317 m |
| y melintang, RMS | 0,018 m | 0,014 m |
| vx, RMS | 0,021 m/s | 0,038 m/s |
| Deteksi pertama (sapuan 53 -> 9 m) | 44,0 m (111/140 tick) | 41,7 m (104/140 tick) |
| Lebar lajur, bias / RMS | -0,053 / 0,076 m | -0,049 / 0,076 m |
| Simpangan ego, RMS | 0,018 m | 0,020 m |
| Sudut hadap, RMS | 0,086 deg | 0,091 deg |
| Dimensi: tinggi | 1,507 m (+1,1%) | 1,529 m (+2,6%) |
| Dimensi: lebar | 1,642 m (-10,6%) | 1,437 m (-21,8%) |
| Dimensi: panjang | 4,046 m (-17,3%) | 4,233 m (-13,5%) |

**Deteksi terhadap jarak** (`check_detection.py`, MKZ). Tabel bagian 18.2
diukur dengan Nissan Patrol, jadi TIDAK bisa dibandingkan langsung:

| Jarak | Lajur ego, conf | Lajur menyalip, conf |
|---|---|---|
| 10-30 m | 0,98-0,99 | 0,98-0,99 |
| 40 m | **0,70** | **0,68** |
| 50-80 m | tidak terdeteksi | tidak terdeteksi |

Positif palsu nol di semua ambang 0,3-0,7. Jangkauan 40 m sama dengan
sebelumnya, tetapi keyakinan pada 40 m tinggal ~0,7: masih di atas
`DETECTION_CONF = 0,5`, marginnya lebih tipis.

**Lebar kendaraan memburuk (-21,8%)**, tetapi tidak menyentuh keselamatan: zona
aman memakai kendaraan desain, bukan taksiran (bagian 28.2). Taksiran dimensi
hanya masuk `face_correction`, dan bias jarak memanjang justru sedikit membaik.

**Segmentasi berubah watak.** Area jalan kini mencakup **bahu jalan kiri sampai
pembatas beton** dan ikut menandai garis tepi jalan. Checkpoint lama berhenti di
garis tepi dan menarik "garis lajur" di atas pembatas. Gambar:
`out/detection_30m_lane0.png`.

### 31.2 Tahap 9 dengan checkpoint baru

| Metrik S1 | MPC + vision, epoch 263 (bagian 29) | **MPC + vision, epoch 92** | MPC + GT |
|---|---|---|---|
| Vonis | 10/10 | **10/10** | 5/5 |
| Jarak min antar bodi | 1,797 ± 0,017 m | **1,858 ± 0,017 m** | 1,432 m |
| Durasi manuver | 12,12 ± 0,05 s | 12,26 ± 0,08 s | 11,65 s |
| Deviasi lajur, SEBELUM manuver | 0,0176 ± 0,0016 m | **0,0111 ± 0,0005 m** | 0,0000 m |
| Deviasi lajur, ekor SESUDAH | 0,1303 ± 0,0012 m | 0,1274 ± 0,0010 m | 0,0286 m |
| Perlambatan terdalam | -2,75 ± 0,26 m/s² | -2,87 ± 0,18 m/s² | -0,14 m/s² |
| Tick tanpa kandidat planner | 36,6 ± 0,9 | 37,6 ± 0,8 | 4 |
| Galat prediksi @ 0,5 s, RMS | 0,0394 m | 0,0361 m | 0,0203 m |
| XTE ke lajur terdekat, RMS | 0,570 m | 0,581 m | 0,535 m |
| Kegagalan solver | 0 | 0 | 0 |

Jalur GT tidak bergantung checkpoint dan tetap identik bit-per-bit. S3 GT kini
juga diukur dengan MKZ: **BERHASIL, 1,52 m, 19,1 s**.

### 31.3 Drivable area masuk jalur kendali

Sampai bagian 30 masker area jalan (`da`) hanya dipakai overlay video. Kini ia
dipakai di tiga tahap, seluruhnya di `lanes.py` (numpy murni, teruji tanpa
simulator), dan seluruhnya `None` di jalur GT:

1. **Garis lajur disaring.** `from_mask(ll, ..., drivable=da)` hanya memakai
   piksel `ll ∩ da`. Garis di atas pembatas, rel, atau jalur seberang dibuang
   sebelum kisi dicocokkan.
2. **Lajur tujuan harus sah.** Dua syarat, keduanya wajib sebelum FSM masuk
   `LANE_CHANGE_OVERTAKE`:
   - *ada* -- `lane_marked`: diapit marka di KEDUA sisi, >= `MIN_PIXELS` (40)
     piksel per garis;
   - *lapang* -- `drivable_fraction`: petak lajur tujuan (x 6-30 m, setengah
     lebar lajur) >= `DRIVABLE_MIN` (0,9) bertanda area jalan.
   Sebelumnya cukup "daftar deteksi kosong", yang tidak bisa membedakan "kosong"
   dari "tidak terlihat".
3. **Kandidat planner dibatasi tepi jalan.** `road_edges` membaca tepi area jalan
   per baris jarak; planner membuang kandidat yang bodinya -- termasuk sapuan
   pojok saat berbelok, `L/2·|sin psi| + W/2·cos psi` -- keluar dari tepi itu.

**Pengukuran yang menentukan rancangannya** (520 tick S1, kedua checkpoint):

| Besaran | epoch 263 | epoch 92 |
|---|---|---|
| Piksel garis tepi KIRI di luar area jalan | 26% | **0%** |
| Piksel garis tepi KANAN di luar area jalan | 13% | **0%** |
| Piksel garis antar-lajur di luar area jalan | 0% | 0% |
| Piksel garis "di luar jalan" di luar area jalan | 100% | 90% |
| Porsi drivable lajur salip (kanan) | 1,00 | 1,00 |
| Porsi drivable sisi KIRI (bahu, tanpa lajur) | **0,00** | **1,00** |
| Porsi drivable lajur yang berisi kendaraan | 0,38 | 0,40 |

Tiga keputusan lahir dari tabel itu:

- **Tanpa toleransi piksel.** Dengan checkpoint lama garis tepi tergerus karena
  area jalan berhenti tepat di marka tepi, sehingga sempat dipakai toleransi 4
  piksel. Dengan epoch 92 garis tepi ikut area jalan; irisan mentah hanya
  membuang 0,3% piksel (semuanya garis di luar jalan) dan menggeser lebar lajur
  paling banyak 0,01 m.
- **"Ada" dipisah dari "lapang".** Epoch 92 menandai bahu jalan sebagai aspal --
  secara fisik benar -- sehingga porsi drivable saja tidak lagi bisa membedakan
  lajur dari bahu. Marka bisa: garis luar lajur salip terukur **>= 109 piksel di
  setiap frame**, bahu jalan **0**. Dihitung dari piksel, bukan dari puncak kisi:
  garis luar itu putus-putus dan hanya menjadi puncak di **54%** frame.
- **Tepi jalan persentil 90, bukan median.** Kendaraan di sebelah ego membolongi
  area jalan di baris yang ditutupinya; median ikut melaporkan sisi kendaraan
  sebagai tepi jalan (persentil 1: **0,54 m**). Persentil 90 ke luar: persentil
  1 **2,72 m**, median 5,09 m, yaitu pembatas beton.

**Biaya:** ~1 ms per tick perception.

### 31.4 Verifikasi

- **133 uji** lolos tanpa simulator, enam di antaranya untuk drivable area. Uji
  tepi jalan sengaja memakai kendaraan yang menutup dua pertiga baris: median
  gagal di sana (-1,00 m), persentil 90 lolos (+1,79 m terhadap 1,75 m).
- **Jalur GT S1 dan S3 identik bit-per-bit** dengan kode sebelum drivable area.
- **Jalur vision, checkpoint SAMA (epoch 92), 10 ulangan per sisi:**

| | Tanpa drivable area | **Dengan drivable area** |
|---|---|---|
| Vonis | 10/10 | 10/10 |
| Jarak min antar bodi | 1,855 ± 0,019 m | 1,849 ± 0,021 m |
| Durasi manuver | 12,26 ± 0,08 s | 12,24 ± 0,09 s |
| Deviasi lajur, SEBELUM manuver | 0,012 ± 0,001 m | 0,012 ± 0,000 m |
| Tick tanpa kandidat planner | 37,6 ± 0,8 | 37,4 ± 0,9 |
| Kegagalan solver | 0 | 0 |

Setara dalam derau. Di S1 gerbang lajur dan saringan tepi jalan dihitung tiap
tick tetapi **tidak pernah menolak**: nilai gerbang sebelum manuver selalu 1,00
dan tepi jalan selalu jauh dari kandidat. Itu benar untuk S1 -- lajur kanan
memang ada dan kosong -- dan harus ditulis apa adanya.

### 31.5 Catatan

- Sebuah klaim saya sendiri yang keliru, dikoreksi: dengan checkpoint lama saya
  menyebut irisan tanpa toleransi membuat `lane_dev` "melompat 3,5 m". Lompatan
  itu terjadi di SATU tick, saat ego tepat di atas garis ketika pindah lajur --
  +1,77 dan -1,76 m menunjuk posisi yang sama dari lajur terdekat yang berganti.
  Bukan galat. Alasan sebenarnya untuk toleransi waktu itu adalah bias kecil di
  garis tepi (lebar lajur 0,10 m).
- Enam berkas di `out/` tidak punya skrip pembuat dan tidak dibangkitkan ulang:
  `lane_annotation_comparison.png`, `town04_test_section.png`,
  `why_q_psi_450.png`, `ttc_threshold.png`, `sensor_rgb_yaw.png`,
  `model_validation_18kmh.*`. Tidak satu pun bergantung pada checkpoint atau
  drivable area.
- Berkas `*_before.*` sengaja tidak dibangkitkan ulang: isinya keadaan SEBELUM
  perbaikan bagian 27.
