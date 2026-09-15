# Rangkuman untuk Penulisan Skripsi

Dokumen ini dibuat untuk ditempel ke percakapan baru saat mulai menulis. Isinya
seluruh angka, sitasi, dan keputusan yang sudah terverifikasi sampai Tahap 4.
Catatan kerja lengkap ada di `CATATAN.md` (1.088 baris); yang ini ringkasannya.

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
| 8 | Perception lengkap (perbaikan leakage) | belum |
| 9 | Eksperimen penuh | belum |

Kode: 3.238 baris, 65 uji otomatis semuanya lolos tanpa perlu menyalakan CARLA.
Terakhir diperbarui 12 September 2026.

**Yang boleh ditulis sekarang:** seluruh bab 3 (metodologi), dan bab 4 untuk
hasil kendali satu run per skenario S1 dan S3 (bagian 14), tuning bobot
(`TUNING_MPC.md` bagian 10-11), serta temuan metodologis (bagian 15).

**Belum boleh ditulis:** success rate (butuh 10-20 ulangan per skenario,
Tahap 9), skenario S2/S4/S5, dan apa pun tentang perception berbasis vision.

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
| Zona aman `A`, `B` | 7,709 m, 3,204 m | diturunkan dari dimensi + `JARAK_AMAN` |
| Geser sumbu belakang -> pusat bodi | 1,433 m | zona diukur antar pusat bodi |
| Jarak ikut `WAKTU_IKUT` | 2,0 s | jarak waktu-tetap, bagian 14.4 |
| Laju menjauh maks untuk kembali | 0,1 m/s | gerbang `DD_KEMBALI` |
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
| `ambang_ttc.png` | Kenapa ambang memakai waktu bukan jarak | Bab 3 atau bab 4 — pembahasan FSM |
| `kenapa_450_derajat.png` | Ilustrasi lompatan sudut 2π | Bab 4 — pembahasan temuan |
| `overtake_topdown.mp4` | Video manuver tampak atas dengan overlay kandidat | Sidang |
| `overtake_planner.mp4` | Video manuver kamera kejar dengan overlay kandidat | Sidang |
| `reference_path.mp4` | Video jalur acuan global planner 300 m | Sidang |

**Label video wajib:** semua video saat ini adalah **playback lintasan planner**,
bukan hasil kendali MPC. Physics dimatikan dan posisi ego ditempelkan ke lintasan.
Beri label "lintasan hasil local planner". Video hasil kendali sudah bisa dibuat
(Tahap 5 selesai), tapi `record_maneuver.py` belum diubah.

**Gambar yang perlu dibangkitkan ulang sebelum masuk skripsi:** `run_s1_mpc.png`
dan seluruh grafik hasil kendali dibuat sebelum perbaikan bagian 15, jadi angkanya
tidak lagi cocok dengan bagian 14. Jalankan ulang `main.py` untuk S1 dan S3, lalu
buat grafiknya dari `out/run_s1_mpc_gt.npz` dan `out/run_s3_mpc_gt.npz`.

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
- **Zona aman dan penilai sama-sama memakai kotak sejajar sumbu.** Pada sudut
  hadap 7° saat pindah lajur, sudut bodi bergeser ~0,31 m yang tidak dihitung.
  Keduanya konsisten satu sama lain, jadi penilaian tidak bias, tapi jarak bodi
  sebenarnya bisa lebih kecil daripada yang dilaporkan. Masuk batasan masalah.
- **Dimensi kendaraan lain dianggap tetap** (Nissan Patrol, yang terbesar di
  skenario); perception tidak mengukur dimensi. Masuk batasan masalah.
- Skenario S2, S4, S5 belum ada. Definisi S3 dibuat tanpa naskah bagian 11.3
  rencana kerja, jadi cocokkan dulu sebelum ditulis.
- `D_SAFE_DEPAN`, `D_SAFE_BELAKANG`, `PASS_MARGIN` masih memakai nilai proposal
  apa adanya, belum dituning.


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

Diukur 12 September 2026, CARLA 0.9.16 kualitas Low, satu run per skenario.
Simulasinya deterministik: dua run berkonfigurasi sama memberi log **identik
bit-per-bit**, jadi satu run menggambarkan konfigurasi itu (ulangan tetap
dibutuhkan di Tahap 9 untuk success rate, bukan untuk ketepatan satu run).

### 14.1 Hasil per skenario

| | S1 (flying overtaking) | S3 (accelerative overtaking) |
|---|---|---|
| Susunan | target 7,0 m/s, 60 m di depan, lajur kanan kosong | idem + kendaraan 13,9 m/s di lajur kanan, mulai 10 m di belakang ego |
| Vonis bagian 11.2 | **BERHASIL** | **BERHASIL** |
| Jarak minimum antar bodi | 1,39 m | 1,49 m |
| Deviasi dari tengah lajur saat `LANE_KEEPING` | 0,011 m rata-rata, 0,152 m maks | 0,012 m rata-rata, 0,131 m maks |
| Durasi manuver | 11,7 s | 19,2 s |
| Kecepatan terendah | 47,8 km/jam | 18,9 km/jam (saat mengikuti; acuan `v_goal` turun sampai 14,5) |
| Perlambatan terdalam | −0,15 m/s² | −1,91 m/s² |
| Waktu solve MPC | 12,9 ms rata-rata, 22,7 ms maks | 11,8 ms rata-rata, 18,2 ms maks |
| Solver gagal | 0 dari 400 | 0 dari 500 |
| Urutan state FSM | lengkap | lengkap |

Anggaran waktu satu tick 50 ms; solve terukur di mesin senggang (lihat catatan
waktu solve di `TUNING_MPC.md` bagian 10).

### 14.2 Bobot dan gain final

| Parameter | Proposal 7.3 | Final | Dasar |
|---|---|---|---|
| `Q` = (X, Y, psi, v) | (1; 20; 10; 2) | **(1; 20; 450; 2)** | `Q[psi]` = titik redaman kritis empiris |
| `Qf` | 5·Q | 5·Q | tidak disentuh |
| `R` = (a, delta) | (0,1; 1,0) | (0,1; 1,0) | tidak disentuh |
| `Rd` = (a, delta) | (1,0; 20,0) | (1,0; 20,0) | disapu, terbukti optimal |
| `rho` | 1000 | 1000 | tidak disentuh |
| `kp` throttle | 0,08 | **0,14** | = 1/gain plant terukur |
| `ki` throttle | 0,25 | 0,25 | tengah geometrik plateau |

Alasan tiap angka, deret sapuan, dan tafsiran fisiknya ada di `TUNING_MPC.md`
bagian 11 — itu bahan langsung untuk sub-bab tuning di bab 3.

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
`d* = A + 1,433 + 2,0·v_depan` dengan kecepatan acuan `v_depan + 2e/T`,
`e = celah − d*`, `T` = durasi manuver terpanjang. Begitu lajur tujuan aman,
ego menyalip dari posisi mengikuti — *accelerative overtaking* menurut
klasifikasi Fabricius dkk. (2022), berbeda dari S1 yang *flying*.

Pemicu memakai `max(v_ego, V_REF)`, bukan `v_ego`: alasan menyalip adalah
kendaraan depan lebih lambat daripada kecepatan yang **diinginkan**. Tanpa itu,
ego yang sudah melambat mengikuti punya TTC tak hingga dan tidak akan pernah
mencoba menyalip lagi.

---

## 15. Temuan metodologis (12 September 2026)

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
buku teks kendali klasik. Nilai `WAKTU_IKUT` karena itu bersandar pada sapuan
eksperimen sendiri, bukan pada standar.

**Perlu keputusan penulis:** apakah aturan 4 tahun berlaku juga untuk sumber
asal konsep di bab 2 — Werling dkk. (2010), Hayward (1972), Flash & Hogan (1985).
Mengganti ketiganya akan melemahkan landasan teori, karena justru merekalah
sumber pertamanya.
