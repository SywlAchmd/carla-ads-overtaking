# Catatan Kerja — Skripsi MPC Overtaking

Ringkasan per tahap: masalah apa yang muncul, kenapa itu masalah, dan bagaimana
diselesaikan. Ditulis untuk dibaca ulang saat menyusun bab metodologi dan saat
menyiapkan sidang.

Lingkungan: CARLA 0.9.16, Ubuntu, Python 3.10. Kode di folder ini.
Rencana kerja lengkap: `rencana_skripsi_mpc_overtaking.md`.

---

## Tahap 1 — Parameter Kendaraan & Validasi Model

**Status: SELESAI** — 7 September 2026
**Rujukan rencana kerja:** bagian 3.1, 3.2, 3.3 (plus verifikasi 0.3)

### Konsep: ini validasi terhadap apa?

Dua hal yang dibandingkan:

- **Plant** — mesin fisika CARLA. Simulasi penuh: slip ban, suspensi, perpindahan
  beban, mesin, transmisi, hambatan udara. Kotak hitam, tidak ada persamaannya
  di mana pun.
- **Prediction model** — kinematic bicycle model. Empat baris matematika yang
  nanti hidup di dalam solver MPC. Tiap tick MPC bertanya ke rumus ini: "kalau
  saya kemudikan sekian selama 2 detik ke depan, mobil sampai di mana?"

Validasinya mengukur seberapa jauh keduanya berpisah kalau diberi input identik:

1. Mobil CARLA dijalankan dengan steer dan throttle tetap, lintasannya dicatat.
2. Kecepatan dan sudut roda yang benar-benar terjadi disuapkan ke rumus bicycle
   model, diintegrasikan maju dari titik awal yang sama.
3. Dua lintasan ditumpuk dan selisihnya diukur.

Rumusnya tidak pernah diberi tahu ke mana mobilnya pergi — dia hanya dapat input
dan titik awal, lalu meramal jalurnya sendiri. Kalau `L` salah, radius belokan
ramalannya beda. Kalau titik referensi salah, lintasannya bergeser tetap.

**Yang sengaja TIDAK divalidasi** (jangan salah klaim di skripsi):

- Bukan terhadap Dodge Charger sungguhan. Yang dikendalikan mobil simulasi.
- Bukan peta throttle -> akselerasi. Kecepatan diambil dari hasil ukur. Tahap 5.
- Bukan pemetaan `steer` -> sudut roda. Sudut roda dibaca langsung dari CARLA.

Ketiganya dilepas supaya yang tersisa untuk diuji hanya **geometrinya**: `L`,
titik sumbu belakang, dan tanda konversi koordinat. Kalau dicampur, saat error
muncul kamu tidak tahu yang salah rumusnya atau peta throttle-nya.

**Catatan konseptual untuk skripsi:** plant dan prediction model memang sengaja
berbeda. Selisihnya (model mismatch) dikoreksi oleh feedback receding horizon.

### Hasil yang terkunci

| Nilai | Angka | Sumber |
|---|---|---|
| Wheelbase `L` | 3.044 m | terukur dari posisi roda |
| Offset sumbu belakang | −1.433 m | terukur, frame kendaraan |
| `delta_max` fisik | 1.222 rad (70.0°) | `max_steer_angle` roda depan |
| `delta_max` dipakai MPC | 0.5 rad | batas konservatif |
| Panjang × lebar | 5.008 × 1.882 m | bounding box |
| Massa | 1920 kg | `physics_control` |
| Drag coefficient | 0.30 | `physics_control` |
| Satuan sumbu-x `steering_curve` | **km/jam** | empiris |
| `SIDE_SIGN` | **−1** (kanan = y negatif) | empiris, konsisten 3 run |
| `LANE_WIDTH` | 3.50 m | terukur dari Town04 |
| `SPAWN_IDX` | 75 | lajur paling kiri, 3 lajur di kanan |

### Hasil validasi — dua titik operasi

| | Run kecepatan rendah | Run kecepatan operasi |
|---|---|---|
| Rentang kecepatan | 7–18 km/jam | 34–50 km/jam |
| `a_lat` maksimum | ~0.3 m/s² | 2.05 m/s² |
| Error posisi @ 2 s | 0.052 m | **0.309 m** |
| Error posisi akhir | 0.132 m @ 5 s | 0.538 m @ 3 s |
| File | `out/model_validation_18kmh.*` | `out/model_validation.*` |

Mismatch naik ~6× dari 18 ke 50 km/jam. Ini bukan kegagalan — ini bukti
kuantitatif untuk klaim bagian 0.2 ("kinematic bicycle paling akurat di
kecepatan rendah").

**Angka paling penting: 0.309 m.** Horizon MPC = `N=20 × dt=0.1` = 2 detik, jadi
itulah sejauh mana model dipercaya sebelum feedback mengoreksi — dan koreksinya
datang tiap 50 ms, 40× lebih sering. Ini jawaban untuk pertanyaan sidang
"kenapa model sesederhana ini cukup".

Kedua run beda durasi (5 s vs 3 s), jadi hanya baris "@ 2 s" yang setara. Untuk
tabel skripsi yang bersih, jalankan ulang yang kecepatan rendah dengan
`VALIDATION_DURATION = 3.0` supaya identik kecuali kecepatan.

---

### Masalah yang muncul & solusinya

#### 1. CARLA tidak mengekspos wheelbase

**Solusi:** hitung dari posisi roda. `physics_control.wheels[i].position` ada
dalam **sentimeter, koordinat dunia** (bukan meter, bukan frame lokal — mudah
terlewat). Rata-rata roda depan dan belakang, lalu jaraknya.

#### 2. Titik referensi: origin actor bukan sumbu belakang

**Kenapa masalah:** `get_transform()` mengembalikan origin actor, sedangkan
bicycle model direferensikan ke sumbu belakang. Kalau tidak dikonversi, XTE
punya bias tetap yang tidak hilang berapa pun bobot `Q` dinaikkan — dan
gejalanya menyesatkan, terlihat seperti tuning MPC yang buruk.

**Solusi:** offset diukur langsung, bukan diasumsikan `L/2`. Posisi roda
ditransformasi ke frame kendaraan pakai `get_inverse_matrix()`.

**Angkanya:** offset asli −1.433 m, `L/2` = 1.522 m. Jadi memakai `L/2` meleset
8.9 cm; melewatkan konversi sama sekali meleset **1.43 m**. Rencana kerja
menyebut "bias ~1.3 m" — itu terkonfirmasi, dan itu bias kalau konversinya
dilewatkan, bukan kalau pakai `L/2`.

#### 3. Satuan `steering_curve` tidak terdokumentasi

**Kenapa masalah:** kurva ini menskalakan sudut kemudi menurut kecepatan. Salah
tafsir satuan sumbu-x = salah menghitung `delta` dari perintah `steer` di
Tahap 5.

**Solusi:** dibandingkan dengan data run validasi. Dengan `steer_cmd = 0.05`:

| Tafsiran | MAE terhadap sudut roda terukur |
|---|---|
| km/jam | 0.00083 rad |
| m/s | 0.00387 rad (4.7× lebih buruk) |

**Kesimpulan: km/jam.** Sanity check tambahan — titik terakhir kurva ada di 120,
dan 120 m/s = 432 km/jam, mustahil untuk mobil jalan raya.

#### 4. Steer 0.20 di luar amplop model  *(bug saya sendiri)*

**Gejala:** nilai awal `VALIDATION_STEER = 0.20` yang saya tulis ternyata
menghasilkan radius putar **14 m** dan `a_lat` **7 m/s²**.

**Kenapa masalah:** itu manuver parkiran, ban sudah slip berat. Validasi dengan
angka itu bukan menguji `L`, tapi menguji slip ban — padahal premis bagian 0.2
justru "efek slip minimal".

**Solusi:** dihitung mundur dari titik desain. `a_lat` target ~2.2 m/s² pada
50 km/jam memberi `steer ≈ 0.033` (R ≈ 90 m).

#### 5. Validasi jalan di 18 km/jam, bukan 50

**Gejala:** run pertama sukses (error 0.132 m) tapi kecepatannya cuma 7–18
km/jam, dan di akhir run masih naik +0.29 m/s². Throttle 0.4 terlalu kecil untuk
mobil 1920 kg.

**Kenapa masalah:** yang tervalidasi jadi "model akurat di bawah 18 km/jam",
padahal skripsi jalan di 40–50 km/jam. Slip tumbuh dengan `v²`. Penguji hampir
pasti menanyakan ini.

**Solusi:** `set_target_velocity()` sesaat setelah spawn — mulai langsung di
kecepatan operasi, tanpa menunggu akselerasi. Bonus: deterministik antar-run,
sesuai anjuran bagian 11.1 untuk setup skenario.

#### 6. Throttle tidak menahan kecepatan

**Gejala:** setelah `set_target_velocity(13.9)`, throttle 0.4 justru membuat
mobil melambat 43 → 19 km/jam (−2.3 m/s²). Dinaikkan ke 0.75 malah tembus
38 → 68 km/jam dengan `a_lat` 3.67 m/s².

**Solusi:** interpolasi linear dari dua titik itu → **0.56**. Hasilnya 34–50
km/jam, `a_lat` 2.05 m/s². Pas di titik desain.

Ini sebenarnya dua titik pertama dari kalibrasi throttle map (bagian 7.5).
Simpan angkanya, nanti dipakai di Tahap 5.

#### 7. Diagnosa otomatis salah menuduh

**Gejala:** saat run throttle 0.75 error mencapai 1.007 m, skrip mencetak
"PERIKSA: tersangka `rear_axle_offset_x` salah tanda, `L` salah, konversi yaw".
Padahal ketiganya sudah gugur di run kecepatan rendah — penyebab aslinya slip
ban di `a_lat` 3.67 m/s².

**Kenapa masalah:** pesan diagnosa yang salah lebih berbahaya daripada tidak ada
pesan sama sekali; bisa menghabiskan berjam-jam mencari bug yang tidak ada.

**Solusi:** pesannya dibuat sadar-konteks. `a_lat` maksimum sekarang ikut
dicetak, dan kalau error besar **disertai** `a_lat > 3 m/s²`, yang muncul catatan
slip, bukan tuduhan bug.

#### 8. Risiko keluar aspal tak terdeteksi

**Kenapa masalah:** 5 detik kemudi tetap menghasilkan pergeseran lateral besar.
Kalau mobil keluar ke rumput, gesekan berubah dan perbandingan plant-vs-model
jadi sampah — tapi tidak ada tandanya di angka mana pun.

**Solusi:** tiap tick dicek `get_waypoint(..., project_to_road=False)`. Kalau
mobil sempat keluar area drivable, muncul peringatan berapa detik. Lebih baik
mendeteksi kegagalan daripada menebak parameter aman.

#### 9. Arah menyalip belum terverifikasi

**Kenapa masalah:** rencana kerja mewanti-wanti `SIDE_SIGN` sebagai sumber bug
klasik, dan gejalanya (mobil menyalip ke arah berlawanan) baru terlihat saat
integrasi — jauh setelah penyebabnya dibuat.

**Solusi:** digabung ke run validasi, tidak perlu eksperimen terpisah. Steer
positif = belok kanan, lalu tanda deviasi lateral di frame right-handed diukur.
Hasil: **kanan = y negatif, `SIDE_SIGN = −1`**, konsisten di ketiga run.

#### 10. Pemilihan spawn point

**Gejala:** scan pertama memberi 10 kandidat yang semuanya identik — lurus
0.00°, "4 lajur searah", lebar 3.50 m. Tabelnya tidak bisa memutuskan.

**Kenapa masalah:** "4 lajur searah" tidak memberi tahu ego ada di lajur mana.
Kalau ego di lajur paling kanan, tidak ada tempat untuk menyalip ke kanan.

**Solusi:** hitungan lajur dipisah kanan/kiri, dan urutan tabel diprioritaskan ke
lajur kanan terbanyak. **Spawn 75** terpilih: lajur paling kiri, 3 lajur di
kanan, 14 m aspal tersedia.

#### 11. `LANE_WIDTH` masih asumsi

**Kenapa masalah:** kalau lebar lajur Town04 ternyata 3.7 m sementara planner
menggeser 3.5 m, mobil berhenti sedikit di luar tengah lajur — bias tetap lagi.

**Solusi:** diukur dari peta lewat `waypoint.lane_width`, dilacak sepanjang probe
150 m, dan spawn dengan lebar berubah-ubah ditandai `~` supaya dihindari.

**Hasil: 3.50 m, konstan.** Kebetulan sama dengan asumsi rencana kerja. Satu
asumsi berubah jadi hasil ukur.

> ⚠ BELUM TERVERIFIKASI: klaim bahwa 3.50 m masuk rentang standar jalan
> perkotaan Indonesia masih perlu rujukan Bina Marga. Lihat bagian
> "Status validitas data" di bawah.

---

### Yang masih terbuka

**1. `delta_max` vs `delta_max_phys` — perlu diperhatikan di Tahap 5.**

Rencana kerja bagian 7.5 menulis `control.steer = delta_opt / delta_max`. Dengan
`delta_max = 0.5` itu **salah**: CARLA memetakan `steer → delta_phys × curve(v)`,
jadi `steer = 1.0` menghasilkan 70°, bukan 28.6°. Pakai rumus itu, mobil
understeer ~2.4×. Dua angka ini beda peran:

- `delta_max = 0.5 rad` → batas *constraint* di dalam solver MPC
- `delta_max_phys = 1.222 rad` → penyebut *normalisasi* saat delta jadi perintah
  `steer`, masih harus dibagi `curve(v)` dengan v dalam **km/jam**

**2. Spawn 75 baru terverifikasi lurus sejauh 150 m.** Cukup untuk validasi 3
detik, tapi skenario S1 butuh ~250–300 m (jarak awal 60 m + manuver ~180 m).
Waktu masuk Tahap 6, naikkan `probe_m` di `scan_straight_spawns` lalu scan ulang
sebelum mengunci spawn skenario.

**3. Sumber untuk lebar lajur.** Angka 3.50 m sudah terukur dari peta, tapi untuk
argumen "sesuai kondisi Indonesia" perlu rujukan Bina Marga (pedoman geometrik
jalan perkotaan). Lihat "Status validitas data" di bawah.

**6. Pilihan kendaraan ego — DITUNDA.** Muncul pertanyaan apakah `charger_2020`
(sedan Amerika, L 3.04 m, 1920 kg) pantas untuk konteks Indonesia. CARLA tidak
punya satu pun model pasar Indonesia (41 blueprint, semuanya Amerika/Eropa/
Jepang-global). Kandidat terdekat `toyota.prius` (L 2.82 m, 1375 kg).

Keputusan ditahan karena pembandingnya belum ada sumber. Pembanding yang benar
**bukan** "mobil apa yang laris di Indonesia" (argumen pasar, gampang dibantah)
melainkan **kendaraan rencana** dari standar geometrik jalan Bina Marga — itu
kategori dimensi kendaraan baku untuk perancangan jalan, sitasinya rekayasa.

**Ini punya tenggat.** Ganti mobil sekarang = satu baris `EGO_BP` + jalankan
ulang dua skrip (~10 menit). Ganti setelah Tahap 5 = tuning ulang seluruh bobot
MPC. Putuskan sebelum mulai tuning.

Kalau akhirnya tetap Charger, jawaban siap untuk sidang: CARLA tidak menyediakan
model pasar Indonesia; parameter diekstrak langsung dari simulator sehingga
metodenya tidak bergantung merek; dan kendaraan yang lebih besar dan berat
membuat hasilnya konservatif.

**4. Tabel dua-kecepatan belum setara.** Run kecepatan rendah masih 5 detik.
Ulangi dengan `VALIDATION_DURATION = 3.0`.

**5. `LANE_WIDTH` dan `V_MAX` belum ada di `config.py`** — sengaja, belum ada
yang membacanya. `LANE_WIDTH` masuk di Tahap 3 (planner), `V_MAX` di Tahap 5
(constraint MPC), `SEED` di Tahap 6 (TrafficManager).

---

### File & cara menjalankan ulang

```
skripsi/
├── config.py             semua konstanta
├── simulation.py         koneksi CARLA, mode sinkron, spawn ego
├── extract_params.py     bagian 3.1 -> out/vehicle_params.json
├── validate_model.py     bagian 3.2 + 3.3 + verifikasi SIDE_SIGN
├── tests/
│   └── test_bicycle_model.py   uji tanda belok, garis lurus, radius putar
└── out/                  hasil: json, png, csv
```

```bash
python3 extract_params.py            # ekstraksi parameter
python3 validate_model.py --scan     # cari spawn point di ruas lurus
python3 validate_model.py            # validasi + grafik + csv
python3 tests/test_bicycle_model.py  # tanpa CARLA, beberapa detik
```

Konversi koordinat CARLA (left-handed, Y ke bawah) ke right-handed ada di
`to_rh_rear_axle()` dalam `validate_model.py`. Sudah terbukti benar — pindahkan
ke `localization.py` di Tahap 2, jangan tulis ulang.

**Housekeeping:** setelah audit over-engineering, kode dipangkas dari 527 ke 398
baris tanpa mengubah perilaku (buang fallback API yang tidak perlu, field JSON
tanpa konsumen, wrapper satu-pemanggil, format CSV manual -> `np.savetxt`).
Sekarang 483 baris setelah penambahan deteksi keluar aspal dan kolom lajur.

---

## Dasar Akademis — Kenapa Quintic Polynomial

Dicari 7 September 2026, untuk bab 2 dan bab 3.

### Alasan matematis: quintic bukan pilihan, melainkan jawaban

Pindah lajur yang nyaman = pindah lajur dengan **jerk** minimum. Jerk adalah laju
perubahan percepatan (turunan ketiga posisi). Penumpang tidak merasakan
percepatan, mereka merasakan perubahannya.

Rumuskan sebagai masalah optimasi: cari `y(t)` yang meminimumkan

```
J = integral dari 0 sampai T  (d3y/dt3)^2 dt
```

dengan 6 syarat batas — posisi, kecepatan, percepatan lateral, masing-masing di
awal dan akhir. Selesaikan dengan kalkulus variasi (Euler-Lagrange), hasilnya
`y^(6)(t) = 0`, yang solusi umumnya **polinomial derajat lima**.

Cara hitung sederhana yang sampai ke jawaban sama: 6 syarat batas butuh 6
koefisien bebas, dan polinomial berkoefisien 6 itu derajat 5.

**Konsekuensinya: memakai derajat selain 5 berarti tidak sedang meminimumkan
jerk.** Ini argumen yang tidak bisa dibantah, bukan "banyak orang memakainya".

Quartic longitudinal ikut logika sama: posisi akhir tidak dipedulikan (hanya
kecepatan yang ditargetkan), jadi syarat tinggal 5 -> koefisien 5 -> derajat 4.

### Sumber asal (untuk menerangkan *kenapa*)

- **Flash, T. & Hogan, N. (1985).** Model minimum-jerk pada gerak lengan manusia;
  asal mula hasil bahwa lintasan jerk-minimum adalah polinomial derajat lima.
  *Belum diverifikasi langsung — cek sitasi lengkapnya sebelum dipakai.*
- **Werling, M., Ziegler, J., Kammel, S. & Thrun, S. (2010).** *Optimal
  Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame.* IEEE
  ICRA 2010, hlm. 987-993. Membawa hasil di atas ke kendaraan otonom; menunjukkan
  bahwa lintasan teroptimasi-jerk berada dalam himpunan polinomial derajat lima.

Catatan: Werling memakai kerangka Frenet, sedangkan rencana kerja bagian 5
sengaja tidak memakainya karena jalannya lurus. Ini tidak melemahkan sitasi —
penurunan polinomialnya tidak bergantung kerangka koordinat. Nyatakan eksplisit
di bab metodologi.

### Sumber 5 tahun terakhir (untuk menunjukkan metode ini masih terkini)

- **Bai, R., Xu, R., Rui, T., Liu, J., Oung, Q.W., Lee, H.L., Tian, Z. & Yuan, F.
  (2025).** *Safe and Efficient Lane-Changing for Autonomous Vehicles: An Improved
  Double Quintic Polynomial Approach with Time-to-Collision Evaluation.* Journal
  of King Saud University - Computer and Information Sciences. arXiv:2509.00582.
  **Dibaca penuh.** Persamaan (7)-(13) identik dengan rencana kerja bagian 5.2:
  enam syarat batas diselesaikan sebagai sistem linear `Ma = b`. Melaporkan jerk
  lateral tetap dalam ambang kenyamanan +-2,0 m/s^3.
- **Jin, M., Qu, M., Gao, Q., Huang, Z., Su, T. & Liang, Z. (2024).** *Advanced
  Trajectory Planning and Control for Autonomous Vehicles with Quintic
  Polynomials.* Sensors, 24(24), 7928. DOI 10.3390/s24247928.
- **Li et al. (2023).** *Dynamic Trajectory Planning for Automated Lane Changing
  Using the Quintic Polynomial Curve.* Journal of Advanced Transportation.
- **(2024).** *Lane-changing and overtaking trajectory planning for autonomous
  vehicles with multi-performance optimization considering static and dynamic
  obstacles.* Robotics and Autonomous Systems. Paling dekat dengan judul skripsi.

### Cara mengutip di skripsi

Kutip **keduanya**. Werling 2010 untuk *kenapa* derajat lima; paper 2023-2025
untuk *bahwa* metode ini masih praktik terkini dan sudah diperluas dengan TTC,
optimasi multi-objektif, dan halangan dinamis. Mengutip hanya paper 2024-2025
untuk hasil tahun 1985 akan memancing pertanyaan "ini asalnya dari mana".

---

## Tahap 2 — Localization

**Status: SELESAI** — 7 September 2026
**Rujukan rencana kerja:** bagian 4 (plus kontrak bagian 2.2)

### Apa yang dibuat

`localization.py` — satu-satunya tempat konversi koordinat terjadi:

- CARLA left-handed, Y ke bawah -> right-handed: `y -> -y`, `yaw -> -yaw`
- origin actor -> titik sumbu belakang, memakai offset **terukur** (−1,433 m),
  bukan `L/2`
- `v_long = v.x·cos(yaw) + v.y·sin(yaw)` (get_velocity di frame dunia)
- `yaw_rate` dari `get_angular_velocity().z`, deg/s -> rad/s, dinegasikan

Isi: `EgoState` (dataclass beku), `to_rh_rear_axle()`, `CarlaGTLocalization`.
`validate_model.py` sekarang mengimpor dari sini — salinan lokalnya dihapus,
sehingga fungsi konversi hanya ada satu.

### Verifikasi

| Yang diuji | Hasil |
|---|---|
| `yaw_rate` terlog vs d(yaw)/dt numerik | −0,1259 vs −0,1263 rad/s — selisih 0,3%, **tanda benar** |
| Regresi setelah refactor | error posisi 0,538 m / 0,309 m @ 2 s — **identik** dengan sebelum refactor |
| Determinisme simulasi | angka sama sampai 3 desimal antar-run |

Verifikasi menumpang `validate_model.py`, tidak perlu skrip baru: kalau konversi
rusak, error posisi langsung meledak dari 0,3 m.

### Keputusan desain

**`EgoState` ditaruh di `localization.py`, bukan `types.py`.** Rencana kerja
bagian 2.1 merancang `types.py` sebagai tempat dataclass bersama, tapi sekarang
`EgoState` punya satu produsen dan nol konsumen — `planning.py` dan `control.py`
belum ada. Pindahkan saat modul kedua benar-benar membacanya.

**`update()` tanpa argumen.** Kontrak bagian 2.2 menulis
`localization.update(sensor_data)`. Versi ground truth tidak membutuhkan
`sensor_data` — membaca aktor langsung. Tambahkan parameter itu kalau EKF
(Tahap opsional) benar-benar ditulis; saat ini pemanggilnya cuma satu.

### Global planner (bagian 5.1) — SELESAI

`simulation.reference_path(world, location, length_m, step)` -> ndarray (3, N)
= `[x, y, psi]` di frame right-handed. Centerline lajur, **tidak berubah saat
menyalip** — manuver dinyatakan sebagai deviasi lateral terhadapnya.

Ditaruh di `simulation.py`, bukan `planning.py`, karena pembangkitannya butuh
akses peta CARLA sedangkan aturan bagian 2.4 melarang `planning.py` mengimpor
`carla`. Konversi koordinatnya tetap memanggil `localization.carla_xy_yaw_to_rh`
supaya konvensi tanda hanya didefinisikan di satu tempat.

Loop penelusuran lajur diekstrak jadi `walk_lane()` dan dipakai bersama oleh
`reference_path` dan `scan_straight_spawns`.

### Dua bug yang ditangkap saat verifikasi global planner

#### 1. `psi` melompat 2π  *(serius)*

**Gejala:** rentang psi sepanjang path lurus terbaca 360,06° — nilai yaw
berputar antara −89,8° dan +270,3°, padahal itu arah yang sama.

**Kenapa masalah:** kalau ini masuk `psi_ref`, error MPC `e = x − x_ref` meledak
jadi ~6,28 rad tepat di titik lompatan, dan kemudi menyentak keras. Bug ini baru
akan terlihat di Tahap 5 dan sangat sulit dilacak dari sana — gejalanya
menyerupai bobot `Q` yang salah tuning.

**Solusi bagian 1:** `np.unwrap(psi)` sebelum path dikembalikan. Sudah dipasang.

**Solusi bagian 2 — BELUM, dibutuhkan di Tahap 5.** `unwrap` merapikan jalur
acuan, tapi `yaw` mobil sendiri juga datang dari CARLA dan bisa melompat
terlepas dari acuannya. Jadi **setiap pengurangan sudut** harus dinormalisasi ke
[-pi, pi]:

```python
e_psi = (yaw - psi_ref + np.pi) % (2 * np.pi) - np.pi
```

Berlaku di: suku error MPC `e_k = x_k - x_ref,k` komponen psi, transisi FSM yang
membandingkan arah, dan perhitungan `psi_ref = atan2(y', x')` di local planner.
Pola ini sudah dipakai di `validate_model.analyse()` — salin dari sana.

**Penyebab sebenarnya (sudah diperiksa ke datanya):** bukan noise, bukan
goyangan fisika. Sumbernya `waypoint.transform.rotation.yaw`, yaitu **data peta
statis** dari file OpenDRIVE Town04.

Nilai mentah CARLA di spawn 75:

```
+89,78  +89,78  +89,78  +89,78  +89,78  -270,22  -270,22  -270,22 ...
```

Selisih tepat 360,0000. Arah jalan sesungguhnya 89,78° — diverifikasi terpisah
dengan `atan2` dari perpindahan posisi (y naik dari -187,89 ke -157,90), tanpa
melihat `yaw` sama sekali.

Polanya menetap, bukan acak: 22 titik memakai +89,78, 39 titik memakai -270,22.
Artinya **file peta menuliskan arah yang sama dengan dua konvensi berbeda di
ruas yang berbeda**, dan CARLA meneruskannya apa adanya.

Konsekuensinya lebih buruk daripada noise: karena datanya statis, lompatan
selalu terjadi **di titik yang sama persis setiap run**. Mobil menyentak di
lokasi yang itu-itu juga, berulang, dan terlihat seperti ada masalah pada satu
tempat tertentu di jalan.

Catatan: -89,8 dan +270,2 yang sempat tercatat adalah nilai **setelah** konversi
ke right-handed (tanda dibalik). Mentahnya +89,78 dan -270,22.

**Akar penyebab — sudah dilacak sampai isi file peta** (`map.to_opendrive()`):

| road_id | `hdg` tertulis di OpenDRIVE | junction |
|---|---|---|
| 48 | 90,22° | -1 |
| 49 | 90,22° | -1 |
| **902** | **450,22°** | **890** |

Road 902 tertulis **450,22° = 90,22 + 360**. Arah sama, ditulis dengan satu
putaran penuh ekstra.

CARLA lalu menghitung arah lajur `yaw = 180 - hdg` (dibalik karena lane_id
positif berarti sisi kiri garis acuan, lalu dinegasikan karena frame CARLA
left-handed) **tanpa menormalkan hasilnya**:

```
road  49:  180 - 90,22  =   89,78
road 902:  180 - 450,22 = -270,22
```

Cocok persis dengan yang terbaca di waypoint.

**Kenapa road 902 tertulis 450,22:** `junction = 890` — ruas ini adalah penghubung
persimpangan, yang **dibangkitkan otomatis** oleh program pembuat peta, bukan
digambar manual. Program menjumlahkan rotasi dari ruas sebelumnya dan tidak
memutarnya kembali saat melewati 360°. Angka itu sisa perhitungan yang tidak
pernah dibersihkan.

Jadi bukan kecerobohan pembuat peta dan bukan bug CARLA — sisa perhitungan
otomatis yang diteruskan apa adanya. Sepanjang 60 m pertama ada 3 `road_id`
(48, 49, 902) dan lompatannya terjadi tepat di meter ke-5, saat masuk ruas 902.

### Temuan sampingan: 40% jalur berada di dalam junction

Diperiksa saat menelusuri asal lompatan sudut. Sepanjang 300 m dari spawn 75,
**40% waypoint bertanda `is_junction = True`**. Ini bukan perempatan berlampu,
melainkan area pertemuan ramp masuk/keluar di highway Town04.

| spawn | bagian junction | lajur kanan di dalam junction |
|---|---|---|
| 75 (dipakai) | 40% | 119/119 — utuh |
| 79 / 80 | 32% | 96/96 — utuh |
| 104 | 49% | 148/148 — utuh |

**Tidak ada yang rusak.** Risiko utamanya adalah lajur kanan hilang di dalam
junction sehingga manuver menyalip kehilangan lajur tujuan — dan itu terbukti
tidak terjadi, di semua kandidat lajur kanan terdefinisi 100%. Geometri juga
tetap lurus (0,06° sepanjang 400 m).

**Keputusan: SPAWN_IDX tetap 75.** Spawn 79 sedikit lebih bersih (32%) tapi
selisihnya tidak cukup berarti untuk mengulang validasi yang sudah
terdokumentasi.

**⚠ Untuk Tahap 6:** TrafficManager punya perilaku khusus di dalam junction.
Kalau kendaraan target berperilaku aneh saat autopilot dinyalakan, ini tersangka
pertamanya, dan **spawn 79 adalah cadangannya**.

**Akibat kalau lolos:** mobil melenceng 0,8° terbaca sebagai error -359,2°.
Dengan `Q[psi] = 10` dan biaya kuadrat, itu ~200.000x lebih besar dari
seharusnya. Solver membanting setir di satu titik tetap sepanjang jalur.
Gejalanya menyerupai bobot salah tuning, jadi mudah salah kejar.

#### 2. Path bisa membelok tanpa peringatan

**Gejala:** guard panjang tidak memicu walau diminta 100 km, karena highway
Town04 adalah lingkaran tertutup — `next()` terus mengembalikan waypoint valid.

**Kenapa masalah:** seluruh skripsi mengasumsikan ruas lurus (bagian 0.1 dan 5,
alasan Frenet tidak dipakai). Path yang diam-diam membelok mematahkan asumsi
Cartesian tanpa gejala.

**Solusi:** guard kelurusan — tolak path yang simpangan psi-nya melebihi
`max_drift_deg` (default 5°).

### Kelurusan spawn 75 — terverifikasi 400 m

| Panjang path | Simpangan psi | Status |
|---|---|---|
| 150 m | 0,00° | OK |
| 300 m | 0,06° | OK |
| **400 m** | **0,06°** | **OK** |
| 600 m | 55,3° | ditolak guard |

Ini menutup poin terbuka Tahap 1 nomor 2. Skenario S1 butuh ~250–300 m; tersedia
400 m. Tidak perlu scan ulang dengan `probe_m` lebih besar.

### Verifikasi akhir Tahap 2

| Yang diuji | Hasil |
|---|---|
| `reference_path` 300 m | shape (3, 301), spasi 1,000–1,006 m, psi −89,756° konstan |
| Tanda y path vs waypoint CARLA mentah | terbalik dengan benar |
| Guard kelurusan | menolak 600 m, menerima 400 m |
| `scan_straight_spawns` setelah pakai `walk_lane` | keluaran identik dengan sebelumnya |
| Validasi model setelah semua refactor | 0,538 m — identik, tidak ada regresi |

---

## Status Validitas Data

Dipisahkan supaya tidak ada angka tak bersumber yang lolos ke skripsi.

### Terukur — aman dikutip, bisa direproduksi

Seluruh parameter kendaraan (`L`, offset sumbu belakang, `delta_max`, dimensi,
massa, drag, `steering_curve`), `LANE_WIDTH` 3.50 m, geometri spawn point, dan
semua hasil validasi model. Sumbernya CARLA API langsung.

Cara verifikasi: jalankan ulang `extract_params.py` dan `validate_model.py`.
Untuk perbandingan antar-blueprint, spawn tiap kendaraan lalu panggil
`get_physics_control()` — sama seperti yang dilakukan `extract_params.py`.

Di bab metodologi, tulis sumbernya sebagai CARLA Python API beserta versinya,
bukan katalog kendaraan. Skripnya sendiri yang jadi bukti reproducibility.

### Sudah ada dasar hukum — tinggal dicantumkan

| Klaim | Dasar |
|---|---|
| Batas kecepatan 50 km/jam kawasan perkotaan | UU No. 22/2009 Pasal 21; PP No. 79/2013 Pasal 23 ayat 4 |
| Menyalip dari sebelah kanan | UU No. 22/2009 Pasal 109 |

### Sumber — SUDAH DIVERIFIKASI (7 Sep 2026)

**Dokumen acuan utama: Pedoman Desain Geometrik Jalan (PDGJ), Pedoman No.
13/P/BM/2021**, ditetapkan lewat SE Dirjen Bina Marga No. 20/SE/Db/2021,
27 Oktober 2021. Dokumen terbaru dan berlaku. Semua angka di bawah dari sini.

*Uji keaslian:* salinan penuh diambil dari mirror, lalu prakatanya diadu dengan
preview resmi binamarga.pu.go.id — identik kata-per-kata, nomor dokumen cocok,
ukuran 14,85 MB terhadap 15,50 MB yang disebut situs resmi. Untuk daftar pustaka
tetap unduh dari situs resmi.

#### Lebar lajur — Tabel 5-58 "Lebar lajur minimum"

| Kecepatan desain V_D | Lebar lajur paling kecil |
|---|---|
| tinggi: V_D ≥ 80 km/jam | 3,60 m |
| **sedang: 40 ≤ V_D < 80 km/jam** | **3,50 m** |
| rendah: V_D < 40 km/jam | 2,75 m |

Kecepatan desain skripsi 50 km/jam → pita "sedang" → **3,50 m**.
Town04 terukur **3,50 m**. **Cocok persis.**

Dasar hukum yang dirujuk tabel ini: Permen PU No. 19/PRT/M/2011.

#### Kendaraan desain — Tabel 5-9

Memakai kendaraan Indonesia nyata dan mencantumkan **jarak antar sumbu**
(wheelbase) — parameter yang persis masuk ke bicycle model. Ini yang tidak
dipunyai standar 1997 maupun 2004. Sumber tabel: Lawalata dan Rahman, 2020.

Kategori "dapat beroperasi pada jalan kelas 1, 2, dan 3" (satuan m):

| Kendaraan | Panjang | Lebar | Tinggi | Jarak antar sumbu |
|---|---|---|---|---|
| Toyota Avanza | 4,19 | 1,66 | 1,69 | 2,65 |
| **Toyota Hiace** | **5,38** | **1,88** | 2,29 | **3,11** |
| Isuzu ELF NLR 55 BLX | 6,17 | 1,84 | 2,17 | 3,36 |

Pembanding CARLA (terukur):

| Kendaraan | Panjang | Lebar | Jarak antar sumbu |
|---|---|---|---|
| dodge.charger_2020 | 5,01 | 1,88 | 3,04 |
| toyota.prius | 4,51 | 2,01 | 2,82 |

### KEPUTUSAN FINAL: kendaraan ego TETAP `dodge.charger_2020`

Lebar Charger identik dengan Toyota Hiace (1,88 m, selisih 0,1%), jarak antar
sumbu beda 2%, panjangnya lebih pendek. Charger duduk di antara Avanza dan
Hiace — keduanya kendaraan desain untuk jalan kelas 1, 2, dan 3 menurut standar
nasional yang berlaku. Tidak ada alasan mengganti.

Kalimat untuk bab metodologi:

> Dimensi kendaraan uji (5,01 x 1,88 m, jarak antar sumbu 3,04 m) berpadanan
> dengan Toyota Hiace (5,38 x 1,88 m, jarak antar sumbu 3,11 m), salah satu
> kendaraan desain untuk jalan kelas 1, 2, dan 3 menurut Pedoman Desain
> Geometrik Jalan No. 13/P/BM/2021 Tabel 5-9, dan memenuhi batas lebar kendaraan
> bermotor menurut PP No. 55 Tahun 2012 (maksimum 2.100 mm).

**Riwayat keputusan ini** — sempat direkomendasikan ganti ke `toyota.prius` atas
dasar "rentang mobil penumpang Indonesia 2,4-2,8 m / 1000-1500 kg". Angka itu
dari ingatan, tanpa sumber, dan keliru sebagai pembanding: itu rentang pasar,
bukan standar perancangan. Terhadap standar yang benar, Charger sah.

### Sumber pendukung

| Dokumen | Isi | Status |
|---|---|---|
| PDGJ No. 13/P/BM/2021 (SE 20/SE/Db/2021) | Tabel 5-9, Tabel 5-58 | **berlaku, acuan utama** |
| PP No. 55 Tahun 2012 tentang Kendaraan | lebar maks 2.100 mm | berlaku |
| Permen PU No. 19/PRT/M/2011 | dasar lebar lajur minimum | dirujuk PDGJ 2021 |
| UU No. 22/2009 Ps. 21; PP No. 79/2013 Ps. 23(4) | batas kecepatan 50 km/jam | berlaku |
| UU No. 22/2009 Ps. 109 | menyalip dari kanan | berlaku |
| TPGJAK No. 038/TBM/1997 | — | **dicabut** oleh PDGJ 2021 |
| RSNI T-14-2004 Geometri Jalan Perkotaan | — | **status belum jelas**, lihat di bawah |

### ⚠ Satu hal yang belum tuntas

**Status RSNI T-14-2004.** Prakata resmi PDGJ 2021 hanya mencabut satu dokumen:
Pedoman Tata Cara Perencanaan Geometrik Jalan Antar Kota 1997. RSNI T-14-2004
tidak disebut. Beberapa sumber sekunder mengklaim daftar pencabutan yang lebih
luas, tapi klaim itu tidak ditemukan di teks pedoman — kemungkinan ada di badan
Surat Edaran, yang tidak termuat di PDF pedoman.

**Tidak menghalangi.** Cukup pakai PDGJ 2021 sebagai acuan; semua angka yang
dibutuhkan ada di sana, dan statusnya jelas berlaku. RSNI T-14-2004 tidak perlu
dikutip sama sekali.

### Tautan

- PDGJ 2021, halaman resmi Bina Marga: https://binamarga.pu.go.id/index.php/nspk/detail/surat-edaran-direktur-jenderal-bina-marga-nomor-20sedb2021-tentang-pedoman-desain-geometrik-jalan-pedoman-nomor-13pbm2021
- PP 55/2012, JDIH BPK: https://peraturan.bpk.go.id/Details/5268/pp-no-55-tahun-2012

---

## Tahap 3 — Local Planner

**Status: SELESAI** — 7 September 2026
**Rujukan rencana kerja:** bagian 5.2-5.5

### Apa yang dibuat

`planning.py` — murni numerik, **tidak mengimpor carla** (aturan 2.4). Terbukti:
saat server CARLA mati, `plot_candidates()` tetap menghasilkan gambar sebelum
koneksi dicoba.

| Fungsi | Isi |
|---|---|
| `quintic_coeffs` | 6 syarat batas -> sistem linear 3x3 -> koefisien a0..a5 |
| `quartic_coeffs` | 5 syarat batas (posisi akhir bebas) -> b0..b4 |
| `peak_lateral_accel` | `(10/sqrt(3)) * |dy| / T^2` — saringan analitik, tanpa evaluasi numerik |
| `plan_lane_change` | sampling 3 offset x 3 durasi, saring kelayakan, pilih biaya minimum |
| `Trajectory` | `(4, N+1)` = `[x_ref, y_ref, psi_ref, v_ref]` + `sample_at()` |

Saringan kelayakan: percepatan lateral > 3 m/s², kelengkungan > 1/R_min, dan
tabrakan elips terhadap prediksi kecepatan-konstan halangan.

Biaya: `J = W_LAT*J_lat + W_LON*J_lon + W_COL*J_col`, dengan `J_lat` = integral
jerk lateral kuadrat + penalti durasi + penalti simpangan dari tengah lajur.

### Verifikasi — 11 uji, semua lolos, tanpa CARLA

| Uji | Hasil |
|---|---|
| Percepatan lateral puncak, dy=3,5 m T=3 s | **2,245 m/s²** — cocok dengan rencana kerja (~2,24) |
| Analitik vs numerik untuk puncak yang sama | selisih < 1e-3 |
| Bentuk tertutup syarat awal nol | `a3=10dy/T^3`, `a4=-15dy/T^4`, `a5=6dy/T^5` tepat |
| Enam syarat batas quintic | terpenuhi sampai 1e-9 |
| Quartic | kecepatan akhir tepat, percepatan akhir nol, derajat 4 |
| Tanpa halangan | terpilih offset 3,5 m = tepat tengah lajur tujuan |
| Arah mengikuti `SIDE_SIGN` | benar untuk -1 dan +1 |
| Halangan diam di lajur tujuan | seluruh 9 kandidat ditolak, kembali `(None, [])` |
| `psi_ref`, `v_ref` konsisten dengan turunan lintasan | selisih < 5e-3 |

### Perilaku yang perlu diingat saat tuning Tahap 4

**Tanpa halangan, pilihan selalu T = 4,0 s** — durasi terpanjang punya jerk
terkecil, dan itu memang yang diminimumkan. Wajar, bukan bug.

**`J_col` bekerja sebagai gradien lunak, bukan tolak-langsung.** Halangan 45 m di
depan pada lajur tujuan yang bergerak 7 m/s tidak menolak kandidat mana pun —
memang benar, karena ego menempuh 55,6 m dalam 4 detik sementara halangan sudah
maju ke 73 m, sisa jarak 17 m. Yang berubah hanya biayanya (9,07 -> 9,23).
Penolakan keras hanya terjadi kalau elipsnya benar-benar dilanggar.

**Kalau nanti FSM terlalu sering abort**, periksa dulu apakah `LATERAL_OFFSETS`
dan `MANEUVER_TIMES` memang menghasilkan kandidat — rencana kerja bagian 11.5
menyarankan mencatat jumlah kandidat yang lolos sebagai metrik diagnostik.

### Keputusan desain

**Halangan diterima sebagai ndarray `(M, 4)` = `[x, y, vx, vy]`, bukan dataclass
`TrackedObject`.** Ukuran per-halangan tidak dibutuhkan karena bagian 7.2 memakai
elips dengan `a_e`, `b_e` tetap dari config, bukan dimensi tiap kendaraan.
Dataclass menyusul saat perception benar-benar memproduksinya (Tahap 8).

**`K_DEV` dinaikkan 5,0 -> 20,0.** Dengan nilai awal 5,0, kandidat offset 3,0 m
justru menang karena jerknya lebih kecil — planner memilih berhenti di luar
tengah lajur. Penalti simpangan harus cukup besar untuk mengalahkan selisih
jerk antar-offset.

### Gambar

`out/planner_candidates.png` — 9 kandidat, yang terpilih ditebalkan, plus profil
percepatan lateral terhadap batas 3 m/s². Dihasilkan `show_lanes.py`, tidak butuh
CARLA.

### Yang masih terbuka

**Normalisasi sudut belum dipakai di sini.** `psi_ref = atan2(y', x')` selalu
mengembalikan (-pi, pi], jadi aman. Tapi begitu MPC mengurangi `yaw - psi_ref` di
Tahap 5, normalisasi wajib dipasang — lihat bagian bug psi di atas.

**FSM belum ada.** `plan_lane_change` masih dipanggil langsung dengan `y_target`
implisit dari `SIDE_SIGN`. Tahap 4 yang memutuskan *kapan* memanggilnya.

---

## Visualisasi Planner (`record_maneuver.py`)

Dibuat 7 September 2026. Bukan bagian tahap mana pun, tapi jadi alat verifikasi
dan sumber gambar/video untuk bab metodologi (bagian 11.6 minta video direkam
sejak awal, bukan setelah eksperimen selesai).

Menghasilkan playback manuver menyalip dengan **overlay kandidat**: biru = 9
kemungkinan lintasan, hijau = yang dipilih. Dua sudut kamera (`--kamera atas`
potret / `--kamera kejar`).

**Ini playback, bukan hasil kendali.** Physics dimatikan, posisi ego ditempelkan
ke lintasan planner. Beri label "lintasan hasil local planner" kalau dipakai di
sidang, jangan "hasil kendali MPC".

### Dua bug yang tertangkap justru lewat visualisasi ini

#### 1. Planner dipanggil tanpa `obstacles` -- nyaris menabrak

**Gejala:** terlihat di video, ego melewati kendaraan target terlalu mepet.

**Ukuran:** jarak minimum 2,26 m, selisih longitudinal 0,67 m (praktis
bersebelahan), simpangan ego baru -2,16 m dari -3,50 (masih separuh di lajur
lama). Constraint elips `g = 0,709` -- **dilanggar**.

**Penyebab:** `plan_lane_change` dipanggil tanpa argumen `obstacles`. Planner
tidak tahu ada kendaraan di sana. Ditambah `GAP_AWAL = 30 m` yang membuat TTC
(2,35 s) lebih pendek dari durasi manuver (4 s) -- persis kondisi yang dilarang
bagian 6.

**Setelah diperbaiki:** jarak minimum 3,50 m, simpangan penuh -3,50 saat
berpapasan, `g = 2,531` aman.

**Pelajaran:** cacat ini tidak terlihat di uji unit maupun grafik kandidat.
Baru kelihatan saat lintasan ditaruh di dunia 3D bersama kendaraan lain.

#### 2. `world.debug.draw_line()` tidak bisa dipakai untuk overlay

Garis debug CARLA bersifat emissive; auto-exposure kamera membuatnya tampak putih
polos sehingga warna hilang total dan sembilan kandidat menyatu jadi satu
gumpalan. Mematikan `bloom_intensity` tidak menolong.

**Solusi:** rekam frame bersih, lalu proyeksikan titik lintasan ke bidang gambar
memakai matriks kamera (`proyeksi()`) dan gambar dengan PIL setelahnya. Kontrol
penuh atas warna, ketebalan, transparansi, dan bisa menyembunyikan bagian
lintasan di belakang mobil.

**Catatan komposisi:** frame **potret** (540x960), bukan lanskap. Jalan sempit
(15 m) tapi manuver panjang (55 m); frame lebar memboroskan setengah gambar.

### Receding horizon 10 Hz

Versi awal memanggil planner **sekali** per manuver -- tidak reaktif, dan itu
terlihat: kipas kandidat diam di tempat. Diganti dengan `rangkai_receding()`
yang mengulang seluruh proses tiap 100 ms dari state lateral ego saat itu,
sesuai frekuensi di bagian 2.

Agar state lateral bisa diteruskan **persis** antar-langkah, `Trajectory`
sekarang menyimpan koefisien polinomialnya dan punya `lateral_at(t)` yang
mengembalikan `(y, y', y'')` eksak -- bukan hasil diferensiasi numerik dari
sampel.

`plan_lane_change` juga mendapat parameter `y_goal`: manuver kembali ke lajur
asal menargetkan `y = 0`, yang tidak bisa dinyatakan sebagai
`side_sign * LANE_WIDTH`.

**Hasil satu run (skenario S1, 16 detik, 160 langkah replan):**

| | |
|---|---|
| Langkah dengan kandidat | 110 (selalu 9 dari 9 lolos) |
| Langkah menahan lajur | 50 (37 sebelum pemicu, 13 setelah kembali) |
| Langkah abort | **0** -- lajur kanan memang kosong |
| Jarak minimum ego-target | 2,89 m, elips `g = 1,523` aman |

Garis waktu fase, cocok dengan tabel bagian 6:

| t | Fase | Pemicu |
|---|---|---|
| 0 - 3,6 s | `ikuti_lajur` | celah 50 -> 25 m |
| 3,7 s | -> `menyalip` | celah 24,5 m < ambang 25 m |
| 9,0 s | -> `kembali` | ego unggul 12,1 m |
| 14,7 s | kembali di lajur asal | simpangan < 0,05 m |

**Pemicu fase di skrip ini masih sementara** -- tanpa histeresis, tanpa dwell
time, tanpa jalur abort. FSM sungguhan dibuat di Tahap 4; struktur fasenya sudah
mengikuti bagian 6 supaya tinggal dipindahkan.

**Yang perlu diperhatikan saat Tahap 4:** dengan ambang 25 m dan selisih
kecepatan 6,9 m/s, jarak berpapasan turun ke 2,89 m (dari 3,50 m saat manuver
dimulai lebih awal). Masih aman menurut elips dan jauh di atas syarat 1,0 m di
bagian 11.2, tapi ambang pemicu jelas berinteraksi dengan selisih kecepatan --
kandidat pertama untuk dituning.

---

## Tahap 4 — Behavior Planner (FSM)

**Status: SELESAI** — 7 September 2026
**Rujukan rencana kerja:** bagian 6

### Apa yang dibuat

`planning.BehaviorFSM` — lima state sesuai tabel bagian 6. Memutuskan **kapan**
menyalip; local planner memutuskan **bagaimana**.

Setiap ambang punya pasangan histeresis (`D_TRIGGER`/`D_TRIGGER_EXIT`,
`DV_TRIGGER`/`DV_EXIT`), dan setiap transisi harus diminta terus-menerus selama
`FSM_DWELL = 0,3 s` sebelum dieksekusi. `y_goal` mengikuti state dan diteruskan
langsung ke `plan_lane_change`.

Sudah dipasang ke `record_maneuver.py`, menggantikan logika fase sementara.

### Verifikasi — 17 uji, semua lolos, tanpa CARLA

Termasuk: dwell menunda transisi (0,25 s belum, 0,35 s sudah), histeresis tidak
bolak-balik saat celah bergoyang di sekitar ambang, lajur tujuan terisi menahan
transisi (depan maupun belakang), TTC tidak cukup menahan transisi, urutan
lengkap `LANE_KEEPING -> CHECK_OVERTAKE -> LANE_CHANGE_OVERTAKE -> OVERTAKING ->
LANE_CHANGE_RETURN -> LANE_KEEPING` pada skenario S1 sintetis, dan celah pemicu
yang menyesuaikan selisih kecepatan.

### TEMUAN: ambang jarak diganti ambang TTC

**Gejala:** uji urutan lengkap gagal. FSM masuk `CHECK_OVERTAKE` lalu **terjebak
di sana selamanya** dan akhirnya kembali ke `LANE_KEEPING` saat kendaraan depan
terlewati begitu saja tanpa pernah menyalip.

**Penelusuran** (dv = 6,9 m/s, ego 50 vs target 25 km/jam, ambang jarak 25 m):

| t | celah | TTC | state | syarat waktu |
|---|---|---|---|---|
| 2,15 s | 25,2 m | 3,65 s | LANE_KEEPING | terpenuhi |
| 2,55 s | 22,4 m | 3,25 s | CHECK_OVERTAKE | terpenuhi |
| **2,85 s** | **20,3 m** | **2,95 s** | CHECK_OVERTAKE | **gugur** |

Manuver tercepat 3,0 s butuh celah minimal `3,0 x 6,9 = 20,7 m`. Dengan pemicu di
25 m, margin yang tersedia cuma `(25 - 20,7)/6,9 = 0,62 s`, sedangkan **dua**
dwell (LANE_KEEPING->CHECK, CHECK->LANE_CHANGE) menghabiskan 0,6 s. Syarat TTC
gugur tepat sebelum dwell kedua selesai.

**Akar masalahnya bukan nilai ambangnya, tapi satuannya.** Jarak yang aman
bergantung selisih kecepatan: 25 m aman pada dv = 3 m/s dan berbahaya pada
dv = 8 m/s. Ambang jarak tetap tidak bisa benar untuk seluruh rentang kecepatan
di bagian 0.2 (target 20-35 km/jam terhadap ego 40-50 km/jam).

**Solusi: pemicu memakai TTC, bukan jarak.**

```
TTC_TRIGGER >= T_tercepat + 2 x FSM_DWELL + margin = 3,0 + 0,6 + 1,4 = 5,0 s
```

`TTC_TRIGGER = 5,0 s`, histeresis `TTC_EXIT = 7,0 s`. Konstanta `D_TRIGGER` dan
`D_TRIGGER_EXIT` dihapus seluruhnya.

**Hasilnya menyesuaikan sendiri:**

| Kecepatan target | dv | Celah pemicu | TTC |
|---|---|---|---|
| 35 km/jam | 4,2 m/s | 20 m | 4,79 s |
| 30 km/jam | 5,6 m/s | 27 m | 4,85 s |
| 25 km/jam | 7,0 m/s | 34 m | 4,89 s |
| 20 km/jam | 8,3 m/s | 41 m | 4,91 s |

**Untuk skripsi — ini temuan yang layak ditulis di bab pembahasan.** Perhatikan
baris pertama: 20 m, persis batas bawah rentang 20-25 m di bagian 6. Jadi angka
rencana kerja **tidak salah**, dia benar untuk selisih kecepatan kecil, lalu
menjadi tidak memadai saat selisihnya membesar. Ambang pemicu, durasi manuver,
dan dwell time terikat satu pertidaksamaan; menyatakannya dalam satuan waktu
membuat keterikatan itu eksplisit dan satu konstanta menutupi seluruh rentang
kecepatan.

Uji `test_ambang_menyesuaikan_selisih_kecepatan` mengunci sifat ini: celah
pemicu harus naik seiring selisih kecepatan, dan rasio celah/dv harus tetap di
sekitar `TTC_TRIGGER`.

### Dasar akademis pemicu TTC

Dicari 7 September 2026, karena mengganti ambang jarak bagian 6 perlu dasar.

**Asal konsep — SUDAH DIVERIFIKASI dari sumber primer** (8 September 2026):

> Hayward, J.C. (1972). *Near-Miss Determination Through Use of a Scale of
> Danger.* Highway Research Record, 384, 24-35. Washington, D.C.: Highway
> Research Board.

Afiliasi penulis (Pennsylvania Transportation and Traffic Safety Center, Penn
State) tercantum di makalahnya, tapi **jangan dimasukkan ke entri daftar
pustaka** — tempat itu untuk penerbit, dan penerbitnya Highway Research Board.

PDF resmi gratis di arsip TRB:
https://onlinepubs.trb.org/Onlinepubs/hrr/1972/384/384-004.pdf
Dipresentasikan di 51st Annual Meeting of the Highway Research Board, Januari
1972. Versi laporan teknisnya bernomor TTSC 7115 (Penn State).

Isinya sudah dibaca langsung. Hayward menyebut ukurannya
**time-measured-to-collision (TMTC)** — istilah "TTC" baru baku belakangan.
Definisinya: waktu yang dibutuhkan dua kendaraan untuk bertabrakan bila keduanya
melanjutkan dengan kecepatan dan lintasan saat itu. Bagian "TIME TO COLLISION"
ada di halaman 27.

**PERINGATAN sitasi.** Sumber sekunder tidak konsisten soal tahunnya — banyak
yang menulis **1971** (mengacu laporan teknis Penn State), sebagian **1972**
(mengacu Highway Research Record). Contoh: Singh dkk. (2024) menulis "Hayward.
Highway Research Board; 1971; pp. 24-35". Arsip TRB sendiri menempatkannya di
Highway Research Record 384 tahun **1972**, halaman **24-35**. Karena sumber
primernya sudah dibuka, pakai versi 1972 di atas.

**Rujukan sekunder terbaru** (bila pembimbing meminta sitasi yang lebih baru,
format "dikutip dalam"):

> Singh, D., Das, P. & Ghosh, I. (2024). *Conflict-Based safety evaluations at
> unsignalized intersections using surrogate safety measures.* Heliyon.
> DOI: 10.1016/j.heliyon.2024.e27665. (akses terbuka)

**Catatan praktis: Hayward mungkin tidak dibutuhkan sama sekali.** Ambang TTC di
skripsi ini sudah didukung Lin dkk. (2023) dan Bai dkk. (2025) yang keduanya
terbaru dan langsung membahas TTC untuk pindah lajur. Hayward hanya diperlukan
kalau ingin menyebut asal-usul konsepnya — dan mengutip sumber 1972 untuk
definisi fundamental adalah hal yang lazim, bukan kelemahan.

**Dipakai untuk keputusan pindah lajur:**

- **Lin, P., Javanmardi, E., Tao, Y., Chauhan, V., Nakazato, J. & Tsukada, M.
  (2023).** *Time-to-Collision-Aware Lane-Change Strategy Based on Potential
  Field and Cubic Polynomial for Autonomous Vehicles.* arXiv:2306.06981.
  Ambang TTC **2,7 detik**, mengacu nilai yang disarankan Mobileye.
- **Bai dkk. (2025)** (sudah dikutip di Tahap 3). Studi sensitivitas ambang TTC:
  baseline **3,0 s**, konservatif **4,5 s**, agresif **1,5 s**, dibandingkan
  terhadap jarak aman dan durasi manuver.

**Pemetaan ke dua ambang di implementasi ini:**

| Ambang | Nilai | Peran | Padanan literatur |
|---|---|---|---|
| `TTC_TRIGGER` | 5,0 s | mulai **mempertimbangkan** (`LANE_KEEPING -> CHECK_OVERTAKE`) | tidak ada padanan langsung |
| `_sempat()` | **3,0 s** | boleh **mengeksekusi** (`CHECK_OVERTAKE -> LANE_CHANGE_OVERTAKE`) | Bai dkk. baseline **3,0 s**; Lin dkk. 2,7 s |

Gerbang eksekusi 3,0 s **persis sama dengan baseline Bai dkk.** dan dekat dengan
2,7 s Lin dkk. Itu bukan kebetulan yang perlu dijelaskan: nilainya datang dari
`min(MANEUVER_TIMES)`, durasi pindah lajur tercepat — sama-sama diturunkan dari
fisika manuvernya, bukan dipilih.

`TTC_TRIGGER = 5,0 s` **bukan** ambang keselamatan melainkan titik keputusan yang
lebih awal (3,0 s manuver + 0,6 s dua dwell + 1,4 s margin), jadi jangan
dibandingkan langsung dengan 4,5 s "konservatif" Bai dkk. — besarannya berbeda
peran.

**Untuk bab hasil:** kalau FSM ternyata terlalu konservatif, rentang wajar di
literatur adalah 2,7-4,5 s, dan Bai dkk. menyediakan kerangka studi sensitivitas
(ambang TTC x jarak aman x durasi manuver) yang bisa ditiru.

### Hasil satu run dengan FSM sungguhan (skenario S1)

| t | State | Celah | Lateral |
|---|---|---|---|
| 0,00 s | `LANE_KEEPING` | 50,0 m | 0,00 |
| 2,70 s | `CHECK_OVERTAKE` | 31,4 m | 0,00 |
| 3,20 s | `LANE_CHANGE_OVERTAKE` | 27,9 m | 0,00 |
| 7,50 s | `OVERTAKING` | -1,8 m | -3,33 |
| 8,80 s | `LANE_CHANGE_RETURN` | -10,7 m | -3,52 |
| 13,10 s | `LANE_KEEPING` | -40,4 m | -0,17 |

| Metrik | Nilai |
|---|---|
| Jarak minimum ke target | 3,20 m |
| Elips `g` minimum | 1,989 — aman |
| Syarat bagian 11.2 (> 1,0 m) | lulus |
| Durasi manuver | 10,4 s — bagian 0.2 memperkirakan 10-13 s |
| Abort | 0 — lajur kanan memang kosong |

### Keputusan desain

**Abort tidak menunggu dwell.** Bagian 6 menulis dwell 0,3 s untuk "setiap
transisi", tapi menunda pembatalan justru menambah risiko. Chattering *masuk* ke
abort jauh lebih aman daripada chattering *keluar* darinya, jadi jalur
`LANE_CHANGE_OVERTAKE -> LANE_KEEPING` dieksekusi seketika lewat `_langsung()`.
Transisi lain tetap memakai dwell.

**`_terdepan` tidak dipakai di state `OVERTAKING`.** Fungsi itu hanya melihat
`x > 0`, sehingga kendaraan yang baru terlewati 1 m sudah dianggap hilang dan FSM
langsung kembali ke lajur — melanggar syarat `PASS_MARGIN = 8 m` di bagian 6.
Diganti dengan pemeriksaan seluruh kendaraan di lajur asal yang posisinya masih
`> -PASS_MARGIN`.

### Yang masih terbuka

**Belum diuji dengan skenario abort sungguhan.** Uji unit sudah mencakup jalur
abort dengan input sintetis, tapi skenario S3 (kendaraan di lajur tujuan, celah
terbatas) belum dijalankan di simulator. Itu bagian dari Tahap 9.

**Ambang lain belum dituning.** `D_SAFE_DEPAN`, `D_SAFE_BELAKANG`, dan
`PASS_MARGIN` masih memakai nilai dari bagian 6 apa adanya. Bagian 11.5
menyarankan mencatat jumlah abort per skenario sebagai diagnostik: sering abort
di S1 (lajur kosong) berarti terlalu konservatif.

---

## Tahap 5 langkah 1 — Inti MPC

**Status: SELESAI** — 10 September 2026
**Rujukan rencana kerja:** bagian 7.1-7.4

### Apa yang dibuat

`control.py` — murni numerik, tidak mengimpor carla (aturan 2.4). Graf CasADi
`Opti()` dibangun **sekali** di `__init__`; tiap tick hanya `set_value` + `solve`.

| Isi | Keterangan |
|---|---|
| `_rk4` | dinamika bicycle simbolik, `dt = 0,1 s`, `N = 20` |
| `MPCController.solve` | -> `(a, delta, waktu_ms, ok)`, warm start + fallback |
| `MPCController.compute` | satu tick penuh -> `ControlCommand` |
| `steer_command` | delta -> perintah steer CARLA, **memakai `delta_max_phys`** |
| `ThrottlePI` | `a_ref` -> throttle/brake tanpa tabel kalibrasi |

### Verifikasi — 13 uji, semua lolos, tanpa CARLA

| Uji | Hasil |
|---|---|
| Menarik kembali ke acuan (melenceng 1 m) | setir ke arah benar, error tinggal < 25% |
| Diam bila sudah pas | `\|delta\|max < 5e-3`, tidak berosilasi |
| Batas laju perubahan setir | `\|Ddelta\| <= DDELTA_MAX` terpenuhi |
| Menjejak lintasan planner melewati halangan | **XTE rata-rata 0,074 m, maks 0,444 m**, `g_min = 2,39` |
| Tetap menjawab saat tidak ada solusi aman | slack aktif, `a`/`delta` tetap terhingga |
| Lompatan sudut 2pi di acuan | perintah identik (selisih < 1e-6) |
| Waktu solve | **rata-rata 24,9 ms, maks 32,8 ms** (anggaran 50 ms) |
| Warm start | **116,6 ms dingin -> 27,7 ms hangat, 4,2x** |

Percepatan warm start 4,2x cocok dengan klaim 3-5x di bagian 7.4.

### TEMUAN: waktu solve awalnya jebol anggaran

Konfigurasi pertama (4 slot obstacle, toleransi IPOPT default) memberi solve
rata-rata **48,7 ms** dengan maksimum **58,4 ms** — melewati anggaran tick 50 ms.

Pengukuran per-tuas (median, 12 solve, warm start aktif):

| Slot obstacle | Toleransi | Median solve |
|---|---|---|
| 1 | default 1e-8 | 45,5 ms |
| 2 | default | 51,5 ms |
| 4 | default | 62,7 ms |
| 4 | 1e-4 | 43,7 ms |
| 1 | 1e-4 | 31,4 ms |

**Solusi, dua perubahan:**

1. `MPC_TOL = 1e-4`. Toleransi default IPOPT 1e-8 dirancang untuk optimasi yang
   jawabannya menjadi produk akhir. Di sini jawabannya dibuang 50 ms kemudian,
   dan 1e-4 rad = 0,006° jauh di bawah `DDELTA_MAX = 0,05 rad` yang bisa
   dieksekusi mobil. Ditambah `acceptable_tol` dan `acceptable_iter = 5` supaya
   solver boleh berhenti lebih awal di titik yang sudah cukup baik.
2. `MPC_MAX_OBSTACLES = 4 -> 2`. Skenario S1-S5 paling banyak butuh 2 kendaraan;
   tiap slot menambah 21 constraint elips dan sekitar 10 ms.

Hasil akhir: **24,9 ms rata-rata, 32,8 ms maksimum**. Uji sekarang mengunci
keduanya terhadap `FIXED_DELTA_SECONDS`.

### TEMUAN: MPC tidak akan memilih sisi sendiri

Uji awal mengharapkan MPC menghindari halangan diam dengan membelok. **Tidak
terjadi** — `delta` tepat 0,00000 sepanjang loop, mobil hanya memperlambat
(13,9 -> 11,9 m/s).

Bukan bug. Halangan tepat di depan pada `y = 0` membuat elipsnya **simetris**:
belok kiri dan kanan berbiaya identik, dan `Q[Y] = 20` menghukum keduanya. Itu
titik stasioner simetris, dan IPOPT diam di sana.

**Ini justru arsitektur yang benar** (bagian 2): MPC adalah **penjejak lintasan
dengan batas aman**, bukan penghindar halangan. Yang memutuskan lewat sisi mana
adalah planner, lewat lintasan acuan yang dihasilkannya. Menambahkan logika
menghindar ke MPC akan menduplikasi planner dan membuat dua modul memutuskan
pertanyaan yang sama.

Uji diganti jadi dua: (a) MPC memperlambat saat halangan masuk horizon, dan
(b) MPC menjejak lintasan yang dihasilkan `plan_lane_change` melewati halangan
itu — uji integrasi Tahap 3 + 5 yang menghasilkan XTE 0,074 m.

### Keputusan desain

**`ThrottlePI`, bukan tabel `throttle_map`.** Bagian 7.5 menawarkan keduanya. PI
mengukur percepatan aktual dan menyesuaikan sendiri: tidak butuh sapuan
kalibrasi di CARLA, ~5 baris, dan mengoreksi diri terhadap tanjakan, drag, atau
ganti kendaraan. Tiga titik data dari Tahap 1 (throttle 0,40 -> -2,3 m/s²;
0,56 -> ~0; 0,75 -> +2,8 m/s²) dipakai menebak gain awal, bukan membangun tabel.
Kalau terbukti kurang responsif, tabel dibangun belakangan.

**Constraint lajur (`|Y - Y_center| <= d_lane`) sengaja tidak dipasang.**
Bagian 7.2 mencantumkannya, tapi constraint keras tambahan berarti sumber
infeasibility tambahan — kalau mobil terdorong keluar batas, solver gagal dan
jatuh ke fallback. Biaya `Q[Y] = 20` sudah menarik mobil ke acuan, dan elips
menangani kendaraan lain. Pasang nanti sebagai constraint lunak kalau memang
terbukti perlu.

**Ambang slack `1e-3`.** IPOPT interior-point tidak pernah mendorong `eps` tepat
ke nol. Terukur 2,2e-06 saat lapang versus 0,999 saat benar-benar melanggar --
terpisah lima orde, jadi ambang di antaranya aman.

### Yang masih terbuka di Tahap 5

- Langkah 2: verifikasi empiris `steer_command` terhadap `get_wheel_steer_angle`
  di CARLA.
- Langkah 3: gain `ThrottlePI` masih tebakan, belum diuji di plant nyata.
- Langkah 4: tuning tertutup urutan bagian 7.6 belum dijalankan; bobot masih
  nilai awal bagian 7.3.
- Langkah 5: `main.py` dan video hasil kendali.
- Toleransi `test_turn_radius` di `tests/test_bicycle_model.py` masih 5%; Euler
  meleset 3,47% sehingga masih lolos. Perketat ke 1% supaya benar-benar mengunci
  RK4.

---

## Tahap 5 langkah 2-5 — kerangka lengkap, menunggu server

**Status: kode selesai, verifikasi di CARLA belum dijalankan** — 10 September 2026

Server CARLA mati saat bagian ini ditulis, jadi semua yang butuh simulator
belum dieksekusi. Kodenya siap dijalankan.

### Yang ditambahkan

| Berkas | Isi |
|---|---|
| `validate_model.py --steer` | verifikasi `steer_command` terhadap `get_wheel_steer_angle`, di 20 dan 50 km/jam, sekaligus menampilkan selisih terhadap rumus bagian 7.5 |
| `localization.PathFrame` | konversi frame dunia -> frame sejajar jalan |
| `localization.wrap` | normalisasi sudut ke (-pi, pi] |
| `perception.py` | `GroundTruthPerception` -> ndarray (M, 4) `[x, y, vx, vy]` |
| `main.py` | main loop lengkap, skenario S1, logging npz |

### Keputusan frame — satu konvensi untuk semua modul hilir

Planner bekerja di koordinat sejajar jalan (`s`, `d`), localization menghasilkan
koordinat dunia. Karena jalan lurus (bagian 5), itu **satu rotasi + translasi**.

`PathFrame` dibangun sekali di awal episode dari `reference_path`, lalu dipakai
di batas localization. Seluruh modul hilir (perception, planner, FSM, MPC)
bekerja di frame ini, jadi tidak ada dua konvensi yang harus dijaga sinkron.
Ditaruh di `localization.py` sesuai aturan 2.4.

**Satu pengecualian yang disengaja:** FSM memakai `x` **relatif** terhadap ego
(`_terdepan` menyaring `x > 0`), sedangkan planner dan MPC memakai `x`
**absolut** di frame jalan. Main loop menggeser satu baris sebelum memanggil
FSM. Mengubah tanda tangan FSM akan memaksa 17 uji ditulis ulang untuk keuntungan
yang tidak ada.

### Keputusan implementasi

**Percepatan terukur dari selisih kecepatan**, bukan `get_acceleration()`. Yang
dibutuhkan `ThrottlePI` adalah percepatan longitudinal yang tercapai;
`get_acceleration()` di frame dunia dan mengandung komponen normal.
`(v - v_prev)/dt` justru tepat, dan tidak menambah API yang framenya harus
diverifikasi lagi.

**`reference_path` sekarang mengembalikan (4, N)**, baris keempat `z` dalam frame
CARLA. Sebelumnya `main.py` menelusuri lajur dua kali — sekali untuk path, sekali
untuk ketinggian penempatan aktor. Dua penelusuran dengan `step` berbeda akan
menggeser posisi diam-diam.

**Kendaraan target dijaga dengan `set_target_velocity()` tiap tick**, bukan
autopilot. Deterministik, sesuai anjuran bagian 11.1.

### Yang harus dijalankan begitu server hidup

1. `python validate_model.py --steer` — konfirmasi penyebut normalisasi setir
2. `python main.py` — loop tertutup pertama; yang dilihat: XTE, waktu solve,
   jumlah solver gagal, urutan state FSM
3. Tuning urutan bagian 7.6, mulai dari `Q[Y]` dan `Q[psi]`
4. `record_maneuver.py` diubah sumber gerakannya ke `apply_control`

### Toleransi uji RK4 diperketat

`test_turn_radius` semula bertoleransi 5%; Euler meleset 3,47% sehingga tetap
lolos. Diperketat ke **1%** supaya benar-benar mengunci RK4.

---

## Tahap 5 langkah 2-5 — LOOP TERTUTUP BERHASIL

**Status: SELESAI** — 10 September 2026. Milestone bagian 8 tercapai: satu
skenario, kendaraan depan lambat, lajur kanan kosong, mobil menyalip dan kembali
tanpa tabrakan. **MPC benar-benar mengemudikan mobil.**

### Hasil run skenario S1 (20 detik, 400 tick)

| Metrik | Nilai |
|---|---|
| Deviasi dari tengah lajur saat `LANE_KEEPING` | **0,125 m rata-rata, 0,462 m maks** |
| Waktu solve | **29,0 ms rata-rata, 47,2 ms maks** (anggaran 50 ms) |
| Solver gagal | **0 dari 400** |
| Rentang kecepatan | 37-50 km/jam |
| Simpangan lateral | 0,00 -> -3,59 -> 0,00 m |
| Urutan state | `LANE_KEEPING -> CHECK_OVERTAKE -> LANE_CHANGE_OVERTAKE -> OVERTAKING -> LANE_CHANGE_RETURN -> LANE_KEEPING` |

Gambar: `out/run_s1_mpc_gt.png` (empat panel: lateral, kecepatan, kemudi, waktu
solve; latar berwarna menandai state FSM). Log mentah: `out/run_s1_mpc_gt.npz`.

---

### TEMUAN 1: `steer_command` kehilangan pembalikan tanda *(paling serius)*

**Gejala run pertama:** XTE rata-rata 2,476 m, maks 13,9 m. Mobil melenceng
empat lajur lalu berhenti dengan kecepatan **-9 km/jam** (mundur). 29 solver
gagal.

**Penelusuran:** pada t=16 s, `y = -4,23` (mobil di kiri acuan) tapi
`delta = +0,4663`. Di frame right-handed, delta positif = belok kiri = `y` naik.
Tapi `y` justru terus turun -- perintah dan akibat berlawanan.

**Sebab:** delta right-handed positif = belok **kiri**; steer CARLA positif =
belok **kanan**. `steer_command` tidak menegasikan. MPC minta koreksi ke kiri,
mobil belok kanan, error membesar, MPC minta lebih keras: umpan balik positif.

**Kenapa lolos dari uji:** `validate_model --steer` membandingkan `delta` yang
diminta dengan sudut roda **mentah CARLA**, bukan yang sudah dikonversi ke
right-handed. Salah tanda ada di dua tempat sekaligus sehingga saling menutupi.
`wheel_delta()` di fungsi `run()` sudah benar menegasikan; uji baru tidak
mengikutinya.

**Solusi:** `steer = -delta / (delta_max_phys x curve(v))`. Uji `--steer`
diperbaiki agar membandingkan di frame yang sama, plus uji unit
`test_steer_membalik_tanda` yang mengunci tandanya.

**Pelajaran:** uji yang menyalin konvensi dari kode yang diujinya tidak menguji
apa-apa. Bug ini hanya muncul di loop tertutup, karena open-loop tidak punya
umpan balik yang bisa meledak.

---

### TEMUAN 2: percepatan terukur terlalu berderau untuk dipakai langsung

**Gejala:** kecepatan ambles 50 -> 7 km/jam pada t=12 s padahal celah ke
kendaraan depan masih 42 m. Bukan penghindaran.

**Ukuran:** `a_ukur = (v - v_prev)/dt` di 20 Hz menghasilkan **-26,8 sampai
+12,8 m/s²**, simpangan baku 3,53, lompatan antar-tick sampai 19,9 -- padahal
batas fisiknya -6 sampai +3. **23 dari 400 tick di luar batas fisik.**

Derau itu masuk ke dua tempat: syarat batas percepatan awal quartic, dan
pengukuran `ThrottlePI`. MPC lalu **setia mengejar acuan yang dibangun dari
derau**.

**Solusi, dua bagian:**

1. `a0` quartic memakai percepatan yang **diperintahkan** tick sebelumnya
   (`cmd.accel_cmd`), bukan hasil ukur. Perintah MPC halus karena suku `Rd`.
2. `ThrottlePI` memakai hasil ukur yang **dipotong ke batas fisik lalu
   dilewatkan low-pass** (`A_FILTER_ALPHA = 0,2`, tetapan waktu ~0,25 s).

---

### TEMUAN 3: batas kecepatan di k=0 membuat masalah infeasible

**Gejala:** 7 solver gagal, semuanya `Infeasible_Problem_Detected`, mengelompok
di `LANE_CHANGE_RETURN`. Sempat dikira batas iterasi; menaikkan `MPC_MAX_ITER`
100 -> 300 tidak menolong sama sekali.

**Sebab:** **setiap** tick yang gagal punya `v > V_MAX` -- sekecil
**0,0003 m/s**. Constraint `X[:,0] == x0` mengunci state awal ke hasil ukur,
lalu `X[3,0] <= V_MAX` bertabrakan dengannya. Masalahnya jadi infeasible karena
**melarang masa lalu**.

**Solusi:** batas state berlaku mulai `k = 1`. State awal adalah pengukuran,
bukan variabel yang bisa dipilih solver.

Setelah akar penyebabnya diperbaiki, `MPC_MAX_ITER` dikembalikan ke 100 dan
waktu solve turun (maks 56 -> 47 ms). Kenaikan batas iterasi memang cuma
tambalan gejala.

Dikunci uji `test_kecepatan_sedikit_di_atas_v_max_tetap_terpecahkan`.

---

### TEMUAN 4: XTE yang diukur pertama kali mengukur nol

Run kedua melaporkan XTE 0,001 m -- mencurigakan bagus. Sebabnya XTE dihitung
terhadap `xref[:,0]`, sedangkan planner **meng-anchor lintasan di posisi ego
saat itu**. Selisih maksimumnya 0,0059 m: metriknya mengukur konstruksi, bukan
kinerja.

**Solusi:** catat juga `y_goal` (tengah lajur yang dituju FSM) dan deviasi
terhadapnya. Angka yang bermakna: **0,125 m rata-rata, 0,462 m maks** saat
`LANE_KEEPING`.

Definisi metrik final ditetapkan saat menulis bab 4; sesuai bagian 11.6 log
disimpan mentah supaya tidak perlu menjalankan ulang.

---

### TEMUAN 5: ego harus mulai di kecepatan operasi

Ego mulai dari diam butuh **7,3 detik** mencapai 47 km/jam, dan selama itu
kendaraan target sudah maju 51 m. Celah tidak pernah turun di bawah 33,8 m
sehingga FSM tidak pernah terpicu. Bagian 11.1 memang menetapkan `ego_v0`;
sebelumnya belum dipasang. `EGO_V0 = 13,9` diset lewat `set_target_velocity`.

---

### Verifikasi `steer_command` di CARLA (`validate_model --steer`)

Setelah tanda diperbaiki dan kecepatan ditahan tiap tick:

| Kecepatan | delta minta | delta nyata | Error relatif |
|---|---|---|---|
| 20,2 km/jam | 0,0200 | 0,0199 | -0,5% |
| 20,2 km/jam | 0,1000 | 0,0975 | -2,5% |
| 50,0 km/jam | 0,0200 | 0,0199 | -0,5% |
| 50,0 km/jam | 0,2000 | 0,1925 | -3,75% |

Meleset 0,5-4% dan membesar seiring sudut -- konsisten dengan nonlinearitas
aktuator, bukan kesalahan penyebut. Bandingkan rumus bagian 7.5: untuk delta
0,02 rad ia memberi `steer = 0,04` yang menghasilkan **0,0489 rad**, meleset
**2,4x**.

Catatan: `set_target_velocity` harus dipanggil **tiap tick**. Dipanggil sekali
lalu mengandalkan throttle membuat mobil melambat dan kurva dievaluasi pada
kecepatan yang salah -- uji pertama sempat memberi error +15-20% karena ini.

---

### Yang terlihat dari gambar dan perlu dituning (langkah 4, bagian 7.6)

1. **Kemudi bergerigi.** Panel delta menunjukkan riak frekuensi tinggi amplitudo
   ~0,001 rad. Bagian 7.6: "kemudi bergerigi hampir selalu berarti `Rd[delta]`
   terlalu kecil". Kandidat pertama untuk dinaikkan dari 20.
2. **Overshoot saat kembali.** `y` melewati tengah lajur sampai -0,46 m lalu
   kembali -- tracking kurang teredam. Tuning `Q[Y]` dan `Q[psi]`.
3. **Dip kecepatan startup** ke 37 km/jam pada t~1,8 s, transien awal.
4. Deviasi 0,462 m saat `LANE_KEEPING` masih bisa diperkecil.

Tuning belum dijalankan; seluruh bobot masih nilai awal bagian 7.3.

---

### TEMUAN 6: transien start memakan 22% run dan merusak metrik

**Pertanyaan yang memicu:** kenapa kedua kendaraan langsung mulai di kecepatan
targetnya?

**Ukuran:** `set_target_velocity` menetapkan kecepatan **bodi**, tapi **roda
masih diam**. Slip longitudinal besar menghasilkan gaya pengereman:

| | |
|---|---|
| Dip terdalam | 36,8 km/jam pada t = 1,85 s |
| Mapan kembali (\|v - v0\| < 0,2 m/s) | t = 4,4 s |
| Porsi run 20 detik | **22%** |
| Deviasi lateral selama transien | 0,005 m (murni masalah memanjang) |

**Solusi: fase pemanasan sebelum pencatatan.** Ego dijalankan `WARMUP_DETIK = 6`
dengan pipeline lengkap tapi tanpa logging. Kendaraan target baru di-spawn
setelah itu, tepat `GAP_AWAL` di depan posisi ego yang **sebenarnya**.

**Dampak pada metrik:**

| | Sebelum | Sesudah |
|---|---|---|
| Deviasi lajur rata-rata | 0,125 m | **0,041 m** |
| Deviasi lajur maks | 0,462 m | **0,205 m** |
| Rentang kecepatan | 37-50 km/jam | **46-50 km/jam** |
| Waktu solve tick pertama | 429 ms | **30 ms** |
| Simpangan lateral puncak | -3,59 m | -3,47 m |

**Deviasi turun 3x.** Sebagian besar yang tadinya dilaporkan sebagai error
tracking ternyata transien start, bukan kinerja controller. Melaporkan 0,125 m
akan merugikan hasil sendiri.

Efek samping: graf CasADi ikut terbangun saat pemanasan, sehingga outlier 429 ms
di tick pertama hilang dari statistik waktu solve.

Bonus ketepatan: jarak awal sekarang **tepat** `GAP_AWAL`. Sebelumnya target
di-spawn relatif `x = 0` sedangkan ego mulai di `x = -1,53`, jadi jarak
sebenarnya 61,5 m -- bagian 11.1 minta jarak awal ditetapkan per skenario.

### Kendaraan target dipaksa kecepatannya tiap tick — disengaja

`set_target_velocity` dipanggil tiap tick untuk kendaraan target, jadi dia tidak
benar-benar dikemudikan, melainkan digerakkan paksa. **Ini sah dan disengaja**: perannya cuma
halangan yang bergerak di kecepatan yang diketahui, dinamika internalnya tidak
relevan, dan memaksa kecepatan memberi nilai persis tanpa variansi -- persis yang
diminta bagian 11.1 untuk reproducibility. **Nyatakan eksplisit di metodologi.**

**Untuk skenario S5** (kendaraan depan mengerem mendadak) ini tidak cukup.
Sarannya tetap `set_target_velocity` tapi dengan **profil kecepatan terjadwal**:
tetap deterministik dan waktu pengereman terkontrol persis. Lebih baik daripada
TrafficManager yang menambah sumber nondeterminisme.

---

### TEMUAN 7: `v_ref` dan `v_max` sempat disetel sama — constraint jadi setpoint

**Pertanyaan yang memicu:** berapa kecepatan kendaraan yang disalip, dan apakah
kecepatan awal ego benar-benar tetap?

Jawabannya: kendaraan target **7,0 m/s = 25,2 km/jam** (skenario S1 bagian 11.3),
ego **13,9 m/s = 50,0 km/jam**. Tapi pemeriksaan itu memunculkan masalah lain.

**Gejala:** `EGO_V0`, `V_MAX`, dan kecepatan referensi yang dikirim ke planner
semuanya diset 13,9 -- angka yang sama persis.

| | |
|---|---|
| Tick dalam 0,5 km/jam dari batas | **260 dari 400 (65%)** |
| Tick yang melewati batas | 1 |
| `a_cmd` minimum | -0,00 m/s² -- praktis tidak pernah mengerem |

Mobil menempel terus di dinding constraint. Constraint keras yang aktif 65%
waktu membuat solver bekerja di tepi kelayakan -- itu yang menyebabkan bug
infeasible di k=0 (Temuan 3). Constraint seharusnya **batas keselamatan, bukan
setpoint**. Bagian 0.2 sendiri sudah memisahkan keduanya (`v_ref` ego 40-50
km/jam, `v_max` constraint 13,9); implementasinya yang menggabungkan.

**Solusi:** `V_REF = 13,4 m/s (48,2 km/jam)` dipisah dari `V_MAX = 13,9`.
`EGO_V0 = V_REF`.

### Determinisme terverifikasi

Tiga run berturut-turut dengan konfigurasi identik menghasilkan angka
**identik bit-per-bit** (deviasi 0,126 m, lateral -4,02 m, dst). Tidak ada
nondeterminisme solver pada skala ini. Bagian 11.1 terpenuhi, dan artinya
perbedaan antar-konfigurasi bisa dibaca sebagai efek nyata.

### Metrik menyesatkan untuk kedua kalinya

Sapuan `V_REF` menunjukkan tren monoton: makin rendah, makin buruk deviasinya.

| `V_REF` | km/jam | Deviasi rata-rata | Deviasi maks | Lateral puncak |
|---|---|---|---|---|
| 13,9 | 50,0 | 0,041 m | 0,205 m | -3,47 m |
| 13,7 | 49,3 | 0,108 m | 0,313 m | -3,31 m |
| 13,4 | 48,2 | 0,126 m | 0,488 m | -4,02 m |
| 13,0 | 46,8 | 0,132 m | 0,521 m | -3,98 m |

Dugaan pertama -- "constraint mengunci kecepatan sehingga tracking lebih stabil"
-- **salah**: variansi kecepatan hampir identik (0,308 vs 0,314 m/s), dan
`V_REF = 13,4` justru melacak referensi lebih baik (error 0,048 vs 0,223 m/s).

Jawabannya terlihat setelah jumlah kandidat planner ikut dicatat (diagnostik
bagian 11.5):

| | `V_REF` 13,9 | `V_REF` 13,4 |
|---|---|---|
| Kandidat lolos saat manuver | 0-9 | 3-9 |
| **Tick tanpa kandidat sama sekali** | **4** | **0** |
| Offset terpilih | 3,0 dan 3,5 | konsisten 3,5 |

Pada 13,9 planner **empat kali kehabisan kandidat**, mengembalikan `None`,
sehingga acuan jatuh ke mode tahan-lajur dan mobil tertarik balik ke tengah.
Ditambah kadang memilih offset 3,0 yang lebih sempit. Angka deviasi yang
"lebih bagus" itu sebagian **akibat planner gagal**, bukan kendali lebih baik.

**Kesimpulan: `V_REF = 13,4` yang benar.** Yang tersisa adalah overshoot tracking
0,52 m, dan itu memang target tuning bagian 7.6.

**Pelajaran, dan ini kedua kalinya hari ini:** angka metrik bisa membaik karena
alasan yang salah. Pertama XTE yang mengukur konstruksi (Temuan 4), sekarang
deviasi yang membaik karena kegagalan planner. Selalu periksa *kenapa* sebuah
angka membaik, jangan cuma bahwa ia membaik.

### Kolom log bertambah

`n_layak` (jumlah kandidat lolos) dan `offset` (offset terpilih) kini ikut
dicatat per tick -- bagian 11.5 menyebut keduanya sebagai diagnostik apakah
sampling `T` dan `y_target` cocok. Kalau sering nol, samplingnya yang salah.

### Status akhir Tahap 5

| Metrik | Nilai |
|---|---|
| Deviasi tengah lajur saat `LANE_KEEPING` | 0,126 m rata-rata, 0,488 m maks |
| Waktu solve | 28,1 ms rata-rata, 49,2 ms maks |
| Solver gagal | 0 dari 400 |
| Kandidat planner nol | 0 tick |
| Uji otomatis | 46 lolos (3 + 17 + 15 + 11) |

Seluruh bobot MPC masih nilai awal bagian 7.3 -- **tuning bagian 7.6 belum
dijalankan sama sekali.**

---

## Harness Tuning (`tuning.py`)

Dibuat 10 September 2026, sebelum tuning bagian 7.6 dimulai.

### Kenapa harness terpisah, bukan tuning di `main.py`

Skenario S1 penuh tidak bisa dipakai untuk tuning. Mengubah satu bobot ikut
mengubah kapan FSM terpicu, kandidat mana yang lolos, dan berapa lama tiap fase
berlangsung -- efek bobot tidak bisa dipisahkan dari efek skenario.

Bagian 7.6 sendiri menetapkan syarat langkah pertama: *"matikan constraint
tabrakan, `x_ref` = garis lurus"*. `tuning.py` memenuhi itu secara harfiah.

Alasan tambahan: metrik sudah **dua kali menyesatkan** dalam sesi ini (XTE yang
mengukur konstruksinya sendiri, lalu deviasi yang membaik karena planner gagal).
Tuning di atas metrik yang belum bisa dipercaya hanya menghasilkan bobot salah
dengan percaya diri.

### Isi

Step response lateral: ego dimapankan di `y = 0` selama fase pemanasan, lalu
acuan **dilompatkan** ke `y = step` tepat pada `t = 0`. Tanpa halangan, tanpa
FSM, tanpa planner -- acuannya garis lurus tetap.

Lima metrik per konfigurasi:

| Metrik | Arti |
|---|---|
| `overshoot%` | sejauh mana `y` melewati target, searah gerakan |
| `settling s` | waktu sampai `\|e\|` menetap di bawah 5% besar lompatan |
| `sisa m` | rata-rata `\|e\|` pada 1 detik terakhir |
| `jitter mrad` | rata-rata `\|delta_k - delta_{k-1}\|` -- **angka untuk "kemudi bergerigi"** |
| `delta maks` | apakah menyentuh batas kemudi |

Plus waktu solve dan jumlah solver gagal.

`jitter` adalah yang paling penting: bagian 7.6 menyuruh menaikkan `Rd[delta]`
"sampai kemudi tidak bergetar", tapi tidak memberi ukuran. Ini ukurannya.

### Override bobot

`MPCController.__init__` menerima argumen `bobot` yang menimpa nilai `config`
tanpa mengeditnya:

```
python tuning.py --sweep RD_DELTA 5,20,80,200
python tuning.py --sweep Q_Y 5,20,60 --step -1.5
```

Nama yang dikenal: `Q_X Q_Y Q_PSI Q_V R_A R_DELTA RD_A RD_DELTA QF RHO`.
Satu entri berubah, sisanya tetap dari config -- dikunci uji di `tuning.py`.

Satu ego dipakai ulang untuk seluruh sapuan; posisinya di-reset ke spawn dan
dipanaskan lagi tiap konfigurasi, jadi titik awalnya identik.

### Belum dijalankan

Server CARLA mati saat harness selesai ditulis. Fungsi murninya sudah diuji
(override bobot, dan metrik terhadap sinyal underdamped buatan: overshoot 23,4%,
settling 0,75 s, sisa 0,0000 m -- masuk akal).

**Urutan yang akan dijalankan begitu server hidup**, sesuai bagian 7.6:

1. `--sweep Q_Y` lalu `--sweep Q_PSI` -- sampai tracking rapat
2. `--sweep RD_DELTA` -- sampai `jitter` turun; ini yang paling sering perlu naik
3. `--sweep Q_V` dan `--sweep R_A` -- tracking kecepatan
4. Baru aktifkan constraint elips dan uji di `main.py`

### Perbaikan sampingan

**Pengurutan halangan diperbaiki.** `_obstacle_matrix` mengurutkan dengan
`hypot(x_absolut, y)` -- itu jarak dari **titik asal path**, bukan dari ego.
Bedanya baru terlihat saat halangan mengapit ego: urutan absolut menyimpan yang
di belakang dan membuang yang di depan. Belum berdampak di S1 (satu kendaraan
saja) tapi S3 punya dua. Sekarang memakai jarak dari ego, dikunci uji.

**`ddy` diperiksa, ternyata bersih.** Sempat dicurigai berderau seperti `a_ukur`
(rentang -26,8..+12,8 m/s²), tapi terukur hanya -0,49..+0,62 m/s² dengan sd
0,236. Sebabnya `dy = v*sin(yaw)` dan `yaw` datang langsung dari transform,
bukan hasil turunan numerik. Tidak perlu difilter.

**`siapkan_jalan()` diekstrak** ke `main.py` dan dipakai bersama `tuning.py`.

---

## Tuning bagian 7.6 — langkah 1 dan 2

Dijalankan 10 September 2026 dengan `tuning.py`. Step response lateral -1,0 m,
acuan garis lurus, tanpa halangan, tanpa FSM, 48 km/jam.

### Langkah 1 — `Q[Y]` dan `Q[psi]`

Sapuan pertama `Q_Y` memberi hasil **berlawanan intuisi**: makin kecil `Q_Y`,
makin bagus semua metrik.

| `Q_Y` | overshoot | settling | jitter |
|---|---|---|---|
| 0,5 | 4,7% | 1,05 s | 2,53 mrad |
| 5 | 22,3% | 1,95 s | 5,61 mrad |
| 20 (proposal) | 26,4% | 2,35 s | 7,77 mrad |
| 60 | 27,6% | 2,35 s | 9,85 mrad |

Itu ciri sistem **kurang teredam**: `Q_Y` menghukum error posisi, dan tanpa
redaman yang cukup koreksinya jadi agresif lalu overshoot. Tapi `Q_Y = 0,5`
berarti error lateral dibobot lebih ringan daripada error memanjang -- bertentangan
dengan premis bagian 7.3.

**Hipotesis: yang menentukan rasio `Q_psi : Q_Y`, bukan `Q_Y` sendirian.**
Diuji dengan menyapu `Q_psi` pada `Q_Y` default:

| `Q_psi` | overshoot | settling | jitter |
|---|---|---|---|
| 10 (proposal) | 26,4% | 2,35 s | 7,76 mrad |
| 100 | 16,1% | 1,40 s | 5,58 mrad |
| 300 | 3,3% | 0,90 s | 4,03 mrad |
| 600 | 0,0% | 1,25 s | 3,18 mrad |
| 1200 | 0,0% | 1,80 s | 2,53 mrad |
| 2400 | 0,0% | 2,50 s | 1,96 mrad |

Terkonfirmasi, dan **lebih baik daripada menurunkan `Q_Y`**: `Q_psi = 300`
memberi overshoot 3,3% versus 4,7% pada `Q_Y = 0,5`, dengan wewenang lateral
utuh. Di atas 600 overshoot tetap nol tapi settling memanjang -- terlalu teredam.

Penyempitan:

| `Q_psi` | overshoot | settling |
|---|---|---|
| 350 | 1,4% | 0,95 s |
| **450** | **0,0%** | **1,05 s** |
| 550 | 0,0% | 1,15 s |
| 650 | 0,0% | 1,30 s |

**`Q_psi = 450`** -- nilai pertama dengan overshoot nol dan settling tercepat di
antara yang nol.

`Q_Y` lalu **disapu ulang** dengan redaman yang benar:

| `Q_Y` | overshoot | settling |
|---|---|---|
| 5 | 0,0% | 2,20 s |
| **20** | **0,0%** | **1,05 s** |
| 28 | 1,9% | 0,95 s |
| 60 | 11,6% | 1,35 s |

**`Q_Y = 20` -- nilai asli bagian 7.3 -- ternyata optimal.** Dari dua bobot di
langkah ini, satu salah 45x dan satu sudah benar sejak awal.

Hasil langkah 1:

| | proposal | tertuning |
|---|---|---|
| `Q_psi` | 10 | **450** |
| `Q_Y` | 20 | 20 (tetap) |
| Overshoot | 26,4% | **0,0%** |
| Settling | 2,35 s | **1,05 s** |

### Langkah 2 — `Rd[delta]`: ternyata tidak perlu diubah

Sapuan awal terlihat menjanjikan (jitter 3,47 -> 2,42 mrad) tapi overshoot naik
0 -> 4,4%. Kecurigaan: **metriknya kurang tajam** -- `jitter` dirata-ratakan
sepanjang run termasuk **ramp awal**, padahal belokan cepat saat merespons step
itu koreksi yang sah, bukan getaran.

Metrik dipisah jadi `chatter` (hanya 1 detik terakhir, setelah mapan) dan
`jitter tot`:

| `Rd[delta]` | overshoot | chatter | jitter tot |
|---|---|---|---|
| **20 (proposal)** | **0,0%** | **0,0008 mrad** | 3,470 |
| 80 | 0,5% | 0,0001 | 3,289 |
| 300 | 2,5% | 0,0001 | 3,028 |
| 1000 | 4,4% | 0,0002 | 2,419 |

**Chatter praktis nol di semua nilai** -- 0,0008 mrad itu derau numerik. Tidak
ada masalah getaran di step response sama sekali. **`Rd[delta] = 20` tetap.**

### Riak kemudi di skenario penuh berasal dari planner, bukan MPC

Grafik skenario S1 memang menunjukkan riak, tapi sumbernya bukan MPC:

| | chatter |
|---|---|
| Step response (acuan garis lurus tetap) | 0,0008 mrad |
| Skenario S1 saat `LANE_KEEPING` | 0,3378 mrad |
| Rasio | **422x** |

MPC-nya identik di keduanya. Bedanya acuan: harness memakai garis lurus **tetap**,
skenario penuh **membangun ulang** lintasan tiap 100 ms dari state ego saat itu.

Bagian 2 sudah meramalkan ini: *"regenerasi lintasan tiap tick membuat x_ref
sedikit berbeda setiap kali, dan karena bobot Q besar pada error lateral, kemudi
ikut bergerak mengejar referensi yang bergeser."* Itu juga alasan planner
dijalankan 10 Hz, bukan 20 Hz.

**Kesimpulan: `Rd[delta]` knob yang salah untuk riak ini.** Besarnya 0,34 mrad =
0,7% dari `DDELTA_MAX`, setara 0,02° di roda. Tidak perlu ditangani.

**Pelajaran, kali ketiga:** metrik yang mencampur dua fenomena akan menuntun ke
knob yang salah. Memisahkan chatter dari ramp mengubah kesimpulan langkah 2
sepenuhnya -- dari "naikkan Rd" jadi "jangan sentuh Rd".

### Dampak di skenario penuh

| | sebelum tuning | sesudah |
|---|---|---|
| Waktu solve rata-rata | 28,1 ms | **24,0 ms** |
| Waktu solve maks | 49,2 ms | **38,5 ms** |
| Lateral puncak | -4,02 m | **-3,93 m** |
| Solver gagal | 0 | 0 |

### Belum dikerjakan

Langkah 3 bagian 7.6 (`Q[v]` dan `R[a]` untuk tracking kecepatan). Catatan:
kecepatan mengembara +-2 km/jam dan penyebabnya kemungkinan **throttle PI yang
lamban**, bukan bobot MPC. Menyetel `Q[v]` sebelum PI-nya benar berarti menyetel
parameter yang salah -- periksa PI dulu.

---

## Kontrak Perception Diperbaiki + Sensor Tabrakan

10 September 2026.

### Kontrak perception: frame ego, bukan frame jalan

**Masalah:** `GroundTruthPerception` melaporkan koordinat **frame jalan absolut**,
sedangkan bagian 2 menetapkan kontraknya **frame ego**. Untuk GT itu tidak
masalah -- dia membaca posisi semua kendaraan dari CARLA jadi memang tahu
semuanya.

Tapi kamera hanya bisa frame ego: dia melihat kotak di gambar, menghitung
jaraknya lewat depth, dapat "30 meter di depan saya". Dia **tidak tahu** dirinya
berada di meter ke berapa. Kalau kontraknya frame jalan, `VisionPerception`
terpaksa menanyakan posisi ego lalu menjumlahkan sendiri -- artinya modul
perception harus tahu-menahu soal frame jalan, padahal tugasnya cuma melaporkan
apa yang dilihat.

**Solusi:** perception mengeluarkan `(M, 4) = [x, y, vx, vy]` **frame ego**,
posisi **dan kecepatan** relatif terhadap ego. Konversi ke frame jalan pindah ke
`localization.halangan_ego_ke_jalan()` -- rotasi menurut `ego.yaw`, translasi
menurut posisi ego, dan penambahan kecepatan ego (bagian 10.4 menyebut kecepatan
dari perception memang relatif).

`GroundTruthPerception` sekarang sengaja melaporkan **besaran yang sama** dengan
yang nanti dihasilkan `VisionPerception`, supaya perbandingan keduanya di bagian
11.4 setara -- bukan membandingkan informasi berbeda jenis.

**Biaya kalau ditunda:** sekarang 10 baris di dua file yang sama-sama baru
ditulis. Setelah `VisionPerception` ada, yang tersentuh planner, MPC, FSM,
ditambah adaptor di dalam perception.

### Sensor tabrakan (`evaluation.py`)

Berkas baru, sesuai nama yang dicadangkan bagian 2.1.

**Tugas:** memasang `sensor.other.collision` pada ego, mencatat waktu, aktor
yang ditabrak, dan besar impuls tiap kejadian.

**Batas — tidak pernah dibaca `control.py` maupun `planning.py`.** Sensor ini
instrumen pengukuran untuk menilai run, bukan masukan kendali. Kendaraan uji
tidak punya akses terhadapnya.

**Tidak menghentikan simulasi.** Run yang menabrak tetap berjalan sampai selesai;
menabrak adalah HASIL yang dicatat, bukan error. Menghentikan simulasi berarti
kehilangan data setelah tabrakan.

Kalimat untuk batasan masalah:

> Sensor tabrakan CARLA digunakan sebagai instrumen pengukuran untuk menilai
> keberhasilan setiap percobaan, bukan sebagai masukan bagi sistem kendali.
> Kendaraan uji tidak memiliki akses terhadap informasi tersebut.

**Ditegakkan otomatis** oleh `tests/test_arsitektur.py`: memindai `planning.py`
dan `control.py` dengan AST untuk impor terlarang (`carla`, `simulation`,
`perception`, `evaluation`, `localization`, `main`), dan memastikan string
`collision` tidak muncul di ketiga modul jalur kendali.

Aturan bagian 2.4 sekaligus menegakkan batas ini secara **struktural**:
`control.py` tidak bisa membaca sensor tabrakan bahkan kalau ada yang mencoba,
karena dia tidak boleh menyentuh CARLA sama sekali.

### TEMUAN: waktu solve bergantung beban mesin, bukan hanya kode

**Gejala:** setelah dua perubahan di atas, waktu solve melonjak 29,1 -> 59,9 ms
rata-rata dengan maksimum 92,3 ms. Reproducible di 54-60 ms, dan determinisme
antar-run ikut hilang.

**Penelusuran:**

| Yang diuji | Waktu solve |
|---|---|
| Dengan sensor tabrakan | 53,7-59,9 ms |
| **Tanpa** sensor tabrakan | 49,5-52,2 ms |
| `tuning.py` (tanpa perception, tanpa sensor, tanpa halangan) | **70,4 ms** |
| `tuning.py` beberapa jam sebelumnya, kode identik | **26-31 ms** |

Baris ketiga dan keempat menentukan: kode yang sama persis, 2,3x lebih lambat.
Load average 3,75 dan CARLA memakai 141% CPU setelah berjalan berjam-jam.

**Kesimpulan: artefak pengukuran, bukan regresi.** Hasil kendali tetap
deterministik (deviasi 0,120 m dan lateral -3,98 m identik antar-run); hanya
waktunya yang bergoyang.

**Akibat untuk skripsi:** bagian 11.5 meminta waktu solve dilaporkan sebagai
klaim real-time. Angka itu **bukan sifat algoritma** melainkan sifat algoritma
**pada mesin tertentu dalam kondisi tertentu**. Ukur di mesin senggang, dan
laporkan kondisinya.

**Akibat untuk uji:** `test_waktu_solve_di_bawah_anggaran` gagal karena beban,
bukan karena kode. Uji yang gagal karena lingkungan lebih buruk daripada tidak
ada uji -- ia melatih orang mengabaikan kegagalan. Diganti
`test_jumlah_iterasi_solver_wajar` yang meng-assert **jumlah iterasi IPOPT**
(murni algoritmik, tidak terpengaruh beban). Waktu solve tetap dicetak sebagai
informasi.

---

## Keputusan lingkup: Tahap 7 dibatalkan

10 September 2026, keputusan penulis. Baseline Pure Pursuit/Stanley tidak
dikerjakan.

Konsekuensi dan apa yang tetap dimiliki skripsi dicatat di
`RANGKUMAN_PENULISAN.md` bagian 13. Ringkasnya: sumbu perbandingan bergeser
sepenuhnya ke **GT versus vision perception** (bagian 11.4 baris 1 dan 2), yang
untuk skripsi berpipeline YOLOPX justru lebih pusat.

Matriks eksperimen bagian 11.4 dan daftar metrik bagian 11.5 perlu disesuaikan
saat menulis bab 3 -- hapus baris baseline, jangan biarkan tertinggal di
proposal.

---

## Pindah Mesin: Race Perintah, PI Split-Range, Verifikasi Ulang Tuning

11 September 2026. Repo di-clone ke mesin baru (RTX 5060, server CARLA
`-quality-level=Low`). Rincian angka di `TUNING_MPC.md` bagian 10-11.

**Setup.** venv di `.venv` (uv, versi `requirements.txt` persis). torch dan
torchvision milik sistem (`~/.local`) tidak disentuh; dibuat terlihat dari venv
lewat berkas `.pth` yang ditaruh SETELAH site-packages venv, supaya numpy 2.2.6
milik venv yang menang. 57 uji lolos.

### TEMUAN: klaim "deterministik bit-per-bit" tidak berlaku

Lima run `main.py` berkonfigurasi identik: lima hasil berbeda sejak tick pertama
(kecepatan ego setelah pemanasan 11,92-13,21 m/s), satu **gagal
`lane_departure`** -- FSM terjebak di `CHECK_OVERTAKE`, ego membelok ke kiri
sampai y = +2,05 m dan melewati target dengan jarak bodi 0.

Penelusuran berurutan:

| Hipotesis | Uji | Hasil |
|---|---|---|
| World perlu di-reload antar-episode | reload -> run, x2 | tetap berbeda -- gugur |
| Fisika CARLA tidak deterministik | throttle tetap tanpa pipeline, 4 run | selisih 0 -- gugur |
| IPOPT berbatas waktu CPU | periksa opsi solver | tidak ada -- gugur |
| Perintah asinkron balapan dengan tick | kirim lewat `apply_batch_sync` dulu | **3 run identik** |

Tanda di log run gagal: throttle berselang-seling 0,37/0,19 tiap tick (rata-rata
perubahan 0,091 vs 0,014 run normal) -- ciri perintah yang berlaku satu frame
terlambat.

**Kenapa di mesin lama tampak deterministik:** tidak diketahui. Race bergantung
penjadwalan thread server; mesin berbeda, peluangnya berbeda. Pelajarannya:
determinisme yang tidak ditegakkan oleh kode hanyalah kebetulan.

**Perbaikan:** `simulation.tick(world, perintah)` -- satu-satunya jalan perintah
aktor. `tests/test_arsitektur.py` menolak `.apply_control(`,
`.set_target_velocity(`, `.set_transform(` di luar `simulation.py`.

### TEMUAN: optimum `kp = 0,3` adalah artefak

`ThrottlePI` lama me-reset integrator pada setiap `a_ref < 0`. Satu permintaan
-0,0016 m/s² memutus throttle 0,38 -> 0,02. `kp = 0,3` "optimal" hanya karena
`a_cmd`-nya kebetulan tidak pernah negatif. Diganti logika split-range (rem hanya
bila throttle jenuh di nol, integrator dibekukan). Setelah itu `kp` datar di
0,035-0,3; dipilih **0,14 = 1/gain plant terukur**. `ki = 0,25`, `Q_psi = 450`,
`Q_Y = 20`, `Rd_delta = 20` terkonfirmasi ulang.

Plant throttle -> percepatan terukur sangat nonlinier: throttle 0 meluncur
-0,10 m/s² (netral), throttle 0,1 justru -3,40 m/s² (turun ke gigi 1, engine
braking).

### TEMUAN: deviasi `LANE_KEEPING` mengukur ekor manuver

Sebelum manuver 0,001 m, setelah kembali 0,21-0,27 m. Metrik README 0,120 m
hampir seluruhnya ekor. Metrik menyesatkan keempat.

### Yang masih terbuka

- **FSM terjebak di `CHECK_OVERTAKE`** tanpa perilaku mengikuti kendaraan depan.
  Tidak terpicu di S1 setelah race diperbaiki, tapi S3 (lajur tujuan terisi)
  pasti memicunya.
- Overshoot saat kembali ke lajur naik +0,15 -> +0,31 m setelah kecepatan
  tertahan lebih rapat. Masih < 0,5 m, belum dijelaskan.
- `validate_model.py --steer` baris 20 km/jam, delta 0,20: kecepatan terbaca
  -4,7 km/jam. Belum diperiksa; baris 50 km/jam meleset <= 1,5%.
- `out/model_validation.csv` dibangkitkan ulang dengan kode baru: selisih
  <= 1,6e-5, angka ringkasan identik (0,309 m @ 2 s, 0,538 m akhir).

---

## FSM: Mengikuti Kendaraan Depan dan Menyalip Ulang (S3)

11 September 2026. Keputusan penulis: setelah mengikuti, ego **boleh mencoba
menyalip lagi**. Rincian dan angka di `TUNING_MPC.md` bagian 12.

**Yang diubah (`planning.BehaviorFSM`):** `v_goal` mengikuti kendaraan depan bila
menyalip tidak mungkin; pemicu dan batal memakai `max(v_ego, V_REF)` supaya ego
yang sudah melambat tetap bisa memicu menyalip. Hasilnya perilaku *accelerative
overtaking*. `main.py` kini menjalankan skenario dari `config.SKENARIO`.

**Jalan buntu yang ditempuh, berurutan:**

1. **Hukum akar `dv = sqrt(2ae)`** -- kebablasan: celah turun ke 14,3 m (d* 17,5),
   ego mundur ke 4,7 m/s di belakang kendaraan 7 m/s. Mengabaikan jeda quartic
   planner 3-4 s. Diganti hukum linier `2e/T` yang diturunkan dari planner itu
   sendiri; konstanta `A_IKUT` hilang.
2. **Tabrakan guardrail di t ~ 30 s** -- bukan FSM: run 35 detik melewati ujung ruas
   lurus 400 m. FSM lama menabrak di titik yang sama. S3 dijalankan 25 detik.
3. **Jarak 0,87 m ke penghalang saat ego diam di lajurnya** -- kendaraan skenario
   hanya dipaksa kecepatannya searah hadapnya sendiri, arah hadap berputar oleh
   gaya ban, penghalang bergeser -3,50 -> -2,83 m. Kecepatan kini dipaksa searah
   jalan; drift tinggal <= 0,08 m.
4. **`WAKTU_IKUT` 2,0 s memicu pelambatan di S1** sebelum pemicu menyalip (d* 21 m,
   hukum aktif di celah < 33,8 m). Syarat "hanya bila menyalip tidak mungkin"
   diberlakukan juga di `LANE_KEEPING`; dikunci `test_tidak_melambat_bila_bisa_menyalip`.

**Hasil:** S3 BERHASIL (jarak bodi 1,42 m, durasi 18,9 s), FSM lama GAGAL (0,00 m).
S1 tetap BERHASIL; jarak minimum 1,71 -> 1,43 m karena target kini benar-benar
di tengah lajur (dulu bergeser +0,3 m menjauhi ego -- angka lama terlalu optimis).

**Sitasi:** penulis menetapkan sitasi baru harus terbit <= 4 tahun, ber-URL, dan
isinya dibuka. ISO 15622 (2018), Rajamani (2012), Li dkk. (2011), dan buku teks
kendali klasik karena itu **tidak dipakai**. Nilai time gap ISO 15622 dari sumber
<= 4 tahun tidak ditemukan -- `WAKTU_IKUT` bersandar pada sapuan eksperimen.

**Terbuka:** `ELLIPSE_B` 2,2 m tidak menjamin `JARAK_AMAN` 1,0 m (README #1).

---

## Zona Aman yang Menjamin Jarak Aman, dan Lup Planner-MPC

11-12 September 2026. Angka lengkap di `TUNING_MPC.md` bagian 13.

**Pertanyaan awal:** apakah constraint yang dipakai planner dan MPC menjamin
syarat lulus jarak bodi 1,0 m? **Tidak.** Elips lama (A=7,0, B=2,2 dari sumbu
belakang) hanya setara jarak bodi 0,29 m saat berpapasan. Jarak yang selama ini
tercapai datang dari lebar lajur, bukan dari constraint.

**Zona baru.** Elips-super pangkat 4 antar pusat bodi, memuat seluruh persegi
terlarang (setengah sisi 5,81 x 2,91 m = dimensi kedua kendaraan + JARAK_AMAN).
`A = 7,709` dan `B = 3,204` **diturunkan di config dari dimensi terukur**, tidak
diketik. Elips biasa butuh A 11-14 m untuk memuat sudut yang sama, terlalu panjang.
Dikunci dua uji baru.

**Dua jalan buntu sebelum akar masalah ketemu:**

1. **Margin B.** Setelah zona dipasang, `WAKTU_IKUT` 1,0 dan 2,5 s gagal
   lane_departure. Dikira margin B terhadap jalur berpapasan terlalu tipis;
   disapu B 3,0/3,1/3,2 dengan p=6 -- hasilnya praktis sama, 2,5 s tetap gagal.
   Bentuk zona bukan penyebab.
2. **Gerbang kembali.** Korelasi 16 run bersih: semua run gagal memulai kembali ke
   lajur saat ego masih bergerak menjauh 0,91-1,89 m/s. Dipasang `DD_KEMBALI`
   0,1 m/s (didukung analisis quintic dan derau 0,0014 m/s). Ego memang tidak lagi
   mulai kembali di tengah ayunan, **tapi 1,0 dan 2,5 s tetap gagal** karena
   lemparannya sudah terjadi lebih awal, saat `OVERTAKING`. Gerbang dipertahankan
   sebagai pengaman; setelah akar masalah diperbaiki ia tidak pernah terpicu lagi.

**Akar masalah: lup planner-MPC.** Rekonstruksi planner offline per tick
menunjukkan urutannya: kandidat habis ditolak zona -> acuan cadangan dan constraint
MPC memberi tendangan keluar -> planner memulai rencana berikutnya dari percepatan
lateral **terukur**, MPC mengikuti kelengkungan awalnya, percepatan itu terukur
lagi. Lintasan pilihan planner sendiri makin menukik: ymin -3,79 -> -5,77 m.

Ini persis TEMUAN 2 Tahap 5 di sisi lateral; `a0` longitudinal sudah lama memakai
nilai yang diperintahkan. Perbaikan: `y''` awal diambil dari lintasan rencana
sebelumnya (`Trajectory.lateral_at`), posisi dan kecepatan tetap terukur.

**Hasil.** S1 dan S3 BERHASIL dan deterministik; `WAKTU_IKUT` 1,0-2,5 s semuanya
lolos. Deviasi lajur S1 turun 0,161 -> **0,011 m** (maks 0,530 -> 0,152 m), dan
ego tidak lagi terdorong sampai menyentuh garis lajur ketiga.

**Pelajaran:** pertanyaan "apakah constraint benar-benar menjamin kriteria
penilaian" ternyata membuka tiga cacat berbeda. Dan dua perbaikan pertama --
keduanya masuk akal, keduanya didukung data korelasi -- hanya mengobati gejala.
Yang membedakan perbaikan ketiga: mekanismenya direkonstruksi tick demi tick,
bukan disimpulkan dari korelasi.

**Terbuka:** zona dan penilai sama-sama memakai kotak sejajar sumbu; pada yaw 7
derajat sudut bodi bergeser ~0,31 m yang tidak dihitung (README #1).

---

## Penilai Jarak: Kotak Berputar dan Titik Acuan Bodi

15 September 2026. Menutup butir terbuka terakhir dari bagian 13 `TUNING_MPC.md`.

**Dua cacat di alat ukur, bukan di kendali:**

1. **Kotak ego ditaruh di sumbu belakang**, bukan pusat bodi -- geser 1,433 m di
   arah memanjang, persis kesalahan yang sama dengan bias XTE di Tahap 1.
2. **Sudut hadap diabaikan.** Pada 5 derajat, kotak sejajar sumbu melebihkan jarak
   ~0,2 m.

`evaluation.jarak_kotak` sekarang memutar kedua kotak menurut sudut hadapnya dan
mengukur dari pusat bodi. Poligon cembung yang terpisah selalu punya jarak minimum
di pasangan titik-sudut ke sisi, jadi cukup memeriksa kedua arah, ditambah uji
sumbu pemisah untuk kasus bertumpuk. Yaw kendaraan lain ikut dicatat di log: yang
dipaksa searah jalan hanya kecepatannya, arah hadap bodinya bebas (terukur <= 0,45
derajat, jadi asumsi lama hampir benar -- sekarang tidak perlu diasumsikan).

**Efek kedua cacat hampir saling meniadakan.** S1 1,39 -> 1,43 m, S3 1,49 -> 1,51 m.
Kalau hanya salah satu diperbaiki, angkanya akan menyesatkan ke arah berlawanan.

**Constraint MPC sengaja tidak diubah.** Urutannya dibalik: perbaiki dulu alat
ukurnya, jalankan, baru putuskan. Hasilnya menunjukkan sudut nol memadai -- saat
kedua bodi berdampingan sudut hadap ego <= 5,1 derajat, dan jarak yang tercapai
1,43-1,51 m terhadap syarat 1,0 m. Menggemukkan zona untuk yaw terburuk (13,6
derajat, terjadi jauh sebelum berpapasan) akan menuntut ruang lateral 4,1 m di
lajur selebar 3,5 m dan justru melumpuhkan manuver.

**`plot_run.py`** dibuat untuk grafik bab 4: empat panel (simpangan lateral,
kecepatan, jarak antar bodi tiap kendaraan, kemudi + waktu solve) dengan latar
diwarnai menurut state FSM, dibaca langsung dari log tanpa menjalankan simulasi
ulang. `out/run_s1_mpc_gt.png` dan `out/run_s3_mpc_gt.png` sudah dibangkitkan ulang.

---

## Tahap 8 Langkah 1 — Rig Kamera dan Kalibrasi Deteksi

15 September 2026. Angka lengkap di `RANGKUMAN_PENULISAN.md` bagian 17 dan 18.

**Rig kamera** mengikuti KITTI (keputusan penulis): 1,65 m di atas jalan, 1,68 m di
depan sumbu roda belakang, fov 90 derajat, resolusi 1280x720. Diverifikasi terhadap
simulator, bukan terhadap nilai yang diminta: terukur 1,652 m dan 1,680 m. Depth
camera satu titik dengan kamera warna.

**Weight `epoch-195.pth` bukan model utuh**, melainkan checkpoint berisi `epoch`,
`model`, `state_dict` (914 tensor), dan `optimizer`. Arsitekturnya datang dari repo
penulis sendiri (`SywlAchmd/YOLOPX`, commit c253eb1), di-clone sebagai folder
tetangga supaya repo skripsi tetap bersih. `load_state_dict(strict=True)` melaporkan
seluruh kunci cocok. Tidak ada paket yang menyentuh torch/torchvision.

**Hasil kalibrasi:** terdeteksi di seluruh jarak 10-80 m (conf 0,78-0,92), inferensi
14,4 ms, ambang 0,5 memberi nol positif palsu.

### TEMUAN: depth membaca muka kendaraan, bukan pusatnya

Galat jarak konsisten -2,3 m di semua jarak -- persis setengah panjang Nissan Patrol.
Bukan bug: depth camera mengukur permukaan terdekat yang terlihat, sedangkan ground
truth diukur ke pusat bodi. Terhadap muka kendaraan galatnya tinggal -0,48..+0,19 m.

Kalau ini tidak disadari, `VisionPerception` akan melaporkan kendaraan 2,3 m lebih
dekat daripada `GroundTruthPerception`, dan seluruh perbandingan bagian 11.4 jadi
membandingkan dua besaran berbeda -- persis jenis kesalahan yang sama dengan bias
titik referensi sumbu belakang di Tahap 1.

### Positif palsu hanya soal ambang

Pada ambang bawaan demo 0,3 muncul 4-8 positif palsu per frame, seluruhnya batu,
semak, dan pagar di garis horizon. Pada 0,5 hilang, sementara deteksi benar
terlemah 0,78. Satu positif palsu sempat muncul lalu tidak terulang saat diulang;
render kamera tidak sedeterministik fisika.

---

## Tahap 8 Langkah 2 — Weight yang Benar, dan Pertukaran Jangkauan

15 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 18.

### Jalan buntu: menguji weight yang salah

`epoch-195.pth` yang ada di mesin ternyata **weight resmi YOLOPX hasil latihan
BDD100K**, bukan hasil fine-tuning penulis. Di notebook Kaggle ia dipakai sebagai
`MODEL.PRETRAINED`, dan hasil fine-tuning keluar sebagai `runs/yolopx/best.pth`
yang belum pernah diunduh.

Dugaan awal saya keliru dua kali sebelum ini ketahuan: mula-mula saya menyalahkan
pascaproses `connect_lane` (padahal tidak pernah aktif), lalu menuduh jarak domain
data latih. Yang menyelesaikannya dua ukuran, bukan penalaran:

| Bukti | Hasil |
|---|---|
| Langkah optimizer di checkpoint | 426.496 = ~70.000 citra/epoch x 195 epoch (ukuran BDD100K), bukan ~1.500 citra milik penulis |
| IoU terhadap anotasi penulis, pada data latihnya sendiri | lajur 0,016 dan area jalan 0,429 -- model yang dilatih di situ mustahil serendah itu |

`best.pth` yang benar: epoch 263, lr 4,27e-05, 25.503 langkah (~97 langkah/epoch,
jadi data latih Kaggle ~1.550 citra -- salinan lokal 457 frame hanya sebagian).

### TEMUAN: fine-tuning menukar jangkauan dengan ketelitian

| | Dekat (10-40 m) | Jauh (50-80 m) | Positif palsu @0,3 |
|---|---|---|---|
| BDD100K | conf 0,84-0,92 | terdeteksi semua | 4-8 per frame |
| Fine-tuned | conf 0,97-0,99 | **hilang di atas 40-50 m** | **0** |

Data latih penulis diambil di jalan kota, sehingga kendaraan selebar 16-30 piksel
(50-80 m di Town04) nyaris tidak terwakili. Jangkauan 40-50 m masih cukup untuk
kendali -- pemicu menyalip bekerja di celah ~32 m, horizon MPC 27 m -- tapi
marginnya tipis, dan ini harus masuk pembahasan, bukan disembunyikan.

Kalau jangkauan itu mau dinaikkan, datanya perlu ditambah adegan jalan tol dengan
kendaraan jauh; rig kamera untuk mengumpulkannya sudah siap dan penempatannya
sama persis dengan yang dipakai eksperimen kendali.

### Segmentasi

Dengan `best.pth` keluaran lajur rapi mengikuti marka, dan area jalan bersih.
Dengan BDD, lajur pecah 7-8 komponen dan bahu kanan ikut dicat. Perlu dikonfirmasi
ke anotasi: area jalan versi fine-tuned tidak mencakup lajur yang sedang ditempati
ego.

---

## Tahap 8 Langkah 3 — VisionPerception, dan Jangkar Halangan 1,433 m

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 19.

### Depth CARLA: PLANAR, bukan radial

Diuji terhadap permukaan jalan, yang profil kedalamannya analitik: `Z = h·f/dv`
tetap sepanjang satu baris citra kalau planar, naik ke tepi kalau radial.
Terukur rasio tepi/tengah **1,000x** di empat baris, sedangkan radial
memprediksi 1,28x. RMS terhadap model planar 0,045 m di baris yang seluruhnya
aspal. Tidak butuh kendaraan target dan tidak bergantung isi adegan.

Ini tidak bisa dibedakan oleh `cek_deteksi.py`: targetnya hampir di tengah
citra, dan di sana kedua tafsiran berselisih 1 cm. Selisihnya baru muncul di
lajur sebelah pada jarak dekat -- 0,64 m di 10 m, 1,10 m di 5 m -- yaitu
satu-satunya tempat keputusan S3 diambil. Salah tebak tidak akan pernah
ketahuan di kalibrasi.

Planar juga yang kebetulan paling cocok: muka belakang kendaraan adalah bidang
tegak lurus sumbu jalan, dan depth planar mengukur ke bidang tegak lurus sumbu
optik. Saat sehadap keduanya berimpit, jadi koreksi muka -> pusat (bagian 18.4)
menjadi penambahan satu konstanta di satu sumbu, dan seluruh piksel pada muka
yang sama memberi angka identik. Dengan radial, petak piksel yang sama menyebar
1,10 m di lajur sebelah 5 m. Asumsinya: ego dan target sehadap -- melemah saat
yaw ego mencapai 13,6 derajat waktu pindah lajur, dan itu masuk batasan masalah.

### `tracking.py` dan `VisionPerception`

Asosiasi dua tahap ala ByteTrack (deteksi lemah hanya MENAHAN track, tidak
melahirkan) + Kalman constant-velocity, state frame ego relatif ego. Kompensasi
gerak ego: suku translasi saling meniadakan, jadi hanya rotasi dan perubahan
laju ego yang masuk -- modul ini tidak perlu tahu posisi ego sama sekali.

Validasi `cek_estimasi.py` terhadap ground truth simulator, geometri S1 dengan
kecepatan dipaksa tetap, jarak menyapu 55 -> 9 m:

| | bias | RMS | maks |
|---|---|---|---|
| x memanjang | -0,011 m | **0,046 m** | 0,205 m |
| y melintang | +0,017 m | 0,019 m | 0,030 m |
| vx | -0,007 m/s | **0,020 m/s** | 0,173 m/s |

Deteksi pertama di 45,6 m -- mereproduksi batas 40-50 m bagian 18.2 secara
independen. Kecepatan konvergen ke galat < 0,1 m/s dalam 0,60 s (12 frame).

### TEMUAN: acuan kecepatan ground truth ikut berderau

Tabel pertama menunjukkan bias kecepatan +0,28 m/s yang menetap di semua jarak.
Terlalu rapi untuk derau. Ternyata itu milik acuannya:

| sumber kecepatan relatif | rata-rata | sd |
|---|---|---|
| `get_velocity()` simulator | -6,394 | **0,283** |
| pergeseran posisi GT / dt | -6,396 | 0,018 |
| VisionPerception | -6,402 | **0,006** |

`get_velocity()` berderau 0,28 m/s per tick ketika kecepatan aktor dipaksa tiap
tick. Diukur terhadap pergeseran posisi, RMS vision jatuh 0,589 -> 0,020 m/s --
dan keluaran KF-nya lebih halus daripada pembacaan kecepatan simulator sendiri.
Acuannya sudah diganti di `cek_estimasi.py`. Pola yang sama dengan bagian 15:
angka menyesatkan karena alat ukurnya, bukan karena yang diukur.

### TEMUAN: halangan dijangkar di sumbu belakang, bukan pusat bodi

Terukur `ego.bounding_box.location.x = -0,005 m`: titik asal aktor = pusat bodi.
Jadi `GroundTruthPerception` melaporkan relatif pusat bodi ego, sementara
`halangan_ego_ke_jalan` menambahkan `ego.x` yang sumbu belakang. Halangan di
frame jalan meleset **1,433 m terlalu dekat**.

Arahnya konservatif -- zona aman efektif 7,71 + 1,43 = 9,14 m ke depan -- jadi
ia tidak pernah muncul sebagai kegagalan, dan `ELLIPSE_A` yang diturunkan
hati-hati di bagian 14.3 bukan yang benar-benar berlaku. Ini bias titik acuan
yang KETIGA di proyek ini, setelah bias XTE Tahap 1 dan kotak penilai jarak.

**Satu perbaikan membenarkan kedua pemakainya**, karena masing-masing sudah
menuliskan acuannya sendiri: zona planner & MPC menggeser ego ke pusat bodi lalu
mengurangkan halangan (jadi pusat-ke-pusat), sementara `main.py` mengurangkan
`ego.x` untuk FSM (jadi sumbu belakang -> pusat, persis yang didokumentasikan
`planning._v_ikut`). `main.py` tidak perlu disentuh. Dikunci
`tests/test_localization.py`.

### Hasil setelah perbaikan

| | S1 lama | S1 baru | S3 lama | S3 baru |
|---|---|---|---|---|
| vonis | BERHASIL | BERHASIL | BERHASIL | BERHASIL |
| jarak min antar bodi | 1,43 m | **1,44 m** | 1,51 m | **1,47 m** |
| deviasi lajur | 0,011 m | 0,009 m | 0,012 m | 0,011 m |
| durasi manuver | 11,7 s | 11,6 s | 19,2 s | 18,9 s |

S3 turun 1,51 -> 1,47 m seperti diduga: zona aman tidak lagi kelebihan 1,43 m,
jadi ego boleh mendekat sampai batas rancangannya. Keduanya masih jauh di atas
syarat 1,0 m. S1 mulai memilih offset 4,0 m di sebagian tick (dulu hanya 3,5).

**Empat tick tanpa kandidat di S1 SUDAH ADA sebelum perbaikan** -- diperiksa
dengan menjalankan ulang kode lama lewat `git stash`, bukan diasumsikan. Bukan
regresi, tapi tetap butir terbuka.

---

## Tahap 8 Langkah 4 — Loop Tertutup Vision dan Video Overlay

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 19.6-19.7.

`main.py` dapat flag `--perception vision|gt` dan `--rekam`. Mode gt sengaja
tidak memasang rig kamera sama sekali, supaya hasilnya tetap identik dengan run
sebelumnya tanpa beban render tambahan.

Urutan di main loop digeser sedikit: `a_filt` dihitung SEBELUM perception, karena
vision butuh kinematika ego (laju, percepatan, yaw rate) untuk kompensasi gerak
di langkah prediksi Kalman. Nilainya tidak berubah, hanya dipindah.

**S1 vision BERHASIL**, tapi angkanya menipu: jarak bodi 1,79 m (GT 1,44 m)
dicapai dengan keluar lajur tujuan sejauh −5,46 m selama 23 tick, rem −4,43 m/s²,
dan 42 tick tanpa kandidat planner. Kriteria 11.2 meloloskannya karena kriteria
itu hanya memeriksa kembali ke lajur asal, bukan menjaga lajur selama menyalip.

Dugaan utama: koreksi muka -> pusat gugur saat ego berdampingan, karena kamera
melihat sisi kendaraan, bukan muka belakangnya. Terlihat di video t=8,50 s
(target terbaca 31 km/jam padahal 25). Belum diukur -- `cek_estimasi.py` perlu
diperluas ke lajur sebelah sudut besar.

### ffmpeg tidak ada di mesin ini

`gambar.Perekam.simpan` semula memanggil ffmpeg seperti `record_path.capture`,
dan gagal di akhir run 20 detik. Diganti `cv2.VideoWriter`: OpenCV sudah jadi
dependensi perception, sedangkan ffmpeg dependensi sistem yang ternyata belum
terpasang. Frame-nya selamat karena `rmtree` berada setelah encode, jadi run-nya
tidak perlu diulang. `record_path.py` dan `record_maneuver.py` masih memakai
ffmpeg dan karena itu masih belum bisa dijalankan di mesin ini.

---

## Tahap 8 Langkah 5 — Kenapa Kandidat Habis, dan Batas yang Tidak Dibagi

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 19.8-19.10.

Dua perbaikan dikerjakan. **Koreksi permukaan sadar sudut pandang berhasil**
(galat berdampingan bias +1,31 -> +0,39 m, galat melintang saat target di depan
RMS 0,37 -> 0,07 m). **Acuan cadangan menahan `y` berjalan tidak berhasil** --
laju lateral puncak tetap 4,10 m/s. Sasarannya keliru: yang mendorong banting
setir bukan acuan planner, melainkan constraint elips MPC dengan slack rho=1000.

### TEMUAN: planner dan MPC tidak berbagi ruang kelayakan

Planner memutar ulang dari log (bisa, karena `planning.py` murni numerik).
Penolak kandidat: elips 377, percepatan lateral 73, kelengkungan 0.

Uji pemisah: lintasan ego yang sama, halangan ditukar antara estimasi vision dan
ground truth. **86 tick nol kandidat pada keduanya -- identik.** Perception bukan
penyebabnya begitu ayunan dimulai.

Yang membedakan keadaan ego: laju lateral puncak 1,61 m/s (GT) vs 4,10 (vision),
sudut hadap 6,7 vs 17,1 derajat. Planner membatasi kandidat pada MAX_LATERAL_ACCEL
3,0 m/s^2; MPC tidak punya batas percepatan lateral sama sekali. MPC membawa mobil
ke keadaan yang planner tidak bisa lanjutkan, lalu lupnya menutup sendiri.

Perbaikan 15.3 ternyata baru separuh: `ddy0` sudah dari rencana, `dy0` masih ukur.

### Run vision tidak terulang

Dua run kode identik: simpangan lateral -5,58 vs -4,55 m (selisih 1,03 m), nol
kandidat 50 vs 52, durasi 11,5 vs 10,9 s. Jarak bodi stabil (1,30 vs 1,31 m).
Bagian 11.4 tidak boleh memakai satu run per konfigurasi untuk vision.

---

## Tahap 8 Langkah 6 — Ruang Kelayakan Dibagi, dan Akar yang Tersisa

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 19.11-19.13.

Batas percepatan lateral ditambahkan ke MPC memakai konstanta yang sama dengan
planner, dan `dy0` planner diambil dari rencana. Keduanya bekerja: percepatan
lateral tersaturasi tepat 3,00 m/s^2, laju lateral puncak 4,10 -> 2,95 m/s, yaw
17,1 -> 12,3 derajat, penolak a_lat 73 -> 32, solver tetap 0 gagal dari 400.
Disapu offline, planner memang masih memberi 9 kandidat sampai dy0 = 3,0 m/s.

**Tapi loop tertutupnya tidak membaik**: tick nol kandidat tetap 50, dan
penolaknya kini hampir seluruhnya elips (410 dari 450). Uji tukar halangan: 49
(vision) vs 42 (GT) -- perception tinggal seperlima masalah.

**Ongkosnya nyata**: jarak bodi minimum 1,79 -> 1,30 -> 1,13 m terhadap syarat
1,0 m. MAX_LATERAL_ACCEL itu batas KENYAMANAN; memberlakukannya keras di MPC
berarti kenyamanan mengalahkan pelebaran jarak saat berpapasan. Harus dinyatakan
sebagai keputusan, bukan didiamkan.

### TEMUAN: planner tidak punya komitmen, FSM punya

Rencana muncul/hilang 10 kali (GT 2 kali). Kepingan rencana saat manuver: 2, 11,
2, 2, 1, 53 replan -- lima yang pertama jauh lebih pendek daripada MANEUVER_TIMES
3,0-4,0 s yang direncanakannya sendiri. Inilah "beberapa kali mau menyalip" yang
terlihat di video. FSM diberi FSM_DWELL dan histeresis dengan alasan yang ditulis
eksplisit; planner tidak pernah diberi keduanya.

### TEMUAN: pemicu tidak diturunkan dari zona aman

Zona aman menuntut celah > 7,6 m selama ego di tengah penyeberangan. Celah
menyusut 6,4 m/s dan penyeberangan makan 3,6-4,0 s = 23-26 m, berangkat dari
26-27 m. Marginal secara rancangan. TTC_TRIGGER = 5,0 s diturunkan dari waktu
dwell FSM, bukan dari geometri elips -- dua syarat berbeda yang tidak pernah
dicocokkan. Pola yang sama dengan bagian 15.4.

---

## Tahap 8 Langkah 7 — Komitmen Planner dan Batas Lateral yang Lunak

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 19.13-19.14.

### KOREKSI: klaim "pemicu terlalu lambat" di langkah 6 keliru

Sapuan terhadap planner yang sebenarnya membantahnya: celah saat pemicu 15,0 m
pada dv terkecil (DV_TRIGGER 3,0) versus celah minimum 13,5 m -- cukup, margin
+1,5 m, dan makin longgar pada dv besar. TTC_TRIGGER tidak diubah; yang
ditambahkan uji yang mengunci kecocokan dua turunan terpisah itu.

Yang sebenarnya mengikat: celah minimum MENGECIL seiring ego menyeberang (19,0 m
di y=0 menjadi 10,5 m di y=-3,0), dan mengecil lagi kalau ego sedang bergerak
lateral. Menyeberang itu balapan. **Berhenti di tengah adalah tindakan terburuk**
-- persis yang dilakukan acuan cadangan, dan itu yang membuat rencana berkedip
berakibat fatal.

### Hasil

Vision S1: jarak bodi 1,13 -> 2,12 m, rem -5,80 -> -2,71 m/s2, tick nol kandidat
50 -> 38, kedipan rencana 10 -> 6 kali, kepingan [2,11,2,2,1,53] -> [2,11,5,55],
laju lateral 4,10 -> 2,58 m/s, yaw 17,1 -> 10,8 derajat, durasi 11,9 -> 10,5 s.

**Run vision menjadi terulang**: dua run kode identik memberi 2,12 m / 38 tick /
10,5 s, simpangan lateral -5,38 vs -5,39 m. Sebaran runtuh dari 1,03 m ke 0,01 m.
Render kamera tetap tidak deterministik, tapi tidak lagi diperbesar lup tak stabil.

GT tanpa regresi: S1 1,42 m (dari 1,44), S3 1,47 m (sama), tick nol kandidat 4
dan 0 seperti sebelumnya. Ongkos: solve 12,3 -> 16,9 ms rata-rata karena N slack
tambahan; deviasi lajur GT 0,009 -> 0,015 m.

Masih terbuka: ego tetap melebar ke -5,38 m (tepi lajur -5,25 m), dan 38 tick nol
kandidat masih jauh di atas 4 tick milik GT.

---

## Tahap 8 Langkah 8 — Tuning Ulang di Atas Vision

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 20.

`tuning_vision.py` menyapu parameter di SKENARIO PENUH dengan vision -- sah
karena run vision sudah terulang setelah bagian 19.14. Yang disapu justru yang
tidak bisa disentuh step response: ambang FSM dan bobot pemilihan kandidat.

Jebakan: durasi run 15 detik memberi vonis PALSU `lane_departure` di semua
konfigurasi, karena manuver selesai ~15 s dan run terpotong sebelum LULUS_TAHAN
2,0 s. Disamakan dengan main.py (20 s).

**Hanya satu pasangan berubah**: Q_y 20 -> 150, Q_psi 450 -> 3400. K_DEV,
FSM_DWELL, dan PASS_MARGIN semuanya sudah optimal pada nilai sekarang.

### TEMUAN: redaman ditentukan rasio Q_psi/Q_y

Sapuan skenario dan step response berlawanan: Q_y=150 memperbaiki skenario
(lambungan -5,39 -> -4,75 m) tapi merusak step response (overshoot 7,4 -> 28,9%,
chatter 50x). Sebabnya Q_psi=450 dulu dipilih sebagai titik redaman kritis PADA
Q_y=20. Rasionya 22,5; 22,5 x 150 = 3375, dan sapuan step response independen
memilih 3400. Di situ step response kembali ke kualitas semula (overshoot 7,5%,
settling 1,60 s, chatter 0,0003 mrad -- lebih baik dari 0,0008 semula). Di 6000
chatter meledak 0,1929 mrad.

Bobot yang bersama-sama menentukan satu sifat fisik tidak boleh disapu satu per
satu: sapuan 1-D pada Q_y selalu tampak buruk, dan sapuan 1-D pada Q_psi tidak
akan pernah menemukan 3400.

### FSM_DWELL tidak perlu dinaikkan

Dugaan "vision lebih berisik jadi butuh dwell lebih panjang" terbantah: 0,5 dan
0,8 s jauh lebih buruk (nol kandidat 38 -> 44 -> 68, rem tersaturasi -6,00).
Peredamannya sudah ada di tempat yang lebih tepat -- TRACK_N_INIT 3 frame dan
melayang 5 frame di `tracking.Pelacak`.

### Hasil

Vision S1: jarak bodi 2,10 m, durasi 10,4 s, rem -0,95 m/s2, lambungan -4,71 m
(di DALAM lajur; tepi -5,25 m), 0 solver gagal. GT tidak bergeser sama sekali:
S1 1,42 m / 11,6 s / 4 tick nol, S3 1,47 m / 19,0 s / 0 tick nol.

Masih terbuka: 40 tick nol kandidat vs 4 milik GT, dan angka itu bertahan
38-44 di SELURUH sapuan -- bukan soal tuning, melainkan geometri zona aman.

### Label video

"mutlak" -> "absolute" atas permintaan penulis.

---

## Tahap 9 — Eksperimen Penuh S1

16 September 2026. Angka di `RANGKUMAN_PENULISAN.md` bagian 21.

Server CARLA direstart tepat sebelum pengukuran (README: 26-31 ms senggang vs
70 ms setelah berjam-jam). `eksperimen.py` mengulang satu konfigurasi N kali dan
melaporkan success rate berikut sebaran tiap metrik.

**GT 5/5, vision 10/10, seluruhnya BERHASIL. Solver gagal 0 dari 6.000 solve.**

GT: jarak bodi 1,423 m sd 0,000 -- log identik bit-per-bit di kelima ulangan,
jadi determinisme selamat melewati seluruh perubahan Tahap 8.
Vision: jarak bodi 2,105 m sd 0,004, durasi 10,43 s sd 0,02, simpangan lateral
-4,694 m sd 0,026. Bandingkan sebelum lup distabilkan: dua run berselisih 1,03 m.

### Jebakan alat ukur yang keempat

Uji determinisme mula-mula melaporkan TIDAK padahal semua metrik kendali identik
sampai digit terakhir. Dua sebab: `solve_ms` itu jam dinding, dan `np.array_equal`
memberi False untuk NaN (kolom x_est/y_est berisi NaN saat tidak ada deteksi).
Dibuktikan dengan membandingkan kolom per kolom, bukan dengan menduga. Setelah
kolom waktu dikecualikan dan equal_nan dipakai: identik.

### Anggaran tick

Inferensi 14,4 + solve 17,9 = 32,3 ms rata-rata dari 50 ms. Terburuk 14,4 + 34,5
= 48,9 ms -- praktis menyentuh anggaran. Harus ditulis apa adanya. Solve naik dari
12,3 ms sebelum Tahap 8 karena N slack batas percepatan lateral; kelonggaran itu
memang dibeli. Mesin tidak benar-benar senggang (load ~3).

### Tick nol kandidat 39,4 vs 4,0

Sistematis (sd 0,9), sudah ditelusuri: geometri zona aman, bukan tuning maupun
perception. TIDAK menurunkan keselamatan -- jarak bodi vision justru lebih besar.
Sejak planner berkomitmen, replan gagal bukan lagi kehilangan arah.
