# Tuning Bobot MPC — Catatan Lengkap

Dokumen ini mencatat seluruh proses penyetelan bobot pengendali: dasar teori tiap
bobot, protokol yang diikuti, setiap percobaan beserta angkanya, jalan buntu yang
sempat ditempuh, dan alasan berhenti di titik akhir.

Dijalankan 10 September 2026. Alat: `tuning.py`. Rujukan: rencana kerja bagian
7.3 (nilai awal) dan 7.6 (urutan tuning).

---

## 1. Apa yang sebenarnya disetel

Fungsi biaya MPC:

```
J = SUM_{k=0}^{N-1} [ e_k' Q e_k  +  u_k' R u_k  +  du_k' Rd du_k  +  rho*eps_k² ]
    + e_N' Qf e_N
```

dengan `e = x - x_ref`, `x = [X, Y, psi, v]`, `u = [a, delta]`,
`du = u_k - u_{k-1}`.

Peran tiap bobot:

| Bobot | Menghukum | Efek kalau dinaikkan |
|---|---|---|
| `Q[X]` | error posisi memanjang | mengejar jadwal longitudinal lebih ketat |
| `Q[Y]` | error posisi lateral | koreksi lateral lebih agresif |
| `Q[psi]` | error arah hadap | **meredam** — arah hadap berperan seperti suku turunan posisi lateral |
| `Q[v]` | error kecepatan | menahan kecepatan lebih dekat ke acuan |
| `R[a]`, `R[delta]` | besar input | hemat gas dan kemudi |
| `Rd[a]`, `Rd[delta]` | perubahan input antar-langkah | input lebih halus, tapi lebih lambat bereaksi |
| `rho` | pelanggaran elips | makin enggan menyerempet batas aman |

**Poin teori yang menentukan seluruh langkah 1:** dalam kendali lateral kendaraan,
`Q[Y]` berperan seperti **penguatan proporsional** terhadap simpangan, sedangkan
`Q[psi]` berperan seperti **penguatan turunan**. Alasannya kinematik: pada
kecepatan `v`, laju perubahan posisi lateral adalah `y_dot = v*sin(psi)`, jadi
arah hadap adalah turunan posisi lateral. Menghukum error arah = menghukum laju
perubahan error posisi = redaman.

Sistem yang hanya punya penguatan proporsional besar tanpa redaman akan
overshoot. Itu persis yang terjadi dengan nilai awal bagian 7.3.

---

## 2. Kenapa harness terpisah, bukan menyetel di skenario penuh

Menyetel langsung di `main.py` tidak sah: mengubah satu bobot ikut mengubah kapan
FSM terpicu, kandidat lintasan mana yang lolos saringan, dan berapa lama tiap
fase berlangsung. Efek bobot tercampur efek skenario dan tidak bisa dipisahkan.

Bagian 7.6 sendiri menetapkan syaratnya: *"matikan constraint tabrakan,
`x_ref` = garis lurus"*. `tuning.py` memenuhi itu secara harfiah.

**Uji yang dipakai: step response lateral.** Ego dimapankan pada `y = 0` selama
fase pemanasan, lalu acuan **dilompatkan** ke `y = -1,0 m` tepat pada `t = 0`.
Acuan berupa garis lurus tetap; tidak ada halangan, tidak ada FSM, tidak ada
planner. Setiap konfigurasi memakai ego yang **di-spawn ulang**, jadi titik
awalnya identik.

Step response dipilih karena memisahkan empat sifat yang biasanya tercampur:
kecepatan respons (settling), kestabilan (overshoot), ketelitian akhir (sisa),
dan kehalusan aktuator (chatter).

---

## 3. Metrik

| Metrik | Definisi | Kenapa penting |
|---|---|---|
| `overshoot%` | `max(sign(step) * (y - step)) / abs(step) * 100` | melewati lajur tujuan itu masalah keselamatan |
| `settling s` | waktu terakhir `abs(e) > 5% * abs(step)`, ditambah satu tick | seberapa cepat manuver selesai |
| `sisa m` | rata-rata `abs(e)` pada 1 detik terakhir | error tunak |
| `chatter mrad` | rata-rata `abs(delta_k - delta_{k-1})` **hanya pada 1 detik terakhir** | ukuran "kemudi bergerigi" bagian 7.6 |
| `jitter tot` | sama, tapi sepanjang run | pembanding; mencampur ramp awal |
| `v err`, `v sisa`, `v min` | error kecepatan rata-rata, tunak, dan dip terdalam | untuk menyetel throttle PI |

Definisi `chatter` **berubah di tengah proses**, dan perubahan itu membalikkan
kesimpulan langkah 2 — lihat bagian 5.

---

## 4. Langkah 1 — `Q[Y]` dan `Q[psi]`

### 4.1 Percobaan pertama: sapuan `Q[Y]`

```
python tuning.py --sweep Q_Y 5,20,60
python tuning.py --sweep Q_Y 0.5,1,2,5,10
```

| `Q_Y` | overshoot | settling | jitter tot | delta maks |
|---|---|---|---|---|
| 0,5 | 4,7% | 1,05 s | 2,534 mrad | 0,1166 |
| 1 | 11,5% | 1,60 s | 3,476 | 0,1394 |
| 2 | 17,4% | 1,55 s | 4,348 | 0,1486 |
| 5 | 22,3% | 1,95 s | 5,606 | 0,1500 |
| 10 | 25,0% | 1,95 s | 6,716 | 0,1599 |
| **20 (proposal)** | **26,4%** | **2,35 s** | **7,768** | **0,1677** |
| 60 | 27,6% | 2,35 s | 9,852 | 0,1773 |

Monoton di seluruh rentang: **makin kecil `Q[Y]`, makin baik semua metrik.**

### 4.2 Kenapa hasil itu tidak boleh langsung dipakai

`Q[Y] = 0,5` berarti error lateral dibobot **lebih ringan** daripada error
memanjang (`Q[X] = 1,0`). Itu bertentangan dengan seluruh premis bagian 7.3
yang membobot lateral 20x lebih berat, dan secara fisik janggal: keluar lajur
jauh lebih berbahaya daripada telat satu meter.

Gejalanya — overshoot naik seiring penguatan — adalah ciri baku sistem **kurang
teredam**. Jadi hipotesisnya: yang kurang bukan bobot lateralnya, melainkan
**redamannya**. Menurunkan `Q[Y]` menaikkan rasio `Q[psi] : Q[Y]`, dan itulah
yang sebenarnya memperbaiki keadaan.

Kalau hipotesis itu benar, menaikkan `Q[psi]` harus memberi perbaikan yang sama
**tanpa mengorbankan wewenang lateral**.

### 4.3 Uji hipotesis: sapuan `Q[psi]` pada `Q[Y]` tetap 20

```
python tuning.py --sweep Q_PSI 10,40,100,300
python tuning.py --sweep Q_PSI 300,600,1200,2400
```

| `Q_psi` | overshoot | settling | jitter tot | delta maks |
|---|---|---|---|---|
| **10 (proposal)** | **26,4%** | **2,35 s** | **7,761** | **0,1674** |
| 40 | 22,5% | 1,85 s | 6,773 | 0,1623 |
| 100 | 16,1% | 1,40 s | 5,583 | 0,1500 |
| 300 | 3,3% | 0,90 s | 4,027 | 0,1435 |
| 600 | 0,0% | 1,25 s | 3,184 | 0,1317 |
| 1200 | 0,0% | 1,80 s | 2,528 | 0,1166 |
| 2400 | 0,0% | 2,50 s | 1,961 | 0,1000 |

Hipotesis terkonfirmasi, dan hasilnya **lebih baik daripada menurunkan `Q[Y]`**:
`Q[psi] = 300` memberi overshoot 3,3% versus 4,7% pada `Q[Y] = 0,5`, dengan
wewenang lateral utuh.

Di atas 600 overshoot tetap nol tapi **settling justru memanjang** (1,25 -> 1,80
-> 2,50 s). Itu terlalu teredam: redaman berlebih membuat respons lamban.
Optimumnya ada di antara 300 dan 600.

### 4.4 Penyempitan

```
python tuning.py --sweep Q_PSI 350,450,550,650
```

| `Q_psi` | overshoot | settling | jitter tot |
|---|---|---|---|
| 350 | 1,4% | 0,95 s | 3,792 |
| **450** | **0,0%** | **1,05 s** | 3,470 |
| 550 | 0,0% | 1,15 s | 3,275 |
| 650 | 0,0% | 1,30 s | 3,110 |

**`Q[psi] = 450` dipilih**: nilai terkecil yang memberi overshoot nol, sekaligus
settling tercepat di antara seluruh nilai beroverhoot nol. Menaikkannya lebih
jauh hanya menukar kecepatan respons dengan kehalusan yang sudah tidak
dibutuhkan.

### 4.5 Sapuan ulang `Q[Y]` setelah redaman benar

Sapuan `Q[Y]` pertama dilakukan saat sistem masih kurang teredam, jadi hasilnya
tidak berlaku lagi. Diulang dengan `Q[psi] = 450`:

```
python tuning.py --sweep Q_Y 5,20,60,150
python tuning.py --sweep Q_Y 20,28,38
```

| `Q_Y` | overshoot | settling | jitter tot |
|---|---|---|---|
| 5 | 0,0% | 2,20 s | 2,111 |
| **20 (proposal)** | **0,0%** | **1,05 s** | 3,470 |
| 28 | 1,9% | 0,95 s | 3,977 |
| 38 | 5,7% | 1,20 s | 4,426 |
| 60 | 11,6% | 1,35 s | 5,216 |
| 150 | 20,7% | 1,75 s | 7,098 |

**`Q[Y] = 20` — nilai asli bagian 7.3 — ternyata optimal.** Lebih rendah membuat
lamban (2,20 s), lebih tinggi memunculkan overshoot lagi.

`Q[Y] = 28` menawarkan settling 0,1 detik lebih cepat dengan imbalan overshoot
1,9%. Ditolak: overshoot lateral berarti menerobos ke lajur sebelah, dan 0,1
detik tidak sebanding dengan itu di manuver berdurasi 3-4 detik.

### 4.6 Hasil langkah 1

| | Proposal | Tertuning |
|---|---|---|
| `Q[psi]` | 10 | **450** (45x) |
| `Q[Y]` | 20 | 20 (tetap) |
| Overshoot | 26,4% | **0,0%** |
| Settling | 2,35 s | **1,05 s** |
| Jitter total | 7,77 mrad | 3,47 mrad |

Dari dua bobot, satu salah 45x dan satu sudah benar sejak awal.

---

## 5. Langkah 2 — `Rd[delta]`

Bagian 7.6: *"Naikkan `Rd[delta]` sampai kemudi tidak bergetar. Ini yang paling
sering perlu dinaikkan."*

### 5.1 Percobaan pertama

```
python tuning.py --sweep RD_DELTA 20,80,300,1000
```

| `Rd[delta]` | overshoot | settling | jitter tot |
|---|---|---|---|
| 20 (proposal) | 0,0% | 1,05 s | 3,470 |
| 80 | 0,5% | 1,00 s | 3,289 |
| 300 | 2,6% | 1,00 s | 3,034 |
| 1000 | 4,4% | 1,10 s | 2,427 |

Jitter turun 30% tapi overshoot naik 0 -> 4,4%. Pertukaran yang tampak merugikan.

### 5.2 Metrik diperbaiki, kesimpulan berbalik

Kecurigaan: `jitter` dirata-ratakan **sepanjang run**, termasuk ramp kemudi di
awal saat step direspons. Belokan cepat di situ adalah **koreksi yang sah**,
bukan getaran. Metrik itu mencampur dua fenomena berbeda.

Metrik dipisah: `chatter` dihitung **hanya pada 1 detik terakhir**, setelah
sistem mapan — di situ setiap perubahan kemudi adalah getaran menurut definisi.

| `Rd[delta]` | overshoot | **chatter (mapan)** | jitter tot |
|---|---|---|---|
| **20 (proposal)** | **0,0%** | **0,0008 mrad** | 3,470 |
| 80 | 0,5% | 0,0001 | 3,289 |
| 300 | 2,5% | 0,0001 | 3,028 |
| 1000 | 4,4% | 0,0002 | 2,419 |

**Chatter praktis nol di semua nilai.** 0,0008 mrad = 8e-7 rad, itu derau
numerik solver, bukan getaran fisik. Tidak ada masalah yang perlu diperbaiki.

**`Rd[delta] = 20` dipertahankan.** Menaikkannya hanya menambah overshoot tanpa
manfaat apa pun.

### 5.3 Lalu riak di grafik skenario penuh itu dari mana?

Grafik skenario S1 memang menunjukkan riak berfrekuensi tinggi di kanal kemudi.
Kalau bukan dari MPC, sumbernya di tempat lain. Diukur:

| | chatter |
|---|---|
| Step response (acuan garis lurus **tetap**) | 0,0008 mrad |
| Skenario S1 saat `LANE_KEEPING` | 0,3378 mrad |
| Rasio | **422x** |

MPC-nya identik di kedua kasus. Yang berbeda hanya **acuannya**: harness memakai
garis lurus tetap, skenario penuh **membangun ulang lintasan tiap 100 ms** dari
state ego saat itu.

Bagian 2 rencana kerja sudah meramalkannya:

> *"Mencegah referensi yang goyang — regenerasi lintasan tiap tick membuat
> `x_ref` sedikit berbeda setiap kali, dan karena bobot `Q` besar pada error
> lateral, kemudi ikut bergerak mengejar referensi yang bergeser."*

Itu juga alasan planner dijalankan 10 Hz, bukan 20 Hz.

**Kesimpulan: `Rd[delta]` adalah knob yang salah untuk riak ini.** Besarnya
0,34 mrad = 0,7% dari `DDELTA_MAX`, setara 0,02 derajat di roda. Tidak
memerlukan penanganan.

---

## 6. Langkah 3 — Throttle PI

> **Tidak berlaku lagi sejak 11 September 2026 — lihat bagian 10.2.** Sapuan 6.2
> dan 6.3 diukur dengan `ThrottlePI` yang me-reset integrator setiap `a_ref < 0`.
> "Optimum tajam `kp = 0,3`" ternyata artefak reset itu, bukan sifat pengendali.

Bagian 7.6 langkah 3 menyebut `Q[v]` dan `R[a]`. Tapi kecepatan mengembara
+-2 km/jam, dan tersangka utamanya bukan bobot MPC melainkan **throttle PI** yang
menerjemahkan `a_ref` menjadi throttle. Menyetel `Q[v]` sebelum PI benar berarti
menyetel parameter yang salah, jadi PI diperiksa lebih dulu.

### 6.1 Harness ternyata cacat — hasil bergantung urutan

Sapuan PI pertama memberi hasil yang tidak konsisten: `ki = 0,1` menghasilkan
`v_err` **0,042** di satu sapuan dan **0,263** di sapuan lain, dengan konfigurasi
identik.

Uji repeatability (`--sweep PI_KI 0.25,0.25,0.25`) memberi tiga baris identik,
jadi harness-nya konsisten **dalam satu sapuan**. Perbedaannya muncul antar
sapuan: di sapuan pertama `ki = 0,1` berada di urutan **pertama**, di sapuan
kedua di urutan **ketiga**.

**Sebab:** harness memakai satu ego berulang-ulang dan hanya me-reset posisinya
dengan `set_transform`. Itu memindahkan posisi tapi **tidak** me-reset putaran
roda dan kompresi suspensi, sehingga tiap konfigurasi memulai dari state fisika
yang berbeda tergantung apa yang berjalan sebelumnya.

**Perbaikan:** ego **di-spawn ulang** untuk setiap konfigurasi. Diverifikasi
dengan `--sweep PI_KI 0.1,0.25,0.1` yang kini memberi hasil identik untuk `ki`
yang sama di posisi berbeda (0,262 di posisi pertama dan ketiga).

Seluruh sapuan PI sebelum perbaikan ini **dibuang** dan diulang.

### 6.2 Sapuan `kp` (harness sudah benar)

```
python tuning.py --sweep PI_KP 0.04,0.08,0.16,0.3
python tuning.py --sweep PI_KP 0.3,0.5,0.8,1.5
python tuning.py --sweep PI_KP 0.2,0.25,0.3,0.4
```

| `kp` | `v err` (m/s) | `v sisa` | `v min` (km/jam) |
|---|---|---|---|
| 0,04 | 0,520 | 0,452 | 44,6 |
| **0,08 (proposal)** | **0,110** | **0,002** | **45,8** |
| 0,16 | 0,107 | 0,016 | 46,6 |
| 0,2 | 0,105 | 0,024 | 46,8 |
| 0,25 | 0,105 | 0,041 | 46,9 |
| **0,3** | **0,021** | **0,007** | **48,0** |
| 0,4 | 0,190 | 0,194 | 46,7 |
| 0,5 | 0,203 | 0,177 | 46,4 |
| 0,8 | 0,177 | 0,183 | 46,4 |
| 1,5 | 0,197 | 0,185 | 46,9 |

**`kp = 0,3` optimum dan tajam**: error rata-rata 0,021 m/s versus ~0,105 di
sekitarnya, dan dip saat manuver praktis hilang (48,0 km/jam dari acuan 48,2).
Ketajaman itu sempat mencurigakan, tapi nilai 0,021 muncul ulang di **tiga
sapuan terpisah**.

### 6.3 Sapuan `ki` pada `kp = 0,3`

```
python tuning.py --sweep PI_KI 0.05,0.15,0.25,0.5
```

| `ki` | `v err` | `v sisa` | `v min` |
|---|---|---|---|
| 0,05 | 0,142 | 0,123 | 47,6 |
| 0,15 | 0,056 | 0,032 | 47,8 |
| **0,25 (proposal)** | **0,021** | **0,007** | **48,0** |
| 0,5 | 0,058 | 0,006 | 47,1 |

Puncak bersih dan unimodal di **`ki = 0,25`** — nilai awal. Dari dua gain PI,
satu perlu naik 3,75x dan satu sudah benar.

### 6.4 `Q[v]` dan `R[a]` tidak jadi disetel

Setelah PI benar, error kecepatan tunak tinggal 0,007 m/s (0,025 km/jam) dan dip
saat manuver 0,2 km/jam. Tidak ada lagi yang bisa diperbaiki `Q[v]` maupun
`R[a]`: sisa ayunan di skenario penuh berasal dari kehilangan kecepatan nyata
saat menikung, bukan dari kelemahan pengendali.

Menaikkan `Q[v]` justru berisiko — ia bersaing dengan `Q[Y]` dalam fungsi biaya
yang sama, dan langkah 1 sudah menunjukkan betapa peka keseimbangan itu.

---

## 7. Bobot akhir

| Parameter | Bagian 7.3 | Akhir | Alasan |
|---|---|---|---|
| `Q[X]` | 1,0 | 1,0 | tidak disentuh |
| `Q[Y]` | 20,0 | **20,0** | disapu dua kali, terbukti optimal |
| `Q[psi]` | 10,0 | **450,0** | redaman kurang; overshoot 26,4% -> 0,0% |
| `Q[v]` | 2,0 | 2,0 | tidak disentuh; PI sudah menangani kecepatan |
| `Qf` | 5*Q | 5*Q | tidak disentuh |
| `R` | (0,1; 1,0) | (0,1; 1,0) | tidak disentuh |
| `Rd` | (1,0; 20,0) | **(1,0; 20,0)** | disapu, terbukti sudah benar |
| `rho` | 1000 | 1000 | tidak disentuh |
| `kp` throttle | 0,08 | **0,14** | = 1/gain plant terukur; 0,3 dulu artefak (bagian 10.2) |
| `ki` throttle | 0,25 | **0,25** | tengah geometrik plateau 0,125-0,5 (bagian 10.2) |

**Dua angka berubah dari sepuluh.** Delapan sisanya diverifikasi atau tidak
disentuh karena tidak ada bukti perlu diubah.

### Dampak di skenario S1 penuh

| | Sebelum tuning | Sesudah |
|---|---|---|
| Waktu solve rata-rata | 28,1 ms | 29,1 ms |
| Rentang kecepatan | 46-50 km/jam | **47,2-50,0 km/jam** |
| Lateral puncak | -4,02 m | **-3,98 m** |
| Solver gagal | 0 | 0 |
| Urutan state FSM | lengkap | lengkap |

---

## 8. Kenapa berhenti di sini

Empat alasan, berurutan dari yang paling menentukan.

**1. Metrik langkah 1 sudah menyentuh batas bawahnya.** Overshoot nol tidak bisa
diperbaiki lagi. Settling 1,05 detik masih bisa dipercepat, tapi setiap
percobaan menunjukkan pertukaran langsung dengan overshoot — dan overshoot
lateral berarti menerobos lajur sebelah.

**2. Langkah 2 dan tiga dari empat gain terbukti sudah benar.** `Q[Y]`,
`Rd[delta]`, dan `ki` masing-masing disapu dan optimumnya jatuh tepat di nilai
proposal. Menyetel lebih jauh berarti mengejar selisih di bawah lantai derau uji
— dan sapuan `ki` pada bagian 6.1 menunjukkan bahwa di wilayah itu hasilnya
sudah tidak bisa dibedakan dari artefak.

**3. Sisa ketidaksempurnaan bukan berasal dari bobot.** Riak kemudi di skenario
penuh berasal dari regenerasi acuan planner (422x lebih besar daripada di
harness), dan ayunan kecepatan berasal dari kehilangan kecepatan nyata saat
menikung. Keduanya tidak bisa disentuh bobot MPC.

**4. Perception belum final.** Bagian 10.6 rencana kerja memperingatkan bahwa
deteksi berbasis vision akan jauh lebih berisik daripada ground truth, dan bobot
harus **dituning ulang** setelah model bersih dipasang. Menghabiskan waktu
menyempurnakan bobot di atas perception ideal berarti pekerjaan yang akan terbuang.

**Yang sengaja tidak dikerjakan:** sapuan dua dimensi (`Q[Y]` x `Q[psi]`
bersamaan), penyetelan `Qf`, `R`, dan `rho`, serta step response pada beberapa
kecepatan. Semuanya baru layak setelah perception final.

---

## 9. Pelajaran metodologis

Tiga kali dalam satu sesi, **metrik menuntun ke kesimpulan yang salah**, dan
ketiganya punya pola yang sama: metrik yang mencampur dua fenomena.

| # | Metrik bermasalah | Kesimpulan salah | Setelah diperbaiki |
|---|---|---|---|
| 1 | XTE terhadap acuan yang di-anchor di posisi ego | "tracking 0,001 m" | acuan bergerak bersama mobil; metriknya mengukur konstruksi, bukan kinerja |
| 2 | Deviasi lajur di skenario penuh | "`V_REF` 13,9 lebih baik" | angkanya bagus karena planner **gagal** 4 kali dan mobil tertarik ke tengah |
| 3 | `jitter` sepanjang run | "naikkan `Rd[delta]`" | ramp awal yang sah tercampur getaran; setelah dipisah, **jangan sentuh `Rd`** |

Ditambah satu kegagalan alat: harness memberi hasil bergantung urutan karena ego
tidak di-spawn ulang, sehingga seluruh sapuan PI harus dibuang dan diulang.

**Aturan yang lahir dari itu:** setiap kali sebuah angka membaik, periksa
*kenapa* ia membaik sebelum menerima kesimpulannya. Angka yang membaik karena
alasan yang salah lebih berbahaya daripada angka yang jelek, karena ia
menghentikan pencarian.

---

## 10. Verifikasi ulang — 11 September 2026

Seluruh sapuan di atas diulang setelah dua cacat ditemukan. Server CARLA
`-quality-level=Low`; hasil kendali terbukti identik bit-per-bit dengan kualitas
default (kualitas hanya memengaruhi rendering).

### 10.1 Cacat 1: perintah aktor balapan dengan `world.tick()`

`apply_control` dan `set_target_velocity` dikirim tanpa menunggu, `world.tick()`
menunggu. Server kadang memproses tick sebelum perintahnya tiba, sehingga
perintah berlaku satu frame terlambat secara acak. Lima run `main.py` identik
memberi lima hasil berbeda sejak tick pertama; satu gagal `lane_departure`.
Fisika CARLA sendiri terbukti deterministik (throttle tetap, 4 run, selisih 0).

Perbaikan: semua perintah lewat `simulation.tick` (`apply_batch_sync`, kembali
setelah perintah diterapkan). Ditegakkan `tests/test_arsitektur.py`.

**Dampak pada bobot lateral: kesimpulan tidak berubah.** Overshoot bergeser
0,5-1 poin persen, urutan dan titik optimum sama:

| `Q_psi` | overshoot / settling (lama) | (race diperbaiki) |
|---|---|---|
| 10 | 26,4% / 2,35 s | 27,8% / 2,35 s |
| 300 | 3,3% / 0,90 s | 3,9% / 0,90 s |
| 350 | 1,4% / 0,95 s | 2,0% / 0,95 s |
| **450** | **0,0% / 1,05 s** | **0,0% / 1,05 s** |
| 600 | 0,0% / 1,25 s | 0,0% / 1,20 s |

### 10.2 Cacat 2: `ThrottlePI` me-reset integrator pada `a_ref` negatif sekecil apa pun

Pola sapuan `kp` tidak masuk akal secara teori: `v err` 0,105 di 0,25, **0,021**
di 0,3, 0,096 di 0,35. Pada `kp = 0,3` yang sama, `Q_psi = 600` atau `Q_Y = 28`
juga melonjak ke ~0,10. Tanda kejadian diskret, bukan kualitas pengendali.

Perpindahan gigi diperiksa lebih dulu dan **gugur**: seluruhnya terjadi saat
pemanasan, gigi tetap 3 selama pengukuran. Penyebab sebenarnya, dari log per tick:

| | `kp = 0,25` | `kp = 0,3` |
|---|---|---|
| `a_cmd` minimum | **-0,0016 m/s²** (t = 0,95 s) | +0,0056 |
| Throttle di t = 1,00 s | **0,020** (dari 0,38) | 0,415 |
| Kecepatan minimum | 46,9 km/jam | 48,0 |

Kode lama: `if a_ref < 0: self.i = 0; return rem`. Permintaan -0,0016 m/s² --
praktis nol -- membuang integrator yang menahan throttle jelajah, throttle putus,
mobil melambat. `kp = 0,3` "menang" hanya karena `a_cmd`-nya kebetulan tidak
pernah menyentuh negatif.

**Perbaikan — split-range:** rem hanya aktif bila keluaran PI sudah jenuh di nol;
integrator dibekukan saat jenuh (anti-windup), tidak di-reset. Tanpa konstanta
baru. Dikunci `test_throttle_pi`.

**Peta throttle -> percepatan terukur** (tahan 48 km/jam 4 s, lepas, fit linear
kecepatan 0,5-2,5 s):

| Throttle | Percepatan | Gigi |
|---|---|---|
| 0,0 | -0,10 m/s² | 0 (netral, meluncur) |
| 0,1 | -3,40 | 1 |
| 0,2 | -2,42 | 2 |
| 0,3 | -1,76 | 2 |
| 0,4 | -0,18 | 3 |
| 0,5 | +0,53 | 3 |
| 0,6 | +1,39 | 4 |

Plant sangat nonlinier: throttle kecil memicu turun gigi dan engine braking
lebih kuat daripada throttle nol. Ambang rem berupa satu angka "perlambatan
meluncur" karena itu tidak sah -- alasan split-range dipilih. Di sekitar jelajah
(throttle 0,4-0,6) gain lokal `K = 7,1-8,6 m/s²` per satuan throttle; throttle
jelajah ~0,43.

**Sapuan ulang dengan PI split-range.** `kp` (`ki = 0,25`):

| `kp` | `v err` (m/s) | `v min` (km/jam) | catatan |
|---|---|---|---|
| 0,035 | 0,016 | 48,2 | |
| 0,07 | 0,013 | 48,2 | |
| 0,08 (proposal) | 0,012 | 48,2 | dulu 0,110 |
| **0,14** | **0,011** | **48,2** | = 1/K |
| 0,28 | 0,011 | 48,2 | |
| 0,3 | 0,012 | 48,1 | dulu "optimum" 0,021 |
| 0,56 | 0,183 | 47,0 | berosilasi, chatter kemudi naik |

`ki` (`kp = 0,14`):

| `ki` | `v err` | `v min` |
|---|---|---|
| 0,0625 | 0,096 | 47,6 |
| 0,125 | 0,018 | 48,0 |
| **0,25** | **0,011** | **48,2** |
| 0,5 | 0,013 | 48,2 |
| 1,0 | 0,084 | 47,6 |

Bobot lateral diulang sekali lagi dengan PI baru -- tetap:

| `Q_psi` | 225 | 350 | **450** | 550 | 900 |
|---|---|---|---|---|---|
| overshoot | 7,9% | 2,2% | **0,0%** | 0,0% | 0,0% |
| settling | 1,30 s | 0,95 s | **1,00 s** | 1,10 s | 1,50 s |

| `Q_Y` | 10 | 14 | **20** | 28 | 40 |
|---|---|---|---|---|---|
| overshoot | 0,0% | 0,0% | **0,0%** | 2,9% | 7,6% |
| settling | 1,50 s | 1,25 s | **1,00 s** | 0,90 s | 1,25 s |

| `Rd_delta` | 10 | **20** | 40 |
|---|---|---|---|
| overshoot | 0,0% | **0,0%** | 0,3% |
| settling | 1,05 s | **1,00 s** | 1,00 s |

Pengulangan konfigurasi identik (`ki` 0,25 / 0,1 / 0,25) memberi baris identik.

### 10.3 Dampak di skenario S1 penuh

| | Sebelum | Race diperbaiki | + PI split-range, `kp` 0,14 |
|---|---|---|---|
| Deterministik | **tidak** (5 run, 5 hasil) | ya | ya |
| Vonis | 4 berhasil, **1 gagal** | berhasil | berhasil |
| Jarak minimum antar bodi | 1,32-1,54 m | 1,76 m | 1,71 m |
| Rentang kecepatan | 47-50 km/jam | 47-50 | **48-50** |
| Overshoot saat kembali | +0,27 m | +0,15 m | +0,31 m |
| Solver gagal | 0 | 0 | 0 |

Kecepatan kini tertahan lebih rapat, dan overshoot lateral saat kembali ke
lajur naik ke +0,31 m -- masih di bawah ambang lulus 0,5 m. **Belum dijelaskan**;
tersangka: acuan planner yang di-anchor ulang tiap 100 ms pada kecepatan lebih
tinggi. Periksa sebelum bab 4.

### 10.4 Metrik menyesatkan, kali keempat

"Deviasi dari tengah lajur saat `LANE_KEEPING`" (README: 0,120 m) **bukan ukuran
menjaga lajur**. Dipisah per fase:

| | Sebelum manuver | Setelah kembali |
|---|---|---|
| Deviasi rata-rata | **0,001 m** | 0,21-0,27 m |

FSM sudah menyatakan `LANE_KEEPING` saat `|d| < 0,3 m`, padahal mobil masih
mapan dari manuver kembali. Angka itu mengukur ekor manuver. Pisahkan saat
mendefinisikan metrik bab 4.

---

## 11. Dasar akademis pemilihan nilai dan titik sapuan

### 11.1 Kenapa step response, dan kenapa -1,0 m

Step response dipakai untuk mencirikan respons transien lup tertutup: overshoot
maksimum dan settling time didefinisikan terhadapnya. Settling memakai pita 5%.
Definisi keduanya dinyatakan eksplisit di bagian 3, sehingga tidak bergantung
pada rujukan buku teks.

Besar lompatan 1,0 m dipilih **di bawah setengah lebar lajur** (1,75 m) supaya
yang terukur adalah bobot, bukan saturasi: `delta` maksimum yang terjadi
0,12-0,15 rad, jauh di bawah batas 0,5 rad, dan sudut hadap tetap kecil
sehingga `sin(psi) ~ psi`. Tanda negatif = arah menyalip (`SIDE_SIGN = -1`).

### 11.2 Urutan prioritas kriteria

1. **Overshoot = 0.** Ruang bebas tiap sisi di dalam lajur `(3,50 - 1,88)/2 =
   0,81 m`. Overshoot lateral berarti bodi menerobos lajur sebelah; keselamatan
   didahulukan.
2. **Settling tercepat** di antara yang memenuhi 1.
3. **Kehalusan** (chatter) -- hanya pemecah seri.

### 11.3 Kenapa titik sapuan berkelipatan (deret geometrik)

**Argumen matematis: hanya RASIO bobot yang bermakna.** Mengalikan seluruh bobot
biaya dengan konstanta `c > 0` memberi `J' = c * J`, dan `argmin J' = argmin J`
-- solusi optimal tidak berubah. Jadi yang menentukan perilaku adalah rasio
seperti `Q_psi / Q_Y`, dan rasio tersebar di beberapa orde besaran. Sampling yang
adil terhadap rasio adalah **langkah perkalian tetap** (skala logaritmik), bukan
langkah penjumlahan tetap. Langkah tambah 100 akan memboroskan titik di
100-1000 dan melompati 10-100 seluruhnya.

Deret yang dipakai:

| Tahap | Deret | Pola |
|---|---|---|
| Kasar `Q_Y` | 0,5 1 2 5 10 20 60 | 1-2-5 per dekade (tiap dekade dibagi ~x2, x2,5, x2) |
| Kasar `Q_psi` | 10 40 100 300 600 1200 2400 | ~x2,5-x4, lalu x2 di atas 300 |
| Kasar `Rd` | 20 80 300 1000 | ~x4 |
| `kp` | 0,035 0,07 0,14 0,28 0,56 | x2, berpusat di 1/K plant |
| `ki` | 0,0625 0,125 0,25 0,5 1,0 | x2, berpusat di nilai proposal |
| Halus `Q_Y` | 10 14 20 28 40 | x√2 (setengah oktaf) |
| Halus `Q_psi` | 350 450 550 650 | +100 di dalam kurung [300, 600] |

**Kasar-ke-halus.** Sapuan kasar logaritmik mencari **kurung** tempat perilaku
berubah (overshoot > 0 menjadi = 0 terjadi di antara 300 dan 600). Di dalam
kurung selebar x2, langkah tambah dan langkah kali praktis setara, jadi
penyempitan boleh linear: langkah 100 = ~22% dari 450, cukup halus untuk
menangkap transisi 1,4% -> 0,0%.

**x√2 untuk `Q_Y`.** Titik 10-14-20-28-40 berjarak rasio sama (√2 = 1,41), jadi
tiap langkah punya bobot logaritmik setara dan 20 berada tepat di tengah.

### 11.4 Kenapa satu parameter per waktu tetap sah

Penyetelan dilakukan satu parameter per waktu (koordinat demi koordinat), dengan
`Q_Y` **disapu ulang** setelah `Q_psi` berubah untuk menangkap interaksi. Titik
optimum `Q_Y` tidak bergeser (20 di kedua sapuan), jadi prosesnya sudah
konvergen.

Pemisahan lateral vs kecepatan didukung data, bukan asumsi: di seluruh sapuan
`Q_psi`, `Q_Y`, dan `Rd` error kecepatan tetap 0,008-0,016 m/s; di seluruh
sapuan `kp` dan `ki` overshoot tetap 0,0% dan settling 1,00 s. Kedua lup tidak
saling memengaruhi secara terukur.

### 11.5 `Q_psi = 450` — tafsiran fisik dan analogi redaman kritis

Galat arah `psi` menghasilkan galat lateral `v * tau * psi` setelah `tau` detik.
Menyamakan biayanya, `Q_psi * psi² = Q_Y * (v * tau * psi)²`, memberi:

```
tau = sqrt(Q_psi / Q_Y) / v
```

| | `Q_psi` | `sqrt(Q_psi/Q_Y)` | `tau` (v = 13,4 m/s) |
|---|---|---|---|
| Proposal | 10 | 0,71 m | **0,053 s** ~ satu tick simulasi |
| Tertuning | 450 | 4,74 m | **0,354 s** |

Artinya: bobot proposal praktis tidak menghukum arah hadap -- galat arah
diperlakukan setara galat lateral hanya 50 ms ke depan. Itu sebabnya overshoot
26-28%. Pada 450, MPC menghukum arah hadap seolah galat lateral 0,35 detik
(4,7 m) ke depan.

Pola sapuan mengikuti sistem orde dua terhadap rasio redaman: di bawah 450
kurang teredam (overshoot 7,9% -> 2,2%), di atas 450 terlalu teredam (settling
memanjang monoton 1,00 -> 1,10 -> 1,50 s). Respons tercepat tanpa overshoot pada
sistem orde dua adalah redaman kritis.
**`Q_psi = 450` adalah titik redaman kritis empiris**: nilai terkecil tanpa
overshoot.

**Batasan yang harus ditulis:** `tau` berbanding terbalik dengan `v`. Pada
`V_MAX = 13,9 m/s`, `tau` turun ke 0,341 s, di antara titik 350 (0,31 s,
overshoot 2,2%) dan 450. Pada kecepatan maksimum, overshoot kecil bisa muncul.
Kalau skenario lain berjalan di atas `V_REF`, `Q_psi = 550` (0,0%, settling
1,10 s) adalah pilihan yang lebih bermargin.

### 11.6 `Q_Y = 20` dan `Rd_delta = 20` — nilai proposal dipertahankan dengan bukti

Keduanya disapu dan optimum jatuh tepat di nilai proposal: `Q_Y` 14 lebih lambat
(1,25 s), 28 memunculkan overshoot (2,9%); `Rd` 40 memunculkan overshoot 0,3%
tanpa perbaikan settling, dan chatter mapan di semua nilai ~1e-4 mrad (derau
numerik). Mempertahankan nilai proposal di sini adalah **hasil**, bukan
kelalaian: ada sapuan yang membuktikannya.

### 11.7 `kp = 0,14` — dari identifikasi plant, bukan dari coba-coba

Suku umpan maju `kp * a_ref` dalam `ThrottlePI` bermakna "throttle tambahan per
m/s² yang diminta". Bila `kp = 1/K`, dengan `K` gain plant, suku itu memberi tepat
throttle yang dibutuhkan dan penguatan lup `K * kp ~ 1`.

- `K` terukur 7,1-8,6 m/s² per satuan throttle di jelajah (bagian 10.2), jadi
  `1/K = 0,12-0,14`.
- Plateau `v err` <= 0,016 membentang 0,035-0,3 (hampir satu dekade); rusak di
  0,56 (penguatan lup ~4, berosilasi).
- `kp = 0,14` memberi margin x2 ke titik 0,28 dan x4 ke titik rusak 0,56. Margin
  itu penting karena `K` sendiri berubah dengan gigi (bagian 10.2) dan derau
  perception Tahap 8 akan menambah gangguan.

**Aturan pilih yang dipakai: tengah plateau, bukan titik minimum.** Di dalam
plateau selisih 0,011 vs 0,013 m/s berada di bawah lantai derau uji; memilih
minimum berarti mengejar derau. Tengah plateau memaksimalkan jarak ke kedua tepi
kegagalan -- itu kekokohan, dan itu yang bisa dipertahankan di sidang.

### 11.8 `ki = 0,25` — tengah geometrik plateau

Plateau 0,125-0,5; `sqrt(0,125 * 0,5) = 0,25` tepat. Margin x2 ke kedua tepi:
0,0625 terlalu lamban menutup error (0,096), 1,0 berosilasi (0,084). Waktu
integral `T_i = kp/ki = 0,56 s`, sekitar dua kali tetapan waktu filter
percepatan (~0,25 s): integrator bekerja lebih lambat daripada jeda pengukuran,
syarat umum agar lup PI tidak mengejar derau.

### 11.9 Kenapa PI, bukan PID

1. Yang dikendalikan adalah percepatan -- sudah turunan kecepatan -- dan hasil
   ukurnya berderau berat (-26,8..+12,8 m/s² mentah). Suku D menurunkannya lagi
   dan memperbesar derau.
2. Fungsi antisipasi suku D sudah dijalankan horizon prediksi MPC 2 detik dan
   bobot `Rd`.
3. Plant throttle -> percepatan dekat orde satu di sekitar titik kerja; PI cukup
   untuk menghapus error tunak.

Rujukan (terbit <= 4 tahun, isi sudah dibuka):

- **Yuan & Zhao (2022)** -- MPC di level atas menghasilkan percepatan yang
  diinginkan; level bawah memakai **pengendali PI** untuk mengubahnya menjadi
  perintah throttle/rem, dengan pemilihan throttle vs rem berdasarkan tanda
  percepatan. Skripsi ini memakai struktur yang sama, **kecuali** pemilihan
  berdasarkan tanda: di CARLA itulah penyebab cacat bagian 10.2, karena throttle
  rendah sudah menghasilkan engine braking -- maka diganti split-range.
- **Pitschi dkk. (2025)** -- kendali percepatan mereka memakai feedforward plus
  umpan balik PID; skripsi ini tidak memakai suku D karena alasan butir 1 di atas.
  Makalah yang sama menyatakan mesin memberi *drag torque* pada throttle rendah
  yang harus diperhitungkan saat mengerem -- sejalan dengan hasil ukur bagian
  10.2 dan menjadi dasar logika split-range.

### 11.10 Kenapa MPC tidak langsung mengeluarkan throttle/rem (HLC-LLC)

MPC (pengendali tingkat tinggi, HLC) menghasilkan percepatan dan sudut kemudi;
`ThrottlePI` (pengendali tingkat rendah, LLC) menerjemahkan percepatan menjadi
throttle/rem. Alasannya **bukan keterbatasan perangkat**:

| Alasan | Bukti |
|---|---|
| Model powertrain tidak tersedia | CARLA tidak mengekspos kurva torsi, logika transmisi, maupun rem -- plant kotak hitam |
| Plant tidak mulus, tidak monoton | throttle 0 -> -0,10 m/s², throttle 0,1 -> **-3,40 m/s²** (turun gigi, bagian 10.2); IPOPT berbasis gradien butuh model mulus |
| Pemetaan statis tidak memadai | Pitschi dkk. (2025): peta statis throttle "suboptimal", drag torque pada throttle rendah |
| Model prediksi yang tervalidasi memakai percepatan | validasi Tahap 1 (0,309 m @ 2 s) memakai percepatan dan sudut roda terukur sebagai masukan |
| Perangkat bukan hambatan | solve 13-14 ms dari anggaran 50 ms, di CPU; GPU tidak dipakai IPOPT |
| Struktur lazim | Yuan & Zhao (2022): MPC atas -> percepatan, PI bawah -> throttle/rem |

Di sisi lateral MPC memang "turun" sampai aktuator: sudut kemudi diubah ke
perintah steer dengan satu rumus statis, terverifikasi meleset <= 4% (Tahap 5).
Tidak butuh LLC karena hubungan itu statis, berbeda dengan powertrain.

Kalau MPC memodelkan transmisi, masalahnya menjadi hybrid (gigi diskret) --
bentuk mixed-integer yang tidak bisa diselesaikan IPOPT dan jauh lebih berat.
Di situlah perangkat baru ikut membatasi, tapi hambatan pertamanya tetap
ketiadaan model.

**Daftar pustaka bagian ini:**

- Yuan, T., & Zhao, R. (2022). LQR-MPC-Based Trajectory-Tracking Controller of
  Autonomous Vehicle Subject to Coupling Effects and Driving State Uncertainties.
  *Sensors*, 22(15), 5556. https://doi.org/10.3390/s22155556
- Pitschi, P., Sagmeister, S., Goblirsch, S., Lienkamp, M., & Lohmann, B. (2025).
  Longitudinal Control for Autonomous Racing with Combustion Engine Vehicles.
  arXiv:2504.17418. https://arxiv.org/abs/2504.17418

---

## 12. Behavior FSM — mengikuti kendaraan depan dan menyalip ulang

11 September 2026. Skenario uji: S3 (`config.SKENARIO`) -- target 7,0 m/s 60 m di
depan pada lajur ego, dan kendaraan di lajur tujuan mulai 10 m di belakang ego
pada 13,9 m/s. Ego tidak boleh langsung menyalip; ia harus menunggu.

### 12.1 Masalah pada FSM lama

1. Selama `LANE_KEEPING` dan `CHECK_OVERTAKE` planner selalu diberi `V_REF`. Bila
   lajur tujuan terisi atau `_sempat` (TTC > 3 s) gugur, FSM tertahan di
   `CHECK_OVERTAKE` -- syarat batal butuh TTC > 7 s -- sementara ego terus mendekat.
2. Pemicu memakai TTC terhadap `v_ego`. Ego yang sudah melambat ke kecepatan
   kendaraan depan punya TTC tak hingga, jadi tidak akan pernah memicu menyalip lagi.

Bukti (S3, FSM versi commit sebelumnya): tertahan di `CHECK_OVERTAKE`, jarak bodi
ke target **0,00 m** pada t = 9,05 s, ke kendaraan lajur tujuan 0,04 m -- GAGAL.

### 12.2 Perubahan

| Perubahan | Isi |
|---|---|
| Mengikuti | `v_goal = v_depan + 2e/T`, `e = celah - d*`, `d* = ELLIPSE_A + WAKTU_IKUT·v_depan`, `T = max(MANEUVER_TIMES)` |
| Kapan mengikuti | hanya bila menyalip **tidak mungkin**: selisih kecepatan <= `DV_TRIGGER`, waktu tidak cukup, atau lajur tujuan terisi |
| Pemicu & batal | TTC dihitung dengan `max(v_ego, V_REF)` |

Perilaku yang dihasilkan adalah **accelerative overtaking**: pengemudi melambat dan
mengikuti kendaraan depan beberapa saat sebelum menyalip -- berlawanan dengan
*flying overtaking* yang mempertahankan kecepatan (S1). Definisi keduanya dari
Fabricius dkk. (2022), yang merangkum Dozza dkk. (2016).

### 12.3 Dasar tiap bagian

**Jarak ikut `d*` -- kebijakan jarak waktu-tetap.** Jarak yang diinginkan naik linier
dengan kecepatan: `s_des = s0 + L + tau·v` (El-Baklish dkk., 2025, bagian 2);
kebijakan ini "commonly employed" pada ACC (Lee & Lee, 2025, bagian 2.1). Di sini
`s0 + L` diganti `ELLIPSE_A` (7 m, diukur dari pusat kendaraan depan ke sumbu
belakang ego), sehingga posisi mengikuti **tidak pernah melanggar elips MPC
sendiri**: pada `dy = 0`, `g = (d*/ELLIPSE_A)² >= 1`.

**Gain `2/T` -- diturunkan dari planner, bukan disetel.** Quartic longitudinal
mengubah kecepatan secara halus selama durasi manuver `T` dengan percepatan awal
dan akhir nol. Menutup selisih kecepatan `dv` dengan profil seperti itu menempuh
jarak relatif `dv·T/2`. Agar jarak itu tidak melampaui ruang tersisa `e`:
`dv <= 2e/T`. Dengan `T = 4 s`, gain 0,5 1/s; konstanta perlambatan terpisah tidak
diperlukan.

Hukum yang lebih dulu dicoba, `dv = sqrt(2·a·e)` (gerak berubah beraturan,
`a = 2 m/s²`), menganggap kecepatan berubah seketika. Karena planner butuh 3-4 s,
ego kebablasan: celah 14,3 m terhadap `d*` 17,5 m, lalu mundur ke 4,7 m/s di
belakang kendaraan 7 m/s.

**Hanya bila menyalip tidak mungkin.** Melambat saat menyalip masih mungkin
membuang selisih kecepatan yang justru dipakai untuk menyalip. Tanpa syarat ini,
`d*` 21 m membuat ego mulai melambat di celah 33,8 m, sebelum pemicu menyalip S1
di ~32 m. Dikunci `test_tidak_melambat_bila_bisa_menyalip`.

**`max(v_ego, V_REF)` untuk pemicu.** Alasan menyalip adalah kendaraan depan lebih
lambat daripada kecepatan yang **diinginkan**, bukan kecepatan saat ini. Saat ego
mendekat dengan `v_ego >= V_REF`, nilainya sama dengan sebelumnya; seluruh 17 uji
FSM lama tetap lolos tanpa diubah.

### 12.4 Artefak skenario yang disingkirkan sebelum menyetel

| Gejala | Penyebab | Penanganan |
|---|---|---|
| Tabrakan guardrail di t ~ 30 s | run 35 s melewati ujung ruas lurus 400 m; FSM lama menabrak di titik yang sama | S3 dijalankan 25 s |
| Jarak bodi 0,87 m ke kendaraan lajur tujuan saat ego diam di lajurnya | kecepatan dipaksa searah hadap aktor; gaya ban memutar arah hadap, kendaraan bergeser -3,50 -> -2,83 m | kecepatan dipaksa searah jalan; drift tinggal -3,60..-3,52 m |

### 12.5 Sapuan `WAKTU_IKUT` (S3, 25 s)

> **Kesimpulan bagian ini direvisi di bagian 13.7.** Sapuan di bawah diukur dengan
> elips lama yang tidak menjamin `JARAK_AMAN` (bagian 13.1), jadi titik kritisnya
> ditentukan oleh cacat itu, bukan oleh jarak ikut.

Simulasi deterministik (dua run `WAKTU_IKUT` 2,0 s identik bit-per-bit), jadi satu
run per nilai sudah menggambarkan konfigurasi itu.

| `WAKTU_IKUT` | Celah ikut min | Jarak bodi ke target | Fase jarak terdekat | Perlambatan min | Mulai salip | Vonis |
|---|---|---|---|---|---|---|
| FSM lama | -- | 0,00 m | `CHECK_OVERTAKE` | -1,79 m/s² | tidak pernah | GAGAL |
| 1,0 s | 11,2 m | 0,07 m | `LANE_CHANGE_OVERTAKE` | -1,83 | 12,50 s | GAGAL |
| 1,5 s | 13,5 m | 0,59 m | `LANE_CHANGE_OVERTAKE` | -1,81 | 12,10 s | GAGAL |
| **2,0 s** | **16,6 m** | **1,42 m** | `OVERTAKING` | -1,92 | 11,60 s | **BERHASIL** |
| 2,5 s | 20,0 m | 1,13 m | `OVERTAKING` | -2,00 | 11,10 s | BERHASIL |

Jarak minimum ke kendaraan lajur tujuan 1,62 m di semua nilai.

**Kenapa 2,0 s:**

1. Pada 1,0 dan 1,5 s titik kritis terjadi saat **pindah lajur**: ego mulai dari
   posisi mengikuti yang terlalu dekat dan sudah sampai di sisi target sebelum
   perpindahan lateral selesai. Di sini jarak ikut yang mengikat.
2. Mulai 2,0 s titik kritis pindah ke fase **berpapasan** (`OVERTAKING`), yang
   ditentukan geometri lajur, bukan jarak ikut.
3. Menaikkan ke 2,5 s tidak menambah keselamatan (1,13 < 1,42 m), memperlambat
   manuver (19,2 vs 18,9 s), dan butuh perlambatan lebih besar.

Jadi 2,0 s adalah **nilai terkecil yang lepas dari kekangan fase pindah lajur**.
Batasan: resolusi sapuan 0,5 s dan hanya satu skenario; perlu diulang saat S2, S4,
S5 dan vision perception tersedia.

Tidak ada sumber <= 4 tahun yang terverifikasi untuk nilai time gap standar ACC,
sehingga nilai ini disandarkan pada sapuan di atas, bukan pada standar.

### 12.6 Regresi S1

| | Sebelum FSM diubah | Sesudah |
|---|---|---|
| Vonis | BERHASIL | BERHASIL |
| Jarak bodi minimum | 1,71 m | 1,43 m |
| `v_goal` minimum | -- | 48,2 km/jam (mengikuti tidak pernah aktif) |
| Deterministik | ya | ya |

Penurunan 1,71 -> 1,43 m berasal dari perbaikan drift (12.4): dulu target
bergeser +0,3 m menjauhi lintasan ego, sekarang tetap di tengah lajur. Angka lama
sedikit terlalu optimis.

### 12.7 Terbuka: elips tidak menjamin `JARAK_AMAN` — SELESAI di bagian 13

Dimensi terukur: ego 5,01 × 1,88 m, Nissan Patrol 4,60 × 1,93 m. Tepat di batas
elips saat berpapasan (`dx = 0`), `ELLIPSE_B = 2,2 m` setara jarak bodi
`2,2 - (1,88 + 1,93)/2 = 0,29 m`, jauh di bawah syarat lulus 1,0 m. Jarak berpapasan
1,42 m saat ini dijamin geometri lajur, bukan constraint -- dan sudah turun ke
1,13 m pada `WAKTU_IKUT` 2,5 s.

Agar constraint menjamin syarat lulus: `ELLIPSE_B >= (1,88 + 1,93)/2 + 1,0 = 2,91 m`.
Mengubahnya memengaruhi saringan planner dan constraint MPC, jadi S1 dan S3 harus
diverifikasi ulang. Belum dikerjakan.

### 12.8 Daftar pustaka bagian ini

- Fabricius, V., Habibovic, A., Rizgary, D., Andersson, J., & Wärnestål, P. (2022).
  Interactions between heavy trucks and vulnerable road users—A systematic review
  to inform the interactive capabilities of highly automated trucks. *Frontiers in
  Robotics and AI*, 9, 818019. https://doi.org/10.3389/frobt.2022.818019
- El-Baklish, S. K., Kouvelas, A., & Makridis, M. A. (2025). Driving Towards
  Stability and Efficiency: A Variable Time Gap Strategy for Adaptive Cruise
  Control. arXiv:2402.14110. https://arxiv.org/abs/2402.14110
- Lee, K., & Lee, C. (2025). String Stability Analysis and Design Guidelines for PD
  Controllers in Adaptive Cruise Control Systems. *Sensors*, 25(11), 3518.
  https://doi.org/10.3390/s25113518


---

## 13. Zona aman, gerbang kembali, dan lup planner-MPC

11-12 September 2026. Berangkat dari satu pertanyaan: apakah constraint yang
dipakai planner dan MPC benar-benar menjamin syarat lulus jarak aman 1,0 m?

### 13.1 Jawabannya: tidak

Elips lama `A = 7,0`, `B = 2,2` diukur dari **sumbu belakang** ego. Saat
berpapasan (`dx = 0`) batas elips ada di jarak pusat lateral 2,2 m. Dengan lebar
ego 1,88 m dan Nissan Patrol 1,93 m, itu setara jarak bodi
`2,2 - (1,88 + 1,93)/2 = 0,29 m` -- jauh di bawah `JARAK_AMAN = 1,0 m`.

Jarak berpapasan yang tercapai selama ini (1,4-1,9 m) datang dari geometri lajur,
bukan dari constraint. Pada `WAKTU_IKUT = 2,5 s` angkanya sudah turun ke 1,13 m.

### 13.2 Syarat yang benar dan bentuk zona

Jarak bodi >= `JARAK_AMAN` dijamin bila zona memuat seluruh **persegi terlarang**
antar pusat bodi:

| Setengah sisi | Rumus | Nilai |
|---|---|---|
| memanjang | `(L_ego + L_lain)/2 + JARAK_AMAN` | 5,81 m |
| melintang | `(W_ego + W_lain)/2 + JARAK_AMAN` | 2,91 m |

State MPC adalah sumbu belakang, jadi pusat bodi digeser `SUMBU_KE_PUSAT` = 1,433 m
(terukur, dikunci `test_dimensi_ego_di_config_sama_dengan_hasil_ukur`).

**Elips biasa tidak praktis.** `B` harus lebih kecil dari lebar lajur (3,50 m),
kalau tidak berpapasan di tengah lajur sebelah jadi tidak layak dan menyalip
mustahil. Dengan `B <= 3,4 m`, memuat sudut persegi menuntut `A` 11-14 m.

**Elips-super pangkat 4 cukup dengan `A = 7,71 m`.** Superelipsoid memang dipakai
justru karena alasan ini: geometri persegi seperti kendaraan terwakili lebih akurat
dengan kompleksitas yang sama (Moran dkk., 2024, bagian 7; mereka memakai p = 3).

Bentuknya:

```
g = ((dx/A)^p + (dy/B)^p)^(1/p) >= 1
```

Akar ke-p penting untuk penskalaan: `g` jadi berderajat satu (berskala seperti
jarak). Tanpa akar, slot obstacle kosong MPC yang diparkir 1e4 m memberi gradien
orde 1e9 dan penskalaan solver rusak.

Kedua parameter **diturunkan di `config.py`, bukan diketik**:

- `B` = tengah antara 2,91 m (batas perlu) dan 3,50 m (berpapasan di tengah lajur
  sebelah) = **3,204 m**; margin ~0,30 m ke masing-masing sisi.
- `A` = nilai terkecil yang masih memuat sudut persegi = **7,709 m**.

Dikunci `test_zona_aman_memuat_persegi_terlarang`: seluruh tepi persegi ada di
dalam zona, sementara berpapasan di tengah lajur sebelah tetap layak.

### 13.3 Hipotesis yang gugur: margin `B` kurang

Setelah zona dipasang, `WAKTU_IKUT` 1,0 dan 2,5 s gagal `lane_departure` (ego
terlempar ke y -4,89 dan -5,36 m). Dugaan pertama: margin `B` terhadap jalur
berpapasan terlalu tipis. Diuji dengan p = 6 supaya `B` bisa mengecil tanpa `A`
meledak: (B, A) = (3,0; 7,78), (3,1; 7,02), (3,2; 6,66), masing-masing untuk S1
dan S3 pada `WAKTU_IKUT` 1,5 / 2,0 / 2,5 s.

**Hasilnya praktis sama di ketiganya**, dan `WAKTU_IKUT = 2,5 s` tetap gagal di
semua varian. Bentuk zona bukan penyebabnya. Karena tidak ada beda terukur,
dipakai p = 4, yang paling dekat dengan elips.

### 13.4 Gerbang kembali `DD_KEMBALI` — mengobati gejala

Dari 16 run terkumpul korelasi yang bersih:

| | Laju lateral saat mulai kembali | Hasil |
|---|---|---|
| 5 run gagal | 0,91-1,89 m/s **menjauhi** lajur asal | kebablasan 0,47-1,23 m, lalu berayun ke lajur asal |
| 11 run lolos | 0,19-1,10 m/s **menuju** lajur asal | tidak ada kebablasan |

Quintic kembali (T = 4 s) yang berangkat dengan laju menjauh `u` kebablasan ke
luar: `u = 0,1` -> 0,017 m; `u = 0,91` -> 0,40 m (terukur 0,47 m). Derau laju
lateral saat menjaga lajur maksimum 0,0014 m/s, jadi ambang `DD_KEMBALI = 0,1 m/s`
~70x di atas derau dan jauh di bawah kasus gagal.

Setelah gerbang dipasang, ego memang selalu mulai kembali saat sudah bergerak
masuk -- **tapi 1,0 dan 2,5 s tetap gagal**: ego sudah terlempar ke -5,16 dan
-5,77 m selagi `OVERTAKING`, dan gerbang hanya membuatnya menunggu di sana.
Gerbang mengobati gejala. Ia tetap dipertahankan sebagai pengaman, dan setelah
akar masalahnya diperbaiki (13.5) tidak pernah terpicu lagi (0 tick di semua run),
jadi ia tidak menutupi apa pun.

### 13.5 Akar masalah: lup planner-MPC lewat percepatan lateral terukur

Planner dijalankan ulang secara offline dari log per tick (`WAKTU_IKUT` 2,5 s):

| t | Kejadian |
|---|---|
| 15,5-16,3 s | ego sejajar target; kandidat yang ditolak zona naik 3 -> 8 dari 9 |
| 16,4-16,6 s | **nol kandidat**; acuan cadangan + constraint zona MPC memberi steer +0,059 (tendangan ke luar) |
| 16,7-19,5 s | seluruh 9 kandidat layak lagi dan target tetap -3,50 m, **tapi lintasan pilihan sendiri makin menukik**: ymin -3,79 -> -4,16 -> ... -> -5,77 m |

Kecepatan lateral awal hanya menyumbang ~0,18 m (tabel 13.4). Sisanya dari
**percepatan lateral awal**: planner memakai `y''` hasil ukur sebagai syarat awal,
MPC mengikuti kelengkungan awal lintasan itu, percepatan yang dihasilkan terukur
lagi, lalu jadi syarat awal rencana berikutnya. Umpan balik positif antara planner
dan pengendali; satu tendangan kecil tumbuh jadi 2,3 m keluar lajur.

Ini **pengulangan TEMUAN 2 Tahap 5 di sisi lateral**. Untuk longitudinal, `a0`
quartic sudah lama memakai percepatan yang *diperintahkan*, bukan hasil ukur.
Sisi lateral tidak diperlakukan sama karena deraunya kecil -- dan itu keliru:
masalahnya bukan derau, melainkan lup.

**Perbaikan:** `y''` awal diambil dari lintasan rencana sebelumnya pada waktu
sekarang (`Trajectory.lateral_at`, fungsi yang memang sudah ada untuk replan
receding). Posisi dan kecepatan lateral tetap hasil ukur, supaya planner tetap
ter-anchor ke keadaan nyata.

### 13.6 Hasil

Skenario S1, sebelum dan sesudah seluruh perubahan bagian 13:

| | Elips lama | Zona baru | Zona + `y''` dari rencana |
|---|---|---|---|
| Vonis | BERHASIL | BERHASIL | BERHASIL |
| Jarak bodi minimum | 1,43 m | 1,88 m | 1,39 m |
| Deviasi lajur rata-rata | 0,161 m | 0,122 m | **0,011 m** |
| Deviasi lajur maksimum | 0,530 m | 0,439 m | **0,152 m** |
| Simpangan lateral terjauh | -4,00 m | -4,31 m | -3,70 m |

Jarak 1,88 m pada kolom tengah bukan perbaikan kendali: ego terdorong keluar
sampai -4,31 m, tepat menyentuh garis lajur ketiga. Setelah lup diputus, ego
berpapasan dari tengah lajur tujuan dengan jarak 1,39 m dan tetap di lajurnya.

### 13.7 Pemilihan `WAKTU_IKUT` (revisi bagian 12.5)

S3, 25 detik, zona baru + `y''` dari rencana:

| `WAKTU_IKUT` | Celah ikut min | Jarak bodi | Deviasi lajur | Nol kandidat | Perlambatan min | Durasi | Vonis |
|---|---|---|---|---|---|---|---|
| 1,0 s | 13,2 m | 1,69 m | 0,010 m | 12 tick | -1,61 m/s² | 16,6 s | BERHASIL |
| 1,5 s | 15,5 m | 1,42 m | 0,011 m | 4 tick | -1,88 | 16,9 s | BERHASIL |
| **2,0 s** | **18,6 m** | **1,49 m** | **0,012 m** | **0 tick** | **-1,91** | 18,0 s | **BERHASIL** |
| 2,5 s | 22,0 m | 1,52 m | 0,012 m | 0 tick | -1,96 | 18,1 s | BERHASIL |

Keempatnya kini lolos, dan jarak bodi tidak lagi bergantung `WAKTU_IKUT` -- itu
memang yang diharapkan setelah constraint menjamin jarak aman (13.2).

**2,0 s dipilih sebagai nilai terkecil yang tidak pernah membuat planner kehabisan
kandidat.** Tick nol kandidat berarti acuan jatuh ke mode tahan-lajur, dan tepat
kejadian itulah yang dulu menendang ego keluar (13.5). Menaikkan ke 2,5 s tidak
menambah apa pun selain jarak ikut yang lebih jauh.

Batasan: resolusi sapuan 0,5 s, satu skenario, dan simulasinya deterministik
(dua run `WAKTU_IKUT` 2,0 s identik bit-per-bit) sehingga satu run per nilai
sudah menggambarkan konfigurasi itu. Perlu diulang setelah S2/S4/S5 dan vision.

### 13.8 Terbuka

- **Zona memakai kotak sejajar sumbu.** Pada yaw 7 derajat saat pindah lajur, sudut
  bodi bergeser ~0,31 m yang tidak dihitung. `evaluation.jarak_kotak` memakai
  asumsi yang sama, jadi penilaian konsisten dengan constraint -- tapi jarak bodi
  sebenarnya bisa lebih kecil daripada yang dilaporkan.
- **Dimensi kendaraan lain dianggap Nissan Patrol** (terbesar di skenario);
  perception tidak mengukur dimensi, jadi zona memakai satu ukuran tetap.

### 13.9 Daftar pustaka bagian ini

- Moran, R., Bagley, S., Kasmann, S., Martin, R., Pasley, D., Trimble, S.,
  Dianics, J., & Sopasakis, P. (2024). NMPC for Collision Avoidance by
  Superellipsoid Separation. *Modeling, Estimation, and Control Conference (MECC
  2024)*. arXiv:2404.14257. https://arxiv.org/abs/2404.14257
