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
| `kp` throttle | 0,08 | **0,3** | error kecepatan 0,110 -> 0,021 m/s |
| `ki` throttle | 0,25 | **0,25** | disapu, terbukti optimal |

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
