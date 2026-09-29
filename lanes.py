"""Geometri lajur dari kepala segmentasi YOLOPX, tanpa peta simulator (bagian 28).

MURNI NUMERIK. Tidak mengimpor carla maupun torch: masukannya masker biner yang
sudah dihasilkan `yolopx.infer`, keluarannya offset garis lajur dalam meter di
frame ego. Bisa diuji di terminal tanpa menyalakan simulator maupun GPU.

Kenapa modul ini ada: `main.siapkan_jalan` mengambil centerline langsung dari
`world.get_map()`, yaitu peta HD simulator. Padahal YOLOPX sudah menghitung
segmentasi garis lajur tiap tick -- dan hasilnya dibuang (`kotak, _, _`). Modul
ini memakainya.

Dua asumsi, keduanya sah di ruas uji dan harus ditulis di batasan masalah:

1. **Permukaan jalan datar.** Balik-proyeksinya IPM (inverse perspective
   mapping), bukan lewat depth. Sengaja: IPM hanya butuh kalibrasi kamera, tidak
   menambah ketergantungan pada sensor kedua. Ruas lurus 400 m Town04 datar.
2. **Garis lajur sejajar.** Di jalan lurus seluruh marka punya arah yang sama,
   jadi kemiringannya dicari SEKALI untuk semua dan hanya perpotongannya yang
   berbeda. Ini yang membuat pemisahannya kokoh: satu parameter bersama dicari
   dari ribuan piksel, bukan satu garis dicocokkan dari serpihan marka.
"""
import numpy as np

import config

F_PIKSEL = config.KAMERA_LEBAR / (2.0 * np.tan(np.radians(config.KAMERA_FOV) / 2.0))

JANGKAUAN = (6.0, 45.0)      # m, batas x yang dipercaya
BIN_C = 0.10                 # m, lebar bin histogram perpotongan
CARI_B = 0.30                # kemiringan maks yang dicari (~17 deg hadap)
MIN_PIKSEL = 40              # piksel minimum sebelum hasil dilaporkan
MIN_PUNCAK = 0.25            # porsi terhadap puncak tertinggi


def letterbox_ke_citra(bentuk_masker, bentuk_citra):
    """(skala, pad_u, pad_v): piksel masker -> piksel citra asli.

    `letterbox_for_img(img, 640, auto=True)` mengecilkan dengan satu rasio lalu
    memberi bingkai sampai kelipatan 32. Membalikkannya butuh keduanya -- lupa
    padding 12 piksel menggeser seluruh hasil beberapa meter.
    """
    hm, wm = bentuk_masker
    hc, wc = bentuk_citra[:2]
    r = min(640.0 / hc, 640.0 / wc)
    lebar, tinggi = int(round(wc * r)), int(round(hc * r))
    return r, (wm - lebar) / 2.0, (hm - tinggi) / 2.0


def ipm(u, v, z=config.KAMERA_Z, f=F_PIKSEL,
        cu=config.KAMERA_LEBAR / 2.0, cv=config.KAMERA_TINGGI / 2.0):
    """Piksel citra -> titik di permukaan jalan, frame ego RH (x depan, y kiri).

    Kamera menghadap lurus (pitch 0), jadi baris `v` di bawah horizon memotong
    permukaan pada x = f*z/(v - cv). Sumbu y memakai perjanjian yang sama dengan
    `perception`: y = -x*(u - cu)/f, positif ke kiri.
    """
    dv = np.asarray(v, dtype=float) - cv
    depan = dv > 1e-6
    x = np.where(depan, f * z / np.where(depan, dv, 1.0), np.inf)
    # y dihitung hanya di piksel yang memotong jalan: inf * 0 memberi nan diam-diam
    y = np.where(depan, -x * (np.asarray(u, dtype=float) - cu) / f, np.nan)
    return x, y


def ke_piksel(x, y, z=config.KAMERA_Z, f=F_PIKSEL,
              cu=config.KAMERA_LEBAR / 2.0, cv=config.KAMERA_TINGGI / 2.0):
    """Kebalikan `ipm`: titik di permukaan jalan -> piksel citra. Untuk menggambar."""
    x = np.maximum(np.asarray(x, dtype=float), 1e-3)
    return cu - np.asarray(y, dtype=float) * f / x, cv + f * z / x


def titik_lajur(masker, bentuk_citra, jangkauan=JANGKAUAN):
    """Masker garis lajur -> (N, 2) titik (x, y) meter di frame ego."""
    vm, um = np.nonzero(np.asarray(masker) > 0)
    if len(um) == 0:
        return np.empty((0, 2))
    r, pad_u, pad_v = letterbox_ke_citra(np.shape(masker), bentuk_citra)
    x, y = ipm((um - pad_u) / r, (vm - pad_v) / r)
    pakai = np.isfinite(x) & (x >= jangkauan[0]) & (x <= jangkauan[1])
    return np.column_stack([x[pakai], y[pakai]])


def _kemiringan_bersama(titik, cari=CARI_B, langkah=0.005):
    """Kemiringan yang dipakai BERSAMA semua garis lajur.

    Untuk tiap calon `b`, proyeksikan c = y - b*x lalu histogramkan. Kemiringan
    yang benar membuat tiap marka jatuh ke bin yang sama, sehingga histogramnya
    paling memuncak; yang salah mengoleskannya rata. Ketajaman diukur dengan
    jumlah kuadrat cacah -- maksimum bila massa terkumpul di sedikit bin.
    """
    x, y = titik[:, 0], titik[:, 1]
    tepi = np.arange(-12.0, 12.0 + BIN_C, BIN_C)
    calon = np.arange(-cari, cari + langkah, langkah)
    skor = np.empty(len(calon))
    for i, b in enumerate(calon):
        cacah, _ = np.histogram(y - b * x, bins=tepi)
        skor[i] = float((cacah.astype(float) ** 2).sum())
    return float(calon[int(np.argmax(skor))])


def _puncak(c, min_porsi=MIN_PUNCAK):
    """Perpotongan tiap garis lajur = modus histogram c. -> (offset, bobot)."""
    tepi = np.arange(-12.0, 12.0 + BIN_C, BIN_C)
    cacah, _ = np.histogram(c, bins=tepi)
    # haluskan 3 bin supaya satu marka tidak pecah jadi dua puncak
    halus = np.convolve(cacah.astype(float), np.ones(3) / 3.0, mode='same')
    ambang = max(halus.max() * min_porsi, 1.0)
    tengah = (tepi[:-1] + tepi[1:]) / 2.0
    offset, bobot, i = [], [], 1
    while i < len(halus) - 1:
        if halus[i] >= ambang and halus[i] >= halus[i - 1] and halus[i] > halus[i + 1]:
            # pusat massa lokal +-0,3 m: lebih halus daripada tengah bin
            dekat = np.abs(c - tengah[i]) < 0.3
            offset.append(float(c[dekat].mean()) if dekat.sum() else float(tengah[i]))
            bobot.append(float(dekat.sum()))
            i += 4                      # lompati bahu puncak yang sama
        else:
            i += 1
    if not offset:
        return np.empty(0), np.empty(0)
    urut = np.argsort(offset)
    return np.array(offset)[urut], np.array(bobot)[urut]


def _kisi(offset, bobot, lebar=(2.8, 4.4), langkah=0.01):
    """Cocokkan KISI berjarak sama ke offset garis -> (lebar, fase).

    Median jarak antar garis bersebelahan gagal begitu satu marka terlewat:
    celahnya menjadi dua kali lebar lajur dan seluruh hitungan ikut melar. Tetapi
    marka lajur di jalan berlajur banyak selalu berjarak SAMA, jadi yang dicari
    cukup satu jarak `w` dan satu fase -- dan garis yang hilang hanya menyisakan
    lubang di kisi, tidak merusaknya.

    Fase dicari lewat rerata melingkar: tiap offset dipetakan ke sudut
    2*pi*offset/w, lalu panjang resultannya mengukur seberapa rapat semuanya
    jatuh di kisi yang sama. `w` terbaik memaksimalkan panjang itu.
    """
    if len(offset) < 2:
        return None, None
    calon = np.arange(lebar[0], lebar[1] + langkah, langkah)
    terbaik, w_terbaik, fase_terbaik = -1.0, None, None
    for w in calon:
        z = (bobot * np.exp(2j * np.pi * offset / w)).sum() / max(bobot.sum(), 1e-9)
        if abs(z) > terbaik:
            terbaik, w_terbaik = float(abs(z)), float(w)
            fase_terbaik = float(np.angle(z) * w / (2.0 * np.pi))
    return w_terbaik, fase_terbaik


def _haluskan(titik, b0, iterasi=3):
    """Haluskan kemiringan bersama dengan kuadrat terkecil. -> b

    `_kemiringan_bersama` mencari lewat argmax histogram, dan itu kasar: skornya
    dihitung pada bin selebar 0,10 m, jadi puncaknya rata dan letaknya melenceng.
    Terukur pada masker sintetis, galat sudut hadap RMS 0,210 deg dengan maksimum
    0,305 deg -- padahal kameranya sempurna dan garisnya lurus sempurna. Jadi itu
    galat ESTIMATOR, bukan galat sensor.

    Memperhalus langkah pencarian TIDAK menolong (diuji: 0,005 -> 0,0005 membuat
    RMS-nya 0,210 -> 0,261, malah lebih buruk). Yang menolong mengganti pencarian
    dengan penyelesaian: tugaskan tiap titik ke garis terdekat, lalu cari satu
    kemiringan bersama dari regresi dalam-kelompok terkumpul

        b = sum_k sum_i (x - xbar_k)(y - ybar_k) / sum_k sum_i (x - xbar_k)^2

    yaitu kemiringan yang sama untuk semua garis, dengan perpotongan yang boleh
    berbeda. Hasilnya RMS 0,095 deg dan maksimum 0,102 deg -- 2,2 dan 3,0 kali
    lebih baik. Argmax tetap dipakai untuk masuk ke lembah yang benar; yang
    diganti hanya langkah terakhirnya.
    """
    x, y = titik[:, 0], titik[:, 1]
    b = b0
    for _ in range(iterasi):
        c = y - b * x
        offset, _ = _puncak(c)
        if not len(offset):
            return b
        kel = np.argmin(np.abs(c[:, None] - offset[None, :]), axis=1)
        # buang titik yang jauh dari garis mana pun: marka palsu tidak boleh
        # ikut menarik kemiringan bersama
        dekat = np.abs(c - offset[kel]) < 0.4
        if dekat.sum() < 20:
            return b
        xk, yk, kk = x[dekat], y[dekat], kel[dekat]
        atas = bawah = 0.0
        for g in np.unique(kk):
            m = kk == g
            if m.sum() < 5:
                continue
            dx, dy = xk[m] - xk[m].mean(), yk[m] - yk[m].mean()
            atas += float((dx * dy).sum())
            bawah += float((dx * dx).sum())
        if bawah <= 0.0:
            return b
        b = atas / bawah
    return b


class GeometriLajur:
    """Hasil satu frame. `offset` positif = ke kiri ego, meter."""

    def __init__(self, offset, bobot, kemiringan, n_piksel):
        self.offset = offset            # perpotongan tiap garis lajur di x = 0
        self.bobot = bobot              # piksel pendukung tiap garis
        self.kemiringan = kemiringan    # dy/dx; ~ -tan(yaw ego terhadap jalan)
        self.n_piksel = n_piksel
        self.lebar_lajur, self.fase = _kisi(offset, bobot)

    @property
    def yaw(self):
        """Sudut hadap ego terhadap arah jalan, radian."""
        return -float(np.arctan(self.kemiringan))

    @property
    def sisa_kisi(self):
        """RMS jarak garis terdeteksi ke titik kisi terdekat, meter.

        Ukuran kepercayaan: kisi yang cocok memberi sisa kecil. Frame dengan sisa
        besar berarti marka yang terbaca tidak membentuk pola berjarak sama --
        biasanya ada garis palsu -- dan hasilnya sebaiknya tidak dipakai.
        """
        if self.lebar_lajur is None:
            return None
        d = self.offset - self.fase
        return float(np.sqrt((((d - np.round(d / self.lebar_lajur) * self.lebar_lajur)
                               ** 2) * self.bobot).sum() / max(self.bobot.sum(), 1e-9)))

    @property
    def dev_lajur(self):
        """Simpangan ego dari tengah lajurnya sendiri, meter. None bila tak tentu.

        Tengah lajur = titik tengah antar dua garis kisi, jadi tetap terdefinisi
        walaupun salah satu marka pengapitnya tidak terdeteksi frame itu.
        Positif berarti ego ada di KIRI tengah lajur.
        """
        if self.lebar_lajur is None:
            return None
        w = self.lebar_lajur
        # tengah lajur ada di fase + (n + 1/2) * w; cari yang terdekat ke ego (0)
        n = np.round((0.0 - self.fase) / w - 0.5)
        return -float(self.fase + (n + 0.5) * w)

    def garis(self, jangkauan=JANGKAUAN, n=24):
        """Garis kisi sebagai lintasan di frame ego -> [(x, y), ...] per garis.

        Yang digambar KISI hasil cocokan, bukan piksel maskernya. Itu sebabnya
        garisnya tersambung penuh walaupun markanya putus-putus: yang dicari
        memang satu garis lurus per lajur, bukan sekumpulan penggal.
        """
        if self.lebar_lajur is None:
            return []
        x = np.linspace(jangkauan[0], jangkauan[1], n)
        w = self.lebar_lajur
        # Hanya slot kisi yang BENAR-BENAR didukung marka. Kisinya tak berhingga;
        # menggambar seluruhnya berarti menarik garis lajur di atas tanggul dan
        # pembatas, yaitu mengklaim sesuatu yang tidak pernah diukur.
        keluar = []
        for c in np.unique(np.round((self.offset - self.fase) / w)):
            c = self.fase + c * w
            keluar.append((x, c + self.kemiringan * x))
        return keluar

    def garis_tengah(self, jangkauan=JANGKAUAN, n=24):
        """Sumbu lajur yang sedang ditempati ego, di frame ego."""
        if self.lebar_lajur is None:
            return None
        x = np.linspace(jangkauan[0], jangkauan[1], n)
        c = -self.dev_lajur
        return x, c + self.kemiringan * x

    def tengah_lajur(self, sisi):
        """Tengah lajur sebelah, relatif ego. `sisi` = +1 kiri, -1 kanan.

        Dipakai FSM sebagai `y_goal` menggantikan `SIDE_SIGN * LANE_WIDTH`.
        """
        if self.lebar_lajur is None:
            return None
        return float(-self.dev_lajur + sisi * self.lebar_lajur)

    def __repr__(self):
        w = self.lebar_lajur
        return (f'GeometriLajur({len(self.offset)} garis, lebar='
                f'{"?" if w is None else f"{w:.2f}"} m, '
                f'yaw={np.degrees(self.yaw):+.1f} deg, {self.n_piksel} px)')


def dari_masker(masker, bentuk_citra, jangkauan=JANGKAUAN):
    """Masker garis lajur YOLOPX -> `GeometriLajur`, atau None bila terlalu sedikit."""
    titik = titik_lajur(masker, bentuk_citra, jangkauan)
    if len(titik) < MIN_PIKSEL:
        return None
    b = _haluskan(titik, _kemiringan_bersama(titik))
    offset, bobot = _puncak(titik[:, 1] - b * titik[:, 0])
    if not len(offset):
        return None
    return GeometriLajur(offset, bobot, b, len(titik))
