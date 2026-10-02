"""Geometri lajur dari kepala segmentasi YOLOPX, tanpa peta simulator (bagian 28).

MURNI NUMERIK. Tidak mengimpor carla maupun torch: masukannya masker biner yang
sudah dihasilkan `yolopx.infer`, keluarannya offset garis lajur dalam meter di
frame ego. Bisa diuji di terminal tanpa menyalakan simulator maupun GPU.

Kenapa modul ini ada: `main.prepare_road` mengambil centerline langsung dari
`world.get_map()`, yaitu peta HD simulator. Padahal YOLOPX sudah menghitung
segmentasi garis lajur tiap tick -- dan hasilnya dibuang (`box, _, _`). Modul
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

F_PIXEL = config.CAMERA_WIDTH / (2.0 * np.tan(np.radians(config.CAMERA_FOV) / 2.0))

RANGE = (6.0, 45.0)      # m, batas x yang dipercaya
BIN_C = 0.10                 # m, lebar bin histogram perpotongan
SEARCH_B = 0.30                # kemiringan maks yang dicari (~17 deg hadap)
MIN_PIXELS = 40              # piksel minimum sebelum hasil dilaporkan
MIN_PEAK = 0.25            # porsi terhadap puncak tertinggi


def letterbox_to_image(mask_shape, image_shape):
    """(skala, pad_u, pad_v): piksel masker -> piksel citra asli.

    `letterbox_for_img(img, 640, auto=True)` mengecilkan dengan satu rasio lalu
    memberi bingkai sampai kelipatan 32. Membalikkannya butuh keduanya -- lupa
    padding 12 piksel menggeser seluruh hasil beberapa meter.
    """
    hm, wm = mask_shape
    hc, wc = image_shape[:2]
    r = min(640.0 / hc, 640.0 / wc)
    width, height = int(round(wc * r)), int(round(hc * r))
    return r, (wm - width) / 2.0, (hm - height) / 2.0


def ipm(u, v, z=config.CAMERA_Z, f=F_PIXEL,
        cu=config.CAMERA_WIDTH / 2.0, cv=config.CAMERA_HEIGHT / 2.0):
    """Piksel citra -> titik di permukaan jalan, frame ego RH (x depan, y kiri).

    Kamera menghadap lurus (pitch 0), jadi baris `v` di bawah horizon memotong
    permukaan pada x = f*z/(v - cv). Sumbu y memakai perjanjian yang sama dengan
    `perception`: y = -x*(u - cu)/f, positif ke kiri.
    """
    dv = np.asarray(v, dtype=float) - cv
    front = dv > 1e-6
    x = np.where(front, f * z / np.where(front, dv, 1.0), np.inf)
    # y dihitung hanya di piksel yang memotong jalan: inf * 0 memberi nan diam-diam
    y = np.where(front, -x * (np.asarray(u, dtype=float) - cu) / f, np.nan)
    return x, y


def to_pixel(x, y, z=config.CAMERA_Z, f=F_PIXEL,
              cu=config.CAMERA_WIDTH / 2.0, cv=config.CAMERA_HEIGHT / 2.0):
    """Kebalikan `ipm`: titik di permukaan jalan -> piksel citra. Untuk menggambar."""
    x = np.maximum(np.asarray(x, dtype=float), 1e-3)
    return cu - np.asarray(y, dtype=float) * f / x, cv + f * z / x


def lane_points(mask, image_shape, reach=RANGE):
    """Masker garis lajur -> (N, 2) titik (x, y) meter di frame ego."""
    vm, um = np.nonzero(np.asarray(mask) > 0)
    if len(um) == 0:
        return np.empty((0, 2))
    r, pad_u, pad_v = letterbox_to_image(np.shape(mask), image_shape)
    x, y = ipm((um - pad_u) / r, (vm - pad_v) / r)
    use = np.isfinite(x) & (x >= reach[0]) & (x <= reach[1])
    return np.column_stack([x[use], y[use]])


# --- Area jalan (drivable area) ---------------------------------------------
# Rentang x petak yang diperiksa. Lebih pendek dari RANGE: kendaraan di depan
# menutupi jalan di belakangnya, dan di 30 m satu piksel masker sudah ~0,1 m.
DRIVABLE_REACH = (6.0, 30.0)

def _sample(mask, image_shape, x, y):
    """Nilai masker di titik jalan (x, y) frame ego. Di luar citra = False."""
    r, pad_u, pad_v = letterbox_to_image(np.shape(mask), image_shape)
    u, v = np.broadcast_arrays(*to_pixel(x, y))
    um = np.round(u * r + pad_u).astype(int)
    vm = np.round(v * r + pad_v).astype(int)
    ok = (um >= 0) & (um < mask.shape[1]) & (vm >= 0) & (vm < mask.shape[0])
    out = np.zeros(u.shape, dtype=bool)
    out[ok] = mask[vm[ok], um[ok]] > 0
    return out


def drivable_fraction(da, image_shape, center, slope, half_width, reach=DRIVABLE_REACH):
    """Porsi petak lajur yang ditandai area jalan, 0..1.

    Petak = x dalam `reach`, y = center + slope*x +- half_width (frame ego).
    Terukur di S1: lajur salip 1,00, lajur ego dengan kendaraan di depannya turun
    sampai 0,40 -- kendaraan membolongi masker, jadi angka ini menjawab "lajurnya
    LAPANG". Ia TIDAK menjawab "lajurnya ADA": bahu jalan kiri juga aspal dan
    terbaca 1,00. Itu urusan `LaneGeometry.lane_marked`.
    """
    x, dy = np.meshgrid(np.linspace(*reach, 12), np.linspace(-half_width, half_width, 5))
    return float(_sample(da, image_shape, x, center + slope * x + dy).mean())


def road_edges(da, image_shape, start, slope, reach=DRIVABLE_REACH):
    """(kanan, kiri): tepi area jalan yang memuat `start`, perpotongan di x = 0.

    Tiap baris x mencari run drivable yang memuat `start`. Antar baris diambil
    persentil 90 ke LUAR, bukan median: kendaraan di sebelah ego memotong run di
    baris yang ditutupinya, dan median ikut melaporkan sisi kendaraan itu sebagai
    tepi jalan (terukur 0,54 m padahal tepinya ~5 m). Kendaraan sudah urusan
    zona aman. None bila `start` tidak drivable di baris mana pun.
    """
    c = np.arange(-15.0, 15.0 + 1e-9, 0.1)
    i0 = int(np.argmin(np.abs(c - start)))
    right, left = [], []
    for x in np.linspace(*reach, 12):
        row = _sample(da, image_shape, x, c + slope * x)
        if not row[i0]:
            continue
        gap = np.flatnonzero(~row)
        hi, lo = gap[gap > i0], gap[gap < i0]
        left.append(c[hi[0] - 1] if len(hi) else c[-1])
        right.append(c[lo[-1] + 1] if len(lo) else c[0])
    if not left:
        return None
    return float(np.percentile(right, 10)), float(np.percentile(left, 90))


def _shared_slope(points, search=SEARCH_B, step=0.005):
    """Kemiringan yang dipakai BERSAMA semua garis lajur.

    Untuk tiap calon `b`, proyeksikan c = y - b*x lalu histogramkan. Kemiringan
    yang benar membuat tiap marka jatuh ke bin yang sama, sehingga histogramnya
    paling memuncak; yang salah mengoleskannya rata. Ketajaman diukur dengan
    jumlah kuadrat cacah -- maksimum bila massa terkumpul di sedikit bin.
    """
    x, y = points[:, 0], points[:, 1]
    edges = np.arange(-12.0, 12.0 + BIN_C, BIN_C)
    candidate = np.arange(-search, search + step, step)
    score = np.empty(len(candidate))
    for i, b in enumerate(candidate):
        counts, _ = np.histogram(y - b * x, bins=edges)
        score[i] = float((counts.astype(float) ** 2).sum())
    return float(candidate[int(np.argmax(score))])


def _peaks(c, min_fraction=MIN_PEAK):
    """Perpotongan tiap garis lajur = modus histogram c. -> (offset, bobot)."""
    edges = np.arange(-12.0, 12.0 + BIN_C, BIN_C)
    counts, _ = np.histogram(c, bins=edges)
    # haluskan 3 bin supaya satu marka tidak pecah jadi dua puncak
    smooth = np.convolve(counts.astype(float), np.ones(3) / 3.0, mode='same')
    threshold = max(smooth.max() * min_fraction, 1.0)
    center = (edges[:-1] + edges[1:]) / 2.0
    offset, weight, i = [], [], 1
    while i < len(smooth) - 1:
        if smooth[i] >= threshold and smooth[i] >= smooth[i - 1] and smooth[i] > smooth[i + 1]:
            # pusat massa lokal +-0,3 m: lebih halus daripada tengah bin
            near = np.abs(c - center[i]) < 0.3
            offset.append(float(c[near].mean()) if near.sum() else float(center[i]))
            weight.append(float(near.sum()))
            i += 4                      # lompati bahu puncak yang sama
        else:
            i += 1
    if not offset:
        return np.empty(0), np.empty(0)
    order = np.argsort(offset)
    return np.array(offset)[order], np.array(weight)[order]


def _lattice(offset, weight, width=(2.8, 4.4), step=0.01):
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
    candidate = np.arange(width[0], width[1] + step, step)
    best, w_best, best_phase = -1.0, None, None
    for w in candidate:
        z = (weight * np.exp(2j * np.pi * offset / w)).sum() / max(weight.sum(), 1e-9)
        if abs(z) > best:
            best, w_best = float(abs(z)), float(w)
            best_phase = float(np.angle(z) * w / (2.0 * np.pi))
    return w_best, best_phase


def _refine(points, b0, iterations=3):
    """Haluskan kemiringan bersama dengan kuadrat terkecil. -> b

    `_shared_slope` mencari lewat argmax histogram, dan itu kasar: skornya
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
    x, y = points[:, 0], points[:, 1]
    b = b0
    for _ in range(iterations):
        c = y - b * x
        offset, _ = _peaks(c)
        if not len(offset):
            return b
        cluster = np.argmin(np.abs(c[:, None] - offset[None, :]), axis=1)
        # buang titik yang jauh dari garis mana pun: marka palsu tidak boleh
        # ikut menarik kemiringan bersama
        near = np.abs(c - offset[cluster]) < 0.4
        if near.sum() < 20:
            return b
        xk, yk, kk = x[near], y[near], cluster[near]
        top = bottom = 0.0
        for g in np.unique(kk):
            m = kk == g
            if m.sum() < 5:
                continue
            dx, dy = xk[m] - xk[m].mean(), yk[m] - yk[m].mean()
            top += float((dx * dy).sum())
            bottom += float((dx * dx).sum())
        if bottom <= 0.0:
            return b
        b = top / bottom
    return b


class LaneGeometry:
    """Hasil satu frame. `offset` positif = ke kiri ego, meter."""

    def __init__(self, offset, weight, slope, n_pixels, c=None):
        self.offset = offset            # perpotongan tiap garis lajur di x = 0
        self.c = c                      # perpotongan TIAP piksel marka, untuk lane_marked
        self.weight = weight              # piksel pendukung tiap garis
        self.slope = slope    # dy/dx; ~ -tan(yaw ego terhadap jalan)
        self.n_pixels = n_pixels
        self.lane_width, self.phase = _lattice(offset, weight)

    @property
    def yaw(self):
        """Sudut hadap ego terhadap arah jalan, radian."""
        return -float(np.arctan(self.slope))

    @property
    def lattice_residual(self):
        """RMS jarak garis terdeteksi ke titik kisi terdekat, meter.

        Ukuran kepercayaan: kisi yang cocok memberi sisa kecil. Frame dengan sisa
        besar berarti marka yang terbaca tidak membentuk pola berjarak sama --
        biasanya ada garis palsu -- dan hasilnya sebaiknya tidak dipakai.
        """
        if self.lane_width is None:
            return None
        d = self.offset - self.phase
        return float(np.sqrt((((d - np.round(d / self.lane_width) * self.lane_width)
                               ** 2) * self.weight).sum() / max(self.weight.sum(), 1e-9)))

    @property
    def lane_dev(self):
        """Simpangan ego dari tengah lajurnya sendiri, meter. None bila tak tentu.

        Tengah lajur = titik tengah antar dua garis kisi, jadi tetap terdefinisi
        walaupun salah satu marka pengapitnya tidak terdeteksi frame itu.
        Positif berarti ego ada di KIRI tengah lajur.
        """
        if self.lane_width is None:
            return None
        w = self.lane_width
        # tengah lajur ada di fase + (n + 1/2) * w; cari yang terdekat ke ego (0)
        n = np.round((0.0 - self.phase) / w - 0.5)
        return -float(self.phase + (n + 0.5) * w)

    def lines(self, reach=RANGE, n=24):
        """Garis kisi sebagai lintasan di frame ego -> [(x, y), ...] per garis.

        Yang digambar KISI hasil cocokan, bukan piksel maskernya. Itu sebabnya
        garisnya tersambung penuh walaupun markanya putus-putus: yang dicari
        memang satu garis lurus per lajur, bukan sekumpulan penggal.
        """
        if self.lane_width is None:
            return []
        x = np.linspace(reach[0], reach[1], n)
        w = self.lane_width
        # Hanya slot kisi yang BENAR-BENAR didukung marka. Kisinya tak berhingga;
        # menggambar seluruhnya berarti menarik garis lajur di atas tanggul dan
        # pembatas, yaitu mengklaim sesuatu yang tidak pernah diukur.
        out = []
        for c in np.unique(np.round((self.offset - self.phase) / w)):
            c = self.phase + c * w
            out.append((x, c + self.slope * x))
        return out

    def center_line(self, reach=RANGE, n=24):
        """Sumbu lajur yang sedang ditempati ego, di frame ego."""
        if self.lane_width is None:
            return None
        x = np.linspace(reach[0], reach[1], n)
        c = -self.lane_dev
        return x, c + self.slope * x

    def lane_center(self, side):
        """Tengah lajur sebelah, relatif ego. `side` = +1 kiri, -1 kanan.

        Dipakai FSM sebagai `y_goal` menggantikan `SIDE_SIGN * LANE_WIDTH`.
        """
        if self.lane_width is None:
            return None
        return float(-self.lane_dev + side * self.lane_width)

    def lane_marked(self, side, tol=0.3):
        """Lajur sebelah diapit marka di KEDUA sisinya? `side` = +1 kiri, -1 kanan.

        Area jalan tidak bisa membedakan lajur dari bahu jalan: keduanya aspal.
        Markanya bisa -- bahu jalan hanya punya garis tepi di satu sisi. Dihitung
        dari piksel, bukan dari puncak `offset`: garis luar lajur salip putus-putus
        dan hanya jadi puncak di 54% frame, sedangkan pikselnya terukur >= 109 di
        setiap frame (bahu jalan kiri: 0).
        """
        if self.lane_width is None or self.c is None:
            return False
        mid = self.lane_center(side)
        return all((np.abs(self.c - (mid + s * self.lane_width / 2)) < tol).sum() >= MIN_PIXELS
                   for s in (-1, 1))

    def __repr__(self):
        w = self.lane_width
        return (f'LaneGeometry({len(self.offset)} lines, width='
                f'{"?" if w is None else f"{w:.2f}"} m, '
                f'yaw={np.degrees(self.yaw):+.1f} deg, {self.n_pixels} px)')


def from_mask(mask, image_shape, reach=RANGE, drivable=None):
    """Masker garis lajur YOLOPX -> `LaneGeometry`, atau None bila terlalu sedikit.

    `drivable` = masker area jalan; bila diberikan, hanya garis lajur di dalam
    area jalan yang dipakai. Terukur di S1: 0,3% piksel garis lajur di luarnya,
    seluruhnya garis di luar jalan; garis tepi jalan 0% (ikut ditandai area jalan).
    """
    if drivable is not None:
        mask = (np.asarray(mask) > 0) & (np.asarray(drivable) > 0)
    points = lane_points(mask, image_shape, reach)
    if len(points) < MIN_PIXELS:
        return None
    b = _refine(points, _shared_slope(points))
    offset, weight = _peaks(points[:, 1] - b * points[:, 0])
    if not len(offset):
        return None
    return LaneGeometry(offset, weight, b, len(points), c=points[:, 1] - b * points[:, 0])
