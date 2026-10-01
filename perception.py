"""Perception (rencana kerja bagian 2).

Keluaran: ndarray (M, 4) = [x, y, vx, vy] dalam **frame ego** -- posisi dan
kecepatan RELATIF terhadap ego, satuan meter dan m/s.

Frame ego dipilih karena itu satu-satunya yang bisa dihasilkan kamera: dia
melihat kotak di gambar dan menghitung jaraknya, tanpa tahu dirinya berada di
mana. Konversi ke frame jalan dilakukan `localization.obstacles_ego_to_road`.

`GroundTruthPerception` sengaja melaporkan besaran yang SAMA dengan yang nanti
dihasilkan `VisionPerception`, supaya perbandingan keduanya di bagian 11.4
setara -- bukan membandingkan informasi yang berbeda jenis.
"""
import math

import numpy as np

import config
import lanes
import sensors
import tracking

RANGE = 80.0        # m ke depan; di luar ini tidak relevan untuk horizon 2 detik
REAR = 15.0         # m ke belakang; untuk memeriksa lajur tujuan


class GroundTruthPerception:
    def __init__(self, world, ego, reach=RANGE):
        self.world, self.ego, self.reach = world, ego, reach

    def update(self):
        tf = self.ego.get_transform()
        v = self.ego.get_velocity()
        yaw = math.radians(tf.rotation.yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        # kecepatan ego di frame CARLA, untuk mengubah absolut -> relatif
        vx_e, vy_e = v.x, v.y

        out = []
        for a in self.world.get_actors().filter('vehicle.*'):
            if a.id == self.ego.id:
                continue
            loc, vel = a.get_transform().location, a.get_velocity()
            dx, dy = loc.x - tf.location.x, loc.y - tf.location.y
            # putar ke frame ego, lalu balik tanda y (CARLA left-handed -> RH)
            x = c * dx + s * dy
            y = -(-s * dx + c * dy)
            if not (-REAR < x < self.reach):
                continue
            dvx, dvy = vel.x - vx_e, vel.y - vy_e
            out.append([x, y, c * dvx + s * dvy, -(-s * dvx + c * dvy)])
        return np.array(out) if out else np.empty((0, 4))


F_PIXEL = config.CAMERA_WIDTH / (2.0 * math.tan(math.radians(config.CAMERA_FOV) / 2.0))


def face_correction(theta, length=config.OTHER_LENGTH, width=config.OTHER_WIDTH):
    """Geseran dari permukaan yang TERLIHAT ke pusat bodi -> (dx memanjang, dy melintang).

    `theta` = sudut garis pandang ke target, radian, 0 = tepat di depan.
    `length`, `width` = dimensi target. Bawaannya dimensi bounding box simulator;
    jalur vision memasok dimensi yang DIUKUR sendiri (`VehicleDimensions`).

    Depth membaca permukaan terdekat yang terlihat (bagian 18.4), dan permukaan
    itu berganti selama manuver: saat target di depan yang terlihat muka
    BELAKANG, saat berdampingan yang terlihat SISI. Menambahkan setengah panjang
    di sumbu memanjang tanpa syarat -- seperti versi pertama -- melaporkan target
    sampai +3,82 m terlalu jauh ke depan begitu ego berdampingan (bagian 19.6).

    Versi kedua membedakannya dari rasio lebar/tinggi kotak. Itu GAGAL justru saat
    berdampingan: kotak terpotong tepi citra, rasionya menyusut, dan koreksi
    melintang praktis tidak diterapkan -- galat -1,06 m ~ OTHER_WIDTH/2 = 0,966 m
    tepat ke arah ego, yang membuat MPC menghindari halangan yang tidak pernah ada
    (bagian 26.2).

    Versi ketiga memakai geometri, bukan penampakan. Pada sudut pandang `theta`
    lebar siluet target adalah PANJANG*sin(theta) dari sisi ditambah
    LEBAR*cos(theta) dari buritan; porsi sisi itulah bobot campurannya. Sudutnya
    datang dari kalibrasi kamera, yang tidak pernah terpotong tepi citra.

    ponytail: `theta` diukur di frame ego, bukan frame jalan, jadi sudut hadap ego
    terhadap jalan (sampai 10,8 deg) ikut terhitung. Galatnya <= 0,08 m melintang
    -- satu orde di bawah 1,06 m yang diperbaiki. Membenarkannya menuntut
    perception mengetahui yaw ego terhadap jalan, yaitu ketergantungan pada
    localization yang belum layak dibayar.

    ponytail: satu jenis kendaraan, dimensi dianggap tetap, dan lawan dianggap
    sehadap jalan (sudah di batasan masalah). Kalau skenario nanti memuat
    kendaraan beragam, dimensi ini harus datang dari kelas deteksi.
    """
    side = length * abs(math.sin(theta))
    rear = width * abs(math.cos(theta))
    f = side / max(side + rear, 1e-9)
    return (1.0 - f) * length / 2.0, f * width / 2.0


class VehicleDimensions:
    """Panjang, lebar, dan tinggi kendaraan lain, DIUKUR dari kotak deteksi.

    Sampai bagian 28 dimensi ini konstanta dari bounding box simulator -- satu
    dari enam ketergantungan ground truth yang tersisa di jalur kendali. Padahal
    kotak deteksi plus depth sudah memuat jawabannya.

    **Tinggi** langsung: H = h_piksel * d / f, dan tidak bergantung sudut pandang
    sama sekali. Dipakai sebagai bukti bahwa balik-proyeksinya benar, bukan
    sebagai masukan kendali.

    **Panjang dan lebar** tidak bisa dipisahkan dari satu kotak: yang terukur
    lebar SILUET, dan siluet pada sudut pandang theta adalah

        W(theta) = L*|sin theta| + W*|cos theta|

    Satu persamaan, dua anu. Tetapi theta menyapu 0 sampai ~40 derajat selama
    menyalip, jadi beberapa pengamatan pada sudut berbeda memberi sistem yang
    penuh. Diselesaikan dengan kuadrat terkecil terbobot yang tumbuh tiap frame,
    dengan prior kendaraan desain PDGJ 2021 supaya frame-frame awal -- saat
    theta masih nol dan L belum teramati -- tetap memberi angka yang masuk akal.

    ponytail: SATU taksiran dipakai bersama semua deteksi, karena skenario memuat
    satu jenis kendaraan dan dimensinya memang sudah dianggap tetap di batasan
    masalah. Kendaraan beragam menuntut satu penaksir per track.
    """

    def __init__(self, length0=None, width0=None, weight0=None):
        p0 = config.PRIOR_LENGTH if length0 is None else length0
        l0 = config.PRIOR_WIDTH if width0 is None else width0
        w0 = config.PRIOR_WEIGHT if weight0 is None else weight0
        # persamaan normal 2x2 dengan prior sebagai regularisasi Tikhonov
        self._A0 = w0 * np.eye(2)          # prior, dipisah supaya `observed` jujur
        self._A = self._A0.copy()
        self._b = w0 * np.array([p0, l0], dtype=float)
        self._h_sum = self._h_weight = 0.0
        self.n_observations = 0

    def observe(self, box, d, theta):
        """Satu deteksi. `box` piksel citra asli, `d` depth planar, `theta` radian.

        Kotak yang menyentuh tepi citra DIABAIKAN: lebarnya terpotong, dan justru
        itu yang terjadi saat ego berdampingan -- kasus yang paling dibutuhkan.
        Memakainya akan menarik taksiran panjang ke bawah persis di sudut pandang
        yang paling informatif.
        """
        x1, y1, x2, y2 = (float(v) for v in box[:4])
        if x1 <= 1.0 or x2 >= config.CAMERA_WIDTH - 2.0:
            return False
        if not 1.0 < d < RANGE:
            return False
        silhouette_width = (x2 - x1) * d / F_PIXEL
        if not 0.5 < silhouette_width < 12.0:
            return False
        # Bobot turun dengan jarak: satu piksel bernilai d/f meter, jadi kotak
        # jauh mengukur dimensi jauh lebih kasar daripada kotak dekat.
        w = 1.0 / (1.0 + (d / 20.0) ** 2)
        rows = np.array([abs(math.sin(theta)), abs(math.cos(theta))])
        self._A += w * np.outer(rows, rows)
        self._b += w * rows * silhouette_width
        self.n_observations += 1
        if y1 > 1.0 and y2 < config.CAMERA_HEIGHT - 2.0:
            self._h_sum += w * (y2 - y1) * d / F_PIXEL
            self._h_weight += w
        return True

    def size(self):
        """(panjang, lebar) meter. Selalu terdefinisi -- prior menanggung awalnya."""
        pj, lb = np.linalg.solve(self._A, self._b)
        # Jepit ke rentang kendaraan yang mungkin: sistem yang nyaris singular
        # (theta belum menyapu) bisa melempar panjang ke angka tak masuk akal.
        # Batas lebar 2,1 m = PP No. 55 Tahun 2012, sumber yang sudah disitasi
        # untuk kendaraan uji (WRITING_SUMMARY.md bagian 3) -- bukan angka karangan.
        return float(np.clip(pj, 3.0, 12.0)), float(np.clip(lb, 1.4, 2.1))

    @property
    def height(self):
        """Meter, atau None bila belum ada kotak yang utuh secara vertikal."""
        if self._h_weight <= 0.0:
            return None
        return float(self._h_sum / self._h_weight)

    @property
    def observed(self):
        """Seberapa jauh sudut pandang sudah menyapu -- 0 berarti masih prior.

        Angka kondisi persamaan normal: selama target hanya terlihat dari
        belakang, baris [0, 1] terus berulang dan panjang TIDAK teramati berapa
        pun banyaknya frame. Ini yang memberi tahu kapan taksiran boleh dipercaya.

        Dihitung dari matriks DATA saja. Menyertakan prior membuatnya selalu
        tampak terkondisi baik -- prior itu kelipatan identitas, jadi ia
        menyembunyikan persis kekurangan yang ingin diukur.
        """
        value = np.linalg.eigvalsh(self._A - self._A0)
        return float(max(value.min(), 0.0) / max(value.max(), 1e-9))

    def __repr__(self):
        pj, lb = self.size()
        tg = self.height
        return (f'VehicleDimensions(p={pj:.2f} l={lb:.2f} '
                f't={"?" if tg is None else f"{tg:.2f}"} m, '
                f'n={self.n_observations}, observed={self.observed:.3f})')


class VisionPerception:
    """YOLOPX + depth camera + pelacak. Kontrak keluaran sama dengan GT di atas.

    Depth CARLA terukur **planar** (sepanjang sumbu optik), jadi balik-proyeksinya
    langsung tanpa faktor sinar: Z = depth, y = -Z*du/f. Depth membaca permukaan
    yang terlihat, bukan pusat bodi; koreksinya di `face_correction`.
    Dimensi kendaraan lain dianggap tetap -- masuk batasan masalah.
    """

    def __init__(self, net, rig, measure_dimensions=True):
        self.net, self.rig = net, rig
        self.tracker = tracking.Tracker()
        # None = pakai konstanta bounding box simulator (perilaku sebelum bagian 28)
        self.dimensions = VehicleDimensions() if measure_dimensions else None
        self.lane = None                  # LaneGeometry frame terakhir, atau None
        self.mask = None                 # (area jalan, garis lajur) frame terakhir

    def update(self, frame, dt, ego_v=0.0, ego_a=0.0, ego_w=0.0):
        depth = sensors.depth_meter(frame['depth'])
        rgb = sensors.rgb_array(frame['rgb'])
        # Kepala segmentasi lajur ikut dipakai, bukan dibuang seperti sebelum
        # bagian 28. Inferensinya sudah berjalan tiap tick; yang ditambahkan
        # hanya balik-proyeksi maskernya, ~1 ms.
        box, da, ll = self.net.infer(rgb, conf=config.TRACK_CONF_LOW)
        self.lane = lanes.from_mask(ll, rgb.shape)
        self.mask = (da, ll)        # untuk overlay video; kendali tidak memakainya
        use, z, R = [], [], []
        for b in box:
            u, v = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
            # ponytail: median petak 7x7 di pusat kotak -- terukur tepat di bagian
            # 18.4 sampai 50 m. Ganti kalau kotak pernah lebih kecil dari 7 px.
            d = float(np.median(depth[max(v - 3, 0):v + 4, max(u - 3, 0):u + 4]))
            if not 0.5 < d < RANGE:
                continue
            use.append(b)
            y = -d * (u - config.CAMERA_WIDTH / 2.0) / F_PIXEL
            # Sudut pandang dihitung ke pusat kotak, bukan ke pusat bodi -- selisihnya
            # orde kedua pada jarak kerja dan hilang di derau kuantisasi kotak.
            theta = math.atan2(y, d)
            # Diamati SEBELUM dipakai: taksiran frame ini memakai dimensi dari
            # frame-frame sebelumnya. Dimensi kendaraan tidak berubah, jadi
            # keterlambatan satu frame tidak berarti apa-apa -- sedangkan memakai
            # hasil ukur frame ini untuk mengoreksi frame ini sendiri menutup lup.
            if self.dimensions is not None:
                # Diukur pada depth PERMUKAAN, bukan pusat bodi. Sempat saya geser
                # ke d + dx dengan alasan yang sama seperti `face_correction`, dan itu
                # KELIRU: tinggi terbentang di muka yang terlihat, yang memang ada
                # di depth d. Menggesernya merusak tinggi dari galat +1,1% menjadi
                # +14,3% dan membuat lebar menabrak batas jepitnya, sehingga zona
                # aman ikut berubah dan run gagal lane_departure. Yang ditaksir di
                # sini ukuran BENDA, bukan letak pusatnya.
                self.dimensions.observe(b, d, theta)
                dx, dy = face_correction(theta, *self.dimensions.size())
            else:
                dx, dy = face_correction(theta)
            # koreksi melintang menjauhi ego: pusat bodi ada di BALIK sisi yang terlihat
            z.append([d + self.rig.x + dx, y + math.copysign(dy, y)])
            # derau melintang tumbuh dengan jarak: kuantisasi kotak x d/f
            R.append(np.diag([config.TRACK_SIGMA_D ** 2,
                              (config.TRACK_SIGMA_PIXEL * d / F_PIXEL) ** 2]))
        if not use:
            return self.tracker.update(np.empty((0, 4)), [], np.empty((0, 2)),
                                       np.empty((0, 2, 2)), dt, ego_v, ego_a, ego_w)
        use = np.array(use)
        return self.tracker.update(use[:, :4], use[:, 4], z, R,
                                   dt, ego_v, ego_a, ego_w)
