"""Pembungkus model YOLOPX (rencana kerja bagian 10.1).

Repo model dan weight berada di luar repo skripsi; jalurnya di config. Modul ini
hanya membungkus: pramuat citra, inferensi, dan penyaringan kotak. Perubahan
frame dan penghitungan posisi ada di `perception.py`.
"""
import sys

import numpy as np
import torch

import config

_TF = None


def _siapkan_jalur():
    if config.YOLOPX_DIR not in sys.path:
        sys.path.insert(0, config.YOLOPX_DIR)


class YOLOPX:
    """Satu model, tiga keluaran: deteksi kendaraan, area jalan, garis lajur."""

    def __init__(self, weight=None, device='cuda', half=True):
        _siapkan_jalur()
        global _TF
        import torchvision.transforms as T
        from lib.config import cfg
        from lib.models import get_net
        self.cfg = cfg
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.half = half and self.device.type == 'cuda'
        # normalisasi ImageNet, sama persis dengan pelatihan (tools/demo.py)
        _TF = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485, 0.456, 0.406],
                                                   std=[0.229, 0.224, 0.225])])
        ck = torch.load(weight or config.YOLOPX_WEIGHT, map_location='cpu', weights_only=True)
        self.model = get_net(cfg)
        self.model.load_state_dict(ck['state_dict'])
        self.model.gr, self.model.nc = 1.0, 1          # satu kelas: kendaraan
        self.model.to(self.device).eval()
        if self.half:
            self.model.half()
        self.epoch = ck.get('epoch')

    def _masukan(self, rgb):
        from lib.utils.augmentations import letterbox_for_img
        img, _, _ = letterbox_for_img(rgb, 640, auto=True)
        x = _TF(img).unsqueeze(0).to(self.device)
        return x.half() if self.half else x

    def infer(self, rgb, conf=None, iou=None):
        """rgb = ndarray (H, W, 3). Kembali (kotak, area_jalan, garis_lajur).

        `kotak` = (M, 5) = [x1, y1, x2, y2, conf] dalam piksel citra asli.
        Dua peta segmentasi dikembalikan pada ukuran masukan jaringan.
        """
        from lib.core.general import non_max_suppression, scale_coords
        x = self._masukan(rgb)
        with torch.no_grad():
            det_out, da_seg, ll_seg = self.model(x)
        det = non_max_suppression(det_out[0], conf_thres=conf or config.DETEKSI_CONF,
                                  iou_thres=iou or config.DETEKSI_IOU)[0]
        if det is None or not len(det):
            kotak = np.empty((0, 5))
        else:
            det[:, :4] = scale_coords(x.shape[2:], det[:, :4], rgb.shape).round()
            kotak = det[:, :5].float().cpu().numpy()
        return kotak, da_seg.float().argmax(1)[0].cpu().numpy(), ll_seg.float().argmax(1)[0].cpu().numpy()
