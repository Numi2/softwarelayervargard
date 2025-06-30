"""
YOLOv5 inference plugin using torch.hub.
"""
import torch
from ..inference_plugin import InferencePlugin


class YOLOv5Plugin(InferencePlugin):
    def __init__(self):
        super().__init__()
        self.model = None

    def initialize(self, params: dict):
        # model_path can be local .pt or pretrained
        model_path = params.get('model_path', 'yolov5s')
        # load model
        try:
            self.model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=model_path)
        except Exception:
            # fallback to default
            self.model = torch.hub.load('ultralytics/yolov5', model_path)
        # optional confidence threshold
        conf_thres = params.get('confidence_threshold')
        if conf_thres is not None:
            self.model.conf = float(conf_thres)

    def process(self, image, camera_info) -> list:
        # image: numpy.ndarray BGR
        if self.model is None:
            return []
        # run inference
        results = self.model(image)
        dets = []
        # each row: [x1, y1, x2, y2, conf, cls]
        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            x1, y1, x2, y2 = [float(v) for v in box]
            dets.append({
                'bbox': [x1, y1, x2 - x1, y2 - y1],
                'confidence': float(conf),
                'class': self.model.names[int(cls)]
            })
        return dets
