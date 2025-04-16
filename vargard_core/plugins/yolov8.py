"""
YOLOv8 inference plugin using Ultralytics YOLOv8.
"""
from ..inference_plugin import InferencePlugin

class YOLOv8Plugin(InferencePlugin):
    def __init__(self):
        super().__init__()
        self.model = None

    def initialize(self, params: dict):
        # Lazy import ultralytics to avoid top-level dependency
        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError('ultralytics package is required for YOLOv8Plugin')
        # model_path can be local .pt or model name (e.g. 'yolov8s.pt')
        model_path = params.get('model_path', 'yolov8s.pt')
        # load YOLOv8 model
        self.model = YOLO(model_path)
        # optional confidence threshold
        conf_thres = params.get('confidence_threshold')
        if conf_thres is not None:
            try:
                self.model.conf = float(conf_thres)
            except Exception:
                pass

    def process(self, image, camera_info) -> list:
        # image: numpy.ndarray BGR
        if self.model is None:
            return []
        # run inference
        results = self.model(image)
        dets = []
        # results is a list of Results objects (one per image)
        for r in results:
            # each r.boxes has tensor fields: xyxy, conf, cls
            try:
                xyxy = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                clss = r.boxes.cls.cpu().numpy().astype(int)
            except Exception:
                continue
            for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, clss):
                dets.append({
                    'bbox': [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                    'confidence': float(conf),
                    'class': self.model.names.get(int(cls_id), str(cls_id))
                })
        return dets