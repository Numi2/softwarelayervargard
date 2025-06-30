"""
YOLOv8 inference plugin using Ultralytics YOLOv8 with enhanced capabilities.
"""
import os
from ..inference_plugin import InferencePlugin, PluginStatus


class YOLOv8Plugin(InferencePlugin):
    def __init__(self):
        super().__init__()
        self.model = None
        self.name = "YOLOv8"
        self.version = "1.1.0"
        self.description = "Object detection using Ultralytics YOLOv8"

    def get_required_params(self):
        """YOLOv8 has no strictly required params (uses defaults)."""
        return []

    def get_supported_formats(self):
        """YOLOv8 supports multiple input formats."""
        return ['BGR', 'RGB']

    def validate_config(self, params: dict) -> bool:
        """Validate YOLOv8 configuration parameters."""
        model_path = params.get('model_path', 'yolov8s.pt')

        # Check if model file exists (if it's a local file)
        if not model_path.startswith('yolov8') and not os.path.exists(model_path):
            return False

        # Validate confidence threshold
        conf_thres = params.get('confidence_threshold')
        if conf_thres is not None:
            try:
                conf_val = float(conf_thres)
                if not (0.0 <= conf_val <= 1.0):
                    return False
            except (ValueError, TypeError):
                return False

        return True

    def initialize(self, params: dict):
        """Initialize YOLOv8 model with enhanced error handling."""
        try:
            self.status = PluginStatus.INITIALIZING

            # Validate configuration first
            if not self.validate_config(params):
                raise RuntimeError('Invalid YOLOv8 configuration parameters')

            # Store configuration
            self.config = params.copy()

            # Lazy import ultralytics to avoid top-level dependency
            try:
                from ultralytics import YOLO
            except ImportError:
                raise RuntimeError('ultralytics package is required for YOLOv8Plugin. Install with: pip install ultralytics')

            # Model path can be local .pt file or model name (e.g. 'yolov8s.pt')
            model_path = params.get('model_path', 'yolov8s.pt')

            # Load YOLOv8 model
            self.model = YOLO(model_path)

            # Configure model parameters
            conf_thres = params.get('confidence_threshold', 0.25)
            if conf_thres is not None:
                self.model.conf = float(conf_thres)

            iou_thres = params.get('iou_threshold', 0.45)
            if iou_thres is not None:
                self.model.iou = float(iou_thres)

            # Set device preference
            device = params.get(
                'device',
                'auto')  # 'auto', 'cpu', 'cuda', etc.
            if device != 'auto':
                self.model.to(device)

            # Test inference with dummy data to ensure model works
            import numpy as np
            dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model(dummy_image, verbose=False)  # Run test inference

            self.status = PluginStatus.READY

        except Exception as e:
            self.status = PluginStatus.ERROR
            raise RuntimeError(f'Failed to initialize YOLOv8 plugin: {e}')

    def process(self, image, camera_info) -> list:
        """Process image with YOLOv8 inference."""
        if self.model is None or self.status != PluginStatus.READY:
            return []

        try:
            # Run inference with error handling
            results = self.model(image, verbose=False)
            detections = []

            # Process results
            for r in results:
                if r.boxes is None or len(r.boxes) == 0:
                    continue

                try:
                    # Extract detection data
                    xyxy = r.boxes.xyxy.cpu().numpy()
                    confs = r.boxes.conf.cpu().numpy()
                    clss = r.boxes.cls.cpu().numpy().astype(int)

                    # Convert to detection format
                    for (x1, y1, x2, y2), conf, cls_id in zip(
                            xyxy, confs, clss):
                        detection = {
                            'bbox': [float(
                                x1),
                                float(y1),
                                float(x2 - x1),
                                float(y2 - y1)],
                            'confidence': float(conf),
                            'class': self.model.names.get(
                                int(cls_id),
                                f'class_{cls_id}'),
                            'class_id': int(cls_id),
                            'model': 'yolov8'
                        }

                        # Add additional metadata if available
                        if camera_info is not None:
                            detection['camera_info'] = {
                                'width': camera_info.width,
                                'height': camera_info.height,
                                'frame_id': camera_info.header.frame_id
                            }

                        detections.append(detection)

                except Exception as e:
                    # Log error but continue processing
                    print(f"Error processing YOLOv8 detection: {e}")
                    continue

            return detections

        except Exception as e:
            self.error_count += 1
            print(f"YOLOv8 inference error: {e}")
            return []

    def cleanup(self):
        """Clean up YOLOv8 resources."""
        if self.model is not None:
            # Clear model from memory
            del self.model
            self.model = None

        # Clear GPU cache if using CUDA
        try:
            import torch
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except ImportError:
            pass
