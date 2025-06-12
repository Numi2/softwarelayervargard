import pkg_resources
import numpy as np
import importlib
import pytest

torch = importlib.import_module('torch') if importlib.util.find_spec('torch') else None

def test_entry_point():
    eps = list(pkg_resources.iter_entry_points('vargard.inference_plugins'))
    if not eps:
        pytest.skip('entry points not installed')
    names = {ep.name for ep in eps}
    assert {'yolov8', 'yolov5'}.issubset(names)

@pytest.mark.skipif(torch is None, reason="torch not installed")
def test_dummy_yolov8(monkeypatch):
    # Dummy model returns a single detection in YOLOv8 style
    import torch

    class DummyBoxes:
        def __init__(self):
            # single detection: x1,y1,x2,y2
            self.xyxy = torch.tensor([[0.0, 0.0, 10.0, 10.0]])
            self.conf = torch.tensor([0.5])
            self.cls = torch.tensor([0])

    class DummyR:
        def __init__(self):
            self.boxes = DummyBoxes()

    class DummyModel:
        names = {0: 'class0'}
        def __call__(self, img):
            # return a list of one result
            return [DummyR()]

    # Ensure ultralytics.YOLO is stubbed
    import sys, types
    if 'ultralytics' not in sys.modules:
        sys.modules['ultralytics'] = types.SimpleNamespace(YOLO=lambda path: DummyModel())
    else:
        monkeypatch.setattr(sys.modules['ultralytics'], 'YOLO', lambda path: DummyModel())

    from vargard_core.plugins.yolov8 import YOLOv8Plugin
    plugin = YOLOv8Plugin()
    plugin.initialize({'model_path': 'dummy'})
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    dets = plugin.process(img, None)
    assert isinstance(dets, list)
    assert len(dets) == 1
    det = dets[0]
    assert 'bbox' in det and 'confidence' in det and 'class' in det
