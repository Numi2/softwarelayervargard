import pkg_resources
import numpy as np
import torch

def test_entry_point():
    eps = list(pkg_resources.iter_entry_points('vargard.inference_plugins'))
    assert any(ep.name == 'yolov5' for ep in eps), 'YOLOv5 entry point not registered'

def test_dummy_yolov5(monkeypatch):
    # Dummy model returns a single detection
    class DummyModel:
        names = {0: 'class0'}
        def __call__(self, img):
            # create dummy results with xyxy as torch.Tensor
            return type('R', (), {'xyxy': [torch.tensor([[0, 0, 10, 10, 0.5, 0]])]})

    # Monkeypatch torch.hub.load
    monkeypatch.setattr('torch.hub.load', lambda *args, **kwargs: DummyModel())

    from vargard_core.plugins.yolov5 import YOLOv5Plugin
    plugin = YOLOv5Plugin()
    plugin.initialize({'model_path': 'dummy'})
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    dets = plugin.process(img, None)
    assert isinstance(dets, list)
    assert len(dets) == 1
    det = dets[0]
    assert 'bbox' in det and 'confidence' in det and 'class' in det