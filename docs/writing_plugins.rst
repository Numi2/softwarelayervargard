Writing Inference Plugins
=========================

This guide explains how to create custom inference plugins for Vargard.

Entry Points
------------
Plugins are discovered via the ``vargard.inference_plugins`` entry point.
To register a plugin, add an entry in ``vargard_core/setup.py``::

    'vargard.inference_plugins': [
        'yolov8 = vargard_core.plugins.yolov8:YOLOv8Plugin',
        'my_plugin = my_pkg.my_module:MyPlugin'
    ]

Interface
---------
Plugins must implement :class:`vargard_core.inference_plugin.InferencePlugin`.
At minimum provide ``initialize(params: dict)`` and ``process(image, camera_info)``.
``process`` should return a list of detection dictionaries::

    [
        {
            'bbox': [x, y, w, h],
            'confidence': 0.9,
            'class': 'person'
        }
    ]

Example
-------
A simple plugin using a PyTorch model::

    from vargard_core.inference_plugin import InferencePlugin
    import torch

    class MyPlugin(InferencePlugin):
        def __init__(self):
            self.model = None

        def initialize(self, params: dict):
            self.model = torch.jit.load(params['model_path'])

        def process(self, image, camera_info):
            outputs = self.model(image)
            return [{'bbox': [0,0,10,10], 'confidence': 1.0, 'class': 'dummy'}]

Place your plugin module on the Python path and ensure it is registered via the
entry point so ``InferenceNode`` can load it at runtime.
