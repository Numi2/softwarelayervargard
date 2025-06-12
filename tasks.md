# Development Tasks

## 1. Web Dashboard Expansion
- Implement full Next.js dashboard according to `designreq.md`.
- Add pages for events, rules, plugins and settings.
- Hook dashboard to Flask backend and MQTT/WebSocket streams.
- Implement authentication and role-based access control.

## 2. Dynamic Topic Handling Tests
- Write unit tests for new dynamic subscription logic in `InferenceNode` and `EventManager`.
- Test hot-plugging behaviour in `SensorNode` with mocked sensors.

## 3. Plugin Management Improvements
- Support enabling/disabling plugins at runtime via CLI and dashboard.
- Document plugin configuration parameters and environment setup.
- Package example plugins (yolov5, yolov8) for easy installation via pip.

## 4. Helm Chart Enhancements
- Separate deployments for sensor, inference and event components.
- Provide configurable resources, environment variables and ROS security options.
- Add ingress configuration and TLS secrets.

## 5. Continuous Integration
- Extend GitHub Actions to build documentation and publish Docker images.
- Add code formatting (black) and linting steps to CI pipeline.

## 6. Additional Documentation
- Expand Sphinx docs: sensor hot‑plugging, telemetry topics, dashboard usage.
- Provide end‑to‑end examples for deploying on Jetson devices.

