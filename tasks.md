# Development Tasks

## Core System Development

### Rule Engine Implementation ✓
- **Status**: Complete
- **Files Modified**: 
  - `vargard_core/rule_engine.py`
  - `vargard_core/event_manager.py`
- **Key Features**:
  - Dynamic rule loading from YAML
  - Event filtering with multiple condition types
  - Action execution (webhooks, Slack alerts)
  - Rule validation and error handling

### Event Manager Enhancement ✓
- **Status**: Complete
- **Key Changes**:
  - Dynamic topic discovery
  - Rule engine integration
  - Alert generation with metadata
  - External action support

## Testing Implementation

### Test Infrastructure ✓
- **Status**: Complete
- **Files Created**:
  - `tests/conftest.py`
  - `tests/test_inference_engine.py`
  - `tests/test_sensor_layer.py`

### Test Coverage
- Sensor layer components
- Inference engine functionality
- Plugin system
- Rule engine
- Event processing

## Linting and Code Quality

### Flake8 Error Fixes ✓
- **Status**: Complete
- **Initial State**: 528 flake8 errors
- **Final State**: 0 syntax errors, ~193 style errors remaining
- **Key Fixes Applied**:
  1. Fixed all syntax errors (3 critical files)
  2. Fixed trailing whitespace (W291, W293)
  3. Added newlines at end of files (W292)
  4. Fixed most blank line issues (E301, E302, E305)
  5. Attempted to fix line length issues (E501)

### Syntax Errors Fixed
1. **vargard_core/event_manager.py** (line 97)
   - Issue: Unterminated f-string literal
   - Fix: Properly split f-string across multiple lines using concatenation

2. **vargard_core/plugins/yolov8.py** (line 121)
   - Issue: Invalid syntax in for loop tuple unpacking
   - Fix: Reformatted the for loop to keep tuple unpacking on one line

3. **vargard_sensor_layer/sensor_node.py** (line 245)
   - Issue: Unterminated f-string literal
   - Fix: Properly split f-string across multiple lines using concatenation

### Remaining Issues
- E501: Line too long (most common, ~150 occurrences)
- E302/E305: Blank line issues between functions/classes
- E127/E128: Continuation line indentation issues

These remaining issues are primarily style-related and don't affect functionality.

## Next Steps
1. Manual review of E501 errors for context-aware line breaking
2. Fix remaining blank line issues
3. Address continuation line indentation
4. Consider configuring flake8 with custom line length if needed

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

