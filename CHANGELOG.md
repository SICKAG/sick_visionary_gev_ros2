# Changelog

## 1.1.0 (2025-10-10)

### Changed

### Added
- Added toggle parameter for tf_broadcast
- PtpEnable parameter to documentation
- Extended FAQ Section in README

### Supported devices
- Visionary-B Two, with `Visionary_GigE_Vision_Basic` app
- Visionary-S AP, with `Visionary_GigE_Vision_Basic` app
- Visionary-T Mini AP, with `Visionary_GigE_Vision_Basic` app

### Known Issues
- Windows build not fully supported

## 1.0.0 - initial public release (2025-04-01)

### Changed
- Use params.yaml file for node and camera configuration

### Added
- Support for multiple camera configuration
- Camera meshes for visualisation inside RVIZ2
- Launchfile for visionary publisher
- Launchfile for pointcloud publisher
- Extensive unit tests
- Add transform for camera_link and anchor

### Supported devices
- Visionary-B Two, with `Visionary_GigE_Vision_Basic` app
- Visionary-S AP, with `Visionary_GigE_Vision_Basic` app
- Visionary-T Mini AP, with `Visionary_GigE_Vision_Basic` app

### Known Issues
- Windows build not fully supported

## 0.0.0 - ROS2 Wrapper internal pre-release (2024-09-XX)
- Use gev_recording compatible .cfg configuration file
- Use official ROS message definitions for publishing
- Optional output toggling of different publisher nodes
- Add support for composable nodes
- Add Support for Windows x64, Linux x64/ARM64
- Add support for standalone deployment (bundled GenIStream/GenAPI releases)

### Supported devices
- Visionary B-Two, with `Visionary_GigE_Vision_Basic` app

### Known Issues
- Can only configure device of one type (B-Two/T-mini currently)
