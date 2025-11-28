// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_SICKPUBLISHER_HPP
#define SICK_PUBLISHER_SRC_SICKPUBLISHER_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "sick_publisher/CameraParameters.hpp"

namespace rclcpp
{
class NodeOptions;
class Time;
} // namespace rclcpp

namespace sick
{
class ICameraControl;
class ICameraFrame;
} // namespace sick

namespace sick
{

/// @brief VisionaryPublisher class.
/// @details This class is responsible for publishing the data from the camera.
class VisionaryPublisher : public rclcpp::Node

{
public:
  /// @brief Constructor.
  /// @param options Node options for rclcpp.
  VisionaryPublisher(const rclcpp::NodeOptions& options);

  /// @brief Destructor.
  ~VisionaryPublisher() override;

  /// @brief Searches for the camera in all discovered cameras and get the camera control.
  void initCamera();

  /// @brief Main loop for the camera.
  void cameraLoop();

  /// @brief Configures the camera by connecting and loading parameters.
  /// @details This method connects to the camera and loads the necessary parameters.
  /// It also checks for active components like IMU, Range, and Intensity.
  void configureCamera();

  /// @brief Starts the camera streaming.
  /// @details This method initiates the camera streaming process.
  /// If the streaming fails, the application will exit.
  void startStreaming();

  /// @brief Processes the next frame from the camera.
  /// @details This method retrieves the next frame from the camera and processes it.
  /// It also handles the initial setup for point cloud parameters.
  void processFrame();

  /// @brief Publishes the components of the frame.
  /// @param frame The frame to be published.
  /// @param timeStamp The timestamp of the frame.
  /// @details This method publishes various components of the frame such as IMU data, camera pose, RGB or grayscale images, and depth information.
  void publishComponents(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp);

  /// @brief Publishes the depth data from the camera.
  /// @param frame The frame data from the camera.
  /// @param timeStamp The timestamp of the received frame.
  void onPublishDepth(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp);

  /// @brief Publishes the RGB data from the camera.
  /// @param frame The frame data from the camera.
  /// @param timeStamp The timestamp of the received frame.
  void onPublishRgb(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp);

  /// @brief Publishes the grayscale data from the camera.
  /// @param frame The frame data from the camera.
  /// @param timeStamp The timestamp of the received frame.
  void onPublishGrayscale(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp);

  /// @brief Publishes the camera info.
  /// @param timeStamp The timestamp of the received frame.
  void onPublishCamInfo(const rclcpp::Time& timeStamp);

  /// @brief Publishes the IMU data from the camera.
  /// @param frame The frame data from the camera.
  void onPublishImu(const std::shared_ptr<ICameraFrame>& frame);

  /// @brief Publishes diagnostic information.
  void publishDiagnostics();

  /// @brief Gets the relevant parameters for pointcloud creation from the camera
  void getPointCloudParameters();

  /// @brief Gets rotation/translation parameters for anchor-to-reference coordinate system transform.
  void getAnchorToRefTransform();

  /// @brief Reads and declares params
  void setupConfig();

  /// @brief Reads and declares the serial_number parameter
  bool setSerialNumber();

private:
  /// @brief Configure device
  void configDevice();

  /// @brief Declares common parameters.
  void declareCommonParams();

  /// @brief Sets up parameter handling.
  void setupParameterHandling();

  /// @brief Parses the gev params and declares them
  void declareGevParams();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_paramsCbHandle;

  // Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr m_diagnosticsPub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_intensityImagePublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depthImagePublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camInfoPublisher;

  // Tf2
  tf2_ros::StaticTransformBroadcaster m_staticBroadcaster;

  // Callbackgroups
  rclcpp::CallbackGroup::SharedPtr m_cameraLoopCallbackgroup;
  rclcpp::CallbackGroup::SharedPtr m_pubCallbackgroup;

  // Messages
  sensor_msgs::msg::CameraInfo::SharedPtr m_camInfoMsg;

  // Timers
  rclcpp::TimerBase::SharedPtr m_cameraLoopTimer;

  // Logging
  std::string m_logNodeName{std::string(this->get_namespace()) + '/' + std::string(this->get_name())};
  std::string m_camModelName;
  std::string m_nameSpace;

  // GenICam control interface
  std::shared_ptr<sick::ICameraControl> m_cameraControl{nullptr};

  // Flow control
  /// Indicates if the device configuration is completed or not
  bool m_isConfigured{false};
  /// Indicates if the device started the stream
  bool m_isStreaming{false};
  /// indicates if the camera intrinsics chunk has been processed
  bool m_firstChunkProcessed{false};
  bool m_isConnected{false};

  // Frame components
  /// Indicates if the frame contains the IMU component
  bool m_hasIMUComp{false};
  /// Indicates if the frame contains the Range component
  bool m_hasRangeComp{false};
  /// Indicates if the frame contains the Intensity component
  bool m_hasIntensityComp{false};

  // Config
  YAML::Node m_config;

  // Publishing settings
  /// Toggle for rgb topic
  bool m_publishIntensity{true};
  /// Toggle for depth topic
  bool m_publishDepth{true};
  /// Toggle for depth topic
  bool m_setStreaming{true};
  /// Toggle for broadcasting tf
  bool m_broadcastTransform{false};

  // Camera specific parameters
  int m_serialNumber;
  std::string m_cameraFrame;
  CameraParameters m_camParams{};
  std::vector<double> m_cameraRotation{0., 0., 0.};
  std::vector<double> m_cameraTranslation{0., 0., 0.};
  std::vector<double> m_anchorToRefTranslation{0., 0., 0.};
  std::vector<double> m_anchorToRefRotation{0., 0., 0.};
  std::string m_intensityPixelFormat{};
  std::string m_rangePixelFormat{};

  // GigE-Vision parameters
  std::vector<std::string> m_gevComponents{"Intensity"};
  std::vector<std::pair<std::string, std::string>> m_gevParams;
};
} // namespace sick

#endif // VISIONARY_PUBLISHER_HPP
