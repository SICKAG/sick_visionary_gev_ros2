// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "SICKPublisher.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <ratio>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <yaml-cpp/yaml.h>

#include "GenIStreamSDK/CameraDiscovery.hpp"
#include "base/ErrorCode.hpp"
#include "base/ErrorTrace.hpp"
#include "base/Logging.hpp"
#include "base/Result.hpp"
#include "sick_publisher/CameraIntrinsics.hpp"
#include "sick_publisher/ICameraControl.hpp"
#include "sick_publisher/ICameraFrame.hpp"
#include "utils/FrameTransformations.hpp"
#include "utils/ParamsFileUtils.hpp"
#include "utils/TransformUtils.hpp"

#ifndef M_PI
  #define M_PI (3.14159265358979323846)
#endif

namespace sick
{
VisionaryPublisher::VisionaryPublisher(const rclcpp::NodeOptions& options)
  : Node("visionary_publisher", options)
  , m_staticBroadcaster(this)
  , m_camInfoMsg(nullptr)
  , m_serialNumber{0}
{
  std::string ns = this->get_namespace();
  if (!ns.empty() && ns.front() == '/')
  {
    ns.erase(ns.begin());
  }
  m_nameSpace = ns;

  // Get config file
  namespace fs = std::filesystem;
  const std::string shareDir = ament_index_cpp::get_package_share_directory("sick_visionary_gev_ros2");
  m_config = YAML::LoadFile(fs::path(shareDir + "/config/params.yaml").make_preferred().string());

  if (!setSerialNumber())
  {
    errLog(m_logNodeName) << "Failed to set serial number!";
  }

  m_cameraFrame = "camera_frame_" + std::to_string(m_serialNumber);

  setupConfig();

  if (m_cameraControl == nullptr)
  {
    initCamera();
  }

  setupParameterHandling();

  // Quality of service
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Callbackgroups
  m_cameraLoopCallbackgroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_pubCallbackgroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // clang-format off
  auto topic = [this](const std::string& suffix) { return std::string(get_name()) + suffix; };
  // clang-format on

  // Publishers
  rclcpp::PublisherOptions pubCbOpts = rclcpp::PublisherOptions();
  pubCbOpts.callback_group = m_pubCallbackgroup;

  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(topic("/imu"), qos, pubCbOpts);
  m_camInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>(topic("/camera_info"), qos, pubCbOpts);
  m_depthImagePublisher = create_publisher<sensor_msgs::msg::Image>(topic("/depth"), qos, pubCbOpts);
  m_diagnosticsPub = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(topic("/diagnostics"), qos, pubCbOpts);

  std::string intensityTopic;
  if (m_camModelName.find("Visionary-T Mini") != std::string::npos)
  {
    intensityTopic = "/grayscale";
  }
  else if (m_camModelName.find("Visionary-B Two") != std::string::npos ||
           m_camModelName.find("Visionary-S") != std::string::npos)
  {
    intensityTopic = "/rgb";
  }

  m_intensityImagePublisher = create_publisher<sensor_msgs::msg::Image>(topic(intensityTopic), qos, pubCbOpts);

  // Cameraloop timer
  auto fps = std::chrono::duration<double>(1.0 / 30.0);
  m_cameraLoopTimer =
    this->create_wall_timer(fps, std::bind(&VisionaryPublisher::cameraLoop, this), m_cameraLoopCallbackgroup);
}

VisionaryPublisher::~VisionaryPublisher()
{
  if (m_isStreaming)
  {
    m_cameraControl->stopStreaming();
    m_isStreaming = false;
    infoLog(m_logNodeName) << "Stopped stream";
  }

  if (m_isConnected && !m_isStreaming)
  {
    m_cameraControl->disableFilters();
    infoLog(m_logNodeName) << "Disabled all configured filters";
    m_cameraControl->disconnect();
    infoLog(m_logNodeName) << "Disconnected";
  }
}

void VisionaryPublisher::initCamera()
{
  // Scan the network for cameras
  auto& discovery = CameraDiscovery::getInstance();

  discovery.discoverCameras();

  auto cameraControlResult = discovery.getCameraControl(std::to_string(m_serialNumber));
  if (cameraControlResult.isSuccess())
  {
    m_cameraControl = cameraControlResult.getSuccess();
    infoLog(m_logNodeName) << "Established camera control connection";
  }
  if (cameraControlResult.isError())
  {
    errLog(m_logNodeName) << cameraControlResult.getError()->getMessage();
  }

  m_camModelName = m_cameraControl->getCameraParameter("DeviceModelName").getSuccess();
  infoLog(m_logNodeName) << "Camera Model: " << m_camModelName;
}

void VisionaryPublisher::cameraLoop()
{
  if (!m_isConfigured)
  {
    configureCamera();
  }

  publishDiagnostics();

  if (!m_isStreaming && !m_firstChunkProcessed)
  {
    startStreaming();
  }

  if (m_isStreaming)
  {
    processFrame();
  }
}

void VisionaryPublisher::configureCamera()
{
  if (m_cameraControl->connect() != ErrorCode::OK)
  {
    errLog(m_logNodeName) << "Failed to connect to camera";
    std::exit(EXIT_FAILURE);
  }
  else
  {
    infoLog(m_logNodeName) << "Connected to camera";
    m_isConnected = true;
    m_isConfigured = m_cameraControl->loadConfig(m_gevComponents, m_gevParams) == ErrorCode::OK;

    // TODO(xfealal/DDDSP-2230): Clean up Visionary-T Mini IMU related if cases and comments
    if (m_camModelName.find("Visionary-T Mini") == std::string::npos)
    {
      m_hasIMUComp = m_cameraControl->isComponentActive("ImuBasic");
    }

    m_hasIntensityComp = m_cameraControl->isComponentActive("Intensity");
    if (m_hasIntensityComp)
    {
      m_intensityPixelFormat = m_cameraControl->getComponentPixelFormat("Intensity");
    }

    m_hasRangeComp = m_cameraControl->isComponentActive("Range");
    if (m_hasRangeComp)
    {
      m_rangePixelFormat = m_cameraControl->getComponentPixelFormat("Range");
    }
  }
}

void VisionaryPublisher::startStreaming()
{
  if (m_cameraControl->startStreaming() != ErrorCode::OK)
  {
    errLog(m_logNodeName) << "Failed to start stream";
    std::exit(EXIT_FAILURE);
  }
  else
  {
    m_isStreaming = true;
    infoLog(m_logNodeName) << "Initial start of stream";
  }
}

void VisionaryPublisher::processFrame()
{
  auto result = m_cameraControl->getNextFrame();
  if (result.isSuccess())
  {
    const auto frame = std::static_pointer_cast<ICameraFrame>(result.getSuccess());
    auto timeStamp = this->get_clock()->now();

    if (!m_firstChunkProcessed)
    {
      getPointCloudParameters();
      std::vector<geometry_msgs::msg::TransformStamped> transforms;

      // Setup camera mounting options
      if (m_cameraRotation.size() != 3 || m_cameraTranslation.size() != 3)
      {
        errLog(m_logNodeName) << "Parameters 'camera_rotation' and 'camera_translation' must each contain three values";
      }
      else
      {
        transforms.push_back(
          createStaticTransforms("map", m_cameraFrame, m_cameraTranslation, m_cameraRotation, timeStamp));
      }

      // Broadcast anchor-to-reference coordinate system transform
      getAnchorToRefTransform();
      transforms.push_back(
        createStaticTransforms(m_cameraFrame, "anchor", m_anchorToRefTranslation, m_anchorToRefRotation, timeStamp));
      m_staticBroadcaster.sendTransform(transforms);

      m_firstChunkProcessed = true;
    }
    onPublishCamInfo(timeStamp);
    publishComponents(frame, timeStamp);
  }
  else
  {
    errLog(m_logNodeName) << result.getError()->getMessage();
  }
}

void VisionaryPublisher::publishComponents(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp)
{
  if (m_hasIMUComp)
  {
    onPublishImu(frame);
  }

  if (m_hasIntensityComp && m_publishIntensity)
  {
    // By default, the Visionary-B Two and Visionary-S cameras provide intensity images in BGR8 format.
    // The Visionary-T Mini, on the other hand, provides intensity images in Mono16 format.
    // Note that these default formats may vary based on camera settings, so it's crucial to verify the Pixelformat of the component.
    if (m_intensityPixelFormat.find("BGR8") != std::string::npos)
    {
      onPublishRgb(frame, timeStamp);
    }
    else if (m_intensityPixelFormat.find("Mono16") != std::string::npos)
    {
      onPublishGrayscale(frame, timeStamp);
    }
  }

  if (m_hasRangeComp && m_publishDepth)
  {
    onPublishDepth(frame, timeStamp);
  }
}

void VisionaryPublisher::onPublishDepth(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp)
{
  cv::Mat depthImage = frameToDepth(frame);

  const auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depthImage).toImageMsg();
  msg->header.frame_id = m_cameraFrame;
  msg->header.stamp = timeStamp;
  m_depthImagePublisher->publish(*msg);
}

void VisionaryPublisher::onPublishImu(const std::shared_ptr<ICameraFrame>& frame)
{
  std::vector<sensor_msgs::msg::Imu> imuVec = frameToImu(frame);
  for (auto& imuMsg : imuVec)
  {
    m_imuPublisher->publish(imuMsg);
  }
}

void VisionaryPublisher::onPublishRgb(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp)
{
  const auto cameraFrame = frameToRgb(frame);
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cameraFrame).toImageMsg();
  msg->header.frame_id = m_cameraFrame;
  msg->header.stamp = timeStamp;
  m_intensityImagePublisher->publish(*msg);
}

void VisionaryPublisher::onPublishGrayscale(const std::shared_ptr<ICameraFrame>& frame, const rclcpp::Time& timeStamp)
{
  const auto cameraFrame = frameToGrayscale(frame);
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", cameraFrame).toImageMsg();
  msg->header.frame_id = m_cameraFrame;
  msg->header.stamp = timeStamp;
  m_intensityImagePublisher->publish(*msg);
}

void VisionaryPublisher::onPublishCamInfo(const rclcpp::Time& timeStamp)
{
  if (m_camInfoMsg == nullptr)
  {
    m_camInfoMsg = m_camParams;
  }

  m_camInfoMsg->header.stamp = timeStamp;
  m_camInfoPublisher->publish(*m_camInfoMsg);
}

void VisionaryPublisher::publishDiagnostics()
{
  auto diagnosticsStatus = diagnostic_msgs::msg::DiagnosticStatus();
  diagnosticsStatus.level = 0;
  diagnosticsStatus.name = m_camModelName;
  diagnosticsStatus.message = "To be filled with ok/warn/error msg";
  diagnosticsStatus.hardware_id = std::to_string(m_serialNumber);

  m_diagnosticsPub->publish(diagnosticsStatus);
}

void VisionaryPublisher::setupConfig()
{
  const auto rosYamlNode = m_config[m_nameSpace];
  YAML::Node generalParamsNode;
  YAML::Node specificParamsNode;

  // Set the params from the general params section
  // which applies to all cameras of the same type
  if (rosYamlNode["general"])
  {
    generalParamsNode = rosYamlNode["general"];
    setParamsFromYaml(generalParamsNode,
                      m_publishIntensity,
                      m_publishDepth,
                      m_setStreaming,
                      m_cameraRotation,
                      m_cameraTranslation,
                      m_gevComponents,
                      m_gevParams);
    infoLog(m_logNodeName) << "Parameters set from general section of YAML-file";
  }

  // Override general params if special parameter configuration available
  if (rosYamlNode["cam_" + std::to_string(m_serialNumber)])
  {
    specificParamsNode = rosYamlNode["cam_" + std::to_string(m_serialNumber)];
    setParamsFromYaml(specificParamsNode,
                      m_publishIntensity,
                      m_publishDepth,
                      m_setStreaming,
                      m_cameraRotation,
                      m_cameraTranslation,
                      m_gevComponents,
                      m_gevParams);
    infoLog(m_logNodeName) << "Parameters set from cam_" << std::to_string(m_serialNumber)
                           << " specific section of YAML-file";
  }

  declareCommonParams();
  declareGevParams();
}

void VisionaryPublisher::declareCommonParams()
{
  // Declare common parameters
  declare_parameter<bool>("publish_intensity", m_publishIntensity);
  declare_parameter<bool>("publish_depth", m_publishDepth);
  declare_parameter<bool>("set_streaming", m_setStreaming);
  declare_parameter<std::vector<double>>("camera_rotation", m_cameraRotation);
  declare_parameter<std::vector<double>>("camera_translation", m_cameraTranslation);
}

void VisionaryPublisher::declareGevParams()
{
  for (auto& param : m_gevParams)
  {
    std::string paramName = param.first;
    std::string paramValue = param.second;

    std::string paramDeclaration = "gev_params." + paramName;
    // Check for boolean
    if (paramValue == "true")
    {
      declare_parameter<bool>(paramDeclaration, true);
      continue;
    }
    if (paramValue == "false")
    {
      declare_parameter<bool>(paramDeclaration, false);
      continue;
    }

    // Check for integer
    std::istringstream intStream(paramValue);
    int intValue;
    if (intStream >> intValue && intStream.eof())
    {
      declare_parameter<int>(paramDeclaration, intValue);
      continue;
    }

    // Check for double
    std::istringstream doubleStream(paramValue);
    double doubleValue;
    if (doubleStream >> doubleValue && doubleStream.eof())
    {
      declare_parameter<double>(paramDeclaration, doubleValue);
      continue;
    }

    // String
    declare_parameter<std::string>(paramDeclaration, paramValue);
  }
}

void VisionaryPublisher::setupParameterHandling()
{
  auto parameterChangeCallback = [this](const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto logParamChange = [this](const rclcpp::Parameter& param, bool& controlFlag, const std::string& paramName)
    {
      if (param.get_name() == paramName && param.as_bool() != controlFlag)
      {
        controlFlag = param.as_bool();
        if (controlFlag)
        {
          infoLog(m_logNodeName) << "Publishing " + paramName.substr(8) + "-topic";
        }
        else
        {
          infoLog(m_logNodeName) << "Stopped publishing of " + paramName.substr(8) + "-topic";
        }
      }
    };

    for (const auto& param : params)
    {
      logParamChange(param, m_publishIntensity, "publish_intensity");
      logParamChange(param, m_publishDepth, "publish_depth");

      if (param.get_name() == "set_streaming" && param.as_bool() != m_setStreaming && param.as_bool() != m_isStreaming)
      {
        m_setStreaming = param.as_bool();
        if (m_setStreaming)
        {
          if (m_cameraControl->startStreaming() != ErrorCode::OK)
          {
            errLog(m_logNodeName) << "Failed to restart stream";
            std::exit(EXIT_FAILURE);
          }
          else
          {
            m_isStreaming = true;
            infoLog(m_logNodeName) << "Restarted stream";
          }
        }
        else
        {
          if (m_cameraControl->stopStreaming() != ErrorCode::OK)
          {
            errLog(m_logNodeName) << "Failed to stop stream";
            std::exit(EXIT_FAILURE);
          }
          else
          {
            m_isStreaming = false;
            infoLog(m_logNodeName) << "Stopped stream";
          }
        }
      }
    }

    return result;
  };

  m_paramsCbHandle = add_on_set_parameters_callback(parameterChangeCallback);
}

bool VisionaryPublisher::setSerialNumber()
{
  declare_parameter<int>("serial_number", m_serialNumber);
  m_serialNumber = get_parameter("serial_number").as_int();
  return (m_serialNumber != 0);
}

void VisionaryPublisher::getAnchorToRefTransform()
{
  auto fetchRef = [&](const std::string& feature, const std::string& param, auto& out)
  {
    auto result = m_cameraControl->getCameraParameterFloat(feature, param);
    if (result.isSuccess())
    {
      out = result.getSuccess();
    }
    else
    {
      errLog() << result.getError()->getMessage();
    }
  };

  fetchRef("ChunkScan3dCoordinateReference", "RotationX", m_anchorToRefRotation[0]);
  fetchRef("ChunkScan3dCoordinateReference", "RotationY", m_anchorToRefRotation[1]);
  fetchRef("ChunkScan3dCoordinateReference", "RotationZ", m_anchorToRefRotation[2]);
  fetchRef("ChunkScan3dCoordinateReference", "TranslationX", m_anchorToRefTranslation[0]);
  fetchRef("ChunkScan3dCoordinateReference", "TranslationY", m_anchorToRefTranslation[1]);
  fetchRef("ChunkScan3dCoordinateReference", "TranslationZ", m_anchorToRefTranslation[2]);
  // Convert from mm to m
  m_anchorToRefTranslation[0] = m_anchorToRefTranslation[0] / 1000.;
  m_anchorToRefTranslation[1] = m_anchorToRefTranslation[1] / 1000.;
  m_anchorToRefTranslation[2] = m_anchorToRefTranslation[2] / 1000.;
}

void VisionaryPublisher::getPointCloudParameters()
{
  auto fetchCamParam = [&](const std::string& name, auto& output)
  {
    auto result = m_cameraControl->getCameraParameter(name);
    if (result.isSuccess())
    {
      output = std::stof(result.getSuccess());
    }
    else
    {
      errLog(std::string(get_namespace()) + get_name()) << result.getError()->getMessage();
    }
  };

  // Fetch intrinsic
  CameraIntrinsics intrinsics{};
  fetchCamParam("ChunkScan3dFocalLength", intrinsics.focalLength);
  fetchCamParam("ChunkScan3dAspectRatio", intrinsics.aspectRatio);
  fetchCamParam("ChunkScan3dPrincipalPointU", intrinsics.principalPointU);
  fetchCamParam("ChunkScan3dPrincipalPointV", intrinsics.principalPointV);
  fetchCamParam("ChunkScan3dCoordinateScale", intrinsics.scaleC);
  fetchCamParam("ChunkScan3dCoordinateOffset", intrinsics.offset);

  // Fetch resolution
  Resolution resolution{};
  fetchCamParam("Height", resolution.height);
  fetchCamParam("Width", resolution.width);

  // Fetch binning
  BinningParameters binning{};
  fetchCamParam("BinningHorizontal", binning.binningHorizontal);
  fetchCamParam("BinningVertical", binning.binningVertical);

  // Set camera parameters
  m_camParams = CameraParameters(resolution, binning, {}, {}, {}, {}, intrinsics);

  // Get distortion coefficients
  // TODO(xfealal/DDDSP-1836): Get the cameraInfo parameters from genistream. Flash new FW on cameras.
}

} // namespace sick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(sick::VisionaryPublisher)
