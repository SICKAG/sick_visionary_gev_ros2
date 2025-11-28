// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "ParamsFileUtils.hpp"

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#include "base/Logging.hpp"

namespace sick
{

void setPublishingAndStreamingParams(const YAML::Node& yamlNode,
                                     bool& publishIntensity,
                                     bool& publishDepth,
                                     bool& setStreaming,
                                     bool& broadcastTF)
{
  if (yamlNode["publish_intensity"])
  {
    publishIntensity = yamlNode["publish_intensity"].as<bool>();
  }

  if (yamlNode["publish_depth"])
  {
    publishDepth = yamlNode["publish_depth"].as<bool>();
  }

  if (yamlNode["set_streaming"])
  {
    setStreaming = yamlNode["set_streaming"].as<bool>();
  }

  if (yamlNode["broadcast_tf"])
  {
    broadcastTF = yamlNode["broadcast_tf"].as<bool>();
  }
}

void setMountingParams(const YAML::Node& yamlNode,
                       std::vector<double>& cameraRotation,
                       std::vector<double>& cameraTranslation)
{
  if (yamlNode["camera_rotation"])
  {
    cameraRotation = yamlNode["camera_rotation"].as<std::vector<double>>();
  }

  if (yamlNode["camera_translation"])
  {
    cameraTranslation = yamlNode["camera_translation"].as<std::vector<double>>();
  }
}

void setGevComponents(const YAML::Node& yamlNode, std::vector<std::string>& gevComponents)
{
  if (yamlNode["gev_config"])
  {
    const auto gevComponentsNode = yamlNode["gev_config"]["ComponentList"];
    gevComponents = gevComponentsNode.as<std::vector<std::string>>();
  }
}

void setGevParams(const YAML::Node& yamlNode, std::vector<std::pair<std::string, std::string>>& gevParams)
{
  if (yamlNode["gev_params"])
  {
    const auto gevParamsNode = yamlNode["gev_params"];
    for (const auto& param : gevParamsNode)
    {
      std::string paramName = param.first.as<std::string>();
      YAML::Node paramValue = param.second;
      bool found = false;

      for (auto& existingParam : gevParams)
      {
        if (existingParam.first == paramName)
        {
          existingParam.second = paramValue.as<std::string>();
          found = true;
          break;
        }
      }

      if (!found)
      {
        gevParams.push_back(std::make_pair(paramName, paramValue.as<std::string>()));
      }
    }
  }
}

void setParamsFromYaml(const YAML::Node& yamlNode,
                       bool& publishIntensity,
                       bool& publishDepth,
                       bool& setStreaming,
                       bool& broadcastTF,
                       std::vector<double>& cameraRotation,
                       std::vector<double>& cameraTranslation,
                       std::vector<std::string>& gevComponents,
                       std::vector<std::pair<std::string, std::string>>& gevParams)
{
  setPublishingAndStreamingParams(yamlNode, publishIntensity, publishDepth, setStreaming, broadcastTF);
  setMountingParams(yamlNode, cameraRotation, cameraTranslation);
  setGevComponents(yamlNode, gevComponents);
  setGevParams(yamlNode, gevParams);
}

std::map<std::string, std::vector<std::string>> parseSerialNumbers(const std::string& yamlFilePath)
{
  std::map<std::string, std::vector<std::string>> serialNumbers;
  try
  {
    YAML::Node config = YAML::LoadFile(yamlFilePath);
    for (const auto& node : config["serial_numbers"])
    {
      std::string modelName = node.first.as<std::string>();
      std::vector<std::string> serialNumber = node.second.as<std::vector<std::string>>();
      serialNumbers[modelName] = serialNumber;
    }
  }
  catch (const YAML::Exception& e)
  {
    errLog("ParamsFileUtils.cpp") << "Error parsing YAML file: " << e.what();
  }
  return serialNumbers;
}

} // namespace sick
