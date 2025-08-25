// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_UTILS_PARAMSFILEUTILS_HPP
#define SICK_PUBLISHER_SRC_UTILS_PARAMSFILEUTILS_HPP

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace YAML
{
class Node;
}

namespace sick
{

/// @brief Sets the publishing and streaming parameters from the YAML Node.
/// @param yamlNode YAML Node with the parameters.
/// @param publishIntensity Reference to the publish intensity flag.
/// @param publishDepth Reference to the publish depth flag.
/// @param setStreaming Reference to the set streaming flag.
void setPublishingAndStreamingParams(const YAML::Node& yamlNode,
                                     bool& publishIntensity,
                                     bool& publishDepth,
                                     bool& setStreaming);

/// @brief Sets the mounting parameters from the YAML Node.
/// @param yamlNode YAML Node with the parameters.
/// @param cameraRotation Reference to the camera rotation vector.
/// @param cameraTranslation Reference to the camera translation vector.
void setMountingParams(const YAML::Node& yamlNode,
                       std::vector<double>& cameraRotation,
                       std::vector<double>& cameraTranslation);

/// @brief Sets the Gev components from the YAML Node.
/// @param yamlNode YAML Node with the parameters.
/// @param gevComponents Reference to the Gev components vector.
void setGevComponents(const YAML::Node& yamlNode, std::vector<std::string>& gevComponents);

/// @brief Sets the Gev parameters from the YAML Node.
/// @param yamlNode YAML Node with the parameters.
/// @param gevParams Reference to the Gev parameters vector.
void setGevParams(const YAML::Node& yamlNode, std::vector<std::pair<std::string, std::string>>& gevParams);

/// @brief Reads the parameters from the YAML Node and sets the provided variables.
/// @param yamlNode YAML Node with the parameters.
/// @param publishIntensity Reference to the publish intensity flag.
/// @param publishDepth Reference to the publish depth flag.
/// @param setStreaming Reference to the set streaming flag.
/// @param cameraRotation Reference to the camera rotation vector.
/// @param cameraTranslation Reference to the camera translation vector.
/// @param gevComponents Reference to the Gev components vector.
/// @param gevParams Reference to the Gev parameters vector.
void setParamsFromYaml(const YAML::Node& yamlNode,
                       bool& publishIntensity,
                       bool& publishDepth,
                       bool& setStreaming,
                       bool& broadcastTF,
                       std::vector<double>& cameraRotation,
                       std::vector<double>& cameraTranslation,
                       std::vector<std::string>& gevComponents,
                       std::vector<std::pair<std::string, std::string>>& gevParams);

/// @brief Parses the serial numbers from a YAML file.
/// @param yamlFilePath Path to the YAML file.
/// @return A map where the key is the model name and the value is a vector of serial numbers.
std::map<std::string, std::vector<std::string>> parseSerialNumbers(const std::string& yamlFilePath);

} // namespace sick
#endif //SICK_PUBLISHER_SRC_UTILS_PARAMSFILEUTILS_HPP
