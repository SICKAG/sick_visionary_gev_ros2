// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/ParamsFileUtils.hpp"

#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

using namespace sick;

TEST(ParamsFileUtilsTest, SetParamsFromYaml)
{
  YAML::Node yamlNode;
  yamlNode["publish_intensity"] = true;
  yamlNode["publish_depth"] = false;
  yamlNode["set_streaming"] = true;
  yamlNode["broadcast_tf"] = false;
  yamlNode["camera_rotation"] = std::vector<double>{0.0, 1.0, 2.0};
  yamlNode["camera_translation"] = std::vector<double>{3.0, 4.0, 5.0};
  yamlNode["gev_config"]["ComponentList"] = std::vector<std::string>{"Component1", "Component2"};
  yamlNode["gev_params"]["param1"] = "value1";
  yamlNode["gev_params"]["param2"] = "value2";

  bool publishIntensity = false;
  bool publishDepth = true;
  bool setStreaming = false;
  bool broadcastTF = true;
  std::vector<double> cameraRotation;
  std::vector<double> cameraTranslation;
  std::vector<std::string> gevComponents;
  std::vector<std::pair<std::string, std::string>> gevParams;

  setParamsFromYaml(yamlNode,
                    publishIntensity,
                    publishDepth,
                    setStreaming,
                    broadcastTF,
                    cameraRotation,
                    cameraTranslation,
                    gevComponents,
                    gevParams);

  EXPECT_TRUE(publishIntensity);
  EXPECT_FALSE(publishDepth);
  EXPECT_TRUE(setStreaming);
  EXPECT_FALSE(broadcastTF);

  EXPECT_EQ(cameraRotation.size(), 3);
  EXPECT_EQ(cameraRotation[0], 0.0);
  EXPECT_EQ(cameraRotation[1], 1.0);
  EXPECT_EQ(cameraRotation[2], 2.0);

  EXPECT_EQ(cameraTranslation.size(), 3);
  EXPECT_EQ(cameraTranslation[0], 3.0);
  EXPECT_EQ(cameraTranslation[1], 4.0);
  EXPECT_EQ(cameraTranslation[2], 5.0);

  EXPECT_EQ(gevComponents.size(), 2);
  EXPECT_EQ(gevComponents[0], "Component1");
  EXPECT_EQ(gevComponents[1], "Component2");

  EXPECT_EQ(gevParams.size(), 2);
  EXPECT_EQ(gevParams[0].first, "param1");
  EXPECT_EQ(gevParams[0].second, "value1");
  EXPECT_EQ(gevParams[1].first, "param2");
  EXPECT_EQ(gevParams[1].second, "value2");
}

TEST(ParamsFileUtilsTest, ParseSerialNumbers)
{
  std::string yamlContent = R"(
serial_numbers:
  visionary_b_two: ["1111111", "2222222"]
  visionary_s: ["3333333"]
  visionary_t_mini: []
  )";

  YAML::Node yamlNode = YAML::Load(yamlContent);
  std::string yamlFilePath = "test_serial_numbers.yaml";

  // Write the YAML content to a file
  std::ofstream outFile(yamlFilePath);
  outFile << yamlNode;
  outFile.close();

  std::map<std::string, std::vector<std::string>> serialNumbers = parseSerialNumbers(yamlFilePath);

  EXPECT_EQ(serialNumbers.size(), 3);
  EXPECT_EQ(serialNumbers["visionary_b_two"].size(), 2);
  EXPECT_EQ(serialNumbers["visionary_b_two"][0], "1111111");
  EXPECT_EQ(serialNumbers["visionary_b_two"][1], "2222222");
  EXPECT_EQ(serialNumbers["visionary_s"][0], "3333333");
  EXPECT_TRUE(serialNumbers["visionary_t_mini"].empty());

  // Clean up the test file
  std::remove(yamlFilePath.c_str());
}