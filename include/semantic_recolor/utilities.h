#pragma once
#include "semantic_recolor/segmenter.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <map>

namespace semantic_recolor {

struct DemoConfig {
  DemoConfig(const ros::NodeHandle &nh) {
    if (!nh.getParam("input_file", input_file)) {
      ROS_FATAL("Missing input_file");
      throw std::runtime_error("missing param!");
    }

    if (!nh.getParam("output_file", output_file)) {
      ROS_FATAL("Missing output_file");
      throw std::runtime_error("missing param!");
    }

    nh.getParam("saturation", saturation);
    nh.getParam("luminance", luminance);
    nh.getParam("max_classes", max_classes);
    nh.getParam("num_timing_inferences", num_timing_inferences);
  }

  std::string input_file;
  std::string output_file;
  double saturation = 0.85;
  double luminance = 0.75;
  int max_classes = 150;
  int num_timing_inferences = 10;
};

class SemanticColorConfig {
  public:
  SemanticColorConfig();

  explicit SemanticColorConfig(const ros::NodeHandle &nh);

  void fillColor(int32_t class_id, uint8_t *pixel, size_t pixel_size = 3);

  private:
  bool initialized_;
  std::map<int32_t, std::vector<uint8_t>> color_map_;
  std::vector<uint8_t> default_color_;
  std::set<int32_t> seen_unknown_labels_;
};

void fillSemanticImage(SemanticColorConfig &config,
                       const cv::Mat &classes,
                       cv::Mat &output);

inline size_t getFileSize(std::istream &to_check) {
  to_check.seekg(0, to_check.end);
  size_t size = to_check.tellg();
  to_check.seekg(0, to_check.beg);
  return size;
}

SegmentationConfig readSegmenterConfig(const ros::NodeHandle &nh);

void outputDemoImage(const DemoConfig &config, const cv::Mat &classes);

void showStatistics(const cv::Mat &classes);

void fillNetworkImage(const SegmentationConfig &config,
                      const cv::Mat &input,
                      cv::Mat &output);

}  // namespace semantic_recolor
