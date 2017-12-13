#include "cirp_vision_alg.h"

CirpVisionAlgorithm::CirpVisionAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

void CirpVisionAlgorithm::segmentate(const cv::Mat& in, cv::Mat& feedback,
    const PointCloud& cl, cirp_vision::CirpDetectedPieces& detected_pieces)
{
  segmenter_(in, 0.5);
  feedback = segmenter_.get_feedback();
  const std::vector<cv::Point>& means = segmenter_.get_means();
  detected_pieces.positions.resize(means.size());
  for (int idx = 0; idx < means.size(); ++idx)
  {
    pcl::PointXYZ point = cl(means[idx].x, means[idx].y);
    detected_pieces.positions[idx].x = point.x;
    detected_pieces.positions[idx].y = point.y;
    detected_pieces.positions[idx].z = point.z;
  }
  const std::vector<std::string>& colors = segmenter_.get_labels();
  detected_pieces.colors.resize(colors.size());
  for (int idx = 0; idx < colors.size(); ++idx)
  {
    detected_pieces.colors[idx] = colors[idx];
  }
}

CirpVisionAlgorithm::~CirpVisionAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CirpVisionAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;

  // roi
  segmenter_.set_roi(config_.crop_top, config_.crop_bottom, config_.crop_left,
      config_.crop_right);

  // tolerance
  segmenter_.set_tolerance(cv::Vec3b(config_.h_tolerance, config_.s_tolerance,
        config_.v_tolerance));

  // other
  segmenter_.set_fg_erosion_radius(config_.fg_erosion_r);
  segmenter_.set_bg_erosion_radius(config_.bg_erosion_r);
  segmenter_.set_min_area(config_.min_area);
  segmenter_.set_max_area(config_.max_area);

  // colors
  segmenter_.clear_colors();
  segmenter_.set_color("red", cv::Vec3b(config_.red_h, config_.red_s,
        config_.red_v));
  segmenter_.set_color("blue", cv::Vec3b(config_.blue_h, config_.blue_s,
        config_.blue_v));
  segmenter_.set_color("green", cv::Vec3b(config_.green_h, config_.green_s,
        config_.green_v));
  segmenter_.set_color("yellow", cv::Vec3b(config_.yellow_h, config_.yellow_s,
        config_.yellow_v));

  this->unlock();
}

// CirpVisionAlgorithm Public API
