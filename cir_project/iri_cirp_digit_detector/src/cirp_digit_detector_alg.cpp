#include "cirp_digit_detector_alg.h"

CirpDigitDetectorAlgorithm::CirpDigitDetectorAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CirpDigitDetectorAlgorithm::~CirpDigitDetectorAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CirpDigitDetectorAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CirpDigitDetectorAlgorithm Public API
