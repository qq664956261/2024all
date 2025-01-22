#include <memory>
#include "interface.h"
#include "manager.h"

std::unique_ptr<SLAMManager> impl_ = nullptr;

int PutRelocalizationData(const Eigen::Matrix4d &relocalization_pose, const Eigen::Matrix4d &water_entry_pose, 
                          uint8_t relocalization_result, uint8_t build_frames_result, const uint8_t &curr_task) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdRelocalizationData(relocalization_pose, water_entry_pose, relocalization_result, build_frames_result, curr_task);
  return 0;
}

int PutTaskData(const uint8_t &curr_task) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdTaskData(curr_task);
  return 0;
}

int PutImuData(const IMUData &imu_data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdImuData(imu_data);
  return 0;
}

int PutSocImuData(const SOCIMUData &soc_imu_data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdSocImuData(soc_imu_data);
  return 0;
}

int PutMagData(const MAGData &mag_data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdMagData(mag_data);
  return 0;
}

int PutEncoderData(const ENcoderData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutEncoderData(data, data.exposure_ts);
  return 0;
}

int PutUltraData(const ULtraData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdUltraData(data);
  return 0;
}

int PutDepthData(const DEPData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdDepthData(data);
  return 0;
}

#ifdef X9
int PutLeftFrontUltraData(const LEFTFrontData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdLeftFrontUltraData(data);
  return 0;
}

int PutLeftBackUltraData(const LEFTBackData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdLeftBackUltraData(data);
  return 0;
}
#endif

#ifdef T1_pro
int PutLeftTofData(const LEFTTofData &data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutHmdLeftTofData(data);
  return 0;
}
#endif

int PutNaviSlamData(const NAVISlamData &navi_slam_data) {
  if (!impl_) {
    return -1;
  }
  impl_->PutNaviSlamData(navi_slam_data);
  return 0;
}


int32_t SensorFusionInitialImpl(const std::string calibration_directory, const std::string output_directory,
                                const std::string relocalization_directory, const std::string vocabulary) {
  if (!impl_) {
    impl_.reset(new SLAMManager(calibration_directory, output_directory, relocalization_directory, vocabulary));
    return 0;
  }
  return -1;
}