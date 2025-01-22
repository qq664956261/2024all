#pragma once

#include "log.h"
#include "rapidjson/document.h"
#include <fstream>  
#include <iostream> 
#include <typeinfo>

class TrackingConfigParameters{
public:
  TrackingConfigParameters() = default;
  void loadParameters(const rapidjson::Value &config){
    HJ_INFO("********parse tracking parameter********");
    earth_rotation_speed_ = config["earth_param"]["earth_rotation_speed"].GetDouble();

    earth_gravity_ = config["earth_param"]["earth_gravity"].GetDouble();
    position_error_prior_std_ = config["filter_param"]["para1"].GetDouble();
    velocity_error_prior_std_ = config["filter_param"]["para2"].GetDouble();
    rotation_error_prior_std_ = config["filter_param"]["para3"].GetDouble();
    gyro_bias_error_prior_std_ = config["filter_param"]["para4"].GetDouble();
    accelerometer_bias_error_prior_std_ = config["filter_param"]["para5"].GetDouble();

    gyro_noise_std_ = config["imu_param"]["para1"].GetDouble();
    accelerometer_noise_std_ = config["imu_param"]["para2"].GetDouble();
    accelerometer_bias_[0] = config["imu_param"]["para3"].GetDouble();
    accelerometer_bias_[1] = config["imu_param"]["para4"].GetDouble();
    accelerometer_bias_[2] = config["imu_param"]["para5"].GetDouble();
    gyro_bias_[0] = config["imu_param"]["para6"].GetDouble();
    gyro_bias_[1] = config["imu_param"]["para7"].GetDouble();
    gyro_bias_[2] = config["imu_param"]["para8"].GetDouble();

    Twb_mcu_imu_[0] = config["imu_axis"]["mat_0_0"].GetDouble();
    Twb_mcu_imu_[1] = config["imu_axis"]["mat_0_1"].GetDouble();
    Twb_mcu_imu_[2] = config["imu_axis"]["mat_0_2"].GetDouble();
    Twb_mcu_imu_[3] = config["imu_axis"]["mat_1_0"].GetDouble();
    Twb_mcu_imu_[4] = config["imu_axis"]["mat_1_1"].GetDouble();
    Twb_mcu_imu_[5] = config["imu_axis"]["mat_1_2"].GetDouble();
    Twb_mcu_imu_[6] = config["imu_axis"]["mat_2_0"].GetDouble();
    Twb_mcu_imu_[7] = config["imu_axis"]["mat_2_1"].GetDouble();
    Twb_mcu_imu_[8] = config["imu_axis"]["mat_2_2"].GetDouble();
    Twb_mcu_euler_[0] = config["imu_euler"]["mat_0_0"].GetDouble();
    Twb_mcu_euler_[1] = config["imu_euler"]["mat_0_1"].GetDouble();
    Twb_mcu_euler_[2] = config["imu_euler"]["mat_0_2"].GetDouble();
    Twb_mcu_euler_[3] = config["imu_euler"]["mat_1_0"].GetDouble();
    Twb_mcu_euler_[4] = config["imu_euler"]["mat_1_1"].GetDouble();
    Twb_mcu_euler_[5] = config["imu_euler"]["mat_1_2"].GetDouble();
    Twb_mcu_euler_[6] = config["imu_euler"]["mat_2_0"].GetDouble();
    Twb_mcu_euler_[7] = config["imu_euler"]["mat_2_1"].GetDouble();
    Twb_mcu_euler_[8] = config["imu_euler"]["mat_2_2"].GetDouble();
    Twb_soc_imu_[0] = config["soc_imu_axis"]["mat_0_0"].GetDouble();
    Twb_soc_imu_[1] = config["soc_imu_axis"]["mat_0_1"].GetDouble();
    Twb_soc_imu_[2] = config["soc_imu_axis"]["mat_0_2"].GetDouble();
    Twb_soc_imu_[3] = config["soc_imu_axis"]["mat_1_0"].GetDouble();
    Twb_soc_imu_[4] = config["soc_imu_axis"]["mat_1_1"].GetDouble();
    Twb_soc_imu_[5] = config["soc_imu_axis"]["mat_1_2"].GetDouble();
    Twb_soc_imu_[6] = config["soc_imu_axis"]["mat_2_0"].GetDouble();
    Twb_soc_imu_[7] = config["soc_imu_axis"]["mat_2_1"].GetDouble();
    Twb_soc_imu_[8] = config["soc_imu_axis"]["mat_2_2"].GetDouble();

    encoder_position_x_std_ = config["encoder_param"]["encoder_para1"].GetDouble();
    encoder_position_y_std_ = config["encoder_param"]["encoder_para2"].GetDouble();
    encoder_position_z_std_ = config["encoder_param"]["encoder_para3"].GetDouble();
    encoder_velocity_x_std_ = config["encoder_param"]["encoder_para4"].GetDouble();
    encoder_velocity_y_std_ = config["encoder_param"]["encoder_para5"].GetDouble();
    encoder_velocity_z_std_ = config["encoder_param"]["encoder_para6"].GetDouble();

    encoder_rc_ = config["encoder_param"]["encoder_rc"].GetDouble();
    encoder_scale_ = config["encoder_param"]["encoder_scale"].GetDouble();
    magnetic_offset_x_ = config["magnetic_param"]["magnetic_offset_x"].GetDouble();
    magnetic_offset_y_ = config["magnetic_param"]["magnetic_offset_y"].GetDouble();
    magnetic_offset_z_ = config["magnetic_param"]["magnetic_offset_z"].GetDouble();
    magnetic_x_std_ = config["magnetic_param"]["magnetic_x_std"].GetDouble();
    magnetic_y_std_ = config["magnetic_param"]["magnetic_y_std"].GetDouble();
    magnetic_z_std_ = config["magnetic_param"]["magnetic_z_std"].GetDouble();
    only_prediction_ = config["only_prediction"].GetBool();
    use_earth_model_ = config["use_earth_model"].GetBool();
    use_magnetic_model_ = config["use_magnetic_model"].GetBool();

    HJ_INFO("####### Config Parameters #######");
    HJ_INFO("earth_rotation_speed: %f", earth_rotation_speed_);
    HJ_INFO("earth_gravity: %f", earth_gravity_);
    HJ_INFO("position_error_prior_std: %f", position_error_prior_std_);
    HJ_INFO("velocity_error_prior_std: %f", velocity_error_prior_std_);
    HJ_INFO("rotation_error_prior_std: %f", rotation_error_prior_std_);
    HJ_INFO("accelerometer_bias_error_prior_std: %f", accelerometer_bias_error_prior_std_);
    HJ_INFO("gyro_bias_error_prior_std: %f", gyro_bias_error_prior_std_);
    HJ_INFO("gyro_noise_std: %f", gyro_noise_std_);
    HJ_INFO("accelerometer_noise_std: %f", accelerometer_noise_std_);
    HJ_INFO("encoder_position_x_std: %f", encoder_position_x_std_);
    HJ_INFO("encoder_position_y_std: %f", encoder_position_y_std_);
    HJ_INFO("encoder_position_z_std: %f", encoder_position_z_std_);
    HJ_INFO("encoder_velocity_x_std: %f", encoder_velocity_x_std_);
    HJ_INFO("encoder_velocity_y_std: %f", encoder_velocity_y_std_);
    HJ_INFO("encoder_velocity_z_std: %f", encoder_velocity_z_std_);
    HJ_INFO("encoder_rc: %f", encoder_rc_);
    HJ_INFO("encoder_scale: %f", encoder_scale_);
    HJ_INFO("magnetic_offset_x_: %f", magnetic_offset_x_);
    HJ_INFO("magnetic_offset_y_: %f", magnetic_offset_y_);
    HJ_INFO("magnetic_offset_z_: %f", magnetic_offset_z_);
    HJ_INFO("magnetic_x_std_: %f", magnetic_x_std_);
    HJ_INFO("magnetic_y_std_: %f", magnetic_y_std_);
    HJ_INFO("magnetic_z_std_: %f", magnetic_z_std_);
    HJ_INFO("only_prediction: %d", only_prediction_);
    HJ_INFO("use_earth_model: %d", use_earth_model_);
    HJ_INFO("use_magnetic_model: %d", use_magnetic_model_);
  }

public:
  // earth
  double earth_rotation_speed_{};
  double earth_gravity_{};

  // kalman prediction process std
  double position_error_prior_std_{};
  double velocity_error_prior_std_{};
  double rotation_error_prior_std_{};
  double accelerometer_bias_error_prior_std_{};
  double gyro_bias_error_prior_std_{};

  // imu sensor noise
  double gyro_noise_std_{};
  double accelerometer_noise_std_{};

  //imu sensor bias
  double accelerometer_bias_[3] = {};
  double gyro_bias_[3] = {};

  double encoder_rc_{};
  double encoder_scale_{};

  // kalman measurement process std
  double encoder_position_x_std_{};
  double encoder_position_y_std_{};
  double encoder_position_z_std_{};
  double encoder_velocity_x_std_{};
  double encoder_velocity_y_std_{};
  double encoder_velocity_z_std_{};

  //magnetic measurement process
  double magnetic_offset_x_{};
  double magnetic_offset_y_{};
  double magnetic_offset_z_{};
  double magnetic_x_std_{};
  double magnetic_y_std_{};
  double magnetic_z_std_{};

  //external imu axis
  double Twb_mcu_imu_[9]{};
  double Twb_soc_imu_[9]{};
  double Twb_mcu_euler_[9]{};

  // only using IMU to integration
  bool only_prediction_{};

  bool use_earth_model_{};

  bool use_magnetic_model_{};
};

class ICPConfigParameters{
public:
  ICPConfigParameters() = default;
  void loadParameters(const rapidjson::Value &json_conf){
    HJ_INFO("********parse optimize parameter********");
    if (json_conf.HasMember("savelog_path") && json_conf["savelog_path"].IsString()) {
      savelog_path = json_conf["savelog_path"].GetString();
      HJ_INFO("parse savelog_path parameter:: %s", savelog_path.c_str());
    }
    else {
      HJ_ERROR("Error: cannot parse savelog_path parameter!");
    }
    
    // if (json_conf.HasMember("load_map_as_target") && json_conf["load_map_as_target"].IsBool()) {
    //   load_map_as_target = json_conf["load_map_as_target"].GetBool();
    //   HJ_INFO("parse load_map_as_target parameter:: %d", load_map_as_target);
    // }
    // else {
    //   HJ_ERROR("Error: cannot parse load_map_as_target parameter!");
    // }
  }
public:
  std::string savelog_path{};
  // bool load_map_as_target{};
};


class MappingConfigParameters{
public:
  MappingConfigParameters() = default;
  void loadParameters(const rapidjson::Value &json_conf){
    HJ_INFO("********parse mapping parameter********");
    savelog_path = json_conf["savelog_path"].GetString();
    save_map = json_conf["save_map"].GetBool();
    save_map_path = json_conf["save_map_path"].GetString();
    load_map = json_conf["load_map"].GetBool();
    load_map_path = json_conf["load_map_path"].GetString();
    sonar_left_front_fov_rad = json_conf["ultra_param"]["sonar_left_front_fov_rad"].GetDouble();
    sonar_left_front_base_x = json_conf["ultra_param"]["sonar_left_front_base_x"].GetDouble();
    sonar_left_front_base_y = json_conf["ultra_param"]["sonar_left_front_base_y"].GetDouble();
    sonar_left_front_base_yaw = json_conf["ultra_param"]["sonar_left_front_base_yaw"].GetDouble();
    sonar_left_back_fov_rad = json_conf["ultra_param"]["sonar_left_back_fov_rad"].GetDouble();
    sonar_left_back_base_x = json_conf["ultra_param"]["sonar_left_back_base_x"].GetDouble();
    sonar_left_back_base_y = json_conf["ultra_param"]["sonar_left_back_base_y"].GetDouble();
    sonar_left_back_base_yaw = json_conf["ultra_param"]["sonar_left_back_base_yaw"].GetDouble();

    HJ_INFO("parse savelog_path parameter:: %s", savelog_path.c_str());
    HJ_INFO("parse save_map parameter: %d", save_map);
    HJ_INFO("parse save_map_path parameter: %s", save_map_path.c_str());
    HJ_INFO("parse load_map parameter: %d", load_map);
    HJ_INFO("parse load_map sonar_left_front_fov_rad: %f", sonar_left_front_fov_rad);
    HJ_INFO("parse load_map sonar_left_front_base_x: %f", sonar_left_front_base_x);
    HJ_INFO("parse load_map sonar_left_front_base_y: %f", sonar_left_front_base_y);
    HJ_INFO("parse load_map sonar_left_front_base_yaw: %f", sonar_left_front_base_yaw);
    HJ_INFO("parse load_map sonar_left_back_fov_rad: %f", sonar_left_back_fov_rad);
    HJ_INFO("parse load_map sonar_left_back_base_x: %f", sonar_left_back_base_x);
    HJ_INFO("parse load_map sonar_left_back_base_y: %f", sonar_left_back_base_y);
    HJ_INFO("parse load_map sonar_left_back_base_yaw: %f", sonar_left_back_base_yaw);

  }
public:
  std::string savelog_path{};
 
  bool save_map{};
  std::string save_map_path{};
  bool load_map{};
  std::string load_map_path{};
  double sonar_left_front_fov_rad{};
  double sonar_left_front_base_x{};
  double sonar_left_front_base_y{};
  double sonar_left_front_base_yaw{};
  double sonar_left_back_fov_rad{};
  double sonar_left_back_base_x{};
  double sonar_left_back_base_y{};
  double sonar_left_back_base_yaw{};
};