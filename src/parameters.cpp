#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
state_input state_in;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false, cut_frame = false;
bool use_imu_as_input = false, space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro, cut_frame_time_interval = 0.1;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000;
float  DET_RANGE = 450;
bool   imu_en = true;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov;
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
double time_diff_lidar_to_imu = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;
int cut_frame_num = 1, orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool cut_frame_init = false; // true;

MeasureGroup Measures;

ofstream fout_out, fout_imu_pbp;

// Global node pointer for ROS2
rclcpp::Node::SharedPtr g_node;

void readParameters(rclcpp::Node::SharedPtr node)
{
  g_node = node;
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  // Declare and get parameters
  node->declare_parameter("prop_at_freq_of_imu", true);
  node->declare_parameter("use_imu_as_input", false);
  node->declare_parameter("check_satu", true);
  node->declare_parameter("init_map_size", 100);
  node->declare_parameter("space_down_sample", true);
  node->declare_parameter("mapping.satu_acc", 3.0);
  node->declare_parameter("mapping.satu_gyro", 35.0);
  node->declare_parameter("mapping.acc_norm", 1.0);
  node->declare_parameter("mapping.plane_thr", 0.05);
  node->declare_parameter("point_filter_num", 2);
  node->declare_parameter("common.lid_topic", std::string("/livox/lidar"));
  node->declare_parameter("common.imu_topic", std::string("/livox/imu"));
  node->declare_parameter("common.con_frame", false);
  node->declare_parameter("common.con_frame_num", 1);
  node->declare_parameter("common.cut_frame", false);
  node->declare_parameter("common.cut_frame_time_interval", 0.1);
  node->declare_parameter("common.time_diff_lidar_to_imu", 0.0);
  node->declare_parameter("filter_size_surf", 0.5);
  node->declare_parameter("filter_size_map", 0.5);
  node->declare_parameter("mapping.det_range", 300.0);
  node->declare_parameter("mapping.fov_degree", 180.0);
  node->declare_parameter("mapping.imu_en", true);
  node->declare_parameter("mapping.extrinsic_est_en", true);
  node->declare_parameter("mapping.imu_time_inte", 0.005);
  node->declare_parameter("mapping.lidar_meas_cov", 0.1);
  node->declare_parameter("mapping.acc_cov_input", 0.1);
  node->declare_parameter("mapping.vel_cov", 20.0);
  node->declare_parameter("mapping.gyr_cov_input", 0.1);
  node->declare_parameter("mapping.gyr_cov_output", 0.1);
  node->declare_parameter("mapping.acc_cov_output", 0.1);
  node->declare_parameter("mapping.b_gyr_cov", 0.0001);
  node->declare_parameter("mapping.b_acc_cov", 0.0001);
  node->declare_parameter("mapping.imu_meas_acc_cov", 0.1);
  node->declare_parameter("mapping.imu_meas_omg_cov", 0.1);
  node->declare_parameter("preprocess.blind", 1.0);
  node->declare_parameter("preprocess.lidar_type", 1);
  node->declare_parameter("preprocess.scan_line", 16);
  node->declare_parameter("preprocess.scan_rate", 10);
  node->declare_parameter("preprocess.timestamp_unit", 1);
  node->declare_parameter("mapping.match_s", 81.0);
  node->declare_parameter("mapping.gravity", std::vector<double>());
  node->declare_parameter("mapping.gravity_init", std::vector<double>());
  node->declare_parameter("mapping.extrinsic_T", std::vector<double>());
  node->declare_parameter("mapping.extrinsic_R", std::vector<double>());
  node->declare_parameter("odometry.publish_odometry_without_downsample", false);
  node->declare_parameter("publish.path_en", true);
  node->declare_parameter("publish.scan_publish_en", true);
  node->declare_parameter("publish.scan_bodyframe_pub_en", true);
  node->declare_parameter("runtime_pos_log_enable", false);
  node->declare_parameter("pcd_save.pcd_save_en", false);
  node->declare_parameter("pcd_save.interval", -1);
  node->declare_parameter("mapping.lidar_time_inte", 0.1);
  node->declare_parameter("mapping.ivox_grid_resolution", 0.2);
  node->declare_parameter("ivox_nearby_type", 18);

  // Get parameter values
  prop_at_freq_of_imu = node->get_parameter("prop_at_freq_of_imu").as_bool();
  use_imu_as_input = node->get_parameter("use_imu_as_input").as_bool();
  check_satu = node->get_parameter("check_satu").as_bool();
  init_map_size = node->get_parameter("init_map_size").as_int();
  space_down_sample = node->get_parameter("space_down_sample").as_bool();
  satu_acc = node->get_parameter("mapping.satu_acc").as_double();
  satu_gyro = node->get_parameter("mapping.satu_gyro").as_double();
  acc_norm = node->get_parameter("mapping.acc_norm").as_double();
  plane_thr = static_cast<float>(node->get_parameter("mapping.plane_thr").as_double());
  p_pre->point_filter_num = node->get_parameter("point_filter_num").as_int();
  lid_topic = node->get_parameter("common.lid_topic").as_string();
  imu_topic = node->get_parameter("common.imu_topic").as_string();
  con_frame = node->get_parameter("common.con_frame").as_bool();
  con_frame_num = node->get_parameter("common.con_frame_num").as_int();
  cut_frame = node->get_parameter("common.cut_frame").as_bool();
  cut_frame_time_interval = node->get_parameter("common.cut_frame_time_interval").as_double();
  time_diff_lidar_to_imu = node->get_parameter("common.time_diff_lidar_to_imu").as_double();
  filter_size_surf_min = node->get_parameter("filter_size_surf").as_double();
  filter_size_map_min = node->get_parameter("filter_size_map").as_double();
  DET_RANGE = static_cast<float>(node->get_parameter("mapping.det_range").as_double());
  fov_deg = node->get_parameter("mapping.fov_degree").as_double();
  imu_en = node->get_parameter("mapping.imu_en").as_bool();
  extrinsic_est_en = node->get_parameter("mapping.extrinsic_est_en").as_bool();
  imu_time_inte = node->get_parameter("mapping.imu_time_inte").as_double();
  laser_point_cov = node->get_parameter("mapping.lidar_meas_cov").as_double();
  acc_cov_input = node->get_parameter("mapping.acc_cov_input").as_double();
  vel_cov = node->get_parameter("mapping.vel_cov").as_double();
  gyr_cov_input = node->get_parameter("mapping.gyr_cov_input").as_double();
  gyr_cov_output = node->get_parameter("mapping.gyr_cov_output").as_double();
  acc_cov_output = node->get_parameter("mapping.acc_cov_output").as_double();
  b_gyr_cov = node->get_parameter("mapping.b_gyr_cov").as_double();
  b_acc_cov = node->get_parameter("mapping.b_acc_cov").as_double();
  imu_meas_acc_cov = node->get_parameter("mapping.imu_meas_acc_cov").as_double();
  imu_meas_omg_cov = node->get_parameter("mapping.imu_meas_omg_cov").as_double();
  p_pre->blind = node->get_parameter("preprocess.blind").as_double();
  lidar_type = node->get_parameter("preprocess.lidar_type").as_int();
  p_pre->N_SCANS = node->get_parameter("preprocess.scan_line").as_int();
  p_pre->SCAN_RATE = node->get_parameter("preprocess.scan_rate").as_int();
  p_pre->time_unit = node->get_parameter("preprocess.timestamp_unit").as_int();
  match_s = node->get_parameter("mapping.match_s").as_double();
  gravity = node->get_parameter("mapping.gravity").as_double_array();
  gravity_init = node->get_parameter("mapping.gravity_init").as_double_array();
  extrinT = node->get_parameter("mapping.extrinsic_T").as_double_array();
  extrinR = node->get_parameter("mapping.extrinsic_R").as_double_array();
  publish_odometry_without_downsample = node->get_parameter("odometry.publish_odometry_without_downsample").as_bool();
  path_en = node->get_parameter("publish.path_en").as_bool();
  scan_pub_en = node->get_parameter("publish.scan_publish_en").as_bool();
  scan_body_pub_en = node->get_parameter("publish.scan_bodyframe_pub_en").as_bool();
  runtime_pos_log = node->get_parameter("runtime_pos_log_enable").as_bool();
  pcd_save_en = node->get_parameter("pcd_save.pcd_save_en").as_bool();
  pcd_save_interval = node->get_parameter("pcd_save.interval").as_int();
  lidar_time_inte = node->get_parameter("mapping.lidar_time_inte").as_double();
  ivox_options_.resolution_ = static_cast<float>(node->get_parameter("mapping.ivox_grid_resolution").as_double());
  ivox_nearby_type = node->get_parameter("ivox_nearby_type").as_int();

  // Set ivox nearby type
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }

  // Handle empty vectors with defaults
  if (gravity.empty()) {
    gravity = {0.0, 0.0, -9.81};
  }
  if (gravity_init.empty()) {
    gravity_init = {0.0, 0.0, -9.81};
  }
  if (extrinT.empty()) {
    extrinT = {0.0, 0.0, 0.0};
  }
  if (extrinR.empty()) {
    extrinR = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  }

  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot)
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{

    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}
