#include <iostream>
#include <math.h>
#include <ros/publisher.h>
#include <ros/ros.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

#include <quadrotor_msgs/polynomial.h>

class RefPath {
public:
  RefPath(ros::NodeHandle nh, ros::Rate loop_rate, int path_num,
          int path_length, std::string topic);
  void run();

protected:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  ros::Subscriber sub_path_coefficient_;
  ros::Publisher pub_pseudo_path_;
  ros::Publisher pub_real_ref_path_;

  int path_num_;
  int path_length_;
  std::string topic_;

  std::vector<Eigen::Matrix<double, 3, 6>> coefficient_;
  std::vector<double> t_max_;
  int piece_num_;
  std::vector<double> t_step_;
  void PathCoefficientCB(const quadrotor_msgs::polynomial msg);

  geometry_msgs::PoseArray getRealPath();
  geometry_msgs::PoseArray pseudoNextPath2();

  Eigen::Quaterniond
  differentialDirectionPoint(const geometry_msgs::Point &start_p,
                             const geometry_msgs::Point &end_p);
};

RefPath::RefPath(ros::NodeHandle nh, ros::Rate loop_rate, int path_num,
                 int path_length, std::string topic)
    : nh_(nh), loop_rate_(loop_rate), path_num_(path_num),
      path_length_(path_length), topic_(topic) {
  pub_pseudo_path_ =
      nh_.advertise<geometry_msgs::PoseArray>("/next_base_path", 1);
  pub_real_ref_path_ =
      nh.advertise<geometry_msgs::PoseArray>("/real_path", 100);
  sub_path_coefficient_ =
      nh_.subscribe(topic_, 1, &RefPath::PathCoefficientCB, this);
  piece_num_ = 0;
}

/////////// callback ////////////
void RefPath::PathCoefficientCB(const quadrotor_msgs::polynomial msg) {
  std::vector<double> x_coeff = msg.Xcoeff;
  std::vector<double> y_coeff = msg.Ycoeff;
  std::vector<double> z_coeff = msg.Zcoeff;
  t_max_ = msg.Duration;
  piece_num_ = msg.pieceNum;
  Eigen::Matrix<double, 3, 6> coef;
  coefficient_.clear();
  t_step_.clear();

  for (int i = 0; i < piece_num_; i++) {
    t_step_.push_back(t_max_[i] / path_length_);

    coef << x_coeff[i * 6 + 5], x_coeff[i * 6 + 4], x_coeff[i * 6 + 3],
        x_coeff[i * 6 + 2], x_coeff[i * 6 + 1], x_coeff[i * 6 + 0],
        y_coeff[i * 6 + 5], y_coeff[i * 6 + 4], y_coeff[i * 6 + 3],
        y_coeff[i * 6 + 2], y_coeff[i * 6 + 1], y_coeff[i * 6 + 0],
        z_coeff[i * 6 + 5], z_coeff[i * 6 + 4], z_coeff[i * 6 + 3],
        z_coeff[i * 6 + 2], z_coeff[i * 6 + 1], z_coeff[i * 6 + 0];

    coefficient_.push_back(coef);
  }
  std::cout << "ontrollerLog:   piece_num_" << piece_num_ << std::endl;
}

// {
//   int pieceIdx = locatePieceIdx(t);
//   return pieces[pieceIdx].getPos(t);
// }

/////////// run ////////////
void RefPath::run() {
  geometry_msgs::PoseArray base_path;

  while (ros::ok()) {
    if (path_num_ == 0) {
      base_path = getRealPath();
    } else if (path_num_ == 2) {
      base_path = pseudoNextPath2();
    }

    // std::cout << "base_path=======" << base_path << std::endl;
    pub_pseudo_path_.publish(base_path);

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

Eigen::Quaterniond
RefPath::differentialDirectionPoint(const geometry_msgs::Point &start_p,
                                    const geometry_msgs::Point &end_p) {
  Eigen::Quaterniond output_q;
  double dx, dy, dz;
  dx = start_p.x - end_p.x;
  dy = start_p.y - end_p.y;
  dz = start_p.z - end_p.z;

  Eigen::Vector3d euler_angle;
  euler_angle(0) = 0;
  euler_angle(1) =
      std::acos(std::sqrt((dx * dx + dy * dy) / (dx * dx + dy * dy + dz * dz)));
  if (dz < 0) {
    euler_angle(1) = 6.28 - euler_angle(1);
  }
  euler_angle(2) = std::atan2(dy, dx) + 3.14;

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(euler_angle[2], ::Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(euler_angle[1], ::Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler_angle[0], ::Eigen::Vector3d::UnitX());
  output_q = R;
  return output_q;
}

geometry_msgs::PoseArray RefPath::getRealPath() {
  geometry_msgs::PoseArray path;
  path.poses.clear();

  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();

  double t;

  geometry_msgs::Pose pseudo_path_pose;
  double path_p_x[path_length_ * piece_num_],
      path_p_y[path_length_ * piece_num_], path_p_z[path_length_ * piece_num_];
  double path_q_w[path_length_ * piece_num_],
      path_q_x[path_length_ * piece_num_], path_q_y[path_length_ * piece_num_],
      path_q_z[path_length_ * piece_num_];

  for (int num = 0; num < piece_num_; num++) {
    t = 0;
    for (int i = 0; i < path_length_; i++) {
      path_p_x[i + num * path_length_] =
          coefficient_[num](0, 0) + coefficient_[num](0, 1) * t +
          coefficient_[num](0, 2) * t * t +
          coefficient_[num](0, 3) * t * t * t +
          coefficient_[num](0, 4) * t * t * t * t +
          coefficient_[num](0, 5) * t * t * t * t * t;
      path_p_y[i + num * path_length_] =
          coefficient_[num](1, 0) + coefficient_[num](1, 1) * t +
          coefficient_[num](1, 2) * t * t +
          coefficient_[num](1, 3) * t * t * t +
          coefficient_[num](1, 4) * t * t * t * t +
          coefficient_[num](1, 5) * t * t * t * t * t;
      path_p_z[i + num * path_length_] =
          coefficient_[num](2, 0) + coefficient_[num](2, 1) * t +
          coefficient_[num](2, 2) * t * t +
          coefficient_[num](2, 3) * t * t * t +
          coefficient_[num](2, 4) * t * t * t * t +
          coefficient_[num](2, 5) * t * t * t * t * t;
      t = t + t_step_[num];
    }
  }

  for (int j = 0; j < path_length_ * piece_num_; j++) {
    if (j == path_length_ * piece_num_ - 1) {
      path_q_w[j] = 1;
      path_q_x[j] = 0;
      path_q_y[j] = 0;
      path_q_z[j] = 0;
    } else {
      geometry_msgs::Point start_p;
      start_p.x = path_p_x[j];
      start_p.y = path_p_y[j];
      start_p.z = path_p_z[j];

      geometry_msgs::Point end_p;
      end_p.x = path_p_x[j + 1];
      end_p.y = path_p_y[j + 1];
      end_p.z = path_p_z[j + 1];

      Eigen::Quaterniond q;
      q = differentialDirectionPoint(start_p, end_p);
      path_q_w[j] = q.w();
      path_q_x[j] = q.x();
      path_q_y[j] = q.y();
      path_q_z[j] = q.z();
    }
  }
  for (int k = 0; k < path_length_ * piece_num_; k++) {
    pseudo_path_pose.orientation.w = path_q_w[k];
    pseudo_path_pose.orientation.x = path_q_x[k];
    pseudo_path_pose.orientation.y = path_q_y[k];
    pseudo_path_pose.orientation.z = path_q_z[k];

    pseudo_path_pose.position.x = path_p_x[k];
    pseudo_path_pose.position.y = path_p_y[k];
    pseudo_path_pose.position.z = path_p_z[k];

    path.poses.push_back(pseudo_path_pose);
  }
  pub_real_ref_path_.publish(path);

  return path;
}

geometry_msgs::PoseArray RefPath::pseudoNextPath2() {
  geometry_msgs::PoseArray path;
  path.poses.clear();

  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();

  int pathlength = 170;
  geometry_msgs::Pose pseudo_path_pose;
  double path_p_x[pathlength], path_p_y[pathlength], path_p_z[pathlength];
  double path_q_w[pathlength], path_q_x[pathlength], path_q_y[pathlength],
      path_q_z[pathlength];

  for (int i = 0; i < pathlength; i++) {
    if (i < 50) {
      path_p_x[i] = i * 0.1;
      path_p_y[i] = 0;
      path_p_z[i] = -1.63;
    }
    if (i >= 50 && i < 90) {
      path_p_x[i] = 2 * std::cos((i - 50) * 3.14 / 40 - 1.57) + 5.0;
      path_p_y[i] = 2 * std::sin((i - 50) * 3.14 / 40 - 1.57) + 2.0;
      path_p_z[i] = -1.63 - (i - 50) * 0.1;
    }
    if (i >= 90 && i < 110) {
      path_p_x[i] = (i - 90) * (-0.1) + 5;
      path_p_y[i] = 4;
      path_p_z[i] = -5.63 - (i - 90) * 0.1;
    }
    if (i >= 110 && i < 150) {
      path_p_x[i] = -2 * std::cos((i - 110) * 3.14 / 40 - 1.57) + 3.0;
      path_p_y[i] = 2 * std::sin((i - 110) * 3.14 / 40 - 1.57) + 6.0;
      path_p_z[i] = -7.63 - (i - 110) * 0.1;
    }
    if (i >= 150 && i < pathlength) {
      path_p_x[i] = (i - 150) * 0.1 + 3;
      path_p_y[i] = 8;
      path_p_z[i] = -11.63;
    }
  }
  for (int j = 0; j < pathlength; j++) {
    if (j == pathlength - 1) {
      path_q_w[j] = 1;
      path_q_x[j] = 0;
      path_q_y[j] = 0;
      path_q_z[j] = 0;
    } else {
      geometry_msgs::Point start_p;
      start_p.x = path_p_x[j];
      start_p.y = path_p_y[j];
      start_p.z = path_p_z[j];

      geometry_msgs::Point end_p;
      end_p.x = path_p_x[j + 1];
      end_p.y = path_p_y[j + 1];
      end_p.z = path_p_z[j + 1];

      Eigen::Quaterniond q;
      q = differentialDirectionPoint(start_p, end_p);
      path_q_w[j] = q.w();
      path_q_x[j] = q.x();
      path_q_y[j] = q.y();
      path_q_z[j] = q.z();
    }
  }
  for (int k = 0; k < pathlength; k++) {
    pseudo_path_pose.orientation.w = path_q_w[k];
    pseudo_path_pose.orientation.x = path_q_x[k];
    pseudo_path_pose.orientation.y = path_q_y[k];
    pseudo_path_pose.orientation.z = path_q_z[k];

    pseudo_path_pose.position.x = path_p_x[k];
    pseudo_path_pose.position.y = path_p_y[k];
    pseudo_path_pose.position.z = path_p_z[k];

    path.poses.push_back(pseudo_path_pose);
  }

  return path;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pseudo_path_publisher");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  int path_num;
  int path_length;
  std::string topic;

  /// LOADING PARAMETERS FROM THE ROS SERVER
  if (!nh.getParam("path_num", path_num)) {
    ROS_ERROR("Couldn't retrieve the path code.");
    return -1;
  }
  if (!nh.getParam("path_length", path_length)) {
    ROS_ERROR("Couldn't retrieve the path code.");
    return -1;
  }
  if (!nh.getParam("topic", topic)) {
    ROS_ERROR("Couldn't retrieve the topic name for the pseudo path.");
    return -1;
  }

  RefPath ref_path_generate(nh, loop_rate, path_num, path_length, topic);
  ref_path_generate.run();

  return 0;
}