#include "MPCController.h"

MPCController::MPCController(ros::NodeHandle &n, double frequency,
                             std::string topic_base_command,
                             std::string topic_base_state, int Horizon,
                             double dt, double low_v_ref, double high_v_ref,
                             bool sim, std::vector<double> u_min,
                             std::vector<double> u_max,
                             std::vector<double> x_min,
                             std::vector<double> x_max)
    : nh_(n),
      loop_rate_(frequency),
      Horizon_(Horizon),
      dt_(dt),
      low_v_ref_(low_v_ref),
      high_v_ref_(high_v_ref),
      sim_(sim),
      uMin_(u_min.data()),
      uMax_(u_max.data()),
      xMin_(x_min.data()),
      xMax_(x_max.data())
{
    ///// Subscribers
    sub_next_base_path_ =
        nh_.subscribe("/next_base_path", 1, &MPCController::nextBasePathCB, this,
                      ros::TransportHints().reliable().tcpNoDelay());

    sub_debug_odom_ = nh_.subscribe("/mavros/local_position/odom", 50,
                                    &MPCController::debugOdomCB, this);

    sub_odom_ = nh_.subscribe(topic_base_state, 50,
                              &MPCController::perceptionOdomCB, this);

    sub_path_coefficient_ = nh_.subscribe(
        "/reference/polynomial", 1, &MPCController::PathCoefficientCB, this);

    ////// Publishers
    pub_solution_ = nh_.advertise<nav_msgs::Path>("/solution", 1);

    pub_ref_ = nh_.advertise<nav_msgs::Path>("/ref", 1);

    historypath_pub = nh_.advertise<geometry_msgs::PoseArray>("/history_path", 1);

    // pub_cmd_vel_ = nh_.advertise<mavros_msgs::PositionTarget>(
        // "/mavros/setpoint_raw/local", 10);
    pub_cmd_vel_ = nh_.advertise<quadrotor_msgs::PositionCommand>(
        "/controller/position_cmd", 20);

    run_mpc_pub_ = nh_.advertise<std_msgs::Bool>("/mpc_recv", 1);

    // init flags
    frist_run_flag_ = true;
    mpc_ready_flag_ = false;
    path_size_ = 0;
    times_ = 0;
    escape_flag_ = false;
    speed_mode_ = 3;
    planning_stop_flag_ = false;
    have_target_ = false;
    path_finish_flag_ = false;
    mpc_vel_cmd_.setZero();
    ref_path_.poses.clear();
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////

void MPCController::nextBasePathCB(const geometry_msgs::PoseArrayConstPtr msg)
{
    ref_path_.poses.clear();
    for (int i = 0; i < msg->poses.size(); i++)
    {
        ref_path_.poses.push_back(msg->poses[i]);
    }
    path_size_ = ref_path_.poses.size();

    // std::cout << "ControllerLog:  ref path size: " << path_size_ << std::endl;
    if (msg->header.frame_id == "world" && path_size_ > 0)
    {
        mpc_ready_flag_ = true;
    }
}

void MPCController::debugOdomCB(const nav_msgs::Odometry &odom_msg)
{
    if (sim_)
    {
        // base_real_position_ << odom_msg.pose.pose.position.x,
        //     odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
        // base_real_orientation_.coeffs() << odom_msg.pose.pose.orientation.x,
        //     odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
        //     odom_msg.pose.pose.orientation.w;

        // Eigen::Vector3d base_real_euler = quaternion2euler(base_real_orientation_);

        // cur_mpc_state_set_ << base_real_position_, base_real_euler;

        // double curr_yaw = get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
        // ROS_INFO_THROTTLE(
        //     5, "ControllerLog:  GET pose %f %f %f", odom_msg.pose.pose.position.x,
        //     odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    }
}

void MPCController::perceptionOdomCB(const nav_msgs::Odometry &odom_msg)
{
    if (!sim_)
    {
        base_real_position_ << odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
        base_real_orientation_.coeffs() << odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w;

        Eigen::Vector3d base_real_euler = quaternion2euler(base_real_orientation_);

        cur_mpc_state_set_ << base_real_position_, base_real_euler;

        double curr_yaw = get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
        ROS_INFO_THROTTLE(
            5, "ControllerLog:  GET pose %f %f %f", odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    }
}

void MPCController::PathCoefficientCB(const quadrotor_msgs::polynomial msg)
{
    std::vector<double> x_coeff = msg.Xcoeff;
    std::vector<double> y_coeff = msg.Ycoeff;
    std::vector<double> z_coeff = msg.Zcoeff;
    std::vector<double> t_max_ = msg.Duration;
    int piece_num_ = msg.pieceNum;
    planning_stop_flag_ = false;
    if (msg.captured)
    {
        speed_mode_ = 2;
    }
    else
    {
        speed_mode_ = 3;
    }
    // std::cout << "ControllerLog:  speed_mode_ = " << speed_mode_ << std::endl;
    // ROS_INFO("ControllerLog:  planning_stop_flag_ = %d", planning_stop_flag_);
}

///////////////////////////////////////////////////////////////
////////////////////////// Utils /// //////////////////////////
///////////////////////////////////////////////////////////////

void MPCController::escapeMode()
{
    cur_time_ = ros::Time::now().toSec();
    double dt = cur_time_ - last_time_;
    // std::cout << "ControllerLog:   dt: " << dt << std::endl;

    if (dt > 1.3)
    {
        if (abs(last_real_position_(0) - base_real_position_(0)) < 0.04 &&
            abs(last_real_position_(1) - base_real_position_(1)) < 0.04 &&
            abs(last_real_position_(2) - base_real_position_(2)) < 0.04)
        {
            escape_flag_ = true;
            times_ = 100;
            std::cout << "ControllerLog:   escape mode" << std::endl;
        }

        last_time_ = cur_time_;
        last_real_position_ = base_real_position_;
    }
}

int MPCController::findNearestIndex()
{
    double min_dist = 1000.0;
    int nearest_index = -1;
    for (int i = 0; i < path_size_; i++)
    {
        double dx = base_real_position_(0) - ref_path_.poses[i].position.x;
        double dy = base_real_position_(1) - ref_path_.poses[i].position.y;
        double dz = base_real_position_(2) - ref_path_.poses[i].position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_index = i;
        }
    }
    return nearest_index;
}

void MPCController::history_path()
{
    int nearest_idex = findNearestIndex();
    history_path_.header.frame_id = "world";
    if (near_id != nearest_idex)
    {
        geometry_msgs::Pose pseudo_path_pose;

        pseudo_path_pose.orientation.w = base_real_orientation_.w();
        pseudo_path_pose.orientation.x = base_real_orientation_.x();
        pseudo_path_pose.orientation.y = base_real_orientation_.y();
        pseudo_path_pose.orientation.z = base_real_orientation_.z();

        pseudo_path_pose.position.x = base_real_position_(0);
        pseudo_path_pose.position.y = base_real_position_(1);
        pseudo_path_pose.position.z = base_real_position_(2);

        history_path_.poses.push_back(pseudo_path_pose);
        near_id = nearest_idex;
        historypath_pub.publish(history_path_);
    }
}

/// TODO ////////////
void MPCController::getRefPath()
{
    x_ref_.clear();                             // clear the ref path
    int nearest_index = findNearestIndex() + 3; // get the nearest point
    int path_left = path_size_ - nearest_index;
    Eigen::Matrix<double, STATE, 1> x_ref;
    x_ref_.push_back(cur_mpc_state_set_);

    Eigen::Vector3d ref_position;
    Eigen::Quaterniond q;
    Eigen::Vector3d ref_euler;
    Eigen::Vector3d state_diff;
    Eigen::Vector3d new_ref_position;

    if (path_left > Horizon_)
    {
        ref_position << ref_path_.poses[nearest_index].position.x,
            ref_path_.poses[nearest_index].position.y,
            ref_path_.poses[nearest_index].position.z;

        state_diff = cur_mpc_state_set_.head(3) - ref_position;
        double dist = state_diff.norm();
        if (dist > 0.5)
        {
            ref_position << ref_path_.poses[nearest_index + Horizon_].position.x,
                ref_path_.poses[nearest_index + Horizon_].position.y,
                ref_path_.poses[nearest_index + Horizon_].position.z;
            state_diff = cur_mpc_state_set_.head(3) - ref_position;

            for (int i = 1; i < Horizon_ + 1; i++)
            {
                new_ref_position =
                    cur_mpc_state_set_.head(3) - (state_diff * i) / Horizon_;
                q = differentialDirectionVector(cur_mpc_state_set_.head(3),
                                                new_ref_position);
                ref_euler = quaternion2euler(q);
                x_ref << new_ref_position, ref_euler;
                // std::cout << "add_ref: " << x_ref.transpose() << std::endl;
                x_ref_.push_back(x_ref);
            }
            ROS_INFO_THROTTLE(1, "ControllerLog:   dist > 0.5, add_ref ");
        }
        else
        {
            for (int i = nearest_index + 1; i < nearest_index + 1 + Horizon_; i++)
            {
                ref_position << ref_path_.poses[i].position.x,
                    ref_path_.poses[i].position.y, ref_path_.poses[i].position.z;
                q = differentialDirectionPoint(ref_path_.poses[i].position,
                                               ref_path_.poses[i + 1].position);
                ref_euler = quaternion2euler(q);
                x_ref << ref_position, ref_euler;
                x_ref_.push_back(x_ref);
            }
        }
    }
    else
    {
        // std::cout << "ControllerLog:   path_left <= Horizon_ && path_left > 5 "
        // << std::endl;

        ref_position << ref_path_.poses[path_size_ - 1].position.x,
            ref_path_.poses[path_size_ - 1].position.y,
            ref_path_.poses[path_size_ - 1].position.z;
        state_diff = cur_mpc_state_set_.head(3) - ref_position;

        for (int i = 1; i < Horizon_ + 1; i++)
        {
            q = differentialDirectionVector(cur_mpc_state_set_.head(3), ref_position);
            ref_euler = quaternion2euler(q);
            x_ref << cur_mpc_state_set_.head(3) - (state_diff * i) / Horizon_,
                ref_euler;
            x_ref_.push_back(x_ref);
        }
        ROS_WARN_THROTTLE(1, "ControllerLog:   nearly arrive goal, add_ref ");
    }
    // set the
    x_local_ref_.clear();
    Eigen::Matrix<double, STATE, 1> x_local_ref;
    Eigen::Matrix<double, STATE, 1> zero_vec;
    Eigen::Quaterniond ref_q;

    zero_vec.setZero();
    x_local_ref << zero_vec;
    x_local_ref_.push_back(x_local_ref);

    for (int i = 1; i < Horizon_ + 1; i++)
    {
        ref_position = x_ref_[i].head(3);
        ref_euler = x_ref_[i].tail(3);
        ref_q = euler2quaternion(ref_euler);
        Eigen::Quaterniond ref_local_orientation =
            base_real_orientation_.inverse() * ref_q;
        Eigen::Vector3d ref_local_position =
            base_real_orientation_.toRotationMatrix().inverse() *
            (ref_position - base_real_position_);
        Eigen::Vector3d ref_local_euler = quaternion2euler(ref_local_orientation);

        x_local_ref << ref_local_position, ref_local_euler;
        // std::cout << "ControllerLog:   x_local_ref: " << x_local_ref.transpose()
        // << std::endl;
        x_local_ref_.push_back(x_local_ref);
    }
    Eigen::Vector3d goal_position;
    goal_position << ref_path_.poses[path_size_ - 1].position.x,
        ref_path_.poses[path_size_ - 1].position.y,
        ref_path_.poses[path_size_ - 1].position.z;
    double goal_diff = (cur_mpc_state_set_.head(3) - goal_position).norm();
    if (goal_diff > 0.3)
    {
        path_finish_flag_ = false;
    }
    else
    {
        path_finish_flag_ = true;
    }
}

void MPCController::tf()
{
    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "body";
    transformStamped_.transform.translation.x = base_real_position_(0);
    transformStamped_.transform.translation.y = base_real_position_(1);
    transformStamped_.transform.translation.z = base_real_position_(2);
    transformStamped_.transform.rotation.x = base_real_orientation_.x();
    transformStamped_.transform.rotation.y = base_real_orientation_.y();
    transformStamped_.transform.rotation.z = base_real_orientation_.z();
    transformStamped_.transform.rotation.w = base_real_orientation_.w();

    broadcaster_.sendTransform(transformStamped_);
}

void MPCController::visualizeRefPaths()
{
    nav_msgs::Path gui_path;
    geometry_msgs::Pose pose;

    Eigen::Vector3d position;
    Eigen::Vector3d euler;
    Eigen::Quaterniond q;
    gui_path.poses.resize(Horizon_ + 1);
    gui_path.header.frame_id = "body";
    for (int i = 0; i < Horizon_ + 1; i++)
    {
        position = x_local_ref_[i].head(3);
        euler = x_local_ref_[i].tail(3);
        // ROS_INFO_STREAM("solution euler" << solution_euler.transpose());
        q = Eigen::AngleAxisd(euler[2], ::Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler[1], ::Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler[0], ::Eigen::Vector3d::UnitX());

        pose.position.x = position(0);
        pose.position.y = position(1);
        pose.position.z = position(2);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        gui_path.poses[i].pose = pose;
    }

    pub_ref_.publish(gui_path);
}

void MPCController::visualizePaths(const Eigen::VectorXd &solutions)
{
    nav_msgs::Path gui_path;
    geometry_msgs::Pose solution_pose;
    Eigen::Matrix<double, STATE, 1> solution_vector;
    Eigen::Matrix<double, INPUT, 1> solution_ctrl;
    // Eigen::Vector3d ref_position;

    Eigen::Vector3d solution_euler;
    Eigen::Quaterniond solution_q;
    gui_path.poses.resize(Horizon_ + 1);
    gui_path.header.frame_id = "body";
    for (int i = 0; i < Horizon_ + 1; i++)
    {
        solution_vector = solutions.block(STATE * i, 0, STATE, 1);
        solution_euler = solution_vector.tail(3);
        // ROS_INFO_STREAM("solution euler" << solution_euler.transpose());
        solution_q =
            Eigen::AngleAxisd(solution_euler[2], ::Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(solution_euler[1], ::Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(solution_euler[0], ::Eigen::Vector3d::UnitX());

        // ref_position = solution_vector.head(3);
        // Eigen::Quaterniond orientation = base_real_orientation_.inverse() *
        // solution_q; Eigen::Vector3d position =
        // base_real_orientation_.toRotationMatrix().inverse() * (-ref_position +
        // base_real_position_); solution_pose.position.x = position(0);
        // solution_pose.position.y = position(1);
        // solution_pose.position.z = position(2);
        // solution_pose.orientation.x = orientation.x();
        // solution_pose.orientation.y = orientation.y();
        // solution_pose.orientation.z = orientation.z();
        // solution_pose.orientation.w = orientation.w();

        solution_pose.position.x = solution_vector(0);
        solution_pose.position.y = solution_vector(1);
        solution_pose.position.z = solution_vector(2);
        solution_pose.orientation.x = solution_q.x();
        solution_pose.orientation.y = solution_q.y();
        solution_pose.orientation.z = solution_q.z();
        solution_pose.orientation.w = solution_q.w();

        // ROS_INFO_STREAM("ControllerLog:   the solution_pose: " <<
        // solution_vector.transpose());

        gui_path.poses[i].pose = solution_pose;
    }

    pub_solution_.publish(gui_path);
}

Eigen::Vector3d MPCController::quaternion2euler(
    const Eigen::Quaterniond &the_q)
{
    Eigen::Vector3d output_euler;
    double sinr_cosp = 2 * (the_q.w() * the_q.x() + the_q.y() * the_q.z());
    double cosr_cosp = 1 - 2 * (the_q.x() * the_q.x() + the_q.y() * the_q.y());
    double sinp = 2 * (the_q.w() * the_q.y() - the_q.z() * the_q.x());
    double siny_cosp = 2 * (the_q.w() * the_q.z() + the_q.x() * the_q.y());
    double cosy_cosp = 1 - 2 * (the_q.y() * the_q.y() + the_q.z() * the_q.z());
    output_euler(0) = std::atan2(sinr_cosp, cosr_cosp);
    if (std::abs(sinp) >= 1)
        output_euler(1) =
            std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        output_euler(1) = std::asin(sinp);
    output_euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return output_euler;
}

Eigen::Quaterniond MPCController::differentialDirectionVector(
    const Eigen::Vector3d &start_p, const Eigen::Vector3d &end_p)
{
    Eigen::Quaterniond output_q;
    Eigen::Vector3d euler_angle;

    Eigen::Vector3d diff;
    diff = end_p - start_p;

    euler_angle(0) = 0;
    euler_angle(1) = std::acos(
        std::sqrt((diff(0) * diff(0) + diff(1) * diff(1)) /
                  (diff(0) * diff(0) + diff(1) * diff(1) + diff(2) * diff(2))));
    if (diff(2) > 0)
    {
        euler_angle(1) = -euler_angle(1);
    }
    euler_angle(2) = std::atan2(diff(1), diff(0));

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(euler_angle[2], ::Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], ::Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[0], ::Eigen::Vector3d::UnitX());
    output_q = R;
    return output_q;
}

Eigen::Quaterniond MPCController::differentialDirectionPoint(
    const geometry_msgs::Point &start_p, const geometry_msgs::Point &end_p)
{
    Eigen::Quaterniond output_q;
    double dx, dy, dz;
    dx = end_p.x - start_p.x;
    dy = end_p.y - start_p.y;
    dz = end_p.z - start_p.z;

    Eigen::Vector3d euler_angle;
    euler_angle(0) = 0;
    euler_angle(1) =
        std::acos(std::sqrt((dx * dx + dy * dy) / (dx * dx + dy * dy + dz * dz)));
    if (dz > 0)
    {
        euler_angle(1) = -euler_angle(1);
    }
    euler_angle(2) = std::atan2(dy, dx);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(euler_angle[2], ::Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], ::Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[0], ::Eigen::Vector3d::UnitX());
    output_q = R;
    return output_q;
}

Eigen::Quaterniond MPCController::euler2quaternion(
    const Eigen::Vector3d &euler)
{
    Eigen::Quaterniond output_q =
        Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX());
    return output_q;
}

double MPCController::get_yaw_from_quat_msg(
    const geometry_msgs::Quaternion &quat_msg)
{
    tf2::Quaternion quat_tf;
    double roll, pitch, yaw;
    tf2::fromMsg(quat_msg, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    return yaw;
}

///////////////////////////////////////////////////////////////
////////////////////////// Functions //////////////////////////
///////////////////////////////////////////////////////////////
void MPCController::setDynamicsMatrices(Eigen::Matrix<double, STATE, STATE> &A,
                                        Eigen::Matrix<double, STATE, INPUT> &B,
                                        double vdt)
{
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    B << vdt, 0.0, 0.0, 0.0, 0.0, vdt, 0.0, 0.0, 0.0, 0.0, vdt, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, vdt;
}

void MPCController::setStateConstraints(
    Eigen::Matrix<double, STATE, 1> &new_xMax,
    Eigen::Matrix<double, STATE, 1> &new_xMin,
    const Eigen::Matrix<double, STATE, 1> &xMax,
    const Eigen::Matrix<double, STATE, 1> &xMin,
    const Eigen::Matrix<double, STATE, 1> &ref_state)
{
    new_xMin = xMin - ref_state;
    new_xMax = xMax - ref_state;
}

void MPCController::setWeightMatrices(Eigen::DiagonalMatrix<double, STATE> &Q,
                                      Eigen::DiagonalMatrix<double, INPUT> &R)
{
    Q.diagonal() << 600.0, 600.0, 600.0, 0.01, 0.01, 900.0;
    R.diagonal() << 8.0, 8.0, 8.0, 8.0;
}

void MPCController::castMPCToQPHessian(
    const Eigen::DiagonalMatrix<double, STATE> &Q,
    const Eigen::DiagonalMatrix<double, INPUT> &R,
    Eigen::SparseMatrix<double> &hessianMatrix)
{
    hessianMatrix.resize(STATE * (Horizon_ + 1) + INPUT * Horizon_,
                         STATE * (Horizon_ + 1) + INPUT * Horizon_);

    // populate hessian matrix
    for (int i = 0; i < STATE * (Horizon_ + 1) + INPUT * Horizon_; i++)
    {
        if (i < STATE * (Horizon_ + 1))
        {
            int posQ = i % STATE;
            float value = Q.diagonal()[posQ];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
        else
        {
            int posR = i % INPUT;
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}

void MPCController::castMPCToQPGradient(
    const Eigen::DiagonalMatrix<double, STATE> &Q,
    const Eigen::Matrix<double, STATE, 1> &xRef, const int current_id,
    Eigen::VectorXd &gradient)
{
    Eigen::Matrix<double, STATE, 1> Qx_ref;

    // populate the gradient vector
    for (int i = 0; i < STATE; i++)
    {
        Qx_ref = Q * (-xRef);
        float value = Qx_ref(i, 0);
        gradient(STATE * current_id + i, 0) = value;
    }
}

void MPCController::initQPConstraintMatrix(
    Eigen::SparseMatrix<double> &constraint_matrix)
{
    constraint_matrix.resize(
        STATE * (Horizon_ + 1) + STATE * (Horizon_ + 1) + INPUT * Horizon_,
        STATE * (Horizon_ + 1) + INPUT * Horizon_);
    // populate linear constraint matrix
    for (int i = 0; i < STATE * (Horizon_ + 1); i++)
    {
        constraint_matrix.insert(i, i) = -1;
    }

    for (int i = 0; i < STATE * (Horizon_ + 1) + INPUT * Horizon_; i++)
    {
        constraint_matrix.insert(i + (Horizon_ + 1) * STATE, i) = 1;
    }
}

void MPCController::castMPCToQPConstraintMatrix(
    const Eigen::Matrix<double, STATE, STATE> &A,
    const Eigen::Matrix<double, STATE, INPUT> &B, const int current_id,
    Eigen::SparseMatrix<double> &constraint_matrix)
{
    for (int j = 0; j < STATE; j++)
    {
        for (int k = 0; k < STATE; k++)
        {
            float value = A(j, k);
            if (value != 0)
            {
                constraint_matrix.insert(STATE * (current_id + 1) + j,
                                         STATE * current_id + k) = value;
            }
        }
    }

    for (int j = 0; j < STATE; j++)
    {
        for (int k = 0; k < INPUT; k++)
        {
            float value = B(j, k);
            if (value != 0)
            {
                constraint_matrix.insert(
                    STATE * (current_id + 1) + j,
                    INPUT * current_id + k + STATE * (Horizon_ + 1)) = value;
            }
        }
    }
}

void MPCController::castMPCToQPConstraintVectors(
    const Eigen::Matrix<double, INPUT, 1> &uMax,
    const Eigen::Matrix<double, INPUT, 1> &uMin,
    const Eigen::Matrix<double, STATE, 1> &x0, Eigen::VectorXd &lowerBound,
    Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality =
        Eigen::MatrixXd::Zero(STATE * (Horizon_ + 1) + INPUT * Horizon_, 1);
    Eigen::VectorXd upperInequality =
        Eigen::MatrixXd::Zero(STATE * (Horizon_ + 1) + INPUT * Horizon_, 1);

    for (int i = 0; i < Horizon_; i++)
    {
        lowerInequality.block(INPUT * i + STATE * (Horizon_ + 1), 0, INPUT, 1) =
            uMin;
        upperInequality.block(INPUT * i + STATE * (Horizon_ + 1), 0, INPUT, 1) =
            uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality =
        Eigen::MatrixXd::Zero(STATE * (Horizon_ + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, STATE, 1) = -x0;
    upperEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound =
        Eigen::MatrixXd::Zero(2 * STATE * (Horizon_ + 1) + INPUT * Horizon_, 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound =
        Eigen::MatrixXd::Zero(2 * STATE * (Horizon_ + 1) + INPUT * Horizon_, 1);
    upperBound << upperEquality, upperInequality;
}

void MPCController::updateMPCToQPConstraintVectors(
    const Eigen::Matrix<double, STATE, 1> &new_xMax,
    const Eigen::Matrix<double, STATE, 1> &new_xMin,
    const Eigen::Matrix<double, STATE, 1> &sMax,
    const Eigen::Matrix<double, STATE, 1> &sMin, const int current_id,
    Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(STATE * current_id + STATE * (Horizon_ + 1), 0, STATE, 1) =
        new_xMin;
    upperBound.block(STATE * current_id + STATE * (Horizon_ + 1), 0, STATE, 1) =
        new_xMax;

    lowerBound.block(STATE * current_id, 0, STATE, 1) = sMin;
    upperBound.block(STATE * current_id, 0, STATE, 1) = sMax;
}

void MPCController::initQPSolver()
{
    setWeightMatrices(Q_, R_);
    castMPCToQPHessian(Q_, R_, hessian_);
    gradient_ =
        Eigen::VectorXd::Zero(STATE * (Horizon_ + 1) + INPUT * Horizon_, 1);
}

int MPCController::solveQP()
{
    if (frist_run_flag_)
    {
        // ROS_INFO("first run, initializing the QP solver");
        initQPSolver();
    }

    sMax_.setZero();
    sMin_.setZero();

    // ROS_INFO("custom the graident and constraint matrix.");
    initQPConstraintMatrix(linearMatrix_);
    castMPCToQPConstraintVectors(uMax_, uMin_, cur_mpc_state_set_, lowerBound_,
                                 upperBound_);
    double norm_x, norm_y, front_norm_x, front_norm_y, back_norm_x,
        back_norm_y = 0.0;

    Eigen::Matrix<double, STATE, 1> new_xMax;
    Eigen::Matrix<double, STATE, 1> new_xMin;
    double dxr, dyr, dzr;
    double vdt;

    for (int id = 0; id < Horizon_ + 1; id++)
    {
        castMPCToQPGradient(Q_, x_local_ref_[id], id, gradient_);

        if (id < Horizon_)
        {
            dxr = x_local_ref_[id + 1](0) - x_local_ref_[id](0);
            dyr = x_local_ref_[id + 1](1) - x_local_ref_[id](1);
            dzr = x_local_ref_[id + 1](2) - x_local_ref_[id](2);
            // computeRefSpeed();
            switch (speed_mode_)
            {
            case 1:
                vdt = sqrt(dxr * dxr + dyr * dyr + dzr * dzr) / 1;
                break;
            case 2:
                vdt = sqrt(dxr * dxr + dyr * dyr + dzr * dzr) / low_v_ref_;
                break;
            case 3:
                vdt = sqrt(dxr * dxr + dyr * dyr + dzr * dzr) / high_v_ref_;
                break;
            case 4:
                vdt = sqrt(dxr * dxr + dyr * dyr + dzr * dzr) / ref_v_13_;
                break;
            default:
                ROS_ERROR("wrong speed mode");
            }
            setDynamicsMatrices(A_, B_, vdt);
            castMPCToQPConstraintMatrix(A_, B_, id, linearMatrix_);
        }

        setStateConstraints(new_xMax, new_xMin, xMax_, xMin_, x_local_ref_[id]);
        updateMPCToQPConstraintVectors(new_xMax, new_xMin, sMax_, sMin_, id,
                                       lowerBound_, upperBound_);
    }

    // ROS_INFO("solver settings");
    if (frist_run_flag_)
    {
        // ROS_INFO("first run, initializing the QP solver");
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);

        // ROS_INFO("solver information in");
        // set the initial data of the QP solver
        solver_.data()->setNumberOfVariables(STATE * (Horizon_ + 1) +
                                             INPUT * Horizon_);
        solver_.data()->setNumberOfConstraints(2 * STATE * (Horizon_ + 1) +
                                               INPUT * Horizon_);
        if (!solver_.data()->setHessianMatrix(hessian_))
            return 1;
        if (!solver_.data()->setGradient(gradient_))
            return 1;
        if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix_))
            return 1;
        if (!solver_.data()->setLowerBound(lowerBound_))
            return 1;
        if (!solver_.data()->setUpperBound(upperBound_))
            return 1;

        // instantiate the solver
        if (!solver_.initSolver())
        {
            ROS_INFO("ControllerLog:   solver initiation failed");
            return 1;
        }
        // ROS_INFO("solver initiated");
    }
    else
    {
        // solver_.updateHessianMatrix(hessian_);
        solver_.updateLinearConstraintsMatrix(linearMatrix_);
        solver_.updateBounds(lowerBound_, upperBound_);
        solver_.updateGradient(gradient_);
    }

    Eigen::Vector4d ctr;
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    // ROS_INFO("solveing ..........");
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return 1;

    // get the controller input
    QPSolution = solver_.getSolution();
    visualizePaths(QPSolution);
    ctr = QPSolution.block(STATE * (Horizon_ + 1), 0, INPUT, 1);

    mpc_vel_cmd_(0) = ctr(0);
    mpc_vel_cmd_(1) = ctr(1);
    mpc_vel_cmd_(2) = ctr(2);
    mpc_vel_cmd_(3) = ctr(3);

    // std::cout << "ControllerLog: mpc_vel_cmd_  === " << mpc_vel_cmd_(0) << " "
    // << mpc_vel_cmd_(1) << "      " << mpc_vel_cmd_(2) << "      " <<
    // mpc_vel_cmd_(3) << std::endl;

    frist_run_flag_ = false;
    return 0;
}

int MPCController::computeRefSpeed()
{
    // double min_dist = 1000.0;
    // int nearest_index = -1;
    // for (int i = 0; i < circles_.size(); i++)
    // {
    //     double dx = base_real_position_(0) - circles_[i](0);
    //     double dy = base_real_position_(1) - circles_[i](1);
    //     double dz = base_real_position_(2) - circles_[i](2);
    //     double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    //     if (dist < min_dist)
    //     {
    //         min_dist = dist;
    //         nearest_index = i;
    //     }
    // }
    // if (min_dist < 4)
    // {
    //     speed_mode_ = 1;
    // }
    // else if (min_dist >= 3 && min_dist < 9)
    // {
    //     speed_mode_ = 2;
    // }
    // else
    // {
    //     speed_mode_ = 3;
    // }
    // // std::cout << "ControllerLog:   speed_mode_ " << speed_mode_ <<
    // std::endl; return speed_mode_;
}
///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void MPCController::run()
{
    std_msgs::Bool run;
    while (nh_.ok())
    {
        if (!planning_stop_flag_)
        {
            // std::cout << "planning_stop_flag_:" << planning_stop_flag_ << std::endl; 
            if (mpc_ready_flag_)
            {
                std::cout << "mpc_ready_flag_:" << mpc_ready_flag_ << std::endl; 
                // ROS_INFO("ControllerLog:  check the base path.");
                getRefPath();
                ROS_INFO("got the ref path. Ref path size : %d", x_ref_.size());
                // tf();
                visualizeRefPaths();
                history_path();

                if (!path_finish_flag_)
                {
                    // ROS_INFO("solve QP one more time");
                    solveQP();
                    // ROS_INFO("sent control commands to the robot");
                    send_commands_to_robot();
                }
                else
                {
                    ROS_INFO_THROTTLE(1, "controller log: path_finish_flag");
                    send_zero_commands();
                }
                run.data = true;
                run_mpc_pub_.publish(run);
            }
            else
            {
                run.data = false;
                run_mpc_pub_.publish(run);
            }
        }
        else
        {
            // ROS_INFO("sent zero commands to the robot");
            send_zero_commands();
            run.data = false;
            run_mpc_pub_.publish(run);
        }
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////

void MPCController::send_commands_to_robot()
{
    // ROS_INFO_THROTTLE(5, "ControllerLog:  twist commands : %f, %f, %f, %f ",
    //                   mpc_vel_cmd_(0), mpc_vel_cmd_(1), mpc_vel_cmd_(2),
    //                   mpc_vel_cmd_(3));

    // mavros_msgs::PositionTarget pos;
    // pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

    // pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
    //                 mavros_msgs::PositionTarget::IGNORE_PY |
    //                 mavros_msgs::PositionTarget::IGNORE_PZ |
    //                 mavros_msgs::PositionTarget::IGNORE_AFX |
    //                 mavros_msgs::PositionTarget::IGNORE_AFY |
    //                 mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                 mavros_msgs::PositionTarget::IGNORE_YAW;
    // pos.position.x = 0.0f;
    // pos.position.y = 0.0f;
    // pos.position.z = 0.0f;
    // pos.acceleration_or_force.x = 0.0f;
    // pos.acceleration_or_force.y = 0.0f;
    // pos.acceleration_or_force.z = 0.0f;

    // pos.velocity.x = mpc_vel_cmd_(0);
    // pos.velocity.y = mpc_vel_cmd_(1);
    // pos.velocity.z = mpc_vel_cmd_(2);

    // pos.yaw = 0.0f;
    // pos.yaw_rate = mpc_vel_cmd_(3);

    // pub_cmd_vel_.publish(pos);
    
    quadrotor_msgs::PositionCommand cmd;
    ros::Time time_now = ros::Time::now();
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    cmd.position.x = 0.0f;
    cmd.position.y = 0.0f;
    cmd.position.z = 0.0f;

    cmd.velocity.x = mpc_vel_cmd_(0);
    cmd.velocity.y = mpc_vel_cmd_(1);
    cmd.velocity.z = mpc_vel_cmd_(2);

    cmd.acceleration.x = 0.0f;
    cmd.acceleration.y = 0.0f;
    cmd.acceleration.z = 0.0f;

    cmd.jerk.x = 0.0f;
    cmd.jerk.y = 0.0f;
    cmd.jerk.z = 0.0f;

    cmd.yaw = 0.0f;
    cmd.yaw_dot = mpc_vel_cmd_(3);

    cmd.kx[0] = 0.0f;
    cmd.kx[1] = 0.0f;
    cmd.kx[2] = 0.0f;

    cmd.kv[0] = 0.0f;
    cmd.kv[1] = 0.0f;
    cmd.kv[2] = 0.0f;
    
    // std::cout << "pos: " << pos << std::endl;
    // std::cout << "vel: " << vel << std::endl;
    // std::cout << "acc: " << acc << std::endl;
    // std::cout << "yaw: " << cmd.yaw << std::endl;
    // std::cout << "yaw_v: " << cmd.yaw_dot << std::endl;
    
    pub_cmd_vel_.publish(cmd);

}

void MPCController::send_zero_commands()
{
    // mavros_msgs::PositionTarget pos;
    // pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

    // pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
    //                 mavros_msgs::PositionTarget::IGNORE_PY |
    //                 mavros_msgs::PositionTarget::IGNORE_PZ |
    //                 mavros_msgs::PositionTarget::IGNORE_AFX |
    //                 mavros_msgs::PositionTarget::IGNORE_AFY |
    //                 mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                 mavros_msgs::PositionTarget::IGNORE_YAW;
    // pos.position.x = 0.0f;
    // pos.position.y = 0.0f;
    // pos.position.z = 0.0f;
    // pos.acceleration_or_force.x = 0.0f;
    // pos.acceleration_or_force.y = 0.0f;
    // pos.acceleration_or_force.z = 0.0f;

    // pos.velocity.x = 0.0f;
    // pos.velocity.y = 0.0f;
    // pos.velocity.z = 0.0;

    // pos.yaw = 0.0f;
    // pos.yaw_rate = 0.0f;

    // pub_cmd_vel_.publish(pos);
    quadrotor_msgs::PositionCommand cmd;
    ros::Time time_now = ros::Time::now();
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    cmd.position.x = 0.0f;
    cmd.position.y = 0.0f;
    cmd.position.z = 0.0f;

    cmd.velocity.x = 0.0f;
    cmd.velocity.y = 0.0f;
    cmd.velocity.z = 0.0f;

    cmd.acceleration.x = 0.0f;
    cmd.acceleration.y = 0.0f;
    cmd.acceleration.z = 0.0f;

    cmd.jerk.x = 0.0f;
    cmd.jerk.y = 0.0f;
    cmd.jerk.z = 0.0f;

    cmd.yaw = 0.0f;
    cmd.yaw_dot = 0.0f;

    cmd.kx[0] = 0.0f;
    cmd.kx[1] = 0.0f;
    cmd.kx[2] = 0.0f;

    cmd.kv[0] = 0.0f;
    cmd.kv[1] = 0.0f;
    cmd.kv[2] = 0.0f;
    
    // std::cout << "pos: " << pos << std::endl;
    // std::cout << "vel: " << vel << std::endl;
    // std::cout << "acc: " << acc << std::endl;
    // std::cout << "yaw: " << cmd.yaw << std::endl;
    // std::cout << "yaw_v: " << cmd.yaw_dot << std::endl;
    
    pub_cmd_vel_.publish(cmd);

}