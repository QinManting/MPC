#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <OsqpEigen/OsqpEigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// #include "controller/polynomial.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"
// mavros
// #include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/polynomial.h>
#define STATE 6
#define INPUT 4

class MPCController
{
protected:
    ////// ROS VARIABLES:
    // A handle to the node in ros
    ros::NodeHandle nh_;
    // Rate of the run loop
    ros::Rate loop_rate_;

    ///// Subscribers:
    // Subscriber for the base states
    ros::Subscriber sub_base_state_;
    ros::Subscriber sub_next_base_path_;
    ros::Subscriber sub_debug_odom_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_path_coefficient_;
    ros::Subscriber sub_global_circles_;

    ////// Publishers:
    // Publisher for the twist of base
    ros::Publisher pub_base_cmd_;
    ros::Publisher pub_solution_;
    ros::Publisher pub_ref_;
    ros::Publisher historypath_pub;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher anglerate_pub_;
    ros::Publisher run_mpc_pub_;

    tf2_ros::TransformBroadcaster broadcaster_;
    geometry_msgs::TransformStamped transformStamped_;

    ////// input path:
    geometry_msgs::PoseArray ref_path_;

    // OUTPUT COMMANDS
    // final base desired velocity
    Eigen::Matrix<double, INPUT, 1> mpc_vel_cmd_;

    ////// STATE VARIABLES:Matrix
    // state: position, orientation
    Eigen::Matrix<double, 3, 1> base_real_position_;
    Eigen::Quaterniond base_real_orientation_;
    Eigen::Matrix<double, 3, 1> last_real_position_;
    Eigen::Matrix<double, STATE, 1> cur_mpc_state_set_;

    // some data structure for storage
    std::vector<Eigen::Matrix<double, STATE, 1>> x_ref_;
    std::vector<Eigen::Matrix<double, STATE, 1>> x_local_ref_;
    std::vector<Eigen::Vector3d> circles_;

    // Some parameters
    int Horizon_;
    double dt_;
    double low_v_ref_;
    double high_v_ref_;
    int path_size_;
    bool sim_;

    // for escape mode
    double cur_time_;
    double last_time_;
    int times_;

    //////////////////////////////////////////
    ///////////// last circle control ////////
    //////////////////////////////////////////
    Eigen::Vector3d goal_pos_13_;
    double ref_t_13_;
    double period_13_;
    double ref_v_13_;

    /////////////////////////////////////////
    /////////// mpc control /////////////////
    /////////////////////////////////////////
    OsqpEigen::Solver solver_;

    // allocate the dynamics matrices
    Eigen::Matrix<double, STATE, STATE> A_;
    Eigen::Matrix<double, STATE, INPUT> B_;

    // allocate the constraints vector
    Eigen::Matrix<double, STATE, 1> xMax_;
    Eigen::Matrix<double, STATE, 1> xMin_;
    Eigen::Matrix<double, INPUT, 1> uMax_;
    Eigen::Matrix<double, INPUT, 1> uMin_;
    Eigen::Matrix<double, STATE, 1> sMax_;
    Eigen::Matrix<double, STATE, 1> sMin_;
    Eigen::Matrix<double, STATE, 1> ep_;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, STATE> Q_;
    Eigen::DiagonalMatrix<double, INPUT> R_;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linearMatrix_;
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;

    // history path
    int near_id = -1;
    geometry_msgs::PoseArray history_path_;
    void history_path();

    // run flag
    bool frist_run_flag_;
    bool mpc_ready_flag_;
    bool escape_flag_;
    int speed_mode_;
    bool have_target_;
    bool planning_stop_flag_;
    bool path_finish_flag_;

    // Callbacks
    void baseStateCB(const nav_msgs::OdometryConstPtr msg);
    void nextBasePathCB(const geometry_msgs::PoseArrayConstPtr msg);
    void debugOdomCB(const nav_msgs::Odometry &odom_msg);
    void perceptionOdomCB(const nav_msgs::Odometry &odom_msg);
    void PathCoefficientCB(const quadrotor_msgs::polynomial msg);
    void stopFlagCB(const std_msgs::Bool::ConstPtr &msg);

    // MPC FUNC
    void escapeMode();

    void setDynamicsMatrices(Eigen::Matrix<double, STATE, STATE> &A,
                             Eigen::Matrix<double, STATE, INPUT> &B, double vdt);

    void setStateConstraints(Eigen::Matrix<double, STATE, 1> &new_xMax,
                             Eigen::Matrix<double, STATE, 1> &new_xMin,
                             const Eigen::Matrix<double, STATE, 1> &xMax,
                             const Eigen::Matrix<double, STATE, 1> &xMin,
                             const Eigen::Matrix<double, STATE, 1> &ref_state);

    void setModelConstraints(Eigen::Matrix<double, STATE, 1> &sMax,
                             Eigen::Matrix<double, STATE, 1> &sMin,
                             const Eigen::Matrix<double, STATE, 1> &current_ref,
                             const Eigen::Matrix<double, STATE, 1> &prev_ref,
                             const double norm_x, const double norm_y,
                             const double front_norm_x, const double front_norm_y,
                             const double back_norm_x, const double back_norm_y,
                             const double yaw);

    void setWeightMatrices(Eigen::DiagonalMatrix<double, STATE> &Q,
                           Eigen::DiagonalMatrix<double, INPUT> &R);

    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, STATE> &Q,
                            const Eigen::DiagonalMatrix<double, INPUT> &R,
                            Eigen::SparseMatrix<double> &hessianMatrix);

    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, STATE> &Q,
                             const Eigen::Matrix<double, STATE, 1> &xRef,
                             const int current_id, Eigen::VectorXd &gradient);

    void initQPConstraintMatrix(Eigen::SparseMatrix<double> &constraint_matrix);

    void castMPCToQPConstraintMatrix(
        const Eigen::Matrix<double, STATE, STATE> &A,
        const Eigen::Matrix<double, STATE, INPUT> &B, const int current_id,
        Eigen::SparseMatrix<double> &constraint_matrix);

    void castMPCToQPConstraintVectors(const Eigen::Matrix<double, INPUT, 1> &uMax,
                                      const Eigen::Matrix<double, INPUT, 1> &uMin,
                                      const Eigen::Matrix<double, STATE, 1> &x0,
                                      Eigen::VectorXd &lowerBound,
                                      Eigen::VectorXd &upperBound);

    void updateMPCToQPConstraintVectors(
        const Eigen::Matrix<double, STATE, 1> &new_xMax,
        const Eigen::Matrix<double, STATE, 1> &new_xMin,
        const Eigen::Matrix<double, STATE, 1> &sMax,
        const Eigen::Matrix<double, STATE, 1> &sMin, const int current_id,
        Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);

    void initQPSolver();

    int solveQP();

    int computeRefSpeed();

    // Util
    int findNearestIndex();

    void getRefPath();

    void send_commands_to_robot();

    void send_zero_commands();

    void tf();

    void visualizePaths(const Eigen::VectorXd &solutions);

    void visualizeRefPaths();

    Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &the_q);

    Eigen::Quaterniond differentialDirectionVector(const Eigen::Vector3d &start_p,
                                                   const Eigen::Vector3d &end_p);

    Eigen::Quaterniond differentialDirectionPoint(
        const geometry_msgs::Point &start_p, const geometry_msgs::Point &end_p);

    Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &euler);

    double get_yaw_from_quat_msg(const geometry_msgs::Quaternion &quat_msg);

public:
    MPCController(ros::NodeHandle &n, double frequency,
                  std::string topic_base_command, std::string topic_base_state,
                  int Horizon, double dt, double low_v_ref, double high_v_ref,
                  bool sim, std::vector<double> u_min, std::vector<double> u_max,
                  std::vector<double> x_min, std::vector<double> x_max);
    void run();
};

#endif // MPCCONTROLLER_H
