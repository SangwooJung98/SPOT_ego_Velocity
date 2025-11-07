#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <rmw/qos_profiles.h>

#include <fstream>
#include <iomanip>
#include <mutex>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace message_filters;
using namespace Eigen;

class SpotRollNode : public rclcpp::Node
{
public:
  SpotRollNode()
  : Node("spot_msgs_node"),
    foot_sub_(this, "/status/feet",  rmw_qos_profile_sensor_data),  
    joint_sub_(this, "/joint_states",     rmw_qos_profile_sensor_data)
  {
    imu_ang_vel_.setZero();
    imu_integrate_.setIdentity();
    R_wb_prev_.setIdentity();
    prev_contact << 1, 1, 1, 1;
    curr_contact << 1, 1, 1, 1;


    pub_leg_vel_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/leg_velocity", rclcpp::QoS(800));

    sub_spot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::QoS(10000),
      std::bind(&SpotRollNode::spotOdomCb, this, _1));

    using MySyncPolicy = sync_policies::ApproximateTime<
        spot_msgs::msg::FootStateArray,
        sensor_msgs::msg::JointState>;

    sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(
              MySyncPolicy(10), foot_sub_, joint_sub_);

    sync_->registerCallback(
      std::bind(&SpotRollNode::legCb, this, _1, _2));
  }

private:
  void spotOdomCb(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    // std::ofstream writeFile("odom_spot.txt", std::ios::app);
    // if (writeFile.is_open()) {
    //   writeFile << std::fixed << std::setprecision(6)
    //             << odom_msg->header.stamp.sec +
    //                odom_msg->header.stamp.nanosec * 1e-9 << ' '
    //             << odom_msg->pose.pose.position.x << ' '
    //             << odom_msg->pose.pose.position.y << ' '
    //             << odom_msg->pose.pose.position.z << ' '
    //             << odom_msg->pose.pose.orientation.x << ' '
    //             << odom_msg->pose.pose.orientation.y << ' '
    //             << odom_msg->pose.pose.orientation.z << ' '
    //             << odom_msg->pose.pose.orientation.w << '\n';
    // }
  }

  void legCb(
    const spot_msgs::msg::FootStateArray::ConstSharedPtr& foot_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
  {
    static double prev_time = 0.0;
    static double prev_contact_time = 0.0;
    double curr_time = joint_msg->header.stamp.sec +
                       joint_msg->header.stamp.nanosec * 1e-9;
    double dt = curr_time - prev_time;
    if (dt > 100.0 || dt <= 0.0) dt = 0.005;
    if (prev_time == curr_time) return;
    prev_time = curr_time;

    std::lock_guard<std::mutex> lock(mtx);

    Eigen::Matrix3d fl_A12 = exp_skew((Eigen::Vector3d(joint_msg->position[0], 0, 0)));
    Eigen::Matrix3d fl_A23 = exp_skew((Eigen::Vector3d(0, joint_msg->position[1], 0)));
    Eigen::Matrix3d fl_A34 = exp_skew((Eigen::Vector3d(0, joint_msg->position[2], 0)));


    Eigen::Vector3d fl_t1 = Eigen::Vector3d(0, 0.055, 0);
    Eigen::Vector3d fl_t2 = Eigen::Vector3d(0, 0.110945, 0);
    Eigen::Vector3d fl_t3 = Eigen::Vector3d(0.025, 0, -0.3205);
    Eigen::Vector3d fl_t4 = Eigen::Vector3d(0, 0, -0.34);

    // front-right leg
    Eigen::Matrix3d fr_A12 = exp_skew((Eigen::Vector3d(joint_msg->position[3], 0, 0)));
    Eigen::Matrix3d fr_A23 = exp_skew((Eigen::Vector3d(0, joint_msg->position[4], 0)));
    Eigen::Matrix3d fr_A34 = exp_skew((Eigen::Vector3d(0, joint_msg->position[5], 0)));


    Eigen::Vector3d fr_t1 = Eigen::Vector3d(0, -0.055, 0);
    Eigen::Vector3d fr_t2 = Eigen::Vector3d(0, -0.110945, 0);
    Eigen::Vector3d fr_t3 = Eigen::Vector3d(0.025, 0, -0.3205);
    Eigen::Vector3d fr_t4 = Eigen::Vector3d(0, 0, -0.34);

    // rear-left leg
    Eigen::Matrix3d rl_A12 = exp_skew((Eigen::Vector3d(joint_msg->position[6], 0, 0)));
    Eigen::Matrix3d rl_A23 = exp_skew((Eigen::Vector3d(0, joint_msg->position[7], 0)));
    Eigen::Matrix3d rl_A34 = exp_skew((Eigen::Vector3d(0, joint_msg->position[8], 0)));

    Eigen::Vector3d rl_t1 = Eigen::Vector3d(-0.5957, 0.055, 0);
    Eigen::Vector3d rl_t2 = Eigen::Vector3d(0, 0.110945, 0);
    Eigen::Vector3d rl_t3 = Eigen::Vector3d(0.025, 0, -0.3205);
    Eigen::Vector3d rl_t4 = Eigen::Vector3d(0, 0, -0.34);

    // rear-right leg
    Eigen::Matrix3d rr_A12 = exp_skew((Eigen::Vector3d(joint_msg->position[9], 0, 0)));
    Eigen::Matrix3d rr_A23 = exp_skew((Eigen::Vector3d(0, joint_msg->position[10], 0)));
    Eigen::Matrix3d rr_A34 = exp_skew((Eigen::Vector3d(0, joint_msg->position[11], 0)));

    Eigen::Vector3d rr_t1 = Eigen::Vector3d(-0.5957, -0.055, 0);
    Eigen::Vector3d rr_t2 = Eigen::Vector3d(0, -0.110945, 0);
    Eigen::Vector3d rr_t3 = Eigen::Vector3d(0.025, 0, -0.3205);
    Eigen::Vector3d rr_t4 = Eigen::Vector3d(0, 0, -0.34);

    // result of forward-kinematics (for each legs)

    Eigen::Vector3d fl_f_t;

    Eigen::Vector3d fr_f_t;

    Eigen::Vector3d rl_f_t;

    Eigen::Vector3d rr_f_t;


    // front-left leg
    fl_f_t = fl_A12 * fl_A23 * fl_A34 * fl_t4 + fl_A12 * fl_A23 * fl_t3 + fl_A12 * fl_t2 + fl_t1;

    
    // front-right leg
    fr_f_t = fr_A12 * fr_A23 * fr_A34 * fr_t4 + fr_A12 * fr_A23 * fr_t3 + fr_A12 * fr_t2 + fr_t1;


    // rear-left leg
    rl_f_t = rl_A12 * rl_A23 * rl_A34 * rl_t4 + rl_A12 * rl_A23 * rl_t3 + rl_A12 * rl_t2 + rl_t1;
    

    // rear-right leg
    rr_f_t = rr_A12 * rr_A23 * rr_A34 * rr_t4 + rr_A12 * rr_A23 * rr_t3 + rr_A12 * rr_t2 + rr_t1;
    

    if(!leg_init){
        leg_init = true;
        fl_f_t_prev = fl_f_t;
        fr_f_t_prev = fr_f_t;
        rl_f_t_prev = rl_f_t;
        rr_f_t_prev = rr_f_t;

        mtx.unlock();
        return;
    }

    Vector3d fl_lin_vel = -(fl_f_t - fl_f_t_prev) / dt;
    Vector3d fr_lin_vel = -(fr_f_t - fr_f_t_prev) / dt;
    Vector3d rl_lin_vel = -(rl_f_t - rl_f_t_prev) / dt;
    Vector3d rr_lin_vel = -(rr_f_t - rr_f_t_prev) / dt;

    
    int     contact_leg_num = 0;
    Eigen::Vector3d contact_lin_vel;
    contact_lin_vel << 0, 0, 0;

    if((uint)foot_msg->states[0].contact == 1){
        contact_leg_num++;
        contact_lin_vel += fl_lin_vel;
        curr_contact(0) = 1;
    }
    else
        curr_contact(0) = 0;

    // front-right leg
    if((uint)foot_msg->states[1].contact == 1){
        contact_leg_num++;
        contact_lin_vel += fr_lin_vel;
        curr_contact(1) = 1;
    }
    else
        curr_contact(1) = 0;

    // rear-left leg
    if((uint)foot_msg->states[2].contact == 1){
        contact_leg_num++;
        contact_lin_vel += rl_lin_vel;
        curr_contact(2) = 1;
    }
    else
        curr_contact(2) = 0;

    // rear-right leg
    if((uint)foot_msg->states[3].contact == 1){
        contact_leg_num++;
        contact_lin_vel += rr_lin_vel;
        curr_contact(3) = 1;
    }
    else
        curr_contact(3) = 0;

    // detect contact change
    if ((prev_contact - curr_contact).norm() != 0) {
      prev_contact_time = curr_time;
      // RCLCPP_INFO(this->get_logger(), "contact changed");
    }

    if(contact_leg_num != 0){
        contact_lin_vel /= contact_leg_num;
    }

    // std::cout<<"Contact Vel : "<<contact_lin_vel<<std::endl;

    Eigen::Matrix3d cov_xyz = Eigen::Matrix3d::Identity()*1e-4;

    double contact_time = curr_time - prev_contact_time;

    double x_variable = 10.0;

    if((uint)(foot_msg->states[0].contact == 1)){
        double x = fl_f_t(0);
        double y = fl_f_t(1);
        double z = fl_f_t(2);
        double xyz = sqrt(x*x + y*y + z*z);
        Eigen::Vector3d J(x_variable * x / xyz, y / xyz, z / xyz);
        double var_vr = (contact_lin_vel - fl_lin_vel).norm();
        cov_xyz += J * var_vr * J.transpose();
    }

    if((uint)(foot_msg->states[1].contact == 1)){
        double x = fr_f_t(0);
        double y = fr_f_t(1);
        double z = fr_f_t(2);
        double xyz = sqrt(x*x + y*y + z*z);
        Eigen::Vector3d J(x_variable * x / xyz, y / xyz, z / xyz);
        double var_vr = (contact_lin_vel - fr_lin_vel).norm();
        cov_xyz += J * var_vr * J.transpose();
    }

    if((uint)(foot_msg->states[2].contact == 1)){
        double x = rl_f_t(0);
        double y = rl_f_t(1);
        double z = rl_f_t(2);
        double xyz = sqrt(x*x + y*y + z*z);
        Eigen::Vector3d J(x_variable * x / xyz, y / xyz, z / xyz);
        double var_vr = (contact_lin_vel - rl_lin_vel).norm();
        cov_xyz += J * var_vr * J.transpose();
    }

    if((uint)(foot_msg->states[3].contact == 1)){
        double x = rr_f_t(0);
        double y = rr_f_t(1);
        double z = rr_f_t(2);
        double xyz = sqrt(x*x + y*y + z*z);
        Eigen::Vector3d J(x_variable * x / xyz, y / xyz, z / xyz);
        double var_vr = (contact_lin_vel - rr_lin_vel).norm();
        cov_xyz += J * var_vr * J.transpose();
    }


    if (contact_time > 0.0) {
      cov_xyz = (cov_xyz / contact_time) * 10.0;
      geometry_msgs::msg::TwistWithCovarianceStamped msg;
      msg.header.stamp = joint_msg->header.stamp;
      msg.twist.twist.linear.x = contact_lin_vel(0);
      msg.twist.twist.linear.y = contact_lin_vel(1);
      msg.twist.twist.linear.z = contact_lin_vel(2);
      msg.twist.covariance[0]  = cov_xyz(0,0);
      msg.twist.covariance[1]  = cov_xyz(0,1);
      msg.twist.covariance[2]  = cov_xyz(0,2);
      msg.twist.covariance[6]  = cov_xyz(1,0);
      msg.twist.covariance[7]  = cov_xyz(1,1);
      msg.twist.covariance[8]  = cov_xyz(1,2);
      msg.twist.covariance[12] = cov_xyz(2,0);
      msg.twist.covariance[13] = cov_xyz(2,1);
      msg.twist.covariance[14] = cov_xyz(2,2);

      if(contact_leg_num!=0){
        pub_leg_vel_->publish(msg);
        // std::ofstream writeFile("spot_vel.txt", std::ios::app);
        // if (writeFile.is_open()) {
        //   writeFile << std::fixed << std::setprecision(6)
        //             << msg.header.stamp.sec +
        //               msg.header.stamp.nanosec * 1e-9 << ' '
        //             << msg.twist.twist.linear.x << ' '
        //             << msg.twist.twist.linear.y << ' '
        //             << msg.twist.twist.linear.z << '\n';
        // }
        // std::cout<<"Publish!!!!!"<<std::endl;
      }
      
    }

    fl_f_t_prev       = fl_f_t;
    fr_f_t_prev       = fr_f_t;
    rl_f_t_prev       = rl_f_t;
    rr_f_t_prev       = rr_f_t;
    prev_contact  = curr_contact;
  }

  Matrix3d skew(const Vector3d &w)
  {
    Matrix3d w_hat;
    w_hat <<   0, -w.z(),  w.y(),
            w.z(),     0, -w.x(),
           -w.y(),  w.x(),     0;
    return w_hat;
  }

  Eigen::Matrix3d exp_skew(Eigen::Vector3d w){
      double theta = w.norm();
      Eigen::Matrix3d w_hat;
      w_hat << 0, -w(2), w(1),
              w(2), 0, -w(0),
              -w(1), w(0), 0;
      Eigen::Matrix3d R;
      R = Eigen::Matrix3d::Identity() + w_hat * sin(theta) / theta + w_hat * w_hat * (1 - cos(theta)) / (theta * theta);
      return R;
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_leg_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_spot_odom_;

  message_filters::Subscriber<spot_msgs::msg::FootStateArray> foot_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState>      joint_sub_;
  std::shared_ptr<
    Synchronizer<sync_policies::ApproximateTime<
      spot_msgs::msg::FootStateArray,
      sensor_msgs::msg::JointState>>
  > sync_;

  std::mutex mtx;

  Vector3d imu_ang_vel_;
  Matrix3d imu_integrate_;
  Matrix3d R_wb_prev_;

  Eigen::Vector4d prev_contact = Eigen::Vector4d::Zero();

  Eigen::Matrix3d fl_f_R_prev;
  Eigen::Matrix3d fr_f_R_prev;
  Eigen::Matrix3d rl_f_R_prev;
  Eigen::Matrix3d rr_f_R_prev;

  Eigen::Vector3d fl_f_t_prev;
  Eigen::Vector3d fr_f_t_prev;
  Eigen::Vector3d rl_f_t_prev;
  Eigen::Vector3d rr_f_t_prev;

 Eigen::Vector4d curr_contact = Eigen::Vector4d::Zero();

  rclcpp::Subscription<spot_msgs::msg::FootStateArray>::SharedPtr    debug_foot_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     debug_joint_sub_;
  
  bool leg_init = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpotRollNode>());
  rclcpp::shutdown();
  return 0;
}
