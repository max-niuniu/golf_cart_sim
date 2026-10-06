#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace golf_cart_sim {
class Ackermann : public gazebo::ModelPlugin {
  using Clock = std::chrono::steady_clock;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  std::array<gazebo::physics::JointPtr, 6> joints_;
  const std::array<std::string,6> names_{{"front_left_steer_joint", "front_right_steer_joint",
    "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"}};
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  std::mutex mutex_;
  double cmd_v_=0, cmd_a_=0, target_v_=0, target_a_=0, last_=-1, pub_time_=-1, rx_sim_=0;
  bool valid_=false, fresh_=false;
  Clock::time_point received_;
  double L_, tfw_, trw_, r_, vmax_, amax_, accel_, decel_, rate_, torque_, st_, sp_, timeout_;
  static double clamp(double a, double lim) { return std::clamp(a,-lim,lim); }
  static double approach(double x, double goal, double delta) { return x+clamp(goal-x,delta); }
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_=model; base_=model_->GetLink("base_link"); node_=gazebo_ros::Node::Get(sdf);
    auto p=[&](const char *key){ if(!sdf->HasElement(key)) throw std::runtime_error(key); return sdf->Get<double>(key); };
    L_=p("wheelbase"); tfw_=p("front_track"); trw_=p("rear_track"); r_=p("wheel_radius");
    vmax_=p("max_speed"); amax_=p("max_steer"); accel_=p("max_accel"); decel_=p("max_decel");
    rate_=p("steer_rate"); torque_=p("drive_torque"); st_=p("steer_torque");
    sp_=p("steer_position_gain"); timeout_=p("command_timeout");
    if(!base_ || L_<=0 || tfw_<=0 || trw_<=0 || r_<=0 || amax_<=0 || amax_>=1.2 ||
       L_/std::tan(amax_)<=std::max(tfw_,trw_)/2 || vmax_<=0 || accel_<=0 || decel_<=0 ||
       rate_<=0 || torque_<=0 || st_<=0 || timeout_<=0) throw std::runtime_error("Invalid vehicle geometry/limits");
    for(size_t i=0;i<6;++i) {
      joints_[i]=model_->GetJoint(names_[i]);
      if(!joints_[i]) throw std::runtime_error("Missing joint: "+names_[i]);
    }
    // Small contact compliance avoids redundant hard friction constraints on wide cylinders.
    for(const auto &name : {"front_left_wheel","front_right_wheel","rear_left_wheel","rear_right_wheel"}) {
      auto surface=boost::dynamic_pointer_cast<gazebo::physics::ODESurfaceParams>(
        model_->GetLink(name)->GetCollisions().at(0)->GetSurface());
      if(!surface) throw std::runtime_error("This controller requires ODE tyre contacts");
      surface->slip1=surface->slip2=p("contact_slip");
    }
    sub_=node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive",1,
      [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        if(!std::isfinite(msg->drive.speed) || !std::isfinite(msg->drive.steering_angle)) {
          cmd_v_=cmd_a_=0; valid_=false; return;
        }
        cmd_v_=clamp(msg->drive.speed,vmax_); cmd_a_=clamp(msg->drive.steering_angle,amax_);
        received_=Clock::now(); valid_=true; fresh_=true;
      });
    odom_=node_->create_publisher<nav_msgs::msg::Odometry>("odom",10);
    states_=node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    tf_=node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf",10);
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Ackermann::Update,this));
    RCLCPP_INFO(node_->get_logger(),"Physical rear drive / front Ackermann; L=%.3f track=%.3f r=%.3f",L_,tfw_,r_);
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_v_=cmd_a_=target_v_=target_a_=0; valid_=fresh_=false; last_=pub_time_=-1;
  }
  void Update() {
    const auto now=model_->GetWorld()->SimTime(); const double t=now.Double();
    if(last_<0) {last_=t; return;}
    double dt=t-last_; last_=t;
    if(dt<=0) return;
    double v=0,a=0;
    { std::lock_guard<std::mutex> lock(mutex_);
      if(fresh_) {rx_sim_=t; fresh_=false;}
      if(valid_ && t-rx_sim_<timeout_ && std::chrono::duration<double>(Clock::now()-received_).count()<timeout_) {
        v=cmd_v_; a=cmd_a_;
      }
    }
    // Brake to zero before reversing. Limits are acceleration and virtual steering slew.
    if(v*target_v_<0) v=0;
    target_v_=approach(target_v_,v,(std::abs(v)>std::abs(target_v_)?accel_:decel_)*dt);
    target_a_=approach(target_a_,a,rate_*dt);
    const double curvature=std::tan(target_a_)/L_;
    const double left=std::atan2(L_*curvature,1-tfw_*curvature/2);
    const double right=std::atan2(L_*curvature,1+tfw_*curvature/2);
    for(int i=0;i<2;++i) {
      double e=(i==0?left:right)-joints_[i]->Position(0);
      joints_[i]->SetParam("fmax",0,st_);
      joints_[i]->SetParam("vel",0,clamp(sp_*e,1.5));
    }
    // Rear differential wheel speeds. Front wheels roll passively under ground contact.
    for(int i=0;i<2;++i) {
      const double speed=target_v_*(1+(i==0?-1:1)*trw_*curvature/2)/r_;
      joints_[4+i]->SetParam("fmax",0,torque_);
      joints_[4+i]->SetParam("vel",0,speed);
    }
    if(t-pub_time_<0.02) return;
    pub_time_=t;
    const auto stamp=gazebo_ros::Convert<builtin_interfaces::msg::Time>(now);
    sensor_msgs::msg::JointState js; js.header.stamp=stamp;
    for(size_t i=0;i<6;++i) {
      js.name.push_back(names_[i]); js.position.push_back(joints_[i]->Position(0));
      js.velocity.push_back(joints_[i]->GetVelocity(0)); js.effort.push_back(joints_[i]->GetForce(0));
    }
    states_->publish(js);
    // Ground-truth world pose of rear-axle base_link, NOT encoder/localization output.
    auto pose=base_->WorldPose(); auto lin=pose.Rot().RotateVectorReverse(base_->WorldLinearVel());
    auto ang=pose.Rot().RotateVectorReverse(base_->WorldAngularVel());
    nav_msgs::msg::Odometry od; od.header.stamp=stamp; od.header.frame_id="odom"; od.child_frame_id="base_link";
    od.pose.pose.position.x=pose.Pos().X(); od.pose.pose.position.y=pose.Pos().Y(); od.pose.pose.position.z=pose.Pos().Z();
    od.pose.pose.orientation.x=pose.Rot().X(); od.pose.pose.orientation.y=pose.Rot().Y();
    od.pose.pose.orientation.z=pose.Rot().Z(); od.pose.pose.orientation.w=pose.Rot().W();
    od.twist.twist.linear.x=lin.X(); od.twist.twist.linear.y=lin.Y(); od.twist.twist.linear.z=lin.Z();
    od.twist.twist.angular.x=ang.X(); od.twist.twist.angular.y=ang.Y(); od.twist.twist.angular.z=ang.Z();
    odom_->publish(od);
    geometry_msgs::msg::TransformStamped tr; tr.header=od.header; tr.child_frame_id=od.child_frame_id;
    tr.transform.translation.x=pose.Pos().X(); tr.transform.translation.y=pose.Pos().Y(); tr.transform.translation.z=pose.Pos().Z();
    tr.transform.rotation=od.pose.pose.orientation;
    tf2_msgs::msg::TFMessage tf; tf.transforms.push_back(tr); tf_->publish(tf);
  }
};
GZ_REGISTER_MODEL_PLUGIN(Ackermann)
}
