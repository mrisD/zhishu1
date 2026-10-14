// Auto TCP pivot sampler for Gazebo. This executable is SIMULATION ONLY.
// Planner knows the simulated probe offset, but solver only sees flange poses.
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.hpp>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using MGI = moveit::planning_interface::MoveGroupInterface;

struct Sample {
  Eigen::Vector3d p;
  Eigen::Matrix3d R;
  Eigen::Quaterniond q;
  double contact_error_m = 0.0;
};
struct Fit {
  Eigen::Vector3d tcp;
  Eigen::Vector3d pivot;
  Eigen::Matrix3d tcp_rotation = Eigen::Matrix3d::Identity();
  double rms_mm = 0.0, max_mm = 0.0, condition = 0.0;
};

static Eigen::Vector3d vector3(const std::vector<double>& v) {
  if (v.size() != 3) throw std::runtime_error("tip_xyz must have 3 values");
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

static Sample fromTf(const geometry_msgs::msg::TransformStamped& tf) {
  Sample s;
  s.p = Eigen::Vector3d(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z);
  s.q = Eigen::Quaterniond(tf.transform.rotation.w,tf.transform.rotation.x,
                           tf.transform.rotation.y,tf.transform.rotation.z).normalized();
  s.R = s.q.toRotationMatrix();
  return s;
}

// A_i [tcp_flange; pivot_base] = -p_i, where A_i = [R_i | -I].
static Fit solvePivot(const std::vector<Sample>& samples, double max_condition) {
  if (samples.size() < 4) throw std::runtime_error("insufficient accepted samples");
  Eigen::MatrixXd A(3 * samples.size(), 6);
  Eigen::VectorXd b(3 * samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    A.block<3,3>(3*i,0) = samples[i].R;
    A.block<3,3>(3*i,3) = -Eigen::Matrix3d::Identity();
    b.segment<3>(3*i) = -samples[i].p;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  if (singular[5] < 1.e-6)
    throw std::runtime_error("DEGENERATE: pose rotation set has deficient rank");
  Fit fit;
  fit.condition = singular[0] / singular[5];
  if (fit.condition > max_condition)
    throw std::runtime_error("DEGENERATE: design matrix condition number too high");
  const Eigen::VectorXd estimate = svd.solve(b);
  fit.tcp = estimate.head<3>();
  fit.pivot = estimate.tail<3>();
  double sum = 0.0;
  for (const auto& s : samples) {
    const double err_mm = 1000 * (s.p+s.R*fit.tcp-fit.pivot).norm();
    sum += err_mm*err_mm;
    fit.max_mm = std::max(fit.max_mm, err_mm);
  }
  fit.rms_mm = std::sqrt(sum / samples.size());
  return fit;
}

// ABB TCP&Z,X procedure: after pivot fitting, the tip is moved along the
// desired positive tool Z and X directions without changing flange attitude.
// The solver sees only flange poses; the simulated tool truth is used by the
// planner to make those physical calibration motions.
static Eigen::Matrix3d solveFrameOrientation(const Fit& fit, const Sample& reference,
                                             const Sample& z_elongator,
                                             const Sample& x_elongator) {
  const Eigen::Vector3d reference_tip = reference.p + reference.R * fit.tcp;
  const Eigen::Vector3d z_vec = z_elongator.p + z_elongator.R * fit.tcp - reference_tip;
  const Eigen::Vector3d x_vec = x_elongator.p + x_elongator.R * fit.tcp - reference_tip;
  if (z_vec.norm() < 0.03 || x_vec.norm() < 0.03)
    throw std::runtime_error("ABB TCP&Z,X: elongator points must be at least 30 mm from the pivot");
  const Eigen::Vector3d z_axis = z_vec.normalized();
  const Eigen::Vector3d x_projected = x_vec - z_axis * z_axis.dot(x_vec);
  if (x_projected.norm() < 0.02)
    throw std::runtime_error("ABB TCP&Z,X: X and Z elongator directions are nearly collinear");
  Eigen::Vector3d x_axis = x_projected.normalized();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  x_axis = y_axis.cross(z_axis).normalized();
  Eigen::Matrix3d R_base_tcp;
  R_base_tcp.col(0) = x_axis;
  R_base_tcp.col(1) = y_axis;
  R_base_tcp.col(2) = z_axis;
  return reference.R.transpose() * R_base_tcp;
}

static std::vector<Eigen::Vector3d> orientations(double angle) {
  // Diversify all axes: not just spinning about the probe's axis.
  return {
    {0,0,0},{angle,0,0},{0,angle,0},{0,0,angle},
    {-angle,0,0},{0,-angle,0},{0,0,-angle},
    {angle,angle,0},{-angle,angle,0},{angle,-angle,0},{-angle,-angle,0},
    {angle,0,angle},{-angle,0,angle},{0,angle,angle},{0,-angle,angle},
    {angle,angle,angle},{-angle,angle,-angle},{angle,-angle,-angle},
    {-angle,-angle,angle},{0.65*angle,0.45*angle,-angle},
    {-0.7*angle,0.8*angle,0.4*angle},{0.8*angle,-0.6*angle,-0.5*angle},
    {1.2*angle,0.3*angle,-0.5*angle},{-1.2*angle,-0.3*angle,0.5*angle},
    {0.3*angle,1.2*angle,0.5*angle},{-0.3*angle,-1.2*angle,-0.5*angle},
    {0.5*angle,0.5*angle,-1.3*angle},{-0.5*angle,-0.5*angle,1.3*angle},
  };
}

static geometry_msgs::msg::PoseStamped toPose(const Eigen::Vector3d& p,
                                               const Eigen::Matrix3d& R,
                                               const std::string& frame) {
  geometry_msgs::msg::PoseStamped m;
  m.header.frame_id=frame;
  m.pose.position.x=p.x();m.pose.position.y=p.y();m.pose.position.z=p.z();
  Eigen::Quaterniond q(R);
  m.pose.orientation.w=q.w();m.pose.orientation.x=q.x();
  m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();
  return m;
}

static std::string v3(const Eigen::Vector3d& v) {
  std::ostringstream s;
  s << std::setprecision(9) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return s.str();
}

static Eigen::Vector3d rpy(const Eigen::Matrix3d& R) {
  const Eigen::Vector3d zyx = R.eulerAngles(2, 1, 0);
  return Eigen::Vector3d(zyx.z(), zyx.y(), zyx.x());
}

static double rotationErrorDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle() * 180.0 / M_PI;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tcp_auto_sampler",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger=node->get_logger();
  auto param_s = [&](const char* key, const std::string& fallback) {
    return node->has_parameter(key) ? node->get_parameter(key).as_string() : fallback;
  };
  auto param_d = [&](const char* key, double fallback) {
    return node->has_parameter(key) ? node->get_parameter(key).as_double() : fallback;
  };
  auto param_i = [&](const char* key, int fallback) {
    return node->has_parameter(key) ? static_cast<int>(node->get_parameter(key).as_int()) : fallback;
  };
  const std::string arm=param_s("arm","right");
  const std::string tool=param_s("tool","probe");
  const std::string method=param_s("method","multi_point");
  const std::string noise=param_s("noise","ideal");
  const std::string frame=param_s("base_frame","base_link");
  const std::string outdir=param_s("output_dir","/tmp/guohua_tcp_calibration");
  const int required=method == "four_point" ? 4 : param_i("samples",12);
  const int validate_count=param_i("validation_samples",3);
  const double angle=param_d("rotation_step_rad",0.32);
  const double sigma_pos=param_d("position_sigma_mm",0.2)/1000.0;
  const double sigma_rot=param_d("orientation_sigma_deg",0.02) * M_PI / 180.0;
  const double max_contact_mm=param_d("max_tip_error_mm",8.0);
  const double max_condition=param_d("max_condition_number",150.0);
  const double max_fit_rms=param_d("max_fit_rms_mm",3.0);
  const double speed=param_d("speed_scale",0.10);
  const bool spawn_fixture=node->has_parameter("spawn_fixture") ? node->get_parameter("spawn_fixture").as_bool() : true;
  const bool sim_clock = node->has_parameter("use_sim_time") && node->get_parameter("use_sim_time").as_bool();
  if (!sim_clock || (arm!="left" && arm!="right") || (tool!="probe" && tool!="long_probe") ||
      (method!="four_point" && method!="multi_point") || (noise!="ideal" && noise!="gaussian") ||
      required<4 || required>24 || angle<0.05 || angle>0.7 || speed<=0 || speed>0.25) {
    RCLCPP_FATAL(logger,"Invalid arguments or use_sim_time!=true; refusing to run outside simulated time");
    rclcpp::shutdown();return 2;
  }
  std::vector<double> xyz;
  Eigen::Vector3d truth_rpy;
  if (tool == "probe") xyz={0.015,0.0,-0.30};
  else if (arm=="left") xyz={0.010,0.005,-0.36};
  else xyz={-0.010,-0.005,-0.36};
  if (tool == "probe") truth_rpy=Eigen::Vector3d(M_PI+0.12,-0.08,0.18);
  else truth_rpy=Eigen::Vector3d(M_PI-0.10,0.14,-0.22);
  const Eigen::Vector3d truth=vector3(xyz); // planner ONLY; solver receives no truth
  const Eigen::Matrix3d truth_R =
    (Eigen::AngleAxisd(truth_rpy.z(),Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(truth_rpy.y(),Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(truth_rpy.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
  const std::string flange=arm=="left" ? "L_hand_link" : "R_hand_link";
  const std::string group_name=arm=="left" ? "left_arm" : "right_arm";

  auto pose_pub=node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/tcp_calibration/fixture_pose",rclcpp::QoS(1).transient_local().reliable());
  auto marker_pub=node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/tcp_calibration/markers",rclcpp::QoS(1).transient_local().reliable());
  auto ready=std::make_shared<std::atomic<bool>>(false);
  auto ready_sub=node->create_subscription<std_msgs::msg::Bool>(
    "/tcp_calibration/fixture_ready",rclcpp::QoS(1).transient_local().reliable(),
    [ready](const std_msgs::msg::Bool::SharedPtr msg){ready->store(msg->data);});
  (void)ready_sub;
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin([&](){exec.spin();});
  int status=1;
  try {
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer,node,false);
    auto tfpose=[&]() -> Sample {
      return fromTf(tf_buffer.lookupTransform(frame,flange,tf2::TimePointZero,tf2::durationFromSec(2.0)));
    };
    MGI move_group(node,group_name);
    move_group.setPoseReferenceFrame(frame);
    move_group.setPlanningTime(6.0);
    move_group.setNumPlanningAttempts(4);
    move_group.setMaxVelocityScalingFactor(speed);
    move_group.setMaxAccelerationScalingFactor(speed);
    // Pose targets specify flange explicitly; don't rely on an SRDF end-effector.
    const std::string seed_pose=param_s("seed_pose",group_name+"_pose1");
    if(seed_pose!="current") {
      if(!move_group.setNamedTarget(seed_pose))
        throw std::runtime_error("Unknown seed named pose: " + seed_pose);
      MGI::Plan seed_plan;
      if(!static_cast<bool>(move_group.plan(seed_plan)))
        throw std::runtime_error("Unable to plan seed pose; set seed_pose:=current or repair IK/collision");
      if(!static_cast<bool>(move_group.execute(seed_plan)))
        throw std::runtime_error("Failed to execute seed pose");
      std::this_thread::sleep_for(700ms);
    }
    RCLCPP_INFO(logger,"Starting auto calibration: arm=%s tool=%s method=%s samples=%d",
      arm.c_str(),tool.c_str(),method.c_str(),required);
    // Wait for /tf and joint states to become available; do not invent flange state.
    Sample seed;
    bool got_seed=false;
    for (int i=0;i<50 && rclcpp::ok();i++) {
      try {seed=tfpose();got_seed=true;break;} catch (const tf2::TransformException&) {std::this_thread::sleep_for(200ms);}
    }
    if (!got_seed) throw std::runtime_error("TF base->flange not available");
    const Eigen::Vector3d fixture=seed.p+seed.R*truth;
    auto fixture_pose=toPose(fixture,Eigen::Matrix3d::Identity(),frame);
    fixture_pose.header.stamp=node->now();
    pose_pub->publish(fixture_pose);
    if (spawn_fixture) {
      RCLCPP_INFO(logger,"Waiting for Gazebo fixed pivot fixture spawn at [%0.3f %0.3f %0.3f]",
                  fixture.x(),fixture.y(),fixture.z());
      const auto end=std::chrono::steady_clock::now()+30s;
      while(rclcpp::ok() && !ready->load() && std::chrono::steady_clock::now()<end) {
        pose_pub->publish(fixture_pose);
        std::this_thread::sleep_for(250ms);
      }
      if(!ready->load()) throw std::runtime_error("Fixture spawn timeout/failure; no robot movement executed");
    }
    std::mt19937 engine(static_cast<unsigned>(param_i("noise_seed",42)));
    std::normal_distribution<double> normal(0.0,1.0);
    const auto noisy = [&](Sample s) {
      if(noise=="gaussian") {
        for(int j=0;j<3;j++)s.p[j]+=sigma_pos*normal(engine);
        const Eigen::Vector3d w(sigma_rot*normal(engine),sigma_rot*normal(engine),sigma_rot*normal(engine));
        if(w.norm()>1e-12) {
          s.R = s.R * Eigen::AngleAxisd(w.norm(),w.normalized()).toRotationMatrix();
          s.q=Eigen::Quaterniond(s.R);
        }
      }
      return s;
    };
    // For each candidate: plan, execute, let robot settle, verify 3 TF readings.
    std::vector<Sample> train,validate;
    auto accept=[&](const Eigen::Matrix3d& desired_r, bool initial, std::vector<Sample>& list) -> bool {
      const Eigen::Vector3d desired_p=fixture-desired_r*truth;
      if(!initial) {
        auto target=toPose(desired_p,desired_r,frame);
        move_group.setStartStateToCurrentState();
        if(!move_group.setPoseTarget(target,flange)) return false;
        MGI::Plan plan;
        const bool planned = static_cast<bool>(move_group.plan(plan));
        move_group.clearPoseTargets();
        if(!planned) return false;
        if(!static_cast<bool>(move_group.execute(plan))) return false;
      }
      std::this_thread::sleep_for(600ms);
      Sample a=tfpose();
      std::this_thread::sleep_for(140ms);
      Sample b=tfpose();
      std::this_thread::sleep_for(140ms);
      Sample c=tfpose();
      double drift=(a.p-b.p).norm()+(b.p-c.p).norm();
      double orientation_drift=Eigen::AngleAxisd(a.R.transpose()*c.R).angle();
      if(drift>0.0015 || orientation_drift>0.008) {
        RCLCPP_WARN(logger,"TF not settled: drift=%.2f mm; skip",drift*1000);return false;
      }
      c.contact_error_m=(c.p+c.R*truth-fixture).norm();
      if(1000*c.contact_error_m > max_contact_mm) {
        RCLCPP_WARN(logger,"Probe is %.2f mm from pivot, above tolerance; skip",1000*c.contact_error_m);
        return false;
      }
      const Sample measured=noisy(c);
      list.push_back(measured);
      RCLCPP_INFO(logger,"Sample %zu: tip deviation=%.3f mm, drift=%.3f mm",
                  list.size(),1000*c.contact_error_m,1000*drift);
      return true;
    };
    auto angles=orientations(angle);
    for(size_t i=0;i<angles.size() && (int)train.size()<required && rclcpp::ok();i++) {
      const auto& e=angles[i];
      Eigen::Matrix3d R=seed.R *
        (Eigen::AngleAxisd(e.x(),Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(e.y(),Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(e.z(),Eigen::Vector3d::UnitZ())).toRotationMatrix();
      try {
        if(!accept(R,i==0,train)) RCLCPP_WARN(logger,"Candidate %zu rejected",i+1);
      } catch(const std::exception& e) {
        RCLCPP_WARN(logger,"Candidate %zu failed: %s",i+1,e.what());
      }
    }
    if((int)train.size()<required) {
      throw std::runtime_error("Insufficient valid automated samples: " + std::to_string(train.size())+
                               " of "+std::to_string(required)+"; check reachable pivot and collision geometry");
    }
    Fit fit=solvePivot(train,max_condition);
    if(fit.rms_mm>max_fit_rms)
      throw std::runtime_error("Fit RMS exceeds threshold; calibration not accepted");
    // Hold-out validation uses independent candidate rotations not in training.
    for(size_t i=0;i<angles.size() && (int)validate.size()<validate_count && rclcpp::ok();i++) {
      const auto& e=angles[i];
      const Eigen::Vector3d scaled(-1.3*e.y()+0.05,1.2*e.x()-0.07,e.z()+0.13);
      Eigen::Matrix3d R=seed.R *
        (Eigen::AngleAxisd(scaled.x(),Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(scaled.y(),Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(scaled.z(),Eigen::Vector3d::UnitZ())).toRotationMatrix();
      try { (void)accept(R,false,validate); }
      catch(const std::exception& e) {RCLCPP_WARN(logger,"Validation candidate failed: %s",e.what());}
    }
    if((int)validate.size()<validate_count)
      throw std::runtime_error("Insufficient independent validation samples");
    double validation_squares=0.0, validation_max=0.0;
    for(const auto& s:validate) {
      const double err=(s.p+s.R*fit.tcp-fit.pivot).norm()*1000;
      validation_squares+=err*err;
      validation_max=std::max(validation_max,err);
    }
    const double validation_rms=std::sqrt(validation_squares/validate.size());
    const double truth_error=(fit.tcp-truth).norm()*1000;

    // ABB TCP&Z,X: keep the selected reference flange attitude, then move the
    // physical tip along +Z and +X extensions. These two flange poses define
    // orientation after the pivot solver has estimated the TCP translation.
    // Use the measured starting pose as the axis reference. The last pivot
    // validation attitude may be close to joint limits and leave no room for
    // a fixed-attitude Z/X extension.
    const double elongator_length=param_d("elongator_length_m",0.05);
    // Axis measurements take place above the fixed needle, so neither the
    // +Z extension nor the holdouts can drive the tool into the reference rod.
    const Eigen::Vector3d clear_tip=fixture+Eigen::Vector3d(0,0,0.15+elongator_length);
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(toPose(clear_tip-seed.R*truth,seed.R,frame),flange);
    MGI::Plan clearance_plan;
    const bool clearance_planned=static_cast<bool>(move_group.plan(clearance_plan));
    move_group.clearPoseTargets();
    if(!clearance_planned || !static_cast<bool>(move_group.execute(clearance_plan)))
      throw std::runtime_error("Failed to reach free-space axis reference above the fixture");
    std::this_thread::sleep_for(900ms);
    const Sample orientation_reference=tfpose();
    const Eigen::Vector3d orientation_origin=orientation_reference.p+orientation_reference.R*fit.tcp;
    RCLCPP_INFO(logger,"Axis calibration in free space: reference raised %.1f mm above pivot",
                (orientation_origin.z()-fixture.z())*1000.0);
    const auto capture_elongator = [&](const Eigen::Vector3d& axis_tcp,
                                        const char* axis_name) -> Sample {
      const Eigen::Vector3d axis_base=orientation_reference.R*truth_R*axis_tcp;
      const Eigen::Vector3d target_flange=orientation_reference.p+elongator_length*axis_base;
      auto target=toPose(target_flange,orientation_reference.R,frame);
      move_group.setStartStateToCurrentState();
      if(!move_group.setPoseTarget(target,flange))
        throw std::runtime_error(std::string("Cannot set ")+axis_name+" elongator pose");
      MGI::Plan plan;
      const bool planned=static_cast<bool>(move_group.plan(plan));
      move_group.clearPoseTargets();
      if(!planned || !static_cast<bool>(move_group.execute(plan)))
        throw std::runtime_error(std::string("Failed to execute ")+axis_name+" elongator pose");
      std::this_thread::sleep_for(600ms);
      const Sample a=tfpose();
      std::this_thread::sleep_for(140ms);
      const Sample b=tfpose();
      std::this_thread::sleep_for(140ms);
      Sample c=tfpose();
      const double drift=(a.p-b.p).norm()+(b.p-c.p).norm();
      const double angular_drift=rotationErrorDeg(a.R,c.R);
      if(drift>0.0015 || angular_drift>0.008*180.0/M_PI)
        throw std::runtime_error(std::string("Unsettled ")+axis_name+" elongator pose");
      c=noisy(c);
      return c;
    };
    Sample z_elongator=capture_elongator(Eigen::Vector3d::UnitZ(),"+Z");
    Sample x_elongator=capture_elongator(Eigen::Vector3d::UnitX(),"+X");
    fit.tcp_rotation=solveFrameOrientation(fit,orientation_reference,z_elongator,x_elongator);

    // ABB-style independent check: approach three hold-out axis targets using
    // the estimated frame. Neither these measurements nor simulated truth feed
    // the orientation solver.
    const Eigen::Matrix3d estimated_R_base_tcp=orientation_reference.R*fit.tcp_rotation;
    const std::vector<Eigen::Vector3d> frame_check_directions={
      Eigen::Vector3d::UnitY(),-Eigen::Vector3d::UnitX(),-Eigen::Vector3d::UnitZ()};
    double orientation_position_squares=0.0, orientation_angle_squares=0.0;
    double orientation_position_max=0.0, orientation_angle_max=0.0;
    std::vector<Sample> frame_validation;
    for(size_t i=0;i<frame_check_directions.size();++i) {
      const Eigen::Matrix3d target_R_flange=estimated_R_base_tcp*fit.tcp_rotation.transpose();
      Eigen::Vector3d target_tip;
      MGI::Plan plan;
      bool planned=false;
      // Retain all three independent directions while adapting the distance
      // to the reachable workspace. No motion is issued for rejected plans.
      for(const double scale : {1.0, 0.75, 0.5, 0.25}) {
        target_tip=orientation_origin+scale*elongator_length*
          estimated_R_base_tcp*frame_check_directions[i];
        const Eigen::Vector3d target_p_flange=target_tip-target_R_flange*fit.tcp;
        auto target=toPose(target_p_flange,target_R_flange,frame);
        move_group.setStartStateToCurrentState();
        if(!move_group.setPoseTarget(target,flange)) continue;
        planned=static_cast<bool>(move_group.plan(plan));
        move_group.clearPoseTargets();
        if(planned) {
          RCLCPP_INFO(logger,"Frame holdout %zu: distance=%.1f mm",i+1,scale*elongator_length*1000.0);
          break;
        }
      }
      if(!planned || !static_cast<bool>(move_group.execute(plan)))
        throw std::runtime_error("Failed independent tool-frame validation motion");
      std::this_thread::sleep_for(600ms);
      const Sample measured=tfpose();
      frame_validation.push_back(measured);
      const double pos_error=(measured.p+measured.R*fit.tcp-target_tip).norm()*1000.0;
      const double rot_error=rotationErrorDeg(measured.R*fit.tcp_rotation,estimated_R_base_tcp);
      orientation_position_squares+=pos_error*pos_error;
      orientation_angle_squares+=rot_error*rot_error;
      orientation_position_max=std::max(orientation_position_max,pos_error);
      orientation_angle_max=std::max(orientation_angle_max,rot_error);
    }
    const double orientation_position_rms=std::sqrt(orientation_position_squares/frame_validation.size());
    const double orientation_holdout_rms_deg=std::sqrt(orientation_angle_squares/frame_validation.size());
    const double truth_orientation_error_deg=rotationErrorDeg(fit.tcp_rotation,truth_R);
    // Leave the scene at the same physical tip-to-tip pose used to start.
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(toPose(seed.p,seed.R,frame),flange);
    MGI::Plan return_plan;
    const bool return_planned=static_cast<bool>(move_group.plan(return_plan));
    move_group.clearPoseTargets();
    if(!return_planned || !static_cast<bool>(move_group.execute(return_plan)))
      throw std::runtime_error("Failed to return to the starting pivot pose");
    std::this_thread::sleep_for(900ms);
    fs::create_directories(outdir);
    const auto stamp=std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string basename=arm+"_"+tool+"_"+method+"_"+std::to_string(stamp);
    const fs::path yaml=fs::path(outdir)/(basename+".yaml");
    const fs::path csv=fs::path(outdir)/(basename+"_samples.csv");
    const fs::path report=fs::path(outdir)/(basename+"_report.md");
    {
      std::ofstream f(yaml);
      if(!f)throw std::runtime_error("Failed to write result YAML");
      f << std::setprecision(10)
        << "version: 1\narm: "<<arm<<"\ntool_id: "<<tool
        << "\nparent_frame: " << flange
        << "\nreference_frame: " << frame
        << "\ntranslation_xyz_m: " << v3(fit.tcp)
        << "\norientation_rpy_rad: " << v3(rpy(fit.tcp_rotation))
        << "\norientation_quaternion_xyzw: ["
        << Eigen::Quaterniond(fit.tcp_rotation).x() << ", "
        << Eigen::Quaterniond(fit.tcp_rotation).y() << ", "
        << Eigen::Quaterniond(fit.tcp_rotation).z() << ", "
        << Eigen::Quaterniond(fit.tcp_rotation).w() << "]"
        << "\norientation_calibrated: true\norientation_method: ABB_TCP_Z_X\nmethod: " << method
        << "\nsample_count: " << train.size()
        << "\nvalidation_count: " << validate.size()
        << "\nnoise: " << noise
        << "\nmetrics:\n  fit_rms_mm: " <<fit.rms_mm
        << "\n  fit_max_mm: " <<fit.max_mm
        << "\n  validation_rms_mm: " << validation_rms
        << "\n  validation_max_mm: " <<validation_max
        << "\n  truth_error_mm: " <<truth_error
        << "\n  truth_orientation_error_deg: " <<truth_orientation_error_deg
        << "\n  orientation_holdout_position_rms_mm: " <<orientation_position_rms
        << "\n  orientation_holdout_position_max_mm: " <<orientation_position_max
        << "\n  orientation_holdout_rms_deg: " <<orientation_holdout_rms_deg
        << "\n  orientation_holdout_max_deg: " <<orientation_angle_max
        << "\n  design_condition: " <<fit.condition
        << "\n  fixture_position_base_m: "<<v3(fixture)<<"\n";
    }
    {
      std::ofstream f(csv);
      if(!f)throw std::runtime_error("Failed to write CSV");
      f << std::setprecision(12) << "role,x,y,z,qx,qy,qz,qw,tip_distance_from_pivot_mm\n";
      auto write=[&](const std::vector<Sample>& v,const char* role) {
        for(const auto& s:v) {
          f << role << ',' <<s.p.x()<<','<<s.p.y()<<','<<s.p.z()<<','
            <<s.q.x()<<','<<s.q.y()<<','<<s.q.z()<<','<<s.q.w()<<','
            <<(s.p+s.R*fit.tcp-fit.pivot).norm()*1000<<'\n';
        }
      };
      write(train,"pivot_fit");write(validate,"pivot_validation");
      write(std::vector<Sample>{z_elongator},"elongator_z");
      write(std::vector<Sample>{x_elongator},"elongator_x");
      write(std::vector<Sample>{orientation_reference},"orientation_reference");
      write(frame_validation,"orientation_validation");
    }
    {
      std::ofstream f(report);
      if(!f)throw std::runtime_error("Failed to write Markdown report");
      f << "# Gazebo TCP auto calibration\n\n"
        << "- arm/tool: " << arm << " / "<<tool<<"\n"
        << "- flange: " <<flange<<"\n"
        << "- method: "<<method<<"\n"
        << "- train samples: "<<train.size()<<"; independent validation: "<<validate.size()<<"\n"
        << "- noise: "<<noise<<"\n"
        << "- TCP XYZ (flange, m): "<<v3(fit.tcp)<<"\n"
        << "- TCP RPY (flange, rad): "<<v3(rpy(fit.tcp_rotation))<<"\n"
        << "- orientation method: ABB TCP&Z,X\n"
        << "- ground truth (sim only, m): "<<v3(truth)<<"\n"
        << "- fit RMS/MAX (mm): "<<fit.rms_mm<<" / "<<fit.max_mm<<"\n"
        << "- independent validation RMS/MAX (mm): "<<validation_rms<<" / "<<validation_max<<"\n"
        << "- TCP-to-truth error (mm): "<<truth_error<<"\n"
        << "- orientation-to-truth error (deg): "<<truth_orientation_error_deg<<"\n"
        << "- independent frame holdout position RMS (mm): "<<orientation_position_rms<<"\n"
        << "- independent frame holdout orientation RMS (deg): "<<orientation_holdout_rms_deg<<"\n"
        << "- matrix condition: "<<fit.condition<<"\n\n"
        << "Results are not valid for the physical robot without a separate physical calibration.\n";
    }
    visualization_msgs::msg::MarkerArray markers;
    auto sphere=[&](int id,const Eigen::Vector3d& p,double r,double g,double b,double radius) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id=frame;m.header.stamp=node->now();m.ns="tcp_calibration";m.id=id;
      m.type=visualization_msgs::msg::Marker::SPHERE;
      m.action=visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w=1.0;
      m.pose.position.x=p.x();m.pose.position.y=p.y();m.pose.position.z=p.z();
      m.scale.x=radius;m.scale.y=radius;m.scale.z=radius;
      m.color.a=1.0;m.color.r=r;m.color.g=g;m.color.b=b;
      markers.markers.push_back(m);
    };
    sphere(0,fixture,1,0.7,0,0.018);
    Sample current=tfpose();
    sphere(1,current.p+current.R*fit.tcp,0.2,1,0.2,0.018);
    sphere(2,current.p+current.R*truth,0.2,0.4,1,0.009);
    const auto axis_marker=[&](int id,const Eigen::Vector3d& axis,double red,double green,double blue) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id=frame;m.header.stamp=node->now();m.ns="tcp_calibration_frame";m.id=id;
      m.type=visualization_msgs::msg::Marker::ARROW;m.action=visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start,end;
      const Eigen::Vector3d tcp_now=current.p+current.R*fit.tcp;
      start.x=tcp_now.x();start.y=tcp_now.y();start.z=tcp_now.z();
      const Eigen::Vector3d delta=0.12*current.R*fit.tcp_rotation*axis;
      end.x=start.x+delta.x();end.y=start.y+delta.y();end.z=start.z+delta.z();
      m.points={start,end};m.scale.x=0.004;m.scale.y=0.009;m.scale.z=0.014;
      m.color.a=1.0;m.color.r=red;m.color.g=green;m.color.b=blue;
      markers.markers.push_back(m);
    };
    axis_marker(3,Eigen::Vector3d::UnitX(),1.0,0.1,0.1);
    axis_marker(4,Eigen::Vector3d::UnitY(),0.1,1.0,0.1);
    axis_marker(5,Eigen::Vector3d::UnitZ(),0.1,0.3,1.0);
    marker_pub->publish(markers);
    RCLCPP_INFO(logger,"SUCCESS: TCP=%s RPY=%s  RMS=%.3f mm  holdout=%.3f mm  truth errors=%.3f mm/%.3f deg; YAML: %s",
                v3(fit.tcp).c_str(),v3(rpy(fit.tcp_rotation)).c_str(),fit.rms_mm,
                validation_rms,truth_error,truth_orientation_error_deg,yaml.c_str());
    std::this_thread::sleep_for(2s);
    status=0;
  } catch(const std::exception& e) {
    RCLCPP_ERROR(logger,"AUTO TCP CALIBRATION FAILED: %s",e.what());
  }
  exec.cancel();
  if(spin.joinable())spin.join();
  rclcpp::shutdown();
  return status;
}
