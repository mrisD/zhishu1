#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.hpp>
#include <stdexcept>
#include <algorithm>
#include <vector>

namespace pallet_placement
{
// Local damped Newton refinement avoids the general IK solver's dead band.
// Preserve the measured branch and verify FK to sub-micrometre precision.
inline void refineTcpPose(moveit::core::RobotState& state,
                          const moveit::core::JointModelGroup* group,
                          const std::string& tcp, const Eigen::Isometry3d& target)
{
  const auto names = group->getVariableNames();
  const auto reference = state;
  constexpr double epsilon = 1e-6;
  for(const auto& name:names) {
    const auto& bounds=state.getRobotModel()->getVariableBounds(name);
    if(bounds.position_bounded_)
      state.setVariablePosition(name,std::clamp(state.getVariablePosition(name),
        bounds.min_position_+1e-8,bounds.max_position_-1e-8));
  }
  for (int iteration = 0; iteration < 40; ++iteration) {
    state.update();
    const Eigen::Isometry3d current = state.getGlobalLinkTransform(tcp);
    const Eigen::AngleAxisd angle(target.rotation() * current.rotation().transpose());
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target.translation() - current.translation();
    error.tail<3>() = angle.axis() * angle.angle();
    if (error.head<3>().norm() < 1e-7 && error.tail<3>().norm() < 1e-6) {
      if (!state.satisfiesBounds(group)) throw std::runtime_error("Microstep violates joint bounds");
      return;
    }
    Eigen::MatrixXd jacobian(6, names.size());
    for (std::size_t j = 0; j < names.size(); ++j) {
      auto perturbed = state;
      perturbed.setVariablePosition(names[j], state.getVariablePosition(names[j]) + epsilon);
      perturbed.update();
      const Eigen::Isometry3d pose = perturbed.getGlobalLinkTransform(tcp);
      jacobian.block<3, 1>(0, j) = (pose.translation() - current.translation()) / epsilon;
      const Eigen::AngleAxisd rotation(pose.rotation() * current.rotation().transpose());
      jacobian.block<3, 1>(3, j) = rotation.axis() * rotation.angle() / epsilon;
    }
    // Active-set bounded least squares: a saturated joint stays inside its
    // limits while redundant joints supply the remaining Cartesian motion.
    // Clipping an unconstrained IK result after convergence would invalidate
    // the requested pose, so enforce the limits during every Newton step.
    Eigen::VectorXd delta=Eigen::VectorXd::Zero(names.size());
    std::vector<bool> free(names.size(),true);
    std::vector<double> low(names.size()),high(names.size());
    for(std::size_t j=0;j<names.size();++j) {
      const auto& bounds=state.getRobotModel()->getVariableBounds(names[j]);
      low[j]=reference.getVariablePosition(names[j])-.10;
      high[j]=reference.getVariablePosition(names[j])+.10;
      if(bounds.position_bounded_) {
        low[j]=std::max(low[j],bounds.min_position_+1e-8);
        high[j]=std::min(high[j],bounds.max_position_-1e-8);
      }
      if(low[j]>high[j]) throw std::runtime_error("Microstep reference is outside the bounded joint branch");
    }
    for(std::size_t active=0;active<=names.size();++active) {
      auto remaining=error;
      Eigen::MatrixXd available=jacobian;
      for(std::size_t j=0;j<names.size();++j) if(!free[j]) {
        remaining-=jacobian.col(j)*delta[j]; available.col(j).setZero();
      }
      Eigen::Matrix<double,6,6> normal=available*available.transpose();
      normal.diagonal().array()+=1e-10;
      Eigen::VectorXd trial=available.transpose()*normal.ldlt().solve(remaining);
      for(std::size_t j=0;j<names.size();++j) if(!free[j]) trial[j]=delta[j];
      if(!trial.allFinite()) throw std::runtime_error("Non-finite bounded microstep inverse kinematics");
      int blocked=-1; double largest=0.0, clipped=0.0;
      for(std::size_t j=0;j<names.size();++j) if(free[j]) {
        const double position=state.getVariablePosition(names[j])+trial[j];
        const double bounded=std::clamp(position,low[j],high[j]);
        if(std::abs(position-bounded)>largest+1e-12) {
          largest=std::abs(position-bounded); blocked=static_cast<int>(j);
          clipped=bounded-state.getVariablePosition(names[j]);
        }
      }
      if(blocked<0) { delta=trial; break; }
      free[blocked]=false; delta[blocked]=clipped;
    }
    const double maximum = delta.cwiseAbs().maxCoeff();
    if (maximum > 0.02) delta *= 0.02 / maximum;
    for (std::size_t j = 0; j < names.size(); ++j) {
      const double value=std::clamp(state.getVariablePosition(names[j])+delta[j],low[j],high[j]);
      state.setVariablePosition(names[j],value);
    }
  }
  throw std::runtime_error("Microstep FK residual exceeds 0.1 micrometre; no motion sent");
}
}  // namespace pallet_placement
