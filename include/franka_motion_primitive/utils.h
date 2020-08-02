#include <map>
#include <boost/variant.hpp>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <franka_controllers/Gain.h>

#ifndef FRANKA_MOTION_PRIMITIVE_UTILS_H
#define FRANKA_MOTION_PRIMITIVE_UTILS_H

namespace franka_motion_primitive{
  // useful shortcut
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Quaterniond = Eigen::Quaterniond;
  using AngleAxisd = Eigen::AngleAxisd;

  // a frame can be represented by the position of the origin
  //   and orientation of the axes
  struct Pose {
    Vector3d p;     // pos
    Quaterniond q;  // quat
  };

  // status of a primtiive is whether done or executing, and if done,
  // whether because of timeout or success
  enum class Status {
    EXECUTING,
    TIMEOUT,
    SUCCESS
  };

  // Groups of prmitiives
  // NOTE: decaprated, use PrimitiveType.msg instead
  enum class PrimitiveGroup {
    MoveToPose,
    ConstantVelocity,
    AdmittanceMotion
  };

  // High-level command (can shared with franka_controllers packaage)
  // NOTE translational velocity is expressed w.r.t the task frame,
  //      rotational velocity is completely defined by the orientation of
  //        the task frame (not by the origin of the task frame)
  struct ControlSignal {
    Vector6d f;     // force
    Pose pose;
    Vector6d v;     // velocity
    Matrix6d S;    // selection matrix
    // convention: 1 for position control, 0 for force control
  };

  // State vectoc
  struct State {
    Vector6d f;     // external force
    Vector6d f_ee;  // external force, relative to base frame
    Pose pose;
    Vector6d v;     // spatial velocity
  };

  // use to store params of different types,
  // similar to dictionary in python
  using TypeVariant = boost::variant<
        uint8_t,
        bool,
        double,
        // std::vector<double>,
        Vector3d,
        Quaterniond,
        Vector6d,
        franka_controllers::Gain>;

  using ParamMap = std::map<std::string, TypeVariant>;

  // ---- helper functions

  // transform coordinate of a point p1 expressed in Frame 1 to Frame 2
  //  T12: transformation from frame 1 to 2
  inline void transform_pose(Pose& out, const Pose& p1, const Pose& T21) {
    // quat to rotation matrix
    Matrix3d R(T21.q);

    // p2 = T21.linear * p1 + T21.p
    out.p = R*p1.p + T21.p;

    // q2 = q21 * q1
    out.q = T21.q * p1.q;
  }

  // transform spatial vector
  //  v1: spatial velocity/force expressed in frame 1
  //      translation:rotation format
  //  T21: transformation matri
  //  flag_force: 1 if v1 is force vector
  inline void transform_spatial(
      Vector6d& out, const Vector6d& v1, const Pose& T21, int flag_force) {
    // quat to rotation matrix
    Matrix3d R(T21.q);

    // if rotational element is 0 then just need to transform
    // translational element
    if (v1.tail(3) == Vector3d::Zero()) {
      out.tail(3) = Vector3d::Zero();
      out.head(3) = R*v1.head(3);
    }
    // calculate adjoint representation of a transformation matrix
    else {
      Matrix3d p_bar;
      p_bar << 0, -T21.p(2) , T21.p(1),
               T21.p(2), 0,  -T21.p(0),
               -T21.p(1), T21.p(0), 0;
      // transform spatial force
      if (flag_force == 1){
        out.head(3) = p_bar*R*v1.tail(3) + R*v1.head(3);  // linear velocity
        out.tail(3) = R*v1.tail(3);                       // angular velocity
      }
      // transform spatial velocity
      else {
        out.head(3) = R*v1.head(3);                       // force
        out.tail(3) = R*v1.tail(3) + p_bar*R*v1.head(3);  // moment
      }
    }
  }


}

#endif
