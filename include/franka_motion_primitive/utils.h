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
    AdmittanceMotion,

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

  // compliant frame object
  class CompliantFrame {
    private:
      Pose frame_;
      Vector6d fd_;
    public:
      CompliantFrame()
        {frame_.p.setZero(); frame_.q = Quaterniond(1., 0., 0., 0.); fd_.setZero();}
      void get_compliant_frame(Pose& out) {out = frame_;}
      void get_position(Vector3d& out) {out = frame_.p;}
      void get_orientation(Quaterniond& out) {out = frame_.q;}
      void get_fd(Vector6d& out) {out = fd_;}
      void set_compliant_frame(const Pose& in) {frame_.p = in.p; frame_.q = in.q;}
      void set_compliant_frame(const Vector3d& pos, const Quaterniond& quat)
        {frame_.p = pos; frame_.q = quat;}
      /**
       *  update compliant frame based on the current command, current pose and
       *  selection matrix. The compliant frame is set to the current pose in the
       *  force-controlled direction and set to command otherwise
       *
       *  @param cmd   current control command
       *  @param pc    current end-effector pose
       *  @param S     selection matrix in the base frame
       */
      void update_compliant_frame(const Pose& cmd, const Pose& pc, const Matrix6d& S){
        Matrix3d I;
        I = Matrix3d::Identity();
        frame_.p = S.topLeftCorner(3, 3)*cmd.p + (I - S.topLeftCorner(3, 3))*pc.p;
        // NOTE not set based on selection matrix because not do torque control
        frame_.q = cmd.q;
      }
      void set_fd(const Vector6d& f) {fd_ = f;}
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

  inline void transform_selection_matrix(
      Matrix6d& out, const Matrix6d& S1, const Matrix3d& R21) {
    /**
     *  transform the selection matrix in frame 1 to frame 2.
     */
     out.setIdentity();
     out.topLeftCorner(3, 3) = R21 * S1.topLeftCorner(3, 3) * R21.transpose();
     out.bottomRightCorner(3, 3) = R21 * S1.bottomRightCorner(3, 3) * R21.transpose();
  }

  // quaternion error in the base frame
  inline void quat_error(Vector3d& out, Quaterniond& q1, const Quaterniond& q2){
    if (q1.coeffs().dot(q2.coeffs()) < 0.) q1.coeffs() << -q1.coeffs();
    // quaternion "error"
    Quaterniond qe(q2 * q1.inverse());
    // angle axis error
    AngleAxisd aae(qe);
    out << aae.axis() * aae.angle();
    for (size_t i=0; i<3; i++){
      if (std::isnan(out(i))) out(i) =0.;
    }
  }

  // transform force torque
  inline void transform_wrench(Vector6d& out, const Vector6d& f1, const Vector3d& p21, const Quaterniond& q21){
    // rotation matrix
    Matrix3d R21(q21);
    // force
    out.head(3) = R21 * f1.head(3);
    // torque
    Matrix3d p_tilde;
    p_tilde << 0, -p21(2), p21(1),
               p21(2), 0, -p21(0),
               -p21(1), p21(0), 0;
    out.tail(3) << R21 * f1.tail(3) + p_tilde*out.head(3);
  }

  inline void pseudoInverse(Eigen::MatrixXd& M_pinv_, const Eigen::MatrixXd& M_, double damp_ = 0.1) {
    double lambda_ = damp_;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }
}

#endif
