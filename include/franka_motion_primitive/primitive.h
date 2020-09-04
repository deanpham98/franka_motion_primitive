#include "utils.h"

namespace franka_motion_primitive{

  class PrimitiveBase {
    protected:
      // transform world state to task state
      void transform_state(State& task, const State& world);

      // execution time
      double t_exec_;

      // cartesian velocity and acceleration limits
      Vector6d dx_max_ = (Vector6d() << 0.5, 0.5, 0.5, 1.0, 1.0, 1.0).finished();
      Vector6d ddx_max_ = (Vector6d() << 5, 5, 5, 10.0, 10.0, 10.0).finished();
      // initial stiffness
      Vector6d kp_init_ = (Vector6d() << 1000., 1000., 1000., 60., 60., 30).finished();

      // maximum execution time
      double t_max_;

      Pose task_frame_;

      // controller stiffness and damping
      Matrix6d Kp_;
      Matrix6d Kd_;

      // initial state
      State s0_;

    public:
      PrimitiveBase() {
        t_exec_ = 0; t_max_ = 5.; Kp_.setIdentity(); Kd_.setIdentity();
        s0_.pose.p.setZero(); s0_.pose.q = Quaterniond(1., 0., 0., 0.);
        s0_.f.setZero(); s0_.v.setZero();
        task_frame_.p.setZero(); task_frame_.q = Quaterniond(1., 0., 0., 0.);
        // controller_gain_.kDefineDamping = 0;
        // for (size_t i=0; i<6 ; i++){
        //   controller_gain_.kp[i] = kp_init_(i);
        // }
      }

      // update control command and return the execution status of the primitive
      //    u = f(s, t) control input
      //    g = f(s, t) interrupt condition
      // NOTE: the calculated control command and state s
      //       is represented in the base frame
      virtual void update_control(
          ControlSignal& cmd, Status& status,
          const State& s, const double& dt) = 0;

      // set target pose, force for the primitive
      // NOTE: represented in the task frame
      //  - because the input (of the primitive) is represented in the task frame,
      //    while the its output (control command) is represented in the base frame,
      //    there is some transformation between the task frmae and the base frame
      //    internal of the primitive
      virtual void configure(ParamMap& params) = 0;

      // check terminating condition, return the status
      virtual void check_terminate(Status& status, const State& state) = 0;

      // NOTE: only used in admittancemotion
      virtual void SetAdmittanceGain(const Vector6d& kd) {} // do nothing
  };

  class MoveToPose : public PrimitiveBase {
    /**
     *  Move to a target pose
     *
     *  Params:
     *    task_frame: the frame in which the target pose
     *                and the trajectory is defined
     *    target_pose
     *    speed_factor
     *    timeout
     */
    public:
      MoveToPose();

      // update the control command (represented in base frame)
      void update_control(
          ControlSignal& cmd, Status& status,
          const State& s, const double& dt) override;

      // set target for the primitive (represented in task frame)
      void configure(ParamMap& params) override;

      // return the status of the primitive, g(s)
      void check_terminate(Status& status, const State& state) override;

    private:
      // plan for trajectory parameterers
      void plan_trajectory();

      // target
      double speed_factor_;
      Pose pd_;       // in task frame
      Pose pd_base_;  // in base frame
      Vector6d fd_;   // in task frame
      Vector6d fd_base_;   // in base frame
      Matrix6d Sd_;

      // trajectory paramters
      double t_traj_syn_;
      Vector6d dx_traj_max_;

      // distance between current and target
      Vector6d delta_x_;
  };

  class ConstantVelocity : public PrimitiveBase {
    /**
     *  control constant veloctiy of the end-effector
     *
     *  Params:
     *     - speed_factor [double] \in [0, 1] relative to dx_max
     *     - direction [Eigen::Vector6d]
     *     - task_frame [franka_motion_primitive::Pose]
     */
     public:
       ConstantVelocity();

       void update_control(
           ControlSignal& cmd, Status& status,
           const State& s, const double& dt) override;

       // set target for the primitive (represented in task frame)
       void configure(ParamMap& params) override;

       // return the status of the primitive, g(s)
       void check_terminate(Status& status, const State& state) override;
    private:
      void plan_trajectory();

      double speed_factor_;
      // move direction
      // double v_mag_;
      Vector6d move_dir_;
      Vector6d vd_base_;
      // fd
      Vector6d fd_;   // in task frame
      Vector6d fd_base_;   // in base frame
      Matrix6d Sd_;
      // ref pose
      Vector3d pd_;
      Quaterniond qd_;
      // stopping position
      Vector3d ps_;
      Quaterniond qs_;
      // stopping motino
      bool kDetectContact;
      int stopping_step_;
      const int kMaxStoppingStep{100};
      const double kAlphaStop{0.95}; // p = k*p + (1-k)*pd

      // threshhold force
      double f_thresh_;

      // trajectory params
      double t_accel_;
      double t_decel_;
      Eigen::Vector3d ddx_accel_;
      Eigen::Vector3d ddx_decel_;
      Eigen::Vector3d x_accel_end_;
      Eigen::Vector3d x_constant_end_;

  };

  class AdmittanceMotion : public PrimitiveBase {
    public:
      AdmittanceMotion();

      void update_control(
          ControlSignal& cmd, Status& status,
          const State& s, const double& dt) override;

      // set target for the primitive (represented in task frame)
      void configure(ParamMap& params) override;
      // return the status of the primitive, g(s)
      void check_terminate(Status& status, const State& state) override;

      void SetAdmittanceGain(const Vector6d& kd) override {
        kd_.setIdentity();
        kd_.diagonal() = kd;
      }
    private:
      // primitive parameters
      Vector6d fd_;
      Vector6d fd_base_;
      Matrix6d kd_;
      double z_thresh_;

      // desired ee pose
      Vector3d pd_;
      Quaterniond qd_;

      // stopping pose
      Vector3d ps_;
      Quaterniond qs_;
      // stopping motino
      bool kDetectThresh;
      int stopping_step_;
      const int kMaxStoppingStep{100};
      const double kAlphaStop{0.95}; // p = k*p + (1-k)*pd

      // gradually increase force (for tuning )
      double kMaxForce{-20.};

      // store value of t_max
      double timeout_;
  };

  // construct, run primitives
  class PrimitiveLibrary {
    public:

    private:
      int no_primitives;
      int no_types;
  };
}
