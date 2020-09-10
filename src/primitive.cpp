#include <franka_motion_primitive/primitive.h>

namespace franka_motion_primitive{
  void PrimitiveBase::transform_state(State& task, const State& world) {

  }

  MoveToPose::MoveToPose() : PrimitiveBase() {
    speed_factor_ = 0.5;
    pd_.p.setZero();
    pd_.q = Quaterniond(1.0, 0., 0., 0.);
    pd_base_.p.setZero();
    pd_base_.q = Quaterniond(1.0, 0., 0., 0.);

    fd_.setZero();
    Sd_.setIdentity();

    t_traj_syn_ = 1.0;
    dx_traj_max_ = speed_factor_ * dx_max_;
    delta_x_.setZero();
  }
  MoveToPose::MoveToPose(std::shared_ptr<CompliantFrame>& c_frame)
      : PrimitiveBase(c_frame) {
    speed_factor_ = 0.5;
    pd_.p.setZero();
    pd_.q = Quaterniond(1.0, 0., 0., 0.);
    pd_base_.p.setZero();
    pd_base_.q = Quaterniond(1.0, 0., 0., 0.);

    fd_.setZero();
    Sd_.setIdentity();

    t_traj_syn_ = 1.0;
    dx_traj_max_ = speed_factor_ * dx_max_;
    delta_x_.setZero();
  }

  void MoveToPose::update_control(
        ControlSignal& cmd, Status& status, const State& s, const double& dt){
    if (t_exec_ == 0) {
      s0_ = s;
      // plan from compliant frame
      Pose c_pose;
      compliant_frame_->get_compliant_frame(c_pose);
      s0_.pose = c_pose;
      plan_trajectory();
    }

    // update time
    t_exec_ += dt;
    t_max_ -= dt;

    // update status
    check_terminate(status, s);

    // update control
    // force
    Vector6d fd;
    compliant_frame_->get_fd(fd);
    // progressively change the value of controlled force
    cmd.f = fd_base_*0.01 + fd*0.99;

    // motion
    if (status == Status::EXECUTING){
      double k = std::pow(std::sin(0.5 * M_PI * t_exec_ / t_traj_syn_), 2);
      double k_vel = std::sin(M_PI * t_exec_ / t_traj_syn_);
      cmd.pose.p = s0_.pose.p + k * delta_x_.head(3);
      // cmd.pose.q = slerp(s0_.pose.q, pd_.q, k);
      cmd.pose.q = s0_.pose.q.slerp(k, pd_base_.q);
      // std::cout << "q_command" << cmd.pose.q.coeffs() << std::endl;
      cmd.v = k_vel * dx_traj_max_;
    }
    else {
      cmd.pose.p = pd_base_.p;
      cmd.pose.q.coeffs() = pd_base_.q.coeffs();
      cmd.v.setZero();
    }
    cmd.S.setIdentity();
    for (size_t i = 0; i< 6; ++i){
      if (fd_base_(i) != 0.) {cmd.S(i, i) = 0;}
      // if (std::abs(fd_(i)) >= 1e-2) {cmd.S(i, i) = 0;}
    }

    // update compliant frame
    compliant_frame_->set_compliant_frame(cmd.pose);
    compliant_frame_->set_fd(cmd.f);
  }

  void MoveToPose::configure(ParamMap& params) {
    speed_factor_ = boost::get<double>(params["speed_factor"]);
    pd_.p         = boost::get<Vector3d>(params["target_pos"]);
    pd_.q         = boost::get<Quaterniond>(params["target_quat"]);
    fd_           = boost::get<Vector6d>(params["target_force"]);
    task_frame_.p = boost::get<Vector3d>(params["task_frame_pos"]);
    task_frame_.q = boost::get<Quaterniond>(params["task_frame_quat"]);
    t_max_        = boost::get<double>(params["timeout"]);

    // transform target to base frame
    transform_pose(pd_base_, pd_, task_frame_);

    // quat to rotation matrix
    // WARNING: this is redundant (computed in transform_pose)
    //          solution: add rotation matrix to Pose struct?
    Matrix3d R(task_frame_.q);
    // transform target force
    // WARNING: only control force in this case
    // To control torque, there are 2 ways:
    //    1. Change force control law to J_ee * F_ee, and compute fd_ee_ (same as below)
    //    2. Compute fd_ee_ (wrench ee), then transform to (wrench base) and
    //       use the same control law
    fd_base_.head(3) = R*fd_.head(3);
    fd_base_.tail(3) = R*fd_.tail(3);

    // reset time
    t_exec_ = 0;

    // std::cout << pd_base_.p <<std::endl<< std::endl;
    // std::cout << pd_base_.q.coeffs() <<std::endl<< std::endl;
    // std::cout << s0_.pose.p <<std::endl<< std::endl;
    // std::cout << s0_.pose.q.coeffs() <<std::endl<< std::endl;

    // std::cout << fd_base_ <<std::endl<< std::endl;
  }

  void MoveToPose::check_terminate(Status& status, const State& state){
    if (t_exec_ < t_traj_syn_ + 0.005) {status = Status::EXECUTING;}
    else {status = Status::SUCCESS;}
  }

  void MoveToPose::plan_trajectory() {
    // TODO read AND set controller in
    // calculate delta_x
    if (pd_base_.q.coeffs().dot(s0_.pose.q.coeffs()) < 0.0) {
      s0_.pose.q.coeffs() << -s0_.pose.q.coeffs();
    }

    Eigen::Quaterniond qe(pd_base_.q * s0_.pose.q.inverse());
    Eigen::AngleAxisd aa_e(qe);  // quat -> aa

    delta_x_.head(3) = pd_base_.p - s0_.pose.p;
    delta_x_.tail(3) << aa_e.axis() * aa_e.angle();
    for (size_t i=0; i<3; i++){
      if (std::isnan(delta_x_(i+3))) delta_x_(i+3) =0.;
    }

    // calculate synchronized moving time
    Vector6d t_min_traj;
    dx_traj_max_ = speed_factor_ * dx_max_;
    for (size_t i = 0; i< 6; ++i){
      t_min_traj(i) = std::abs(delta_x_(i)) / dx_traj_max_(i) * M_PI / 2;
    }
    t_traj_syn_ = t_min_traj.maxCoeff();

    // update maximum velocity
    if (t_traj_syn_ < 1e-5){
      t_traj_syn_ = 1e-9;
      dx_traj_max_.setZero();
    }
    else{
      for (size_t i = 0; i< 6; ++i){
        dx_traj_max_(i) = delta_x_(i) * M_PI / (2 * t_traj_syn_);
      }
    }
  }

  ConstantVelocity::ConstantVelocity() {
    speed_factor_ = 0;
    // v_dir_.setZero();
    fd_.setZero();
    fd_base_.setZero();
    Sd_.setIdentity();
    f_thresh_ = 10.;
    kDetectContact = false;
    stopping_step_ = 0;
    move_dir_.setZero();
    pd_.setZero();
    ps_.setZero();
    qd_ = Quaterniond(1.0, 0.0, 0.0, 0.0);
    qs_ = Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  ConstantVelocity::ConstantVelocity(std::shared_ptr<CompliantFrame>& c_frame)
      : PrimitiveBase(c_frame){
    speed_factor_ = 0;
    // v_dir_.setZero();
    fd_.setZero();
    fd_base_.setZero();
    Sd_.setIdentity();
    f_thresh_ = 10.;
    kDetectContact = false;
    stopping_step_ = 0;
    move_dir_.setZero();
    pd_.setZero();
    ps_.setZero();
    qd_ = Quaterniond(1.0, 0.0, 0.0, 0.0);
    qs_ = Quaterniond(1.0, 0.0, 0.0, 0.0);
  }

  void ConstantVelocity::update_control(
      ControlSignal& cmd, Status& status,
      const State& s, const double& dt) {
    if (t_exec_ == 0) {
      s0_ = s;
      // pose
      pd_ = s0_.pose.p;
      qd_ = s0_.pose.q;
      // plan_trajectory();
      // set from compliant frame
      Pose c_pose;
      compliant_frame_->get_compliant_frame(c_pose);
      s0_.pose = c_pose;
      pd_ = c_pose.p;
      qd_ = c_pose.q;
      // std::cout << "q_c_init" << s0_.pose.q.coeffs() << std::endl;
    }

    // update status
    check_terminate(status, s);
    // update time
    // t_exec_ += dt;
    if (status == Status::EXECUTING) {t_exec_ += dt;}
    t_max_ -= dt;

    // update control
    // force
    // cmd.f = fd_base_;
    Vector6d fd;
    compliant_frame_->get_fd(fd);
    // progressively change the value of controlled force
    cmd.f = fd_base_*0.01 + fd*0.99;

    if (status == Status::EXECUTING) {
      // motion
      if (!kDetectContact) {
        pd_ = s0_.pose.p + t_exec_*vd_base_.head(3);
        // desired quaternion
        Vector3d rotd;
        rotd = t_exec_*vd_base_.tail(3);
        double angle = rotd.norm();
        if (angle > 1e-9) {
          rotd.normalize(); // normalize vector
          AngleAxisd aa_d(angle, rotd);
          // NOTE different quaternion between the desired and initial orientation,
          //      expressed in the base frame
          Quaterniond qe_d(aa_d);   // angle axis to quaternion
          qd_ = qe_d * s0_.pose.q;
        }
        else {
          qd_ = s0_.pose.q;
        }
        // desired velocity
        cmd.v= vd_base_;
      }
      else {
        // pd_ = kAlphaStop*pd_ + (1-kAlphaStop)*ps_;
        // Quaterniond qslerp = qd_.slerp(1-kAlphaStop, qs_);
        // qd_ = qslerp;
        cmd.v = Vector6d::Zero();
      }
    }
    else {cmd.v = Vector6d::Zero();}

    // setting fd = f_thresh in the moving direction
    // if (status == Status::SUCCESS) {
    //   Vector6d f = move_dir_ * f_thresh_;
    //   cmd.f = fd_base_ + f;
    // }

    cmd.pose.p = pd_;
    cmd.pose.q = qd_;

    // selection matrix
    cmd.S.setIdentity();
    for (size_t i = 0; i< 6; ++i){
      // if (fd_base_(i) != 0.) {cmd.S(i, i) = 0;}
      if (std::abs(fd_base_(i)) >= 1.) {
        cmd.S(i, i) = 0;
        if (i <3)
          cmd.pose.p(i) = s.pose.p(i);
      }
    }
    // set compliant frame
    compliant_frame_->set_compliant_frame(cmd.pose);
    compliant_frame_->set_fd(cmd.f);
  }

  void ConstantVelocity::configure(ParamMap& params) {
    Vector6d move_dir;

    speed_factor_ = boost::get<double>(params["speed_factor"]);
    fd_           = boost::get<Vector6d>(params["target_force"]);
    task_frame_.p = boost::get<Vector3d>(params["task_frame_pos"]);
    task_frame_.q = boost::get<Quaterniond>(params["task_frame_quat"]);
    t_max_        = boost::get<double>(params["timeout"]);
    move_dir        = boost::get<Vector6d>(params["direction"]);
    f_thresh_     = boost::get<double>(params["f_thresh"]);

    // velocity in the task frame v^t_ee
    Vector6d ve_task;
    for (size_t i=0; i<6; ++i)
      ve_task(i) = speed_factor_ * dx_max_(i) * move_dir(i);

    // transform to base frame v^0_ee
    // R^0_task
    Matrix3d R(task_frame_.q);
    vd_base_.head(3) = R*ve_task.head(3);
    vd_base_.tail(3) = R*ve_task.tail(3);
    move_dir_ = vd_base_.normalized();
    // std::cout << vd_base_ << std::endl;

    // transform to base frame
    fd_base_.head(3) = R*fd_.head(3);
    fd_base_.tail(3) = R*fd_.tail(3);

    // std::cout << fd_base_ << std::endl;

    // reset time
    t_exec_ = 0.;
    stopping_step_ =0;
    // reset contact
    kDetectContact = false;
  }

  void ConstantVelocity::check_terminate(Status& status, const State& state){
    // quat to rotatiion matrix
    Matrix3d R(state.pose.q);

    // transofrm fe to f0
    Vector6d f0;
    f0.head(3) = R*state.f_ee.head(3);
    f0.tail(3) = R*state.f_ee.tail(3);

    double f_proj;
    // project external force to move direction
    f_proj= f0.adjoint()*move_dir_;

    // std::cout << f_proj << std::endl;

    if (t_max_ < 0) {status = Status::TIMEOUT; return;}
    else if (f_proj > f_thresh_) {
      if (kDetectContact==false) {
        ps_ = state.pose.p;
        qs_ = state.pose.q;
        kDetectContact = true;
      }
    }

    if (stopping_step_<=kMaxStoppingStep) {
      if (kDetectContact) {stopping_step_ += 1;}
    }
    else {status = Status::SUCCESS; return;}
    status = Status::EXECUTING;
  }

  void ConstantVelocity::plan_trajectory() {
    // NOTE: the formulation might wrong because the velocity representation
    //       is not unified (note that State.v is the twist, while hte desired
    //       velocity is not twist - instead it is the linear and angular velocity
    //       of a frame represented in another frame)

    Eigen::Vector3d t_accel, t_decel;
    t_accel.setZero();
    t_decel.setZero();
    for (size_t i = 0; i < 6; ++i) {
      t_accel(i) = std::abs(vd_base_(i) - s0_.v(i)) / ddx_max_(i);
      t_decel(i) = std::abs(vd_base_(i)) / ddx_max_(i);
    }
    t_accel_ = t_accel.maxCoeff();
    t_decel_ = t_decel.maxCoeff();
    for (size_t i = 0; i < 3; ++i) {
      ddx_accel_(i) = (vd_base_(i) - s0_.v(i)) / t_accel_;
      ddx_decel_(i) = -vd_base_(i) / t_decel_;
    }

    // open loop setpoint based on velocity and moving time
    x_accel_end_ = s0_.pose.p + s0_.v.head(3) * t_accel_ +
                  ddx_accel_ * std::pow(t_accel_, 2) / 2;

    x_constant_end_ = x_accel_end_ + vd_base_.head(3) * t_exec_;

    // if (!kUseForceThreshold){
    //   x_constant_end_ = x_accel_end_ + vd_base_.head(3) * time_move_;
    // }
  }

  AdmittanceMotion::AdmittanceMotion() {
    fd_.setZero();
    fd_base_.setZero();
    kd_.setZero();
    z_thresh_ = 0.;

    pd_.setZero();
    ps_.setZero();
    qd_ = Quaterniond(1., 0., 0., 0.);
    qs_ = Quaterniond(1., 0., 0., 0.);

  }
  AdmittanceMotion::AdmittanceMotion(std::shared_ptr<CompliantFrame>& c_frame)
      : PrimitiveBase(c_frame) {
    fd_.setZero();
    fd_base_.setZero();
    kd_.setZero();
    z_thresh_ = 0.;

    pd_.setZero();
    ps_.setZero();
    qd_ = Quaterniond(1., 0., 0., 0.);
    qs_ = Quaterniond(1., 0., 0., 0.);
  }

  void AdmittanceMotion::update_control(
      ControlSignal& cmd, Status& status,
      const State& s, const double& dt) {
    if (t_exec_ == 0) {
      s0_ = s;
      Pose c_pose;
      compliant_frame_->get_compliant_frame(c_pose);
      s0_.pose = c_pose;
      // pd_ = s.pose.p;
      // qd_ = s.pose.q;
      pd_ = c_pose.p;
      qd_ = c_pose.q;
      fd_ = fd_base_;
      timeout_ = t_max_;
    }

    // update time
    t_exec_ += dt;
    t_max_ -= dt;

    check_terminate(status, s);

    // update control
    // force
    // cmd.f = fd_base_;
    Vector6d fd;
    compliant_frame_->get_fd(fd);
    // progressively change the value of controlled force
    cmd.f = fd_base_*0.01 + fd*0.99;

    // Vector6d fd;
    if (status == Status::EXECUTING) {
      if (!kDetectThresh) {
        // quaternion -> rotation matrix
        Matrix3d R(s.pose.q);
        // transform ee_force to base frame
        Vector6d f0;
        f0.head(3) = R*s.f_ee.head(3);
        f0.tail(3) = R*s.f_ee.tail(3);

        // desired end-effector velocity
        Vector6d v_ee;
        // v_ee = -kd_ * s.f;
        v_ee = -kd_ * f0;
        // pd_ = s.pose.p + dt*v_ee.head(3);
        // OR
        Vector3d pd = pd_ + dt*v_ee.head(3);
        pd_ = pd;

        // desired quaternion
        Vector3d rotd;
        rotd = dt*v_ee.tail(3);
        double angle = rotd.norm();
        if (angle <= 1e-6) {rotd.setZero(); rotd(0) = 1.;}
        else {rotd.normalize();} // normalize vector

        AngleAxisd aa_d(angle, rotd);
        // NOTE different quaternion between the desired and initial orientation,
        //      expressed in the base frame
        Quaterniond qe_d(aa_d);   // angle axis to quaternion

        // qd_ = qe_d * s.pose.q;
        // OR
        Quaterniond qd = qe_d * qd_;
        qd_ = qd;
        cmd.v= v_ee;
      }
      else {
        pd_ = kAlphaStop*pd_ + (1-kAlphaStop)*ps_;
        Quaterniond qslerp = qd_.slerp(1-kAlphaStop, qs_);
        qd_ = qslerp;
        cmd.v = Vector6d::Zero();
      }
      // update dsired force
      // if (fd_base_(2) != 0) {
      //   if (t_exec_ < timeout_)
      //     fd_(2) = fd_base_(2) + t_exec_/timeout_*(kMaxForce - fd_base_(2));
      //   else fd_(2) = kMaxForce;
      // }
      // fd_ = fd_base_;
    }
    else {cmd.v = Vector6d::Zero();}

    cmd.pose.p = pd_;
    cmd.pose.q = qd_;
    // cmd.f = fd_;

    // selection matrix
    cmd.S.setIdentity();
    for (size_t i = 0; i< 6; ++i){
      if (std::abs(fd_base_(i)) >= 1.) {
        cmd.S(i, i) = 0;
        if (i <3)
          cmd.pose.p(i) = s.pose.p(i);
      }
    }
    // update compliant frame
    compliant_frame_->set_compliant_frame(cmd.pose);
    compliant_frame_->set_fd(cmd.f);

  }

  void AdmittanceMotion::configure(ParamMap& params) {
    kd_.diagonal() = boost::get<Vector6d>(params["kd"]);
    fd_ = boost::get<Vector6d>(params["fd"]);
    t_max_ = boost::get<double>(params["timeout"]);
    z_thresh_ = boost::get<double>(params["z_thresh"]);

    // quat to rotatiion matrix
    Matrix3d R(task_frame_.q);
    // transform to base frame
    fd_base_.head(3) = R*fd_.head(3);
    fd_base_.tail(3) = R*fd_.tail(3);

    // reset time
    t_exec_ = 0;
    // reset stopping step
    stopping_step_ = 0;
    kDetectThresh = false;
  }

  void AdmittanceMotion::check_terminate(Status& status, const State& state) {
    if (t_max_ <0) {status = Status::TIMEOUT; return;}
    if (state.pose.p(2) < z_thresh_) {
      if (!kDetectThresh) {
        ps_ = state.pose.p;
        qs_ = state.pose.q;
        kDetectThresh = true;
      }
    }
    if (stopping_step_<=kMaxStoppingStep) {
      if (kDetectThresh) {stopping_step_ += 1;}
    }
    else {status = Status::SUCCESS; return;}
    status = Status::EXECUTING;
  }

  void Displacement::configure(ParamMap& params) {
    Vector6d move_dir;

    speed_factor_ = boost::get<double>(params["speed_factor"]);
    fd_           = boost::get<Vector6d>(params["target_force"]);
    task_frame_.p = boost::get<Vector3d>(params["task_frame_pos"]);
    task_frame_.q = boost::get<Quaterniond>(params["task_frame_quat"]);
    t_max_        = boost::get<double>(params["timeout"]);
    move_dir        = boost::get<Vector6d>(params["direction"]);
    f_thresh_     = boost::get<double>(params["f_thresh"]);
    delta_d_      = boost::get<double>(params["displacement"]);

    // velocity in the task frame v^t_ee
    Vector6d ve_task;
    for (size_t i=0; i<6; ++i)
      ve_task(i) = speed_factor_ * dx_max_(i) * move_dir(i);

    // transform to base frame v^0_ee
    // R^0_task
    Matrix3d R(task_frame_.q);
    vd_base_.head(3) = R*ve_task.head(3);
    vd_base_.tail(3) = R*ve_task.tail(3);
    move_dir_ = vd_base_.normalized();
    // std::cout << vd_base_ << std::endl;

    // transform to base frame
    fd_base_.head(3) = R*fd_.head(3);
    fd_base_.tail(3) = R*fd_.tail(3);

    // std::cout << fd_base_ << std::endl;

    // reset time
    t_exec_ = 0.;
    stopping_step_ =0;
    // reset contact
    kDetectContact = false;
    // displacement goal detect
    kDisplacementReach = false;
  }


  void Displacement::update_control(
      ControlSignal& cmd, Status& status,
      const State& s, const double& dt) {
    if (t_exec_ == 0) {
      // initial compliant frame
      s0_ = s;
      // initial sensed position
      ps0_.p = s.pose.p;
      ps0_.q = s.pose.q;
      // plan_trajectory();
      // set from compliant frame
      Pose c_pose;
      compliant_frame_->get_compliant_frame(c_pose);
      s0_.pose = c_pose;
      pd_ = c_pose.p;
      qd_ = c_pose.q;
    }

    // update displacement
    if (s.pose.q.coeffs().dot(ps0_.q.coeffs()) < 0.0) {
      ps0_.q.coeffs() << -ps0_.q.coeffs();
    }

    Eigen::Quaterniond qe(s.pose.q * ps0_.q.inverse());
    Eigen::AngleAxisd aa_e(qe);  // quat -> aa

    dp_.head(3) = s.pose.p - ps0_.p;
    dp_.tail(3) << aa_e.axis() * aa_e.angle();
    for (size_t i=0; i<3; i++){
      if (std::isnan(dp_(i+3))) dp_(i+3) =0.;
    }

    // update status
    check_terminate(status, s);
    // update time
    // t_exec_ += dt;
    if (status == Status::EXECUTING) {t_exec_ += dt;}
    t_max_ -= dt;

    // update control
    // force
    // cmd.f = fd_base_;
    Vector6d fd;
    compliant_frame_->get_fd(fd);
    // progressively change the value of controlled force
    cmd.f = fd_base_*0.01 + fd*0.99;

    if (status == Status::EXECUTING) {
      // motion
      if (!kDetectContact && !kDisplacementReach) {
        pd_ = s0_.pose.p + t_exec_*vd_base_.head(3);
        // desired quaternion
        Vector3d rotd;
        rotd = t_exec_*vd_base_.tail(3);
        double angle = rotd.norm();
        if (angle > 1e-9) {
          rotd.normalize(); // normalize vector
          AngleAxisd aa_d(angle, rotd);
          // NOTE different quaternion between the desired and initial orientation,
          //      expressed in the base frame
          Quaterniond qe_d(aa_d);   // angle axis to quaternion
          qd_ = qe_d * s0_.pose.q;
        }
        else {
          qd_ = s0_.pose.q;
        }
        // desired velocity
        cmd.v= vd_base_;
      }
      else {
        // pd_ = kAlphaStop*pd_ + (1-kAlphaStop)*ps_;
        // Quaterniond qslerp = qd_.slerp(1-kAlphaStop, qs_);
        // qd_ = qslerp;
        cmd.v = Vector6d::Zero();
      }
    }
    else {cmd.v = Vector6d::Zero();}

    // setting fd = f_thresh in the moving direction
    // if (status == Status::SUCCESS) {
    //   Vector6d f = move_dir_ * f_thresh_;
    //   cmd.f = fd_base_ + f;
    // }

    cmd.pose.p = pd_;
    cmd.pose.q = qd_;

    // selection matrix
    cmd.S.setIdentity();
    for (size_t i = 0; i< 6; ++i){
      // if (fd_base_(i) != 0.) {cmd.S(i, i) = 0;}
      if (std::abs(fd_base_(i)) >= 1.) {
        cmd.S(i, i) = 0;
        if (i <3)
          cmd.pose.p(i) = s.pose.p(i);
      }
    }
    // set compliant frame
    compliant_frame_->set_compliant_frame(cmd.pose);
    compliant_frame_->set_fd(cmd.f);
  }

  void Displacement::check_terminate(Status& status, const State& state){
    // quat to rotatiion matrix
    Matrix3d R(state.pose.q);

    // transofrm fe to f0
    Vector6d f0;
    f0.head(3) = R*state.f_ee.head(3);
    f0.tail(3) = R*state.f_ee.tail(3);

    double f_proj;
    // project external force to move direction
    f_proj= f0.adjoint()*move_dir_;

    // directional displacement
    double dp_proj = dp_.adjoint()*move_dir_;

    // checking
    // std::cout << "f_proj: " << f_proj <<std::endl;
    // std::cout << "dp_proj: " << dp_proj <<std::endl;

    // std::cout << f_proj << ","<< dp_proj<< std::endl;
    if (t_max_ < 0) {status = Status::TIMEOUT; return;}
    else if (f_proj > f_thresh_) {
      if (kDetectContact==false) {
        ps_ = state.pose.p;
        qs_ = state.pose.q;
        kDetectContact = true;
      }
    }

    else if (std::abs(dp_proj) > delta_d_) {
      if (kDisplacementReach==false){
        ps_ = state.pose.p;
        qs_ = state.pose.q;
        kDisplacementReach = true;
      }
    }

    if (stopping_step_<=kMaxStoppingStep) {
      if (kDetectContact || kDisplacementReach)
        {stopping_step_ += 1;}
    }
    else {status = Status::SUCCESS; return;}
    status = Status::EXECUTING;
  }

}
