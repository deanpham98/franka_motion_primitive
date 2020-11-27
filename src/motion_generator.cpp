#include <boost/bind.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <franka_motion_primitive/motion_generator.h>
#include <franka_motion_primitive/PrimitiveType.h>
#include <franka_motion_primitive/PrimitiveStatus.h>

namespace franka_motion_primitive{

  bool MotionGenerator::init(hardware_interface::RobotHW* robot_hw,
        // hardware_interface::ForceTorqueSensorInterface* ft_hw,
        ros::NodeHandle& nh) {
    // TODO: controller ns config
    std::string controller_ns, controller_cmd_topic, gain_config_topic;
    controller_ns = "/variable_impedance_controller";
    controller_cmd_topic = controller_ns + "/command";
    gain_config_topic = controller_ns + "/gain_config";

    // TODO arm_id to config
    std::string arm_id;
    arm_id = "panda";

    // state interface
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM("Error getting state interface");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Exception getting state handle: " << ex.what());
      return false;
    }
    // model interface
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM("Error getting model interface");
      return false;
    }
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Exception getting model handle: " << ex.what());
      return false;
    }

    // TODO: read no_primitive_type_ from config
    no_primitive_types_ = 5; // default value

    // common compliant frame
    c_frame_ = std::make_shared<CompliantFrame>();
    // primitive container
    primitive_container_.resize(no_primitive_types_);
    primitive_container_[PrimitiveType::MoveToPose] =
      std::make_shared<franka_motion_primitive::MoveToPose>(c_frame_);

    primitive_container_[PrimitiveType::ConstantVelocity] =
      std::make_shared<franka_motion_primitive::ConstantVelocity>(c_frame_);

    primitive_container_[PrimitiveType::AdmittanceMotion] =
      std::make_shared<franka_motion_primitive::AdmittanceMotion>(c_frame_);

    primitive_container_[PrimitiveType::Displacement] =
      std::make_shared<franka_motion_primitive::Displacement>(c_frame_);

    primitive_container_[PrimitiveType::MoveToPoseFeedback] =
      std::make_shared<franka_motion_primitive::MoveToPoseFeedback>(c_frame_, nh);

    // set main primitive to move to pose primtiive
    main_primitive_ = primitive_container_[PrimitiveType::MoveToPose];
    // main_primitive_ = primitive_container_[PrimitiveType::ConstantVelocity];

    // pubs and subs
    sub_primitive_cmd_ = nh.subscribe("run_primitive", 1,
        &MotionGenerator::primitiveCommandCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    ft_sensor_.setZero();
    kFtSubscribeInit = false;
    sub_ft_sensor_ = nh.subscribe("/netft_data", 1,
        &MotionGenerator::ftSensorCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    pub_controller_.init(nh, controller_cmd_topic, 1);
    pub_state_.init(nh, "state", 1);
    pub_gain_.init(nh, gain_config_topic, 1);

    // service
    set_initial_force_serv_ =
      nh.advertiseService<SetInitialForce::Request, SetInitialForce::Response>
        ("set_initial_force", boost::bind(&MotionGenerator::set_initial_force, this, _1, _2));

    run_primitive_serv_ =
      nh.advertiseService<RunPrimitive::Request, RunPrimitive::Response>
        ("run_primitive", boost::bind(&MotionGenerator::run_primitive, this, _1, _2));

    // Dynamic reconfigure
    dynamic_admittance_node_ =
      ros::NodeHandle("dynamic_reconfigure_admittance_config");

    dynamic_server_admittance_config_.reset(
      new dynamic_reconfigure::Server<AdmittanceConfig>(
          dynamic_admittance_node_));

    dynamic_server_admittance_config_->setCallback(
      boost::bind(&MotionGenerator::AdmittanceDynamicReconfigureCallback, this, _1, _2));

    return true;
  }

  void MotionGenerator::starting(const ros::Time&){
    franka::RobotState robot_state = state_handle_->getRobotState();
    // curretn position and orientation
    Eigen::Affine3d T(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Vector3d p = T.translation();
    Quaterniond q = Quaterniond(T.linear());

    // set the task frame to be at the base
    // and target pose to be the end-effector pose
    Vector3d zero_vec3d = Vector3d::Zero();
    Vector6d zero_vec6d = Vector6d::Zero();
    Quaterniond zero_quat = Quaterniond(1., 0., 0., 0.);
    primitive_param_["task_frame_pos"] = zero_vec3d;
    primitive_param_["task_frame_quat"] = zero_quat;
    primitive_param_["target_pos"] = p;
    primitive_param_["target_quat"] = q;
    primitive_param_["target_force"] = zero_vec6d;
    primitive_param_["speed_factor"] = 0.1;
    primitive_param_["timeout"] = 5.;

    // set compliant frame
    c_frame_->set_compliant_frame(p, q);

    // setting primitive
    main_primitive_->configure(primitive_param_);

    // initial filter velocity
    dq_filter_ = Vector7d::Map(robot_state.dq.data());
    // initial external force for compensation
    f0_ = Vector6d::Map(robot_state.O_F_ext_hat_K.data());
    f0_ee_ = Vector6d::Map(robot_state.K_F_ext_hat_K.data());
    transform_wrench(f0_ss_, ft_sensor_, p_ee_ft_, q_ee_ft_);
    // estimation force
    std::array<double, 42> body_jacobian_array =
      model_handle_->getBodyJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> Jb(body_jacobian_array.data());
    // -- external torque
    Vector7d tau_ext(Vector7d::Map(robot_state.tau_ext_hat_filtered.data()));
    calc_ext_force(f0_ext_est_, Jb, tau_ext);

    // filter force
    f_filter_ = f0_;
    f_ee_filter_ = f0_ee_;
    // filter gain
    filter_gain_ = kDeltaT / (kDeltaT + 1/(2*M_PI*kCutoffFrequency));
    // std::cout << filter_gain_ <<std::endl;

    target_primitive_type_ = PrimitiveType::MoveToPose;
    execution_status_ = Status::EXECUTING;
    kReceiveCommand = false;
    kSetInitialForce = false;
    kRunPrimitiveReq = false;
  }

  void MotionGenerator::update(const ros::Time& t_now, const ros::Duration& period) {
    // configure primtiive
    // if (kReceiveCommand == true && execution_status_ != Status::EXECUTING) {
    //   // TODO: set controller param (publish to hybrid_controller)
    //
    //   // configure primitive
    //   main_primitive_ = primitive_container_[target_primitive_type_];
    //   main_primitive_->configure(primitive_param_);
    //   kReceiveCommand = false;
    // }

    // configure primtiive
    if (kRunPrimitiveReq == true && execution_status_ != Status::EXECUTING) {
      // set controller gain
      franka_example_controllers::Gain gain =
        boost::get<franka_example_controllers::Gain>(primitive_param_["controller_gain_msg"]);

      if (pub_gain_.trylock()){
        pub_gain_.msg_ = gain;
        pub_gain_.unlockAndPublish();
      }
      // configure primitive
      main_primitive_ = primitive_container_[target_primitive_type_];
      main_primitive_->configure(primitive_param_);
      execution_status_ = Status::EXECUTING;
      kRunPrimitiveReq = false;
    }

    // admittance param
    // if (kReceiveDynamicServer) {
    //   // TODO set controller gain
    //   if (pub_gain_.trylock()){
    //     pub_gain_.msg_.kDefineDamping = 1;
    //     for (size_t i = 0; i < 6; i++){
    //       pub_gain_.msg_.kp[i] = kp_dynamic_(i);
    //       pub_gain_.msg_.kd[i] = kd_dynamic_(i);
    //     }
    //     pub_gain_.unlockAndPublish();
    //   }
    //   main_primitive_->SetAdmittanceGain(kd_admittance_dynamic_);
    //   kReceiveDynamicServer = false;
    // }

    // robot state
    franka::RobotState robot_state = state_handle_->getRobotState();
    // ee transformation
    Eigen::Affine3d T(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Vector3d p = T.translation();
    Quaterniond q = Quaterniond(T.linear());
    // measured external force
    Vector6d f(Vector6d::Map(robot_state.O_F_ext_hat_K.data()));
    Vector6d f_ee(Vector6d::Map(robot_state.K_F_ext_hat_K.data()));
    // update filter velocity
    Vector7d dq(Vector7d::Map(robot_state.dq.data()));
    dq_filter_ = alpha_dq_*dq + (1-alpha_dq_)*dq_filter_;
    // Jacobian matrix
    std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

    // update filter force
    f_filter_ = filter_gain_ * f + (1 - filter_gain_) * f_filter_;
    f_ee_filter_ = filter_gain_ * f_ee + (1 - filter_gain_) * f_ee_filter_;
    // TODO get force torque sensor signals
    Vector6d fe;
    transform_wrench(fe, ft_sensor_, p_ee_ft_, q_ee_ft_);

    // external estimation force
    Vector6d fe_est;
    std::array<double, 42> body_jacobian_array =
      model_handle_->getBodyJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> Jb(body_jacobian_array.data());
    // -- external torque
    Vector7d tau_ext(Vector7d::Map(robot_state.tau_ext_hat_filtered.data()));
    // tau friction
    calc_ext_force(fe_est, Jb, tau_ext);

    // update initial force
    // TODO reset ft sensor force
    if (kSetInitialForce) {
      f0_ = f_filter_;
      f0_ee_ = f_ee_filter_;
      transform_wrench(f0_ss_, ft_sensor_, p_ee_ft_, q_ee_ft_);
      // estimation force
      f0_ext_est_ = fe_est;

      // NOTE reset compliant frame
      main_primitive_->reset_compliant_frame(p, q);
      kSetInitialForce = false;
    }

    // update internal state
    State s;
    s.pose.p = p;
    s.pose.q = q;
    s.v = J*dq_filter_;   // base twist
    s.f = f_filter_ - f0_;
    // s.f_ee = f_ee_filter_ - f0_ee_;
    // TODO set s.f_ee to fe
    s.f_ee = -(fe - f0_ss_);    // FT sensor
    // s.f_ee = fe_est - f0_ext_est_;

    // update control u = f(s, t)
    ControlSignal cmd;
    main_primitive_->update_control(cmd, execution_status_, s, period.toSec());

    // publish to controller
    if (pub_controller_.trylock()){
      for (size_t i = 0; i < 3; ++i){
        // pub_controller_.msg_.f[i] = s.f_ee(i);
        // pub_controller_.msg_.f[i+3] = s.f_ee(i+3);
        // pub_controller_.msg_.S[i] = cmd.S(i, i);
        // pub_controller_.msg_.S[i+3] = cmd.S(i+3, i+3);
        pub_controller_.msg_.v[i] = cmd.v(i);
        pub_controller_.msg_.v[i+3] = cmd.v(i+3);
        pub_controller_.msg_.p[i] = cmd.pose.p[i];
        pub_controller_.msg_.f[i] = cmd.f(i);
        pub_controller_.msg_.f[i + 3] = cmd.f(i + 3);
      }
      pub_controller_.msg_.q[0] = cmd.pose.q.w();
      pub_controller_.msg_.q[1] = cmd.pose.q.x();
      pub_controller_.msg_.q[2] = cmd.pose.q.y();
      pub_controller_.msg_.q[3] = cmd.pose.q.z();

      pub_controller_.unlockAndPublish();
    }

    if (state_publish_rate_() && pub_state_.trylock()) {
      pub_state_.msg_.stamp = t_now.toSec();
      for (size_t i=0; i<6; ++i) {
        // pub_state_.msg_.f_filter[i] = s.f[i];
        pub_state_.msg_.f_filter[i] = fe_est(i) - f0_ext_est_(i);
        pub_state_.msg_.f[i] = f[i] - f0_[i];
        pub_state_.msg_.fd[i] = cmd.f(i);
        pub_state_.msg_.f_ee[i] = f_ee_filter_(i) - f0_ee_(i);
        pub_state_.msg_.f_s[i] = fe(i) - f0_ss_(i);
        pub_state_.msg_.status = int(execution_status_);
      }
      pub_state_.unlockAndPublish();
    }
  }

  void MotionGenerator::primitiveCommandCallback(const RunPrimitiveCommandConstPtr& msg){
    ROS_INFO("receive command");

    target_primitive_type_ = msg->type;
    std::cout <<  int(msg->type) << std::endl;
    msg2PrimitiveParam(primitive_param_, msg);

    kReceiveCommand = true;
  }

  void MotionGenerator::msg2PrimitiveParam(
        ParamMap& out, const RunPrimitiveCommandConstPtr& msg){
    out.clear();
    if (target_primitive_type_ == PrimitiveType::MoveToPose){
      Vector3d task_frame_pos, target_pos;
      Quaterniond task_frame_quat, target_quat;
      Vector6d fd;

      // task frame
      task_frame_pos = Vector3d::Map(msg->move_to_pose_param.task_frame.pos.data());
      // task_frame_quat = Quaterniond(msg->move_to_pose_param.task_frame.quat.data());
      task_frame_quat = Quaterniond(msg->move_to_pose_param.task_frame.quat[0],
                                    msg->move_to_pose_param.task_frame.quat[1],
                                    msg->move_to_pose_param.task_frame.quat[2],
                                  msg->move_to_pose_param.task_frame.quat[3]);

      // target pose
      target_pos = Vector3d::Map(msg->move_to_pose_param.target_pose.pos.data());
      // target_quat = Quaterniond(msg->move_to_pose_param.target_pose.quat.data());
      target_quat = Quaterniond(msg->move_to_pose_param.target_pose.quat[0],
                                    msg->move_to_pose_param.target_pose.quat[1],
                                    msg->move_to_pose_param.target_pose.quat[2],
                                  msg->move_to_pose_param.target_pose.quat[3]);
      // desired force
      fd = Vector6d::Map(msg->move_to_pose_param.fd.data());

      out["task_frame_pos"] = task_frame_pos;
      out["task_frame_quat"] = task_frame_quat;
      out["target_pos"] = target_pos;
      out["target_quat"] = target_quat;
      out["target_force"] = fd;
      out["timeout"] = msg->move_to_pose_param.timeout;
      out["speed_factor"] = msg->move_to_pose_param.speed_factor;
      out["controller_gain_msg"] = msg->move_to_pose_param.controller_gain;
    }
    else if (target_primitive_type_ == PrimitiveType::ConstantVelocity) {
      Vector3d task_frame_pos;
      Quaterniond task_frame_quat;
      Vector6d move_dir, fd;

      // task frame
      task_frame_pos = Vector3d::Map(msg->constant_velocity_param.task_frame.pos.data());
      // task_frame_quat = Quaterniond(msg->constant_velocity_param.task_frame.quat.data());
      task_frame_quat = Quaterniond(msg->constant_velocity_param.task_frame.quat[0],
                                    msg->constant_velocity_param.task_frame.quat[1],
                                    msg->constant_velocity_param.task_frame.quat[2],
                                  msg->constant_velocity_param.task_frame.quat[3]);

      // desired force
      fd = Vector6d::Map(msg->constant_velocity_param.fd.data());

      // move_dir
      move_dir = Vector6d::Map(msg->constant_velocity_param.direction.data());

      out["task_frame_pos"] = task_frame_pos;
      out["task_frame_quat"] = task_frame_quat;
      out["direction"] = move_dir;
      out["target_force"] = fd;
      out["speed_factor"] = msg->constant_velocity_param.speed_factor;
      out["timeout"] = msg->constant_velocity_param.timeout;
      out["f_thresh"] = msg->constant_velocity_param.f_thresh;

    }
    else if (target_primitive_type_ == PrimitiveType::AdmittanceMotion) {
      Vector6d fd, kd;
      kd = Vector6d::Map(msg->admittance_motion_param.kd.data());
      fd = Vector6d::Map(msg->admittance_motion_param.fd.data());

      out["kd"] = kd;
      out["fd"] = fd;
      out["timeout"] = msg->admittance_motion_param.timeout;
      out["z_thresh"] = msg->admittance_motion_param.z_thresh;

    }
  }
  bool MotionGenerator::set_initial_force(SetInitialForce::Request& req,
                          SetInitialForce::Response& res ){
    ROS_INFO("Request to set initial force");
    kSetInitialForce = true;
    return true;
  }

  void MotionGenerator::AdmittanceDynamicReconfigureCallback(
    AdmittanceConfig& config,
    uint32_t level)
  {
    kd_dynamic_.setZero();
    kp_dynamic_.setZero();
    kd_admittance_dynamic_.setZero();
    for (size_t i=0; i<3; ++i){
      kp_dynamic_(i) = config.stiffness_lin;
      kd_dynamic_(i) = config.damping_lin;
      kp_dynamic_(i+3) = config.stiffness_rot;
      kd_dynamic_(i+3) = config.damping_rot;

      kd_admittance_dynamic_(i) = config.kd_lin;
      kd_admittance_dynamic_(i+3) = config.kd_rot;
    }
    kReceiveDynamicServer = true;
  }

  bool MotionGenerator::run_primitive(
        RunPrimitive::Request& req, RunPrimitive::Response& res){
    target_primitive_type_ = req.type;
    ROS_INFO_STREAM("run primitive request received with primitive type " << int(target_primitive_type_));
    service2PrimitiveParam(primitive_param_, req);
    kRunPrimitiveReq = true;

    // sleep rate
    ros::Rate r(100);
    while (kRunPrimitiveReq == true || execution_status_ == Status::EXECUTING) {
      r.sleep();
    }
    res.time = ros::Time::now().toSec() - req.time;

    if (execution_status_ == Status::TIMEOUT) {res.status = PrimitiveStatus::TIMEOUT;}
    else {res.status = PrimitiveStatus::SUCCESS;}
    ROS_INFO("done running service");
    return true;
  }

  void MotionGenerator::service2PrimitiveParam(ParamMap& out, RunPrimitive::Request& req){
    out.clear();
    if (target_primitive_type_ == PrimitiveType::MoveToPose  ||
        target_primitive_type_ == PrimitiveType::MoveToPoseFeedback){
      Vector3d task_frame_pos, target_pos;
      Quaterniond task_frame_quat, target_quat;
      Vector6d fd;

      // task frame
      task_frame_pos = Vector3d::Map(req.move_to_pose_param.task_frame.pos.data());
      // task_frame_quat = Quaterniond(req.move_to_pose_param.task_frame.quat.data());
      task_frame_quat = Quaterniond(req.move_to_pose_param.task_frame.quat[0],
                                    req.move_to_pose_param.task_frame.quat[1],
                                    req.move_to_pose_param.task_frame.quat[2],
                                    req.move_to_pose_param.task_frame.quat[3]);

      // target pose
      target_pos = Vector3d::Map(req.move_to_pose_param.target_pose.pos.data());
      // target_quat = Quaterniond(req.move_to_pose_param.target_pose.quat.data());
      target_quat = Quaterniond(req.move_to_pose_param.target_pose.quat[0],
                                req.move_to_pose_param.target_pose.quat[1],
                                req.move_to_pose_param.target_pose.quat[2],
                                req.move_to_pose_param.target_pose.quat[3]);
      // desired force
      fd = Vector6d::Map(req.move_to_pose_param.fd.data());

      out["task_frame_pos"] = task_frame_pos;
      out["task_frame_quat"] = task_frame_quat;
      out["target_pos"] = target_pos;
      out["target_quat"] = target_quat;
      out["target_force"] = fd;
      out["timeout"] = req.move_to_pose_param.timeout;
      out["speed_factor"] = req.move_to_pose_param.speed_factor;
      out["controller_gain_msg"] = req.move_to_pose_param.controller_gain;
    }
    else if (target_primitive_type_ == PrimitiveType::ConstantVelocity){
      Vector3d task_frame_pos;
      Quaterniond task_frame_quat;
      Vector6d move_dir, fd;

      // task frame
      task_frame_pos = Vector3d::Map(req.constant_velocity_param.task_frame.pos.data());
      // task_frame_quat = Quaterniond(req.constant_velocity_param.task_frame.quat.data());
      task_frame_quat = Quaterniond(req.constant_velocity_param.task_frame.quat[0],
                                    req.constant_velocity_param.task_frame.quat[1],
                                    req.constant_velocity_param.task_frame.quat[2],
                                  req.constant_velocity_param.task_frame.quat[3]);

      // desired force
      fd = Vector6d::Map(req.constant_velocity_param.fd.data());

      // move_dir
      move_dir = Vector6d::Map(req.constant_velocity_param.direction.data());

      out["task_frame_pos"] = task_frame_pos;
      out["task_frame_quat"] = task_frame_quat;
      out["direction"] = move_dir;
      out["target_force"] = fd;
      out["speed_factor"] = req.constant_velocity_param.speed_factor;
      out["timeout"] = req.constant_velocity_param.timeout;
      out["f_thresh"] = req.constant_velocity_param.f_thresh;
      out["controller_gain_msg"] = req.constant_velocity_param.controller_gain;
    }
    else if (target_primitive_type_ == PrimitiveType::AdmittanceMotion) {
      Vector6d fd, kd;
      kd = Vector6d::Map(req.admittance_motion_param.kd.data());
      fd = Vector6d::Map(req.admittance_motion_param.fd.data());

      out["kd"] = kd;
      out["fd"] = fd;
      out["timeout"] = req.admittance_motion_param.timeout;
      out["z_thresh"] = req.admittance_motion_param.z_thresh;
      out["controller_gain_msg"] = req.admittance_motion_param.controller_gain;

    }
    else if (target_primitive_type_ == PrimitiveType::Displacement){
      Vector3d task_frame_pos;
      Quaterniond task_frame_quat;
      Vector6d move_dir, fd;

      // task frame
      task_frame_pos = Vector3d::Map(req.displacement_param.task_frame.pos.data());
      // task_frame_quat = Quaterniond(req.displacement_param.task_frame.quat.data());
      task_frame_quat = Quaterniond(req.displacement_param.task_frame.quat[0],
                                    req.displacement_param.task_frame.quat[1],
                                    req.displacement_param.task_frame.quat[2],
                                  req.displacement_param.task_frame.quat[3]);

      // desired force
      fd = Vector6d::Map(req.displacement_param.fd.data());

      // move_dir
      move_dir = Vector6d::Map(req.displacement_param.direction.data());

      out["task_frame_pos"] = task_frame_pos;
      out["task_frame_quat"] = task_frame_quat;
      out["direction"] = move_dir;
      out["target_force"] = fd;
      out["speed_factor"] = req.displacement_param.speed_factor;
      out["timeout"] = req.displacement_param.timeout;
      out["f_thresh"] = req.displacement_param.f_thresh;
      out["displacement"] = req.displacement_param.displacement;
      out["controller_gain_msg"] = req.displacement_param.controller_gain;
    }
  }

  void MotionGenerator::ftSensorCallback(const geometry_msgs::WrenchStamped& msg) {
    ft_sensor_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  }

  void MotionGenerator::calc_ext_force(Vector6d& out, const Eigen::Matrix<double, 6, 7>& J, const Vector7d& tau_ext) {
    // Eigen::Matrix<double, 6, 7> J_trans_inv;
    Eigen::MatrixXd J_trans_inv;
    pseudoInverse(J_trans_inv, J.transpose(), kInvDamp);
    out = J_trans_inv * tau_ext;
  }
}
PLUGINLIB_EXPORT_CLASS(franka_motion_primitive::MotionGenerator,
                       controller_interface::ControllerBase)
