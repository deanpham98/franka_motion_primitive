#include <boost/bind.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <franka_motion_primitive/motion_generator.h>
#include <franka_motion_primitive/PrimitiveType.h>

namespace franka_motion_primitive{

  bool MotionGenerator::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) {
    // TODO: controller ns config
    std::string controller_ns, controller_cmd_topic, gain_config_topic;
    controller_ns = "/hybrid_controller";
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
    no_primitive_types_ = 3; // default value

    // primitive container
    primitive_container_.resize(no_primitive_types_);
    primitive_container_[PrimitiveType::MoveToPose] =
      std::make_shared<franka_motion_primitive::MoveToPose>();

    primitive_container_[PrimitiveType::ConstantVelocity] =
      std::make_shared<franka_motion_primitive::ConstantVelocity>();

    primitive_container_[PrimitiveType::AdmittanceMotion] =
      std::make_shared<franka_motion_primitive::AdmittanceMotion>();

    // set main primitive to move to pose primtiive
    main_primitive_ = primitive_container_[PrimitiveType::MoveToPose];
    // main_primitive_ = primitive_container_[PrimitiveType::ConstantVelocity];

    // pubs and subs
    sub_primitive_cmd_ = nh.subscribe("run_primitive", 1,
        &MotionGenerator::primitiveCommandCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    pub_controller_.init(nh, controller_cmd_topic, 1);
    pub_state_.init(nh, "state", 1);
    pub_gain_.init(nh, gain_config_topic, 1);

    // service
    set_initial_force_serv_ =
      nh.advertiseService<SetInitialForce::Request, SetInitialForce::Response>
        ("set_initial_force", boost::bind(&MotionGenerator::set_initial_force, this, _1, _2));

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

    // setting primitive
    main_primitive_->configure(primitive_param_);

    // initial filter velocity
    dq_filter_ = Vector7d::Map(robot_state.dq.data());
    // initial external force for compensation
    f0_ = Vector6d::Map(robot_state.O_F_ext_hat_K.data());
    f0_ee_ = Vector6d::Map(robot_state.K_F_ext_hat_K.data());

    // filter force
    f_filter_ = f0_;
    f_ee_filter_ = f0_ee_;
    // filter gain
    filter_gain_ = kDeltaT / (kDeltaT + 1/(2*M_PI*kCutoffFrequency));
    std::cout << filter_gain_ <<std::endl;

    target_primitive_type_ = PrimitiveType::MoveToPose;
    execution_status_ = Status::EXECUTING;
    kReceiveCommand = false;
    kSetInitialForce = false;
  }

  void MotionGenerator::update(const ros::Time& t_now, const ros::Duration& period) {
    // configure primtiive
    if (kReceiveCommand == true && execution_status_ != Status::EXECUTING) {
      // TODO: set controller param (publish to hybrid_controller)

      // configure primitive
      main_primitive_ = primitive_container_[target_primitive_type_];
      main_primitive_->configure(primitive_param_);
      kReceiveCommand = false;
    }

    // admittance param
    if (kReceiveDynamicServer) {
      // TODO set controller gain
      if (pub_gain_.trylock()){
        pub_gain_.msg_.kDefineDamping = 1;
        for (size_t i = 0; i < 6; i++){
          pub_gain_.msg_.kp[i] = kp_dynamic_(i);
          pub_gain_.msg_.kd[i] = kd_dynamic_(i);
        }
        pub_gain_.unlockAndPublish();
      }
      main_primitive_->SetAdmittanceGain(kd_admittance_dynamic_);
      kReceiveDynamicServer = false;
    }

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

    // update initial force
    if (kSetInitialForce) {
      f0_ = f_filter_;
      f0_ee_ = f_ee_filter_;
      kSetInitialForce = false;
    }

    // update internal state
    State s;
    s.pose.p = p;
    s.pose.q = q;
    s.v = J*dq_filter_;   // base twist
    s.f = f_filter_ - f0_;
    s.f_ee = f_ee_filter_ - f0_ee_;

    // update control u = f(s, t)
    ControlSignal cmd;
    main_primitive_->update_control(cmd, execution_status_, s, period.toSec());

    // publish to controller
    if (pub_controller_.trylock()){
      for (size_t i =0; i <3; ++i){
        pub_controller_.msg_.f[i] = cmd.f(i);
        pub_controller_.msg_.f[i+3] = cmd.f(i+3);
        pub_controller_.msg_.S[i] = cmd.S(i, i);
        pub_controller_.msg_.S[i+3] = cmd.S(i+3, i+3);
        pub_controller_.msg_.v[i] = cmd.v(i);
        pub_controller_.msg_.v[i+3] = cmd.v(i+3);
        pub_controller_.msg_.p[i] = cmd.pose.p[i];
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
        pub_state_.msg_.f_filter[i] = s.f[i];
        pub_state_.msg_.f[i] = f[i] - f0_[i];
        pub_state_.msg_.f_ee[i] = f_ee[i];
      }
      pub_state_.unlockAndPublish();
    }

  }

  void MotionGenerator::primitiveCommandCallback(const RunPrimitiveCommandConstPtr& msg){
    ROS_INFO("receive command");

    target_primitive_type_ = msg->type;
    std::cout << int(msg->type) << std::endl;
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
}
PLUGINLIB_EXPORT_CLASS(franka_motion_primitive::MotionGenerator,
                       controller_interface::ControllerBase)
