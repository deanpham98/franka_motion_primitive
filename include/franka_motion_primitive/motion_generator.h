#include <memory>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_controllers/HybridControllerCommand.h>
#include <franka_controllers/Gain.h>

#include <franka_motion_primitive/primitive.h>
#include <franka_motion_primitive/PrimitiveState.h>
#include <franka_motion_primitive/RunPrimitiveCommand.h>
#include <franka_motion_primitive/MotionGeneratorState.h>
#include <geometry_msgs/WrenchStamped.h>
// service
#include <franka_motion_primitive/SetInitialForce.h>
#include <franka_motion_primitive/RunPrimitive.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_motion_primitive/AdmittanceConfig.h>

// ft sensor
#include <hardware_interface/force_torque_sensor_interface.h>

namespace franka_motion_primitive {
  class MotionGenerator : public controller_interface::MultiInterfaceController<
                                    franka_hw::FrankaModelInterface,
                                    franka_hw::FrankaStateInterface> {

  /**
   *  - Store reconfigurable primitives classes (main_primitive_)
   *  - Publish command to the controller through /${CONTROLLER_NAME}/command
   *
   *  Publishes to
   *  - \b  /hybrid_controller/command (franka_controllers::HybridControllerCommand)
   *        publish command to the controller
   *
   *  - \b  /hybrid_controller/gain_config (franka_controllers::Gain)
   *        set gain for the controller
   *
   *  - \b  /motion_generator/state (franka_motion_primitive::MotionGeneratorState)
   *        publish current state of the motion generator
   *
   *  Services
   *  - \b  /motion_generator/set_initial_force (franka_motion_primitive::SetInitialForce)
   *        set the offset for the external force torque signals
   *
   *  - \b  /motion_generator/run_primitive (franka_motion_primitive::RunPrimitive)
   *        execute primitives
   *
   *
   */
  public:
    bool init(hardware_interface::RobotHW* robot_hw,
              ros::NodeHandle& nh) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    // read robot state
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    // ft sensor to ee pos
    // Vector3d p_ee_ft_ =(Vector3d() << 0, 0, -0.069).finished();
    // Quaterniond q_ee_ft_ = Quaterniond(0.7071, 0., 0, 0.7071);
    Vector3d p_ee_ft_ =(Vector3d() << 0, 0, -0.069).finished();
    // Quaterniond q_ee_ft_ = Quaterniond(0.7071, 0., 0, 0.7071);
    Quaterniond q_ee_ft_ = Quaterniond(1., 0., 0, 0);
    // subscribe to read force
    ros::Subscriber sub_ft_sensor_;
    void ftSensorCallback(const geometry_msgs::WrenchStamped& msg);
    Vector6d ft_sensor_;
    bool kFtSubscribeInit;

    // external force estimation
    const double kInvDamp = 0.2;
    void calc_ext_force(Vector6d& out, const Eigen::Matrix<double, 6, 7>& J, const Vector7d& tau_ext);
    Vector6d f0_ext_est_;

    Pose task_frame_;

    // motion primitive execution time
    double t_exec_;

    // publish controller command
    realtime_tools::RealtimePublisher<
      franka_controllers::HybridControllerCommand> pub_controller_;

    // publish controller command
    realtime_tools::RealtimePublisher<
      franka_controllers::Gain> pub_gain_;

    // subscribe to run a primitive
    ros::Subscriber sub_primitive_cmd_;
    void primitiveCommandCallback(const RunPrimitiveCommandConstPtr& msg);
    // convert msg to primitive param
    void msg2PrimitiveParam(
        ParamMap& out, const RunPrimitiveCommandConstPtr& msg);
    ParamMap primitive_param_;
    uint8_t target_primitive_type_;
    bool kReceiveCommand;
    // execution status of the current primitive
    Status execution_status_;

    // publish execution state
    realtime_tools::RealtimePublisher<MotionGeneratorState> pub_state_;
    // TODO: read from yaml file
    franka_hw::TriggerRate state_publish_rate_{200.0};

    // motion generator container for desired moition
    std::vector<std::shared_ptr<PrimitiveBase>> primitive_container_;
    std::shared_ptr<PrimitiveBase> main_primitive_;

    // total number of primitive types
    int no_primitive_types_;

    // offset for measured force
    Vector6d f0_;
    Vector6d f0_ee_;
    Vector6d f0_ss_;
    // filter force
    Vector6d f_filter_;
    Vector6d f_ee_filter_;
    double kCutoffFrequency{200.};
    double kDeltaT{0.001};
    double filter_gain_;

    // filter velocity
    Vector7d dq_filter_;
    double alpha_dq_{0.95};

    // Reset compensation force servce
    ros::ServiceServer set_initial_force_serv_;
    bool set_initial_force(SetInitialForce::Request& req,
                            SetInitialForce::Response& res );
    bool kSetInitialForce;

    // run primitive service
    ros::ServiceServer run_primitive_serv_;
    bool run_primitive(RunPrimitive::Request& req,
                      RunPrimitive::Response& res);
    bool kRunPrimitiveReq;

    void service2PrimitiveParam(ParamMap& out, RunPrimitive::Request& req);

    // dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<AdmittanceConfig>>
      dynamic_server_admittance_config_;

    ros::NodeHandle dynamic_admittance_node_;
    void AdmittanceDynamicReconfigureCallback(
            AdmittanceConfig& config,
            uint32_t level);

    // params received from dynamic server
    Vector6d kd_admittance_dynamic_;   //
    Vector6d kp_dynamic_;
    Vector6d kd_dynamic_;
    bool kReceiveDynamicServer;

    // compliant frame, used to set the initial compliant frame
    std::shared_ptr<CompliantFrame> c_frame_;
};
} // namespace
