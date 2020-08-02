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
#include <franka_motion_primitive/SetInitialForce.h>
#include <franka_motion_primitive/MotionGeneratorState.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_motion_primitive/AdmittanceConfig.h>

namespace franka_motion_primitive {
  class MotionGenerator : public controller_interface::MultiInterfaceController<
                                    franka_hw::FrankaModelInterface,
                                    franka_hw::FrankaStateInterface> {
  /**
   *  - Store library of motion primitives
   *  - Publish command to the controller through /${CONTROLLER_NAME}/command
   *  - receive a sequence and execute it
   *  - receive a primitive and execute it
   */
  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    // read robot state
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

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
    franka_hw::TriggerRate state_publish_rate_{500.0};

    // motion generator container for desired moition
    std::vector<std::shared_ptr<PrimitiveBase>> primitive_container_;
    std::shared_ptr<PrimitiveBase> main_primitive_;

    // total number of primitive types
    int no_primitive_types_;

    // offset for measured force
    Vector6d f0_;
    Vector6d f0_ee_;
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
};
} // namespace
