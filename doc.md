This package aims to provide primitives implementation based on the simulated counterparts (located in `~/n/simulation/experiment/pl-torque/pl_sim`). The requirements are
    - Must work with any controllers that receive "hybrid" command as inputs (pose + force)
    - Decoupled from the controller

The ideas for implementation is based on `ros_control` framework
    - Each primitive "type" (or control mode) is a separate `controller` object in the `ros_control` framework.

    - All primitives' type in a "library" should be "loaded" before execution with the "common" parameters - ones that are the same for any primtiives belong to that type in the library. To be more precise, consider each primitive to have 2 types of paramteres, one is "fixed" - shared with other primitives in the same library, other is "dynamic" - can be changed during execution

    - each real controller/primtiive is the unique representation for all primitives in a primitive family in the library

## `controller_manager` package


## to write a new primitive
- start from writing test and ros_interface
- then process to implement the ros subscriber callback in motion_generator
- add to primitive_container in motion_generator
- implement `configure()` function of the primitive
- implement the primiitve


## 25/7
- State of the primtive framework
    - TODO write constant velocity primitive
    - TODO admittance motion primitives
    - TODO how to send a sequence of primitive with different parametres through ROS?
        - use action_server
    - TODO integrate with trained policy from simulation

- Less priority (if have time)
    - ease writing new controller
    - how to test without hardware?

## 26/7
- the `zeroJacobian` in `libfranka` is neither space or body Jacobian mentioned in the book "modern robotics". It is the one relating `v^b_e` to `dotq`, where `b` stand for base frame and `e` stand for end-effector frame

- check whether K_F_ext_hat_k = R * O_F_ext_K
- change control of interest point ?

# 29
- DONE gravity compensation (internal of the libfranka)
- filter force
- DONE data collection / analysis
- DONE reset compensation force service
- dynamic reconfigure to tune params
-

# 31
- IDEA build a common "franka state handle" for whatever (controller, ...) to use it when needed


# 2/8
- why measured torque with respect to K_frame is less noisy than w.r.t 0_frame?
    - because the torque w.r.t o_frame is at the origin of the 0Frame, so the noise of the linear force exarcebate the torque in 0_frame.
    - NOTE that the `O_F_ext_hat_K` should be `O_F_ext_hat_O`

- why the len of the measured signal is different although they are recorded by the same publisher? need to lock the data and wait until complete writing?

- need modify
    - since `O_F_ext_hat_K` is noisy -> use `K_F_ext_hat_K` instead
    - TODO check zeroJacobian: what does it map?
        - if zeroJacobian is J_K^0, then we don't need to change much, we only need to modify the "filtered force estimation" and the "law in admittanceMotion"

        - otherwise the old control law is incorrect and much works need to be done to modify both the control law and admittance motion

        - IDEA to test Jacobian: J_K^O = [R, 0; 0, R] * J_K^K -> compare with zeroJacobian

# 3/8
- can't fully insert reason
    - biased torque is wrong

- tradeoff: damping increase -> increase unstability, but smoother motion

- kd_rot up to 0.9 still stable and the robot is very light

- NOTE for improve:
    - implement service instead of publisher??

- is the insertino successful because of large insertion force or because the admittance law?

- is the external force estimation scheme on franka accurate?

# 4/8
- test fixed sequence with square and triangle peg
- parameter tuning?
- find clearance by experiment (unknown error due to kinematic error)

# 5/8
- experiment 6/8
    - run sequence for square and triangle peg
    - estimate clearance for square and triangle peg-hole
        - first test whether saver saves properly in estimate_clearance() function

# 6/8
- success for square but not for triangle
- ~1mm clearance for triangle, 0.5mm for square
