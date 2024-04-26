# Cybergear Socketcan Driver Node Parameters

Default Config
```yaml
cybergear_socketcan_driver_node:
  ros__parameters:
    anguler_effort:
      max: 12.0
      min: -12.0
    anguler_position:
      max: 12.56637061
      min: -12.56637061
    anguler_velocity:
      max: 30.0
      min: -30.0
    command_topic_type: trajectory
    device_id: 127.0
    joint_name: cybergear
    pid_gain:
      kd: 0.0475
      kp: 0.3
    pid_gain_range:
      kd:
        max: 5.0
        min: 0.0
      kp:
        max: 500.0
        min: 0.0
    primary_id: 0.0
    send_frequency: 16.0
    temperature:
      scale: 0.1
    torque_constant: 0.87
    update_param_frequency: 1.0

```

## primary_id

Identifier parameter for distinguishing the sender in CAN communication

* Type: `int`
* Default Value: 0

*Constraints:*
 - parameter must be within bounds 0

## device_id

Identifier parameter for specifying the recipient in CAN communication (CyberGear ID)

* Type: `int`
* Default Value: 127

*Constraints:*
 - parameter must be within bounds 0

## send_frequency

Transmission interval parameter for CAN frame (Hz)

* Type: `double`
* Default Value: 16.0
* Read only: True

*Constraints:*
 - greater than 0.0

## update_param_frequency

Cycle for reflecting parameter changes during node execution (Hz)

* Type: `double`
* Default Value: 1.0
* Read only: True

*Constraints:*
 - greater than 0.0

## joint_name

Joint name parameter used in JointTrajectory and JointState

* Type: `string`
* Default Value: "cybergear"
* Read only: True

*Constraints:*
 - parameter is not empty

## command_topic_type

Parameter for selecting either JointTrajectory or single target

* Type: `string`
* Default Value: "trajectory"
* Read only: True

*Constraints:*
 - one of the specified values: ['trajectory', 'setpoint']
 - parameter is not empty

## pid_gain.kp

Proportional gain parameter for PID control

* Type: `double`
* Default Value: 0.3

## pid_gain.kd

Derivative gain parameter for PID control

* Type: `double`
* Default Value: 0.0475

## torque_constant

Torque constant used in current control

* Type: `double`
* Default Value: 0.87
* Read only: True

## temperature.scale

DO NOT CHANGE; CyberGear communication data conversion factor to Celsius

* Type: `double`
* Default Value: 0.1
* Read only: True

## anguler_position.max

DO NOT CHANGE; Upper limit of position during CyberGear communication

* Type: `double`
* Default Value: 12.56637061
* Read only: True

## anguler_position.min

DO NOT CHANGE; Lower limit of position during CyberGear communication

* Type: `double`
* Default Value: -12.56637061
* Read only: True

## anguler_velocity.max

DO NOT CHANGE; Upper limit of velocity during CyberGear communication

* Type: `double`
* Default Value: 30.0
* Read only: True

## anguler_velocity.min

DO NOT CHANGE; Lower limit of velocity during CyberGear communication

* Type: `double`
* Default Value: -30.0
* Read only: True

## anguler_effort.max

DO NOT CHANGE; Upper limit of effort during CyberGear communication

* Type: `double`
* Default Value: 12.0
* Read only: True

## anguler_effort.min

DO NOT CHANGE; Lower limit of effort during CyberGear communication

* Type: `double`
* Default Value: -12.0
* Read only: True

## pid_gain_range.kp.max

DO NOT CHANGE; Upper limit of proportional gain during CyberGear communication

* Type: `double`
* Default Value: 500.0
* Read only: True

## pid_gain_range.kp.min

DO NOT CHANGE; Lower limit of proportional gain during CyberGear communication

* Type: `double`
* Default Value: 0.0
* Read only: True

## pid_gain_range.kd.max

DO NOT CHANGE; Upper limit of derivative gain during CyberGear communication

* Type: `double`
* Default Value: 5.0
* Read only: True

## pid_gain_range.kd.min

DO NOT CHANGE; Lower limit of derivative gain during CyberGear communication

* Type: `double`
* Default Value: 0.0
* Read only: True

