cmake_minimum_required(VERSION 3.8)
project(qarl_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionRegime.msg"
  "msg/BoolArrayStamped.msg"
  "msg/BoolStampedSource.msg"
  "msg/BoolStamped.msg"
  "msg/BoschRexrothControl.msg"
  "msg/BoschRexrothFeedback31.msg"
  "msg/BoschRexrothFeedback33.msg"
  "msg/Buttons.msg"
  "msg/ConnectionsQuality.msg"
  "msg/Control.msg"
  "msg/ControlStatus.msg"
  "msg/ControlStep.msg"
  "msg/EngineState.msg"
  "msg/FloatStamped.msg"
  "msg/Header.msg"
  "msg/IboxAnalogIn.msg"
  "msg/IboxPedalsGear.msg"
  "msg/IboxWAngleFeedback.msg"
  "msg/Load.msg"
  "msg/LoaderActuatorFeedback.msg"
  "msg/LoaderRemoteControl.msg"
  "msg/LoadFinished.msg"
  "msg/LoadingAction.msg"
  "msg/Meatball.msg"
  "msg/NmeaGGA.msg"
  "msg/Orientation.msg"
  "msg/OrientationStamped.msg"
  "msg/PedalsGear.msg"
  "msg/PlannerStatus.msg"
  "msg/PlannerStoppageReport.msg"
  "msg/PlatformControl.msg"
  "msg/PlatformState.msg"
  "msg/Position.msg"
  "msg/PositionStamped.msg"
  "msg/Quality.msg"
  "msg/QualityStamped.msg"
  "msg/RemoteAction.msg"
  "msg/RemoteControl.msg"
  "msg/RemoteControlCommand.msg"
  "msg/RemoteState.msg"
  "msg/SteeringWheelControl.msg"
  "msg/SteeringWheelFeedback.msg"
  "msg/TransmissionContactorsFeedback.msg"
  "msg/TransmissionControl.msg"
  "msg/TransmissionState200.msg"
  "msg/TransmissionStateFF20.msg"
  "msg/Uint8MultiArrayExetended.msg"
  "msg/Speed.msg"
  "msg/SpeedLimit.msg"
  "msg/SpeedStamped.msg"
  "msg/Unload.msg"
  "msg/UnloadingAction.msg"
  "msg/VehicleBehaviorMode.msg"
  "msg/WAngleFeedback.msg"
  "msg/WangleFiltered.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    common_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()