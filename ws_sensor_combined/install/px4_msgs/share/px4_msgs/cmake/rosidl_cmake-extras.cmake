# generated from rosidl_cmake/cmake/rosidl_cmake-extras.cmake.in

set(px4_msgs_IDL_FILES "msg/ActionRequest.idl;msg/ActuatorArmed.idl;msg/ActuatorControlsStatus.idl;msg/ActuatorMotors.idl;msg/ActuatorOutputs.idl;msg/ActuatorServos.idl;msg/ActuatorServosTrim.idl;msg/ActuatorTest.idl;msg/AdcReport.idl;msg/Airspeed.idl;msg/AirspeedValidated.idl;msg/AirspeedWind.idl;msg/ArmingCheckReply.idl;msg/ArmingCheckRequest.idl;msg/AutotuneAttitudeControlStatus.idl;msg/BatteryStatus.idl;msg/Buffer128.idl;msg/ButtonEvent.idl;msg/CameraCapture.idl;msg/CameraStatus.idl;msg/CameraTrigger.idl;msg/CanInterfaceStatus.idl;msg/CellularStatus.idl;msg/CollisionConstraints.idl;msg/CollisionReport.idl;msg/ConfigOverrides.idl;msg/ControlAllocatorStatus.idl;msg/Cpuload.idl;msg/DatamanRequest.idl;msg/DatamanResponse.idl;msg/DebugArray.idl;msg/DebugKeyValue.idl;msg/DebugValue.idl;msg/DebugVect.idl;msg/DifferentialPressure.idl;msg/DistanceSensor.idl;msg/DistanceSensorModeChangeRequest.idl;msg/Ekf2Timestamps.idl;msg/EscReport.idl;msg/EscStatus.idl;msg/EstimatorAidSource1d.idl;msg/EstimatorAidSource2d.idl;msg/EstimatorAidSource3d.idl;msg/EstimatorBias.idl;msg/EstimatorBias3d.idl;msg/EstimatorEventFlags.idl;msg/EstimatorGpsStatus.idl;msg/EstimatorInnovations.idl;msg/EstimatorSelectorStatus.idl;msg/EstimatorSensorBias.idl;msg/EstimatorStates.idl;msg/EstimatorStatus.idl;msg/EstimatorStatusFlags.idl;msg/Event.idl;msg/FailsafeFlags.idl;msg/FailureDetectorStatus.idl;msg/FigureEightStatus.idl;msg/FlightPhaseEstimation.idl;msg/FollowTarget.idl;msg/FollowTargetEstimator.idl;msg/FollowTargetStatus.idl;msg/FuelTankStatus.idl;msg/GeneratorStatus.idl;msg/GeofenceResult.idl;msg/GeofenceStatus.idl;msg/GimbalControls.idl;msg/GimbalDeviceAttitudeStatus.idl;msg/GimbalDeviceInformation.idl;msg/GimbalDeviceSetAttitude.idl;msg/GimbalManagerInformation.idl;msg/GimbalManagerSetAttitude.idl;msg/GimbalManagerSetManualControl.idl;msg/GimbalManagerStatus.idl;msg/GotoSetpoint.idl;msg/GpioConfig.idl;msg/GpioIn.idl;msg/GpioOut.idl;msg/GpioRequest.idl;msg/GpsDump.idl;msg/GpsInjectData.idl;msg/Gripper.idl;msg/HealthReport.idl;msg/HeaterStatus.idl;msg/HomePosition.idl;msg/HoverThrustEstimate.idl;msg/InputRc.idl;msg/InternalCombustionEngineStatus.idl;msg/IridiumsbdStatus.idl;msg/IrlockReport.idl;msg/LandingGear.idl;msg/LandingGearWheel.idl;msg/LandingTargetInnovations.idl;msg/LandingTargetPose.idl;msg/LaunchDetectionStatus.idl;msg/LedControl.idl;msg/LogMessage.idl;msg/LoggerStatus.idl;msg/MagWorkerData.idl;msg/MagnetometerBiasEstimate.idl;msg/ManualControlSetpoint.idl;msg/ManualControlSwitches.idl;msg/MavlinkLog.idl;msg/MavlinkTunnel.idl;msg/MessageFormatRequest.idl;msg/MessageFormatResponse.idl;msg/Mission.idl;msg/MissionResult.idl;msg/ModeCompleted.idl;msg/MountOrientation.idl;msg/NavigatorMissionItem.idl;msg/NavigatorStatus.idl;msg/NormalizedUnsignedSetpoint.idl;msg/NpfgStatus.idl;msg/ObstacleDistance.idl;msg/OffboardControlMode.idl;msg/OnboardComputerStatus.idl;msg/OpenDroneIdArmStatus.idl;msg/OpenDroneIdOperatorId.idl;msg/OpenDroneIdSelfId.idl;msg/OpenDroneIdSystem.idl;msg/OrbTest.idl;msg/OrbTestLarge.idl;msg/OrbTestMedium.idl;msg/OrbitStatus.idl;msg/ParameterResetRequest.idl;msg/ParameterSetUsedRequest.idl;msg/ParameterSetValueRequest.idl;msg/ParameterSetValueResponse.idl;msg/ParameterUpdate.idl;msg/Ping.idl;msg/PositionControllerLandingStatus.idl;msg/PositionControllerStatus.idl;msg/PositionSetpoint.idl;msg/PositionSetpointTriplet.idl;msg/PowerButtonState.idl;msg/PowerMonitor.idl;msg/PpsCapture.idl;msg/PwmInput.idl;msg/Px4ioStatus.idl;msg/QshellReq.idl;msg/QshellRetval.idl;msg/RadioStatus.idl;msg/RateCtrlStatus.idl;msg/RcChannels.idl;msg/RcParameterMap.idl;msg/RegisterExtComponentReply.idl;msg/RegisterExtComponentRequest.idl;msg/RoverAckermannGuidanceStatus.idl;msg/RoverAckermannStatus.idl;msg/RoverDifferentialGuidanceStatus.idl;msg/RoverDifferentialSetpoint.idl;msg/RoverDifferentialStatus.idl;msg/RoverMecanumGuidanceStatus.idl;msg/RoverMecanumSetpoint.idl;msg/RoverMecanumStatus.idl;msg/Rpm.idl;msg/RtlStatus.idl;msg/RtlTimeEstimate.idl;msg/SatelliteInfo.idl;msg/SensorAccel.idl;msg/SensorAccelFifo.idl;msg/SensorAirflow.idl;msg/SensorBaro.idl;msg/SensorCombined.idl;msg/SensorCorrection.idl;msg/SensorGnssRelative.idl;msg/SensorGps.idl;msg/SensorGyro.idl;msg/SensorGyroFft.idl;msg/SensorGyroFifo.idl;msg/SensorHygrometer.idl;msg/SensorMag.idl;msg/SensorOpticalFlow.idl;msg/SensorPreflightMag.idl;msg/SensorSelection.idl;msg/SensorUwb.idl;msg/SensorsStatus.idl;msg/SensorsStatusImu.idl;msg/SystemPower.idl;msg/TakeoffStatus.idl;msg/TaskStackInfo.idl;msg/TecsStatus.idl;msg/TelemetryStatus.idl;msg/TiltrotorExtraControls.idl;msg/TimesyncStatus.idl;msg/TrajectoryBezier.idl;msg/TrajectorySetpoint.idl;msg/TrajectoryWaypoint.idl;msg/TransponderReport.idl;msg/TuneControl.idl;msg/UavcanParameterRequest.idl;msg/UavcanParameterValue.idl;msg/UlogStream.idl;msg/UlogStreamAck.idl;msg/UnregisterExtComponent.idl;msg/VehicleAcceleration.idl;msg/VehicleAirData.idl;msg/VehicleAngularAccelerationSetpoint.idl;msg/VehicleAngularVelocity.idl;msg/VehicleAttitude.idl;msg/VehicleAttitudeSetpoint.idl;msg/VehicleCommand.idl;msg/VehicleCommandAck.idl;msg/VehicleConstraints.idl;msg/VehicleControlMode.idl;msg/VehicleGlobalPosition.idl;msg/VehicleImu.idl;msg/VehicleImuStatus.idl;msg/VehicleLandDetected.idl;msg/VehicleLocalPosition.idl;msg/VehicleLocalPositionSetpoint.idl;msg/VehicleMagnetometer.idl;msg/VehicleOdometry.idl;msg/VehicleOpticalFlow.idl;msg/VehicleOpticalFlowVel.idl;msg/VehicleRatesSetpoint.idl;msg/VehicleRoi.idl;msg/VehicleStatus.idl;msg/VehicleThrustSetpoint.idl;msg/VehicleTorqueSetpoint.idl;msg/VehicleTrajectoryBezier.idl;msg/VehicleTrajectoryWaypoint.idl;msg/VelocityLimits.idl;msg/VtolVehicleStatus.idl;msg/WheelEncoders.idl;msg/Wind.idl;msg/YawEstimatorStatus.idl;srv/VehicleCommand.idl")
set(px4_msgs_INTERFACE_FILES "msg/ActionRequest.msg;msg/ActuatorArmed.msg;msg/ActuatorControlsStatus.msg;msg/ActuatorMotors.msg;msg/ActuatorOutputs.msg;msg/ActuatorServos.msg;msg/ActuatorServosTrim.msg;msg/ActuatorTest.msg;msg/AdcReport.msg;msg/Airspeed.msg;msg/AirspeedValidated.msg;msg/AirspeedWind.msg;msg/ArmingCheckReply.msg;msg/ArmingCheckRequest.msg;msg/AutotuneAttitudeControlStatus.msg;msg/BatteryStatus.msg;msg/Buffer128.msg;msg/ButtonEvent.msg;msg/CameraCapture.msg;msg/CameraStatus.msg;msg/CameraTrigger.msg;msg/CanInterfaceStatus.msg;msg/CellularStatus.msg;msg/CollisionConstraints.msg;msg/CollisionReport.msg;msg/ConfigOverrides.msg;msg/ControlAllocatorStatus.msg;msg/Cpuload.msg;msg/DatamanRequest.msg;msg/DatamanResponse.msg;msg/DebugArray.msg;msg/DebugKeyValue.msg;msg/DebugValue.msg;msg/DebugVect.msg;msg/DifferentialPressure.msg;msg/DistanceSensor.msg;msg/DistanceSensorModeChangeRequest.msg;msg/Ekf2Timestamps.msg;msg/EscReport.msg;msg/EscStatus.msg;msg/EstimatorAidSource1d.msg;msg/EstimatorAidSource2d.msg;msg/EstimatorAidSource3d.msg;msg/EstimatorBias.msg;msg/EstimatorBias3d.msg;msg/EstimatorEventFlags.msg;msg/EstimatorGpsStatus.msg;msg/EstimatorInnovations.msg;msg/EstimatorSelectorStatus.msg;msg/EstimatorSensorBias.msg;msg/EstimatorStates.msg;msg/EstimatorStatus.msg;msg/EstimatorStatusFlags.msg;msg/Event.msg;msg/FailsafeFlags.msg;msg/FailureDetectorStatus.msg;msg/FigureEightStatus.msg;msg/FlightPhaseEstimation.msg;msg/FollowTarget.msg;msg/FollowTargetEstimator.msg;msg/FollowTargetStatus.msg;msg/FuelTankStatus.msg;msg/GeneratorStatus.msg;msg/GeofenceResult.msg;msg/GeofenceStatus.msg;msg/GimbalControls.msg;msg/GimbalDeviceAttitudeStatus.msg;msg/GimbalDeviceInformation.msg;msg/GimbalDeviceSetAttitude.msg;msg/GimbalManagerInformation.msg;msg/GimbalManagerSetAttitude.msg;msg/GimbalManagerSetManualControl.msg;msg/GimbalManagerStatus.msg;msg/GotoSetpoint.msg;msg/GpioConfig.msg;msg/GpioIn.msg;msg/GpioOut.msg;msg/GpioRequest.msg;msg/GpsDump.msg;msg/GpsInjectData.msg;msg/Gripper.msg;msg/HealthReport.msg;msg/HeaterStatus.msg;msg/HomePosition.msg;msg/HoverThrustEstimate.msg;msg/InputRc.msg;msg/InternalCombustionEngineStatus.msg;msg/IridiumsbdStatus.msg;msg/IrlockReport.msg;msg/LandingGear.msg;msg/LandingGearWheel.msg;msg/LandingTargetInnovations.msg;msg/LandingTargetPose.msg;msg/LaunchDetectionStatus.msg;msg/LedControl.msg;msg/LogMessage.msg;msg/LoggerStatus.msg;msg/MagWorkerData.msg;msg/MagnetometerBiasEstimate.msg;msg/ManualControlSetpoint.msg;msg/ManualControlSwitches.msg;msg/MavlinkLog.msg;msg/MavlinkTunnel.msg;msg/MessageFormatRequest.msg;msg/MessageFormatResponse.msg;msg/Mission.msg;msg/MissionResult.msg;msg/ModeCompleted.msg;msg/MountOrientation.msg;msg/NavigatorMissionItem.msg;msg/NavigatorStatus.msg;msg/NormalizedUnsignedSetpoint.msg;msg/NpfgStatus.msg;msg/ObstacleDistance.msg;msg/OffboardControlMode.msg;msg/OnboardComputerStatus.msg;msg/OpenDroneIdArmStatus.msg;msg/OpenDroneIdOperatorId.msg;msg/OpenDroneIdSelfId.msg;msg/OpenDroneIdSystem.msg;msg/OrbTest.msg;msg/OrbTestLarge.msg;msg/OrbTestMedium.msg;msg/OrbitStatus.msg;msg/ParameterResetRequest.msg;msg/ParameterSetUsedRequest.msg;msg/ParameterSetValueRequest.msg;msg/ParameterSetValueResponse.msg;msg/ParameterUpdate.msg;msg/Ping.msg;msg/PositionControllerLandingStatus.msg;msg/PositionControllerStatus.msg;msg/PositionSetpoint.msg;msg/PositionSetpointTriplet.msg;msg/PowerButtonState.msg;msg/PowerMonitor.msg;msg/PpsCapture.msg;msg/PwmInput.msg;msg/Px4ioStatus.msg;msg/QshellReq.msg;msg/QshellRetval.msg;msg/RadioStatus.msg;msg/RateCtrlStatus.msg;msg/RcChannels.msg;msg/RcParameterMap.msg;msg/RegisterExtComponentReply.msg;msg/RegisterExtComponentRequest.msg;msg/RoverAckermannGuidanceStatus.msg;msg/RoverAckermannStatus.msg;msg/RoverDifferentialGuidanceStatus.msg;msg/RoverDifferentialSetpoint.msg;msg/RoverDifferentialStatus.msg;msg/RoverMecanumGuidanceStatus.msg;msg/RoverMecanumSetpoint.msg;msg/RoverMecanumStatus.msg;msg/Rpm.msg;msg/RtlStatus.msg;msg/RtlTimeEstimate.msg;msg/SatelliteInfo.msg;msg/SensorAccel.msg;msg/SensorAccelFifo.msg;msg/SensorAirflow.msg;msg/SensorBaro.msg;msg/SensorCombined.msg;msg/SensorCorrection.msg;msg/SensorGnssRelative.msg;msg/SensorGps.msg;msg/SensorGyro.msg;msg/SensorGyroFft.msg;msg/SensorGyroFifo.msg;msg/SensorHygrometer.msg;msg/SensorMag.msg;msg/SensorOpticalFlow.msg;msg/SensorPreflightMag.msg;msg/SensorSelection.msg;msg/SensorUwb.msg;msg/SensorsStatus.msg;msg/SensorsStatusImu.msg;msg/SystemPower.msg;msg/TakeoffStatus.msg;msg/TaskStackInfo.msg;msg/TecsStatus.msg;msg/TelemetryStatus.msg;msg/TiltrotorExtraControls.msg;msg/TimesyncStatus.msg;msg/TrajectoryBezier.msg;msg/TrajectorySetpoint.msg;msg/TrajectoryWaypoint.msg;msg/TransponderReport.msg;msg/TuneControl.msg;msg/UavcanParameterRequest.msg;msg/UavcanParameterValue.msg;msg/UlogStream.msg;msg/UlogStreamAck.msg;msg/UnregisterExtComponent.msg;msg/VehicleAcceleration.msg;msg/VehicleAirData.msg;msg/VehicleAngularAccelerationSetpoint.msg;msg/VehicleAngularVelocity.msg;msg/VehicleAttitude.msg;msg/VehicleAttitudeSetpoint.msg;msg/VehicleCommand.msg;msg/VehicleCommandAck.msg;msg/VehicleConstraints.msg;msg/VehicleControlMode.msg;msg/VehicleGlobalPosition.msg;msg/VehicleImu.msg;msg/VehicleImuStatus.msg;msg/VehicleLandDetected.msg;msg/VehicleLocalPosition.msg;msg/VehicleLocalPositionSetpoint.msg;msg/VehicleMagnetometer.msg;msg/VehicleOdometry.msg;msg/VehicleOpticalFlow.msg;msg/VehicleOpticalFlowVel.msg;msg/VehicleRatesSetpoint.msg;msg/VehicleRoi.msg;msg/VehicleStatus.msg;msg/VehicleThrustSetpoint.msg;msg/VehicleTorqueSetpoint.msg;msg/VehicleTrajectoryBezier.msg;msg/VehicleTrajectoryWaypoint.msg;msg/VelocityLimits.msg;msg/VtolVehicleStatus.msg;msg/WheelEncoders.msg;msg/Wind.msg;msg/YawEstimatorStatus.msg;srv/VehicleCommand.srv;srv/VehicleCommand_Request.msg;srv/VehicleCommand_Response.msg")
