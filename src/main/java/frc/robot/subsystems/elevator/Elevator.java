//BEYONCEEEEEE
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants.CAN;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.io.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    L1(1),
    L2(2),
    L3(3),
    L4(4),
    Stow(5);
    

    public final double setpoint;

    private ElevatorPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private static final double CALIBRATION_SPEED = -0.1;
  private static final double HARD_STOP_CURRENT_LIMIT = 30;

  private static final double INCREMENT_CONSTANT = 1;
  private static final double DECREMENT_CONSTANT = 1;

  private static final double ELEVATOR_MOTOR_KP = 0.21; // 0.08
  private static final double ELEVATOR_MOTOR_KI = 0; // 0.001
  private static final double ELEVATOR_MOTOR_KD = 0; // 0.002
  private static final double ELEVATOR_PID_VEL = 120;
  private static final double ELEVATOR_PID_ACC = 125;

  private static final double ELEVATOR_MOTOR_TOLERANCE = 0.0;

  private static final double ELEVATOR_MAX_INCHES = 14.0;
  private static final double ELEVATOR_MAX_ROTATIONS = 28.67;

  private static final double MOTOR_CONVERSION = ELEVATOR_MAX_INCHES / ELEVATOR_MAX_ROTATIONS;

  // In pounds
  private static final double ELEVATOR_STAGE1_WEIGHT = 3.75;
  private static final double ELEVATOR_STAGE2_WEIGHT = 0;

  // In inches
  private static final double SPOOL_DIAMETER = 1.6;

  // In pound - inches
  private static final double STALL_TORQUE = 62.75;

  // In amps
  private static final double STALL_CURRENT = 366;

  // In Ohms
  private static final double RESISTANCE = 0.0185;

  private static final double GEAR_RATIO = 7.75;
  private static final double NUMBER_OF_MOTORS = 2;

  private static final double FEEDFORWARD_CONSTANT = ((ELEVATOR_STAGE1_WEIGHT + (2 * ELEVATOR_STAGE2_WEIGHT))
      * SPOOL_DIAMETER * STALL_CURRENT * RESISTANCE) / (NUMBER_OF_MOTORS * GEAR_RATIO * STALL_TORQUE);

  private boolean _isCalibrated;
  private ElevatorPosition _currentPos;

  private ProfiledPIDController _elevatorMotorPID;

  private ElevatorIO _io;
  private ElevatorIOInputs _inputs;

  public Elevator(ElevatorIO io) {
    _io = io;
    _inputs = new ElevatorIOInputs();

    _elevatorMotorPID = new ProfiledPIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD,
        new Constraints(ELEVATOR_PID_VEL, ELEVATOR_PID_ACC));
    _elevatorMotorPID.setTolerance(ELEVATOR_MOTOR_TOLERANCE);
    _elevatorMotorPID.setGoal(0.25);
  }

  // Runs the motors down at the calibration speed
  public void runMotorsDown() {
    _io.setElevatorSpeed(CALIBRATION_SPEED);
  }

  public void stopMotors() {
    _io.setElevatorSpeed(0);
  }

  public void setSetpoint(ElevatorPosition setpoint) {
    setSetpoint(setpoint.setpoint);
    _currentPos = setpoint;
  }

  // Sets the setpoint of the PID
  public void setSetpoint(double setpoint) {
    if (setpoint < 0.25 || setpoint > ELEVATOR_MAX_INCHES)
      return;
    _elevatorMotorPID.setGoal(setpoint);
    _elevatorMotorPID.reset(getCurrentPosInches());
  }

  // Checks if it is at the setpoint
  public boolean isAtSetpoint() {
    return _elevatorMotorPID.atSetpoint();
  }

  // Increases elevator position
  public void incrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getGoal().position + INCREMENT_CONSTANT);
  }

  // Decreases elevator position
  public void decrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getGoal().position - DECREMENT_CONSTANT);
  }

  // Checks if above limit
  public boolean isAboveCurrentLimit() {
    return _inputs._elevatorMotorCurrent > HARD_STOP_CURRENT_LIMIT;
  }

  // Sets the encoder values to 0
  public void resetEncoders() {
    _io.resetEncoder();
    _elevatorMotorPID.reset(getCurrentPosInches()); // TODO: Make it so this runs when auto and teleop init
  }

  // Runs motors with PID
  public void runMotorsWithPID() {
    _io.setElevatorSpeed(_elevatorMotorPID.calculate(getCurrentPosInches()));
  }

  public boolean isCalibrated() {
    return _isCalibrated;
  }

  public void setIsCalibrated(boolean isCalibrated) {
    _isCalibrated = isCalibrated;
  }

  // Converts the motors position from weird units to normal people inches
  public double getCurrentPosInches() {
    return _inputs._elevatorPosition * MOTOR_CONVERSION;
  }

  public ElevatorPosition getCurrentPos() {
    return _currentPos;
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);

    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/currentPos", getCurrentPosInches());
    Logger.recordOutput("Elevator/targetPos", _elevatorMotorPID.getGoal().position);
    Logger.recordOutput("Elevator/isAboveCurrentLimit", isAboveCurrentLimit());
    Logger.recordOutput("Elevator/Current", _inputs._elevatorMotorCurrent);
    Logger.recordOutput("Elevator/motorSpeed", _inputs._elevatorSpeed);
    Logger.recordOutput("Elevator/CurrentSetpoint", _elevatorMotorPID.getSetpoint().position);

    // Logger.recordOutput("Elevator/motorSpeed", _primaryMotor.get());
  }

  public boolean canMoveToPos(ElevatorPosition currentElevator, ArmPosition desiredArm) {
    switch (currentElevator) {
      case L1:
      case L2:
      case L3:
        return (desiredArm != ArmPosition.L4_Score) || (desiredArm != ArmPosition.Loading);
      case L4:
        return desiredArm != ArmPosition.Loading;
      case Stow:
        return desiredArm != ArmPosition.L4_Score;
      default:
        return false;
    }
  }
}
