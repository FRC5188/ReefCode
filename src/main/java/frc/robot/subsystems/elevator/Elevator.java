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
import frc.robot.subsystems.arm.io.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    Middle(7),
    Top(12);

    public final double setpoint;

    private ElevatorPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private static final double CALIBRATION_SPEED = -0.1;
  private static final double HARD_STOP_CURRENT_LIMIT = 30;
  
  private static final double INCREMENT_CONSTANT = 1;
  private static final double DECREMENT_CONSTANT = 1;

  private static final double ELEVATOR_MOTOR_KP = 0.21; //0.08
  private static final double ELEVATOR_MOTOR_KI = 0; //0.001
  private static final double ELEVATOR_MOTOR_KD = 0; //0.002
  private static final double ELEVATOR_PID_VEL = 120;
  private static final double ELEVATOR_PID_ACC = 125;

  private static final double ELEVATOR_MOTOR_TOLERANCE = 0.0;

  private static final double ELEVATOR_MAX_INCHES = 14.0;
  private static final double ELEVATOR_MAX_ROTATIONS = 28.67;

  private static final double MOTOR_CONVERSION = ELEVATOR_MAX_INCHES / ELEVATOR_MAX_ROTATIONS;

  private boolean _isCalibrated;

  private ProfiledPIDController _elevatorMotorPID;

  private ElevatorIO _io;
  private ElevatorIOInputs _inputs;

  public Elevator(ElevatorIO io) {
    _io = io;
    _inputs = new ElevatorIOInputs();

    _elevatorMotorPID = new ProfiledPIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD, new Constraints(ELEVATOR_PID_VEL, ELEVATOR_PID_ACC));
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
}
