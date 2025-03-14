//BEYONCEEEEEE
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    L1(5, 5),
    L2(11, 20),
    L3(27, 38),
    L4(52, 52),
    Stow(0.5, 0.5),
    Manual(0, 0);

    double coralHeight, algaeHeight;

    ElevatorPosition(double coralHeight, double algaeHeight) {
      this.coralHeight = coralHeight;
      this.algaeHeight = algaeHeight;
    }

    double getHeight(GamepieceMode mode) {
      return (mode == GamepieceMode.ALGAE) ? this.algaeHeight : this.coralHeight;
    }
  }

  private static final double CALIBRATION_SPEED = -0.1;
  private static final double HARD_STOP_CURRENT_LIMIT = 37;

  private static final double INCREMENT_CONSTANT = 1;
  private static final double DECREMENT_CONSTANT = 1;

  private static final double ELEVATOR_MOTOR_KP = 0.8; //0.75;
  private static final double ELEVATOR_MOTOR_KI = 0;//0.15; 
  private static final double ELEVATOR_MOTOR_KD = 0;
  private static final double ELEVATOR_PID_VEL = 220;
  private static final double ELEVATOR_PID_ACC = 215;

  private static final double ELEVATOR_MAX_INCHES = 52; //48;
  private static final double ELEVATOR_MAX_ROTATIONS = 87; // 36.4;

  private static final double MOTOR_CONVERSION = ELEVATOR_MAX_INCHES / ELEVATOR_MAX_ROTATIONS;

  private static final double FEEDFORWARD_CONSTANT = 0.225;

  private boolean _isCalibrated;
  private boolean _atSetpoint;
  private ElevatorPosition _currentPos;
  private ElevatorPosition _desiredPos;
  private ElevatorPosition _prevPos;
  private MultiSubsystemCommands.GamepieceMode _currentMode;

  private ProfiledPIDController _elevatorMotorPID;

  private ElevatorIO _io;
  private ElevatorIOInputsAutoLogged _inputs;

  public Elevator(ElevatorIO io) {
    _io = io;
    _inputs = new ElevatorIOInputsAutoLogged();

    _elevatorMotorPID = new ProfiledPIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD,
        new Constraints(ELEVATOR_PID_VEL, ELEVATOR_PID_ACC));
    _elevatorMotorPID.setTolerance(0.5);

    _currentPos = ElevatorPosition.Stow;
    _desiredPos = ElevatorPosition.Stow;
    _prevPos = ElevatorPosition.Stow;
  }

  // Runs the motors down at the calibration speed
  public void runMotorsDown() {
    _io.setElevatorSpeed(CALIBRATION_SPEED);
  }

  public void stopMotors() {
    _io.setElevatorSpeed(0);
  }

  public void setElevatorVoltage(Voltage voltage) {
    _io.setElevatorVoltage(voltage);
  }

  public void setSetpoint(ElevatorPosition setpoint) {
    _prevPos = _currentPos;
    _desiredPos = setpoint;
    setSetpoint(setpoint.getHeight(_currentMode));
  }

  // Sets the setpoint of the PID
  public void setSetpoint(double setpoint) {
    if (setpoint < 0.25 || setpoint > ELEVATOR_MAX_INCHES)
      return;
    _atSetpoint = false;
    _elevatorMotorPID.reset(getCurrentPosInches());
    _elevatorMotorPID.setGoal(setpoint);
  }

  // Checks if it is at the setpoint
  public boolean isAtSetpoint() {
    boolean atSetpoint = Math.abs(_elevatorMotorPID.getGoal().position - getCurrentPosInches()) <= 0.5;
    if (atSetpoint) {
      _currentPos = _desiredPos;
      _atSetpoint = true;
    }
    return atSetpoint;
  }

  // Increases elevator position
  public void incrementElevatorPosition() {
    _currentPos = ElevatorPosition.Manual;
    setSetpoint(_elevatorMotorPID.getGoal().position + INCREMENT_CONSTANT);
  }

  // Decreases elevator position
  public void decrementElevatorPosition() {
    _currentPos = ElevatorPosition.Manual;
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
    _io.setElevatorVoltage(Voltage.ofBaseUnits(_elevatorMotorPID.calculate(getCurrentPosInches()) + FEEDFORWARD_CONSTANT, Volt));
  }

  public boolean isCalibrated() {
    return _isCalibrated;
  }

  public void setIsCalibrated(boolean isCalibrated) {
    _isCalibrated = isCalibrated;
  }

  // Converts the motors position from weird units to normal people inches
  public double getCurrentPosInches() {
    return (_inputs._elevatorPosition * MOTOR_CONVERSION) - 0.25;
  }

  public ElevatorPosition getCurrentPos() {
    return _currentPos;
  }

  public ElevatorPosition getPrevPos() {
    return _prevPos;
  }

  public GamepieceMode getCurrentMode() {
    return _currentMode;
  }

  public void setCurrentMode(GamepieceMode mode) {
    _currentMode = mode;
  }

  public double getElevatorMaxHeight() {
    return ELEVATOR_MAX_INCHES;
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);
    Logger.processInputs("Elevator", _inputs);

    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/currentPos", getCurrentPosInches());
    Logger.recordOutput("Elevator/targetPos", _elevatorMotorPID.getGoal().position);
    Logger.recordOutput("Elevator/isAboveCurrentLimit", isAboveCurrentLimit());
    Logger.recordOutput("Elevator/Current", _inputs._elevatorMotorCurrent);
    Logger.recordOutput("Elevator/motorSpeed", _inputs._elevatorSpeed);
    Logger.recordOutput("Elevator/CurrentSetpoint", _elevatorMotorPID.getSetpoint().position);
    Logger.recordOutput("Elevator/atSetpoint", isAtSetpoint());
    Logger.recordOutput("Elevator/currentPosEnum", _currentPos);
    Logger.recordOutput("Elevator/desiredPosEnum", _desiredPos);
    Logger.recordOutput("Elevator/currentGamepieceMode", _currentMode);
  }

}
