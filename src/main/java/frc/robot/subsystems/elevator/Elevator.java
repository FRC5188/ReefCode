// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    Middle(7);

    public final double setpoint;

    private ElevatorPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private static final double CALIBRATION_SPEED = -0.05;
  private static final double HARD_STOP_CURRENT_LIMIT = 30;
  private static final double INCREMENT_CONSTANT = 1;
  private static final double DECREMENT_CONSTANT = 1;
  private static final double ELEVATOR_MOTOR_KP = 0.01;
  private static final double ELEVATOR_MOTOR_KI = 0.0;
  private static final double ELEVATOR_MOTOR_KD = 0.0;

  private static final double ELEVATOR_MOTOR_TOLERANCE = 0.0;

  private static final double ELEVATOR_MAX_INCHES = 14.0;
  //private static final double ELEVATOR_MAX_ROTATIONS = 28.67;

  //private static final double MOTOR_CONVERSION = ELEVATOR_MAX_INCHES / ELEVATOR_MAX_ROTATIONS;

  private boolean _isCalibrated;
  public SparkFlex _primaryMotor;
  public SparkFlex _secondaryMotor;

  private PIDController _elevatorMotorPID;

  SparkFlex flex = new SparkFlex(1, MotorType.kBrushless);
  public com.revrobotics.spark.config.SparkFlexConfig SparkFlexConfig() {
      return null;}
 

  public Elevator() {

    _elevatorMotorPID = new PIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD);
    _elevatorMotorPID.setTolerance(ELEVATOR_MOTOR_TOLERANCE);

    // Declares both motors

    _primaryMotor = new SparkFlex(1, MotorType.kBrushless);
    _secondaryMotor = new SparkFlex(2, MotorType.kBrushless);

    SparkFlexConfig config = SparkFlexConfig();
    config
      .inverted(true)
      .follow(1);

    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Sets the primary to lead the secondary
    //_leadMotor.(new Follower(2, _isCalibrated));
    // _secondaryMotor.setControl(new Follower(_primaryMotor.getDeviceID(), false));
    //_secondaryMotor.follow(_leadMotor);
  }

  // Runs the motors down at the calibration speed
  public void runMotorsDown() {
    _primaryMotor.set(CALIBRATION_SPEED);
  }

  public void stopMotors() {
    _primaryMotor.set(0);
  }

  public void setSetpoint(ElevatorPosition setpoint) {
    setSetpoint(setpoint.setpoint);
  }

  // Sets the setpoint of the PID
  public void setSetpoint(double setpoint) {
    if (setpoint < 0 || setpoint > ELEVATOR_MAX_INCHES)
      return;
    _elevatorMotorPID.setSetpoint(setpoint);
  }

  // Checks if it is at the setpoint
  public boolean isAtSetpoint() {
    return _elevatorMotorPID.atSetpoint();
  }

  // Increases elevator position
  public void incrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getSetpoint() + INCREMENT_CONSTANT);
  }

  // Decreases elevator position
  public void decrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getSetpoint() - DECREMENT_CONSTANT);
  }

  // Checks if above limit
  public boolean isAboveCurrentLimit() {
    return _primaryMotor.getOutputCurrent() > HARD_STOP_CURRENT_LIMIT;
    //return _primaryMotor.getStatorCurrent().getValueAsDouble() > HARD_STOP_CURRENT_LIMIT;
  }

  // Sets the encoder values to 0
  public void resetEncoders() {
    _primaryMotor.getEncoder().setPosition(0);
    //_primaryMotor.setPosition(0);
  }

  // Runs motors with PID
  public void runMotorsWithPID() {
    // _primaryMotor.set(_elevatorMotorPID.calculate(getCurrentPosInches()));
  }

  public boolean isCalibrated() {
    return _isCalibrated;
  }

  public void setIsCalibrated(boolean isCalibrated) {
    _isCalibrated = isCalibrated;
  }

  // Converts the motors position from weird units to normal people inches
  public double getCurrentPosInches() {
    return _primaryMotor.getEncoder().getPosition();
    //return _primaryMotor.getPosition().getValue().magnitude() * MOTOR_CONVERSION;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/currentPos", getCurrentPosInches());
    Logger.recordOutput("Elevator/targetPos", _elevatorMotorPID.getSetpoint());
    Logger.recordOutput("Elevator/isAboveCurrentLimit", isAboveCurrentLimit());
    Logger.recordOutput("Elevator/Current", _primaryMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/motorSpeed", _elevatorMotorPID.calculate(getCurrentPosInches()));

    // Logger.recordOutput("Elevator/motorSpeed", _primaryMotor.get());
  }
}
