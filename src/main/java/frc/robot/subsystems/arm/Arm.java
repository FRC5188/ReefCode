package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.arm.io.ArmIOInputsAutoLogged;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    Stow(80),
    Loading_Coral(120),
    Loading_Algae(60),
    Loading(120),
    L4_Score(45),
    Algae_Score(60);

    double angle;

    ArmPosition(double angle) {
      this.angle = angle;
    }
  }

  public final Trigger _hasPiece = new Trigger(() -> hasPiece());

  private ArmIO _io;
  private ArmIOInputsAutoLogged _inputs;
  private boolean _prevLightSensorVal;
  private boolean _hasGamepiece;
  private int _intakeSpikeCounter;
  private ArmPosition _currentPos;
  private ArmPosition _desiredPos;
  private MultiSubsystemCommands.GamepieceMode _currentMode;

  private ProfiledPIDController _armPidController;

  private static final double KP = 0.01;//0.09;
  private static final double KI = 0; //0.01;
  private static final double KD = 0;
  private static final double PROFILE_VEL = 160;
  private static final double PROFILE_ACC = 145;

  private static final double HAS_ALGAE_CURRENT = 2;

  private static final double ARM_FEEDFORWARD_COEFF = 0.4;

  SysIdRoutine routine = new SysIdRoutine(new Config(),
      new SysIdRoutine.Mechanism(this::setArmVoltage, this::populateLog, this));

  public Arm(ArmIO io) {
    _io = io;
    _inputs = new ArmIOInputsAutoLogged();

    _armPidController = new ProfiledPIDController(KP, KI, KD, new Constraints(PROFILE_VEL, PROFILE_ACC));
    _armPidController.setTolerance(7);
  }

  public void setArmSetpoint(ArmPosition setpoint) {
    if (setpoint == ArmPosition.Loading)
      setpoint = (_currentMode == GamepieceMode.ALGAE) ? ArmPosition.Loading_Algae : ArmPosition.Loading_Coral;

    _armPidController.reset(_inputs._armEncoderPositionDegrees);
    _armPidController.setGoal(setpoint.angle);
    _desiredPos = setpoint;
  }

  public void setIntakeSpeed(double speed) {
    _io.setIntakeMotorSpeed(speed);
  }

  public void spit() {
    setIntakeSpeed(0.5);
  }

  public void clearHasGamepiece() {
    _hasGamepiece = false;
  }

  public void setArmVoltage(Voltage voltage) {
    _io.setArmMotorVoltage(voltage);
  }

  public void resetIntakeEncoders() {
    _io.resetIntakeEncoders();
  }

  public boolean intakeAtDesiredRotations() {
    return _inputs._intakeMotorPositionRotations <= -2;
  }

  public boolean hasPiece() {
    boolean hasPiece;
    if (_currentMode == GamepieceMode.CORAL) {
      boolean currentState = _inputs._lightSensorState;
      hasPiece = _prevLightSensorVal && !currentState;
      _prevLightSensorVal = currentState;
    } else {
      if (_inputs._intakeMotorCurrent >= HAS_ALGAE_CURRENT) {
        _intakeSpikeCounter++;
      }
      hasPiece = _intakeSpikeCounter >= 5;
    }

    if (hasPiece) _hasGamepiece = true;

    return _hasGamepiece;
  }

  public void resetIntakeSpikeCounter() {
    _intakeSpikeCounter = 0;
  }

  public boolean lightSensorSeesGamepiece() {
    return _inputs._lightSensorState;
  }

  public boolean armAtSetpoint() {
    boolean atSetpoint = _armPidController.atGoal();
    if (atSetpoint)
      _currentPos = _desiredPos;
    return atSetpoint;
  }

  public ArmPosition getCurrentPos() {
    return _currentPos;
  }

  public void runArmPID() {
    double out = _armPidController.calculate(_inputs._armEncoderPositionDegrees)
        + (ARM_FEEDFORWARD_COEFF * Math.cos(Units.degreesToRadians(_inputs._armEncoderPositionDegrees)));
    _io.setArmMotorVoltage(Voltage.ofBaseUnits(out, Volt));
  }

  public GamepieceMode getCurrentMode() {
    return _currentMode;
  }

  public void setCurrentMode(GamepieceMode mode) {
    _currentMode = mode;
  }

  public void populateLog(SysIdRoutineLog log) {
    log.motor("arm_motor")
        .voltage(Voltage.ofBaseUnits(_inputs._armMotorVoltage, Volt))
        .angularPosition(Angle.ofBaseUnits(_inputs._armEncoderPositionDegrees, Degrees))
        .angularVelocity(
            AngularVelocity.ofBaseUnits(_inputs._armEncoderVelocity, RPM));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);
    Logger.processInputs("Arm", _inputs);

    Logger.recordOutput("Arm/desiredPos", _armPidController.getSetpoint().position);
    Logger.recordOutput("Arm/hasPiece", hasPiece());
    Logger.recordOutput("Arm/atSetpoint", armAtSetpoint());
    Logger.recordOutput("Arm/currentPosEnum", _currentPos);
    Logger.recordOutput("Arm/desiredPosEnum", _desiredPos);
  }
}
