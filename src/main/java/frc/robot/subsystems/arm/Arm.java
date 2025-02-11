package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    Stow(80),
    Loading(120),
    L4_Score(45);

    double angle;

    ArmPosition(double angle) {
      this.angle = angle;
    }
  }

  public final Trigger _hasPiece = new Trigger(() -> hasPiece());

  private ArmIO _io;
  private ArmIOInputsAutoLogged _inputs;
  private boolean _prevLightSensorVal;

  private ProfiledPIDController _armPidController;

  private static final double KP = 0.08;
  private static final double KI = 0;
  private static final double KD = 0;
  private static final double PROFILE_VEL = 140;
  private static final double PROFILE_ACC = 125;

  private static final double ARM_WEIGHT_KG = 4;
  private static final double ARM_STALL_TORQUE_NM = 3.6;
  private static final double ARM_STALL_CURRENT = 211;
  private static final double ARM_KT = ARM_STALL_TORQUE_NM / ARM_STALL_CURRENT;
  private static final double ARM_RESISTANCE = 0.057;
  private static final double ARM_MOMENT_METERS = 0.3928;
  private static final double ARM_GEARING = 17;
  private static final double ARM_FEEDFORWARD_COEFF = (ARM_WEIGHT_KG * ARM_MOMENT_METERS * ARM_RESISTANCE) / (ARM_KT * ARM_GEARING);

  SysIdRoutine routine = new SysIdRoutine(new Config(),
      new SysIdRoutine.Mechanism(this::setArmVoltage, this::populateLog, this));

  public Arm(ArmIO io) {
    _io = io;
    _inputs = new ArmIOInputsAutoLogged();

    _armPidController = new ProfiledPIDController(KP, KI, KD, new Constraints(PROFILE_VEL, PROFILE_ACC));
  }

  public void setArmSetpoint(ArmPosition setpoint) {
    _armPidController.reset(_inputs._armEncoderPositionDegrees);
    _armPidController.setGoal(setpoint.angle);
  }

  public void setIntakeSpeed(double speed) {
    _io.setIntakeMotorSpeed(speed);
  }

  public void setArmVoltage(Voltage voltage) {
    _io.setArmMotorVoltage(voltage);
  }

  public void resetIntakeEncoders() {
    _io.resetIntakeEncoders();
  }

  public boolean intakeAtDesiredRotations() {
    return _inputs._intakeMotorPositionRotations >= 2;
  }

  public boolean hasPiece() {
    boolean current = _inputs._lightSensorState;
    boolean hasPiece = _prevLightSensorVal && !current;
    _prevLightSensorVal = current;
    return hasPiece;
  }

  public boolean armAtSetpoint() {
    return _armPidController.atGoal();
  }

  public void runArmPID() {
    double out = _armPidController.calculate(_inputs._armEncoderPositionDegrees) + (ARM_FEEDFORWARD_COEFF * Math.cos(Units.degreesToRadians(_inputs._armEncoderPositionDegrees)));
    Logger.recordOutput("Arm/motorSpeed", out);
    _io.setArmMotorVoltage(Voltage.ofBaseUnits(out, Volt));
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
  }
}
