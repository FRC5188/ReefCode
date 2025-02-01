package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.arm.io.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  public enum ArmPosition {

    Stow(40),
    Loading(30),
    L4_Score(45);

    double angle;
    ArmPosition(double angle) {
      this.angle = angle;
    }
  }

  private ArmIO _io;
  private ArmIOInputs _inputs;
  private boolean _prevLightSensorVal;


  private ProfiledPIDController _armPidController;
  
  private static final double KP = 0;
  private static final double KI = 0;
  private static final double KD = 0;
  private static final double PROFILE_VEL = 0;
  private static final double PROFILE_ACC = 0;

  public Arm(ArmIO io) {
    _io = io;
    _inputs = new ArmIOInputs();

    _armPidController = new ProfiledPIDController(KP, KI, KD, new Constraints(PROFILE_VEL, PROFILE_ACC));
    
  }

  public void setArmSetpoint(ArmPosition setpoint) {
    _armPidController.setGoal(setpoint.angle);
  }

  public void setIntakeSpeed(double speed) {
    _io.setIntakeMotorSpeed(speed);
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
    _io.setArmMotorSpeed(_armPidController.calculate(_inputs._armEncoderPositionDegrees));
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);
  }
}
