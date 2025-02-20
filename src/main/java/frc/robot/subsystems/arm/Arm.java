package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.arm.io.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  public enum ArmPosition {

    Stow(40),
    Loading(30);

    double angle;
    ArmPosition(double angle) {
      this.angle = angle;
    }
  }

  public enum ArmMode {
    Coral,
    Algae;
  }

  private ArmIO _io;
  private ArmIOInputs _inputs;
  private boolean _prevLightSensorVal;
  private ArmMode _armMode;

  private ProfiledPIDController _armPidController;
  
  private static final double KP = 0;
  private static final double KI = 0;
  private static final double KD = 0;
  private static final double PROFILE_VEL = 0;
  private static final double PROFILE_ACC = 0;
  private static final double HAS_ALGAE_CURRENT = 2;

  public Arm(ArmIO io) {
    _io = io;
    _inputs = new ArmIOInputs();

    _armPidController = new ProfiledPIDController(KP, KI, KD, new Constraints(PROFILE_VEL, PROFILE_ACC));
    _armMode = ArmMode.Coral;
  }

  public void setArmSetpoint(ArmPosition setpoint) {
    _armPidController.setGoal(setpoint.angle);
  }

  public void setIntakeSpeed(double speed) {
    _io.setIntakeMotorSpeed(speed);
  }
  
  public boolean hasPiece() {
    boolean hasPiece;
    if (_armMode == ArmMode.Coral) {
      boolean currentState = _inputs._lightSensorState;
      hasPiece = _prevLightSensorVal && !currentState;
      _prevLightSensorVal = currentState;
    } else {
      hasPiece = _inputs._intakeMotorCurrent >= HAS_ALGAE_CURRENT;
    }
    
    return hasPiece;
}

  @Override
  public void periodic() {
    
  }

}
