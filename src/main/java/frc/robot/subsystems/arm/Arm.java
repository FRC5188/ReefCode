package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.io.ArmIO;

public class Arm extends SubsystemBase {
  public enum ArmPosition {

    Stow(40),
    Loading(30);

    double angle;
    ArmPosition(double angle) {
      this.angle = angle;
    }
  }

  private ArmIO _io;

  private ProfiledPIDController _armPidController;
  
  private static final double KP = 0;
  private static final double KI = 0;
  private static final double KD = 0;
  private static final double PROFILE_VEL = 0;
  private static final double PROFILE_ACC = 0;

  public Arm(ArmIO io) {
    _io = io;

    _armPidController = new ProfiledPIDController(KP, KI, KD, new Constraints(PROFILE_VEL, PROFILE_ACC));
  }

  public void setArmSetpoint(ArmPosition setpoint) {
    _armPidController.setGoal(setpoint.angle);
  }

  public void setIntakeSpeed(double speed) {
    _io.setIntakeMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    
  }

}
