package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.io.ArmIO;

public class Arm extends SubsystemBase {
  private ArmIO _io;

  public Arm(ArmIO io) {
    _io = io;
  }

  @Override
  public void periodic() {
    
  }
}
