package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.arm.io.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  private ArmIO _io;
  private ArmIOInputs _inputs;
  private boolean _prevLightSensorVal;


  public Arm(ArmIO io) {
    _io = io;
    _inputs = new ArmIOInputs();
  }
  
  public boolean hasPiece() {
    boolean current = _inputs._lightSensorState;
    boolean hasPiece = _prevLightSensorVal && !current;
    _prevLightSensorVal = current;
    return hasPiece;
}

  @Override
  public void periodic() {
    
  }
}
