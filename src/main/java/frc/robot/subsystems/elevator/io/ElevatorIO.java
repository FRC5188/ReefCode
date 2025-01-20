package frc.robot.subsystems.elevator.io;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ArmIOInputs {
       
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    
}
