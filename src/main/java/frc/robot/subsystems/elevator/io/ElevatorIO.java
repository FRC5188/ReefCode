package frc.robot.subsystems.elevator.io;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
       public double _elevatorMotorCurrent = 0;
       public double _elevatorPosition = 0;
       public double _elevatorSpeed = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorSpeed(double speed) {}

    public default void resetEncoder() {}
}
