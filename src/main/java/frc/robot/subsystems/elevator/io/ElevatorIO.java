package frc.robot.subsystems.elevator.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double _elevatorMotorVoltage = 0;
        public double _elevatorMotorCurrent = 0;
        public double _elevatorPosition = 0;
        public double _elevatorSpeed = 0;
        public double _elevatorVelocity = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setElevatorSpeed(double speed) {
    }

    public default void setElevatorVoltage(Voltage voltage) {
    }

    public default void resetEncoder() {
    }
}
