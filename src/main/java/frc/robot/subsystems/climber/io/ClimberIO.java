package frc.robot.subsystems.climber.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double _climberMotorSpeed = 0.0;
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberMotorSpeed(double speed) {}
}