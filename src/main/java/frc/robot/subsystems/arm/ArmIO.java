package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double _armMotorCurrent = 0.0;
        public double _armMotorVoltage = 0.0;
        public double _armMotorSpeed = 0.0;

        public double _feederMotorCurrent = 0.0;
        public double _feederMotorVoltage = 0.0;
        public double _feederMotorSpeed = 0.0;

        public double _armEncoderPositionDegrees = 0.0;
        public double _armEncoderVelocity = 0.0;

        public double _intakeMotorPositionRotations = 0.0;
        public double _intakeMotorVelocityRotationsPerMin = 0.0;
        public double _intakeMotorCurrent = 0.0;
        public double _intakeMotorVoltage = 0.0;

        public boolean _lightSensorState = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setIntakeMotorSpeed(double speed) {}

    public default void setArmMotorSpeed(double speed) {}

    public default void resetIntakeEncoders() {}

    public default void setArmMotorVoltage(Voltage voltage) {}

    public default void setFeederMotorSpeed(double speed) {}
}
