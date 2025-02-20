package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.HardwareConstants.CAN;

public class RealClimberIO implements ClimberIO{
    private SparkFlex _climberMotor;

    public RealClimberIO() {
        _climberMotor = new SparkFlex(CAN.CLIMBER_MTR_ID, MotorType.kBrushless);

        SparkFlexConfig climberConfig = new SparkFlexConfig();
        climberConfig.idleMode(IdleMode.kBrake);
        _climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs._climberMotorSpeed = _climberMotor.get();
    }

    public void setClimberMotorSpeed(double speed) {
        _climberMotor.set(speed);
    }
}
