package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.HardwareConstants.CAN;

public class RealElevatorIO implements ElevatorIO {
    private SparkFlex _primaryMotor;
    private SparkFlex _secondaryMotor;

    public RealElevatorIO() {
        // Declares both motors
        _primaryMotor = new SparkFlex(CAN.PRIMARY_ELEVATOR_ID, MotorType.kBrushless);
        _secondaryMotor = new SparkFlex(CAN.SECONDARY_ELEVATOR_ID, MotorType.kBrushless);

        SparkFlexConfig primaryConfig = new SparkFlexConfig();
        primaryConfig.inverted(false);
        primaryConfig.idleMode(IdleMode.kBrake);
        _primaryMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig secondaryConfig = new SparkFlexConfig();
        secondaryConfig.follow(CAN.PRIMARY_ELEVATOR_ID);
        secondaryConfig.idleMode(IdleMode.kBrake);
        _secondaryMotor.configure(secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs._elevatorMotorVoltage = _primaryMotor.getAppliedOutput() * _primaryMotor.getBusVoltage();
        inputs._elevatorMotorCurrent = _primaryMotor.getOutputCurrent();
        inputs._elevatorPosition = _primaryMotor.getEncoder().getPosition();
        inputs._elevatorSpeed = _primaryMotor.get();
        inputs._elevatorVelocity = _primaryMotor.getEncoder().getVelocity();
    }

    public void setElevatorSpeed(double speed) {
         _primaryMotor.set(speed);
    }

    public void setElevatorVoltage(Voltage voltage) {
        _primaryMotor.setVoltage(voltage);
    }

    public void resetEncoder() {
        _primaryMotor.getEncoder().setPosition(0);
    }
}
