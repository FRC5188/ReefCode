package frc.robot.subsystems.elevator.io;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.HardwareConstants.CAN;
import frc.robot.subsystems.arm.io.ArmIO;
import frc.robot.subsystems.elevator.io.ElevatorIO.ElevatorIOInputs;

public class RealElevatorIO implements ElevatorIO {
    private SparkFlex _primaryMotor;
    private SparkFlex _secondaryMotor;

    public RealElevatorIO() {
        // Declares both motors
        _primaryMotor = new SparkFlex(CAN.PRIMARY_ELEVATOR_ID, MotorType.kBrushless);
        _secondaryMotor = new SparkFlex(CAN.SECONDARY_ELEVATOR_ID, MotorType.kBrushless);

        SparkFlexConfig primaryConfig = new SparkFlexConfig();
        primaryConfig.inverted(true);
        primaryConfig.idleMode(IdleMode.kCoast);
        _primaryMotor.configure(primaryConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig secondaryConfig = new SparkFlexConfig();
        secondaryConfig.follow(CAN.PRIMARY_ELEVATOR_ID);
        _secondaryMotor.configure(secondaryConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs._elevatorMotorCurrent = _primaryMotor.getOutputCurrent();
        inputs._elevatorPosition = _primaryMotor.getEncoder().getPosition();
        inputs._elevatorSpeed = _primaryMotor.get();
    }

    public void setElevatorSpeed(double speed) {
        _primaryMotor.set(speed);
    }

    public void resetEncoder() {
        _primaryMotor.getEncoder().setPosition(0);
    }
}
