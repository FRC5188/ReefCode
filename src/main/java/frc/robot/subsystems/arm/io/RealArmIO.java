package frc.robot.subsystems.arm.io;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.HardwareConstants.CAN;

public class RealArmIO implements ArmIO {
    private SparkFlex _armMotor;
    private SparkMax _intakeMotor;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);
        _intakeMotor = new SparkMax(CAN.INTAKE_MTR_ID, MotorType.kBrushless);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        inputs._intakeMotorVelocityRotationsPerMin = _intakeMotor.get();
        inputs._intakeMotorCurrent = _intakeMotor.getOutputCurrent();
        inputs._intakeMotorVoltage = _intakeMotor.getAppliedOutput() * _armMotor.getBusVoltage();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        _intakeMotor.set(speed);
    }
}
