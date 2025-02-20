package frc.robot.subsystems.arm.io;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.HardwareConstants.CAN;

public class RealArmIO implements ArmIO {
    private SparkFlex _armMotor;
    private SparkMax _feederMotor;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);
        _feederMotor = new SparkMax(CAN.FEEDER_MTR_ID, MotorType.kBrushless);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        inputs._feederMotorSpeed = _feederMotor.get();
        inputs._feederMotorCurrent = _armMotor.getOutputCurent();
        inputs._feederMotorVoltage = _armMotor.getAppliedOutput() * _feederMotor.getBusVoltage();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }
    
    public void setFeederMotorSpeed(double speed) {
        _feederMotor.set(speed);
    }
}
