package frc.robot.subsystems.arm.io;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareConstants.CAN;
import frc.robot.HardwareConstants.DIO;

public class RealArmIO implements ArmIO {

    public double POS_AT_90 = 0.0;
    public double POS_AT_0 = 0.0;
    public double ENCODER_CONVERSION = (POS_AT_90 - POS_AT_0) * 90;
    
    private double INTAKE_ROTATION_CONVERSION = 1; 

    private SparkFlex _armMotor;
    private DigitalInput _lightSensor;
    private SparkMax _intakeMotor;
    private SparkAbsoluteEncoder _armEncoder;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);
        _intakeMotor = new SparkMax(CAN.INTAKE_MTR_ID, MotorType.kBrushless);
        _lightSensor = new DigitalInput(DIO.LIGHT_SENSOR_CHANNEL);
        _armEncoder = _intakeMotor.getAbsoluteEncoder();
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        inputs._lightSensorState = !_lightSensor.get();
        inputs._intakeMotorVelocityRotationsPerMin = _intakeMotor.get();
        inputs._intakeMotorCurrent = _intakeMotor.getOutputCurrent();
        inputs._intakeMotorVoltage = _intakeMotor.getAppliedOutput() * _armMotor.getBusVoltage();
        inputs._intakeMotorPositionRotations = _intakeMotor.getEncoder().getPosition() * INTAKE_ROTATION_CONVERSION; 
        
        inputs._armEncoderPositionDegrees = _armEncoder.getPosition() * ENCODER_CONVERSION;
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        _intakeMotor.set(speed);
    }

    public void resetIntakeEncoders() {
        _intakeMotor.getEncoder().setPosition(0);
    }

}
