package frc.robot.subsystems.arm.io;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareConstants.CAN;
import frc.robot.HardwareConstants.DIO;

public class RealArmIO implements ArmIO {
    private SparkFlex _armMotor;
    private DigitalInput _lightSensor;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);

        _lightSensor = new DigitalInput(DIO.LIGHT_SENSOR_CHANNEL);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        inputs._lightSensorState = !_lightSensor.get();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }
}
