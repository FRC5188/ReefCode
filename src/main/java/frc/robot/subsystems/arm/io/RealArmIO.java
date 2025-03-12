package frc.robot.subsystems.arm.io;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareConstants.CAN;
import frc.robot.HardwareConstants.DIO;

public class RealArmIO implements ArmIO {

    private static final double POS_AT_90 = 0.711;
    private static final double POS_AT_0 = 0.458;
    private static final double ENCODER_CONVERSION = (POS_AT_90 - POS_AT_0) / 90.0;
    private static final double LASERCAN_DISTANCE = 40;

    private double INTAKE_ROTATION_CONVERSION = 1;

    private SparkFlex _armMotor;
    private LaserCan _laserCan;
    private SparkFlex _intakeMotor;
    private SparkAbsoluteEncoder _armEncoder;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);
        _intakeMotor = new SparkFlex(CAN.INTAKE_MTR_ID, MotorType.kBrushless);
        _armEncoder = _armMotor.getAbsoluteEncoder();

        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(true);
        armConfig.voltageCompensation(12);
        _armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _laserCan = new LaserCan(0);
        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        try {
            _laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            _laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            _laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        LaserCan.Measurement measurement = _laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) 
            inputs._lightSensorState = measurement.distance_mm <= LASERCAN_DISTANCE;
        inputs._intakeMotorVelocityRotationsPerMin = _intakeMotor.get();
        inputs._intakeMotorCurrent = _intakeMotor.getOutputCurrent();
        inputs._intakeMotorVoltage = _intakeMotor.getAppliedOutput() * _armMotor.getBusVoltage();
        inputs._intakeMotorPositionRotations = _intakeMotor.getEncoder().getPosition() * INTAKE_ROTATION_CONVERSION;

        inputs._armEncoderPositionDegrees = (_armEncoder.getPosition() - POS_AT_0) / ENCODER_CONVERSION;
        inputs._armEncoderVelocity = _armEncoder.getVelocity();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }

    public void setIntakeMotorSpeed(double speed) {
        _intakeMotor.set(speed);
    }

    public void resetIntakeEncoders() {
        _intakeMotor.getEncoder().setPosition(0);
    }

    public void setArmMotorVoltage(Voltage voltage) {
        _armMotor.setVoltage(voltage);
    }

}
