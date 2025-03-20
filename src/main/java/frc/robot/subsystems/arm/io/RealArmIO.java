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

    private static final double POS_AT_90 = 0.422;
    private static final double POS_AT_0 = 0.168;
    private static final double ENCODER_CONVERSION = (POS_AT_90 - POS_AT_0) / 90.0;
    private static final double CORAL_LASERCAN_DISTANCE_MM = 50;
    private static final double ALGAE_LASERCAN_DISTANCE_MM = 50;

    private double INTAKE_ROTATION_CONVERSION = 1;

    private SparkFlex _armMotor;
    private LaserCan _upperLaserCan;
    private LaserCan _lowerLaserCan;
    private LaserCan _algaeLaserCan;
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

        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        _intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _upperLaserCan = new LaserCan(CAN.UPPER_CORAL_LASERCAN_ID);
        _lowerLaserCan = new LaserCan(CAN.LOWER_CORAL_LASERCAN_ID);
        _algaeLaserCan = new LaserCan(CAN.ALGAE_LASERCAN_ID);
        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        try {
            _upperLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            _upperLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            _lowerLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            _lowerLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            _algaeLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            _algaeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        LaserCan.Measurement upperMeasurement = _upperLaserCan.getMeasurement();
        if (upperMeasurement != null && upperMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            inputs._upperLightSensorState = upperMeasurement.distance_mm <= CORAL_LASERCAN_DISTANCE_MM;
        LaserCan.Measurement lowerMeasurement = _lowerLaserCan.getMeasurement();
        if (lowerMeasurement != null && lowerMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            inputs._lowerLightSensorState = lowerMeasurement.distance_mm <= CORAL_LASERCAN_DISTANCE_MM;
        LaserCan.Measurement algaeMeasurement = _algaeLaserCan.getMeasurement();
        if (algaeMeasurement != null && algaeMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            inputs._algaeLightSensorState = algaeMeasurement.distance_mm <= ALGAE_LASERCAN_DISTANCE_MM;

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
