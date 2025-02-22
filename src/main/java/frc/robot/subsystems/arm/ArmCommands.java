package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmCommands {
    private Arm _arm;

    public ArmCommands(Arm arm) {
        _arm = arm;
    }

    public Command spit() {
        return new StartEndCommand(
                () -> {
                    if (_arm.getCurrentPos() == ArmPosition.L4_Score) 
                        _arm.setArmSetpoint(ArmPosition.Stow);
                    _arm.spit();
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                    _arm.clearHasGamepiece();
                },
                _arm).withTimeout(0.5);

    }

    public Command setArmPosition(ArmPosition setpoint) {
        if (setpoint == ArmPosition.L4_Score) {
            return new InstantCommand(
                () -> {
                    _arm.setArmSetpoint(setpoint);
                },
                _arm).andThen(intakeForNumberOfRotations());
        }
        return new InstantCommand(
                () -> {
                    _arm.setArmSetpoint(setpoint);
                },
                _arm);
    }

    public Command intake() {
        return new StartEndCommand(
                () -> {
                    _arm.setIntakeSpeed(0.45);
                    _arm.setFeederSpeed(0.45);
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                    _arm.setFeederSpeed(0);
                },
                _arm).until(() -> _arm.hasPiece());
    }

    public Command moveGamepieceToLightSensor() {
        return new StartEndCommand(
            () -> {
                _arm.setIntakeSpeed(-0.3);
            },
            () -> {
                _arm.setIntakeSpeed(0);
            },
            _arm).until(() -> _arm.lightSensorSeesGamepiece());       
    }

    public Command runArmPID() {
        return Commands.run(() -> {
            _arm.runArmPID();
        });
    }

    public Command intakeForNumberOfRotations() {
        return new StartEndCommand(() -> {
            _arm.resetIntakeEncoders();
            _arm.setIntakeSpeed(-0.3);
        },
        () -> {
            _arm.setIntakeSpeed(0);
        },
        _arm).until(() -> _arm.intakeAtDesiredRotations());
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(_arm::armAtSetpoint);
    }
}
