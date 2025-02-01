package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ArmCommands {
    private Arm _arm;

    public ArmCommands(Arm arm) {
        _arm = arm;
    }

    public Command spit() {
        return new StartEndCommand(
                () -> {
                    _arm.setIntakeSpeed(0.5);
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                },
                _arm).withTimeout(1);
    }

    public Command setArmPosition(ArmPosition setpoint) {
        if (setpoint == ArmPosition.L4_Score) {
            return new InstantCommand(
                () -> {
                    _arm.setArmSetpoint(setpoint);
                },
                _arm).alongWith(intakeForNumberOfRotations());
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
                    _arm.setIntakeSpeed(0.5);
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                },
                _arm).until(() -> _arm.hasPiece());
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
}
