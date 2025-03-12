package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;
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
                _arm).withTimeout(1);

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
        return Commands.either(intakeAlgae(), intakeCoral(), () -> _arm.getCurrentMode() == GamepieceMode.ALGAE);
    }

    private Command intakeCoral() {
        return new StartEndCommand(
                () -> {
                        _arm.setIntakeSpeed(0.4);
                },
                () -> {
                        _arm.setIntakeSpeed(0);
                },
                _arm).until(() -> _arm.hasPiece());
    }

    private Command intakeAlgae() {
        return new Command() {
            double intakeSpikeCounter = 0;

            @Override
            public void initialize() {
                intakeSpikeCounter = 0;
                _arm.setIntakeSpeed(0.5);
            }

            @Override
            public void execute() {
                if (_arm.getIntakeCurrent() >= Arm.HAS_ALGAE_CURRENT) {
                    intakeSpikeCounter++;
                }
            }

            @Override
            public void end(boolean interrupted) {
                _arm.setIntakeSpeed(0.05);
            }

            @Override
            public boolean isFinished() {
                return intakeSpikeCounter > 15;
            }
        };
    }

    public Command moveGamepieceToLightSensor() {
        return new StartEndCommand(
            () -> {
                _arm.setIntakeSpeed(-0.1);
            },
            () -> {
                _arm.setIntakeSpeed(0);
            },
            _arm).until(() -> _arm.lightSensorSeesGamepiece()).unless(() -> _arm.lightSensorSeesGamepiece());       
    }

    public Command runArmPID() {
        return Commands.run(() -> {
            _arm.runArmPID();
        });
    }

    public Command intakeForNumberOfRotations() {
        return new StartEndCommand(() -> {
            _arm.resetIntakeEncoders();
            _arm.setIntakeSpeed(-0.1);
        },
        () -> {
            _arm.setIntakeSpeed(0);
        },
        _arm).until(() -> _arm.intakeAtDesiredRotations());
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(_arm::isAtSetpoint);
    }
}
