package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmCommands {
    private Arm _arm;

    public ArmCommands(Arm arm) {
        _arm = arm;
    }

    public Command spit() {
        return new StartEndCommand(
                () -> {
                    _arm.spit();
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                    _arm.clearHasGamepiece();
                }, _arm).withTimeout(1)
                .andThen(Commands.runOnce(() -> _arm.setArmSetpoint(ArmPosition.Stow), _arm).unless(() -> _arm.getCurrentPos() != ArmPosition.L4_Score));
                

    }

    public Command setArmPosition(ArmPosition setpoint) {
        if (setpoint == ArmPosition.L4_Score) {
            return new InstantCommand(
                    () -> {
                        _arm.setArmSetpoint(setpoint);
                    }).andThen(intakeForNumberOfRotations());
        }
        return new InstantCommand(
                () -> {
                    _arm.setArmSetpoint(setpoint);
                });
    }

    public Command intake() {
        return Commands.either(intakeAlgae(), intakeCoralWithAdjust(),
                () -> _arm.getCurrentMode() == GamepieceMode.ALGAE);
    }

    private Command intakeCoralWithAdjust() {
        return intakeCoral()
                .andThen(new WaitCommand(0.25))
                .andThen(moveGamepieceToLightSensor());
    }

    private Command intakeCoral() {
        Command c = new Command() {

            @Override
            public void execute() {
                if (_arm.lowerLightSensorSeesGamepiece()) {
                    _arm.setIntakeSpeed(0.08);
                } else {
                    _arm.setIntakeSpeed(0.25); // 0.35
                }
            }

            @Override
            public void end(boolean interrupted) {
                _arm.setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished() {
                return _arm.hasPiece();
            }
        };
        c.addRequirements(_arm);
        return c;
    }

    private Command intakeAlgae() {
        Command c = new Command() {

            @Override
            public void initialize() {
                this.addRequirements(_arm);
                _arm.setIntakeSpeed(0.5);
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    _arm.setIntakeSpeed(0);
                } else {
                    _arm.setIntakeSpeed(0.05);
                }
            }

            @Override
            public boolean isFinished() {
                return _arm.hasPiece();
            }
        };
        c.addRequirements(_arm);
        return c;
    }

    public Command moveGamepieceToLightSensor() {
        return new Command() {

            @Override
            public void initialize() {
                // if we don't see the gamepiece in the middle, we assume that we don't have it
                // Cancel this command (and everything that follows it) by rescheduling intaking
                if (!_arm.lowerLightSensorSeesGamepiece()) {
                    CommandScheduler.getInstance().schedule(intakeCoralWithAdjust());
                }

                // If we see the gamepiece, we want to move further down in the intake
                // If we don't, it's too far down and needs to go back up
            }

            @Override
            public void execute() {
                double speed = -0.08;
                _arm.setIntakeSpeed(speed);
            }

            @Override
            public void end(boolean interrupted) {
                _arm.setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished() {
                return _arm.upperLightSensorSeesGamepiece();
            }
        };
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
                }).until(() -> _arm.intakeAtDesiredRotations());
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(_arm::isAtSetpoint);
    }

    public Command moveArm(ArmPosition pos) {
        return setArmPosition(pos).andThen(waitUntilAtSetpoint());
    }

    public Command resetArmPID() {
        return Commands.runOnce(() -> _arm.resetPID());
    }

    public Command manualIntake() {
        return Commands.either(Commands.runOnce(
                () -> _arm.setIntakeSpeed(0.1), _arm),
                Commands.runOnce(() -> _arm.setIntakeSpeed(0.5), _arm),
                () -> _arm.getCurrentMode() == GamepieceMode.CORAL);
    }
}
