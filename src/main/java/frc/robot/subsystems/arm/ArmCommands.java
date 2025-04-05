package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
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
                }, _arm).withTimeout(0.5)
                .andThen(Commands.runOnce(() -> _arm.setArmSetpoint(ArmPosition.Stow), _arm)
                        .unless(() -> _arm.getCurrentPos() != ArmPosition.L4_Score))
                        .withName("Spit");

    }

    public Command setArmPosition(ArmPosition setpoint) {
        if (setpoint == ArmPosition.L4_Score) {
            return new InstantCommand(
                    () -> {
                        _arm.setArmSetpoint(setpoint);
                    });//.andThen(intakeForNumberOfRotations());
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
        return intakeCoral().andThen(moveArm(ArmPosition.Stow));
    }

    private Command intakeCoral() {
        Command c = new Command() {
            /*
             * This command will run the intake at a fast speed until the lower light sensor
             * detects the coral
             * Then, the speed will decrease and run until both sensors detect the coral
             * Then, the motor will turn off for a few cycles (~100 ms)
             * Then, the motor will run at a slow speed in reverse until the upper light
             * sensor detects the coral
             * Then, the arm will go back to stow
             * 
             * In the case that the lower and upper light sensors don't detect a piece when
             * running reverse,
             * we assume there is no piece and start running at a fast speed again
             */
            boolean hasPieceDetected = false;
            boolean prevHasPieceDetected = false;
            boolean waiting = false;
            boolean done = false;

            int counter = 0;

            @Override
            public void initialize() {
                // Init flags
                hasPieceDetected = false;
                waiting = false;
                done = false;

                counter = 0;
            }

            @Override
            public void execute() {
                // We don't think we have a piece, so try to intake it
                if (!hasPieceDetected) {
                    if (_arm.upperLightSensorSeesGamepiece()) {
                        _arm.setIntakeSpeed(0.09);
                    } else {
                        _arm.setIntakeSpeed(0.22); // 0.35
                    }

                    // Update hasPiece
                    hasPieceDetected = _arm.hasPiece();
                } else {
                    // Check if we actually see the gamepiece on our lower sensor after waiting
                    if (!_arm.lowerLightSensorSeesGamepiece()) {
                        counter++;
                    }

                    if (counter > 15) {
                        // Assume that at this point we don't have a piece
                        // Clear out hasPiece so normal intaking will start again
                        _arm.clearHasGamepiece();
                        hasPieceDetected = false;
                        counter = 0;
                    } else {
                        // Run the motors backwards until we see the piece in the upper light sensor
                        _arm.setIntakeSpeed(-0.09); // changed to -0.11 to prevent dropping piece while moving, seemed
                                                    // to work well
                        done = _arm.upperLightSensorSeesGamepiece();
                    }
                }
            }

            @Override
            public void end(boolean interrupted) {
                _arm.setIntakeSpeed(0);
            }

            @Override
            public boolean isFinished() {
                return done;
            }
        };
        c.addRequirements(_arm);
        return c;
    }

    public Command intakeCoralAuto() {
        return new Command() {
            @Override
            public void execute() {
                if (_arm.upperLightSensorSeesGamepiece()) {
                    _arm.setIntakeSpeed(0.09);
                } else {
                    _arm.setIntakeSpeed(0.22); // 0.35
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
            public void execute() {
                double speed = -0.09; // -0.08
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

    public Command intakeForNumberOfRotations() {
        return new StartEndCommand(
                () -> {
                    _arm.resetIntakeEncoders();
                    _arm.setIntakeSpeed(-0.175); // -0.1
                },
                () -> {
                    _arm.setIntakeSpeed(0);
                }, _arm).until(() -> _arm.intakeAtDesiredRotations());
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
        return Commands.either(
                new StartEndCommand(
                        () -> _arm.setIntakeSpeed(-0.1),
                        () -> _arm.setIntakeSpeed(0),
                        _arm),
                new StartEndCommand(
                        () -> _arm.setIntakeSpeed(0.5),
                        () -> _arm.setIntakeSpeed(0),
                        _arm),
                () -> _arm.getCurrentMode() == GamepieceMode.CORAL)
                .withName("ManualIntake");
    }
}
