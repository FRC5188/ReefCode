package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public final class ElevatorCommands {
    private Elevator _elevator;

    public ElevatorCommands(Elevator elevator) {
        _elevator = elevator;
    }

    public Command calibrateElevator() {
        return new Command() {
            int spikeCounter = 0;

            @Override
            public void initialize() {
                spikeCounter = 0;
                _elevator.runMotorsDown();
                _elevator.setIsCalibrated(false);
            }

            @Override
            public void execute() {
                if (_elevator.isAboveCurrentLimit()) {
                    spikeCounter++;
                }
            }

            @Override
            public void end(boolean interrupted) {
                _elevator.resetEncoders();
                _elevator.stopMotors();
                _elevator.setIsCalibrated(true);
            }

            @Override
            public boolean isFinished() {
                return spikeCounter >= 3;
            }
        };
    }

    public Command decrementElevatorPosition() {
        return new InstantCommand(
                () -> {
                    _elevator.decrementElevatorPosition();

                }, _elevator);
    }

    public Command incrementElevatorPosition() {
        return new InstantCommand(
                () -> {
                    _elevator.incrementElevatorPosition();

                }, _elevator);
    }

    public Command setElevatorSetpoint(ElevatorPosition setpoint) {
        return new InstantCommand(
                () -> {
                    System.out.println("RUNNING ELEVATOR CMD");
                    _elevator.setSetpoint(setpoint);
                });
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(_elevator::isAtSetpoint);
    }

    public Command moveElevator(ElevatorPosition pos) {
        return setElevatorSetpoint(pos).andThen(waitUntilAtSetpoint());
    }

    public Command resetElevatorPID() {
        return Commands.runOnce(() -> _elevator.resetPID());
    }
}
