package frc.robot.subsystems.multisubsystemcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MultiSubsystemCommands {
    public enum OverallPosition {
        Stow(ElevatorPosition.Stow, ArmPosition.Stow),
        Coral_Loading(ElevatorPosition.Stow, ArmPosition.Loading),
        Algae_Loading_L2(ElevatorPosition.L2, ArmPosition.Loading),
        Algae_Loading_L3(ElevatorPosition.L3, ArmPosition.Loading),
        L1(ElevatorPosition.L1, ArmPosition.Stow),
        L2(ElevatorPosition.L2, ArmPosition.Stow),
        L3(ElevatorPosition.L3, ArmPosition.Stow),
        L4(ElevatorPosition.L4, ArmPosition.L4_Score);

        ElevatorPosition _elevatorSetpoint;
        ArmPosition _armSetpoint;

        OverallPosition(ElevatorPosition elevatorSetpoint, ArmPosition armSetpoint) {
            _elevatorSetpoint = elevatorSetpoint;
            _armSetpoint = armSetpoint;
        }

        ElevatorPosition getElevatorPosition() {
            return _elevatorSetpoint;
        }

        ArmPosition getArmPosition() {
            return _armSetpoint;
        }
    }

    public enum GamepieceMode {
        ALGAE,
        CORAL;
    }

    private Elevator _elevator;
    private Arm _arm;
    private ElevatorCommands _elevatorCommands;
    private ArmCommands _armCommands;

    public MultiSubsystemCommands(Elevator elevator, Arm arm, ElevatorCommands elevatorCommands,
            ArmCommands armCommands) {
        _elevator = elevator;
        _arm = arm;
        _elevatorCommands = elevatorCommands;
        _armCommands = armCommands;
    }

    public Command moveToPosition(OverallPosition setpoint) {
        return _armCommands.moveArm(ArmPosition.Transient)
        .andThen(_elevatorCommands.moveElevator(setpoint.getElevatorPosition()))
        .unless(() -> _elevator.getCurrentPos() == setpoint.getElevatorPosition())
        .andThen(_armCommands.moveArm(setpoint.getArmPosition()));
    }

    public Command setGamepieceMode(GamepieceMode mode) {
        return new InstantCommand(
                () -> {
                    _elevator.setCurrentMode(mode);
                    _arm.setCurrentMode(mode);
                }, _elevator, _arm);
    }

    public Command scoreGamepieceAtPosition(OverallPosition setpoint) {
        return moveToPosition(setpoint)
                .andThen(_armCommands.spit());
    }

    public Command loadCoral() {
        return moveToPosition(OverallPosition.Coral_Loading)
                .alongWith(_armCommands.intake())
                .andThen(new WaitCommand(0.1))
                .andThen(_armCommands.moveGamepieceToLightSensor())
                .andThen(new WaitCommand(0.1))
                .andThen(_armCommands.moveGamepieceToLightSensor().unless(() -> _arm.upperLightSensorSeesGamepiece()))
                .andThen(moveToPosition(OverallPosition.Stow));
    }

    public Command loadAlgae(OverallPosition position) {
        if (position != OverallPosition.Algae_Loading_L2 && position != OverallPosition.Algae_Loading_L3) {
            throw new IllegalArgumentException("Can Only Load Algae @ L2 or L3");
        }
        return moveToPosition(position)
                .alongWith(_armCommands.intake());
    }

}