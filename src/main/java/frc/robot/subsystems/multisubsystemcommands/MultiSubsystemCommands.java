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
        Load_Coral(ElevatorPosition.Stow, ArmPosition.Loading),
        Load_Algae_L2(ElevatorPosition.L2, ArmPosition.Loading),
        Load_Algae_L3(ElevatorPosition.L3, ArmPosition.Loading),
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
                .andThen(_armCommands.moveArm(setpoint.getArmPosition()));
    }

    public Command setGamepieceMode(GamepieceMode mode) {
        return new InstantCommand(
                () -> {
                    _elevator.setCurrentMode(mode);
                    _arm.setCurrentMode(mode);
                }, _elevator, _arm);
    }

    public Command scoreGamepieceAtPosition(Supplier<OverallPosition> setpoint) {
        return scoreGamepieceAtPosition(setpoint.get());
    }

    public Command scoreGamepieceAtPosition(OverallPosition setpoint) {
        if (setpoint == OverallPosition.Stow) {
            throw new RuntimeException("scoreGamepieceAtPosition cannot run to stow");
        }
        return moveToPosition(setpoint)
                .andThen(_armCommands.spit())
                .andThen(moveToPosition(OverallPosition.Stow));
    }

    public Command loadCoral() {
        return (moveToPosition(OverallPosition.Load_Coral))
                .andThen(_armCommands.intake())
                .andThen(new WaitCommand(0.25))
                .andThen(_armCommands.moveGamepieceToLightSensor())
                .andThen(new WaitCommand(0.25))
                .andThen(_armCommands.moveGamepieceToLightSensor().unless(() -> _arm.lightSensorSeesGamepiece()))
                .andThen(_armCommands.moveArm(ArmPosition.Stow))
                .unless(() -> _arm.getCurrentMode() != GamepieceMode.CORAL);
    }

    public Command loadAlgae(OverallPosition position) {
        if (position != OverallPosition.Load_Algae_L2 || position != OverallPosition.Load_Algae_L3)
            throw new IllegalArgumentException("Can only load algae from L2 or L3");
        return moveToPosition(position)
                .andThen(_armCommands.intake())
                .andThen(_armCommands.setArmPosition(ArmPosition.Stow))
                .unless(() -> _arm.getCurrentMode() != GamepieceMode.ALGAE);
    }
}