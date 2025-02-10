package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MultiSubsystemCommands {
    public enum OverallPosition {
        Stow(ElevatorPosition.Stow, ArmPosition.Stow),
        Loading(ElevatorPosition.Stow, ArmPosition.Loading),
        L1(ElevatorPosition.L1, ArmPosition.Stow),
        L2(ElevatorPosition.L2, ArmPosition.Stow),
        L3(ElevatorPosition.L3, ArmPosition.Stow),
        L4(ElevatorPosition.L4, ArmPosition.Stow),
        L4_Score(ElevatorPosition.L4, ArmPosition.L4_Score);

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

    public Command setOverallSetpoint(OverallPosition setpoint) {
        return _elevatorCommands.setElevatorSetpoint(setpoint.getElevatorPosition())
                .alongWith(_armCommands.setArmPosition(setpoint.getArmPosition()))
                .unless(() -> !_elevator.canMoveToPos(_elevator.getCurrentPos(), setpoint.getArmPosition()));
    }

    public Command waitForOverallMechanism() {
        return _elevatorCommands.waitUntilAtSetpoint()
                .alongWith(_armCommands.waitUntilAtSetpoint());
    }

    private Command score(OverallPosition setpoint) {
        if (setpoint == OverallPosition.L4) {
            return setOverallSetpoint(OverallPosition.L4_Score)
                    .andThen(waitForOverallMechanism())
                    .andThen(_armCommands.spit())
                    .andThen(setOverallSetpoint(OverallPosition.L4))
                    .andThen(waitForOverallMechanism());
        } else {
            return _armCommands.spit();
        }
    }

    public Command scoreGamepieceAtPosition(OverallPosition setpoint) {
        
        return new InstantCommand();

        /*
        if (setpoint == OverallPosition.Stow || setpoint == OverallPosition.Loading
                || setpoint == OverallPosition.L4_Score) {
            throw new RuntimeException("scoreGamepieceAtPosition cannot run to stow,loading,or L4_score");
        }
        return setOverallSetpoint(setpoint)
                .andThen(waitForOverallMechanism())
                .andThen(score(setpoint))
                .andThen(setOverallSetpoint(OverallPosition.Stow));
    */
                }
}
