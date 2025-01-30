package frc.robot.subsystems.multisubsystemcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MultiSubsystemCommands {
    private Elevator _elevator;
    private Arm _arm;

    public MultiSubsystemCommands(Elevator elevator, Arm arm) {
        _elevator = elevator;
        _arm = arm;
    }

    public Command setOverallSetpoint(ElevatorPosition elevatorSetpoint, ArmPosition armSetpoint) {
        return new InstantCommand(
            () -> {
             _elevator.setSetpoint(elevatorSetpoint);
             _arm.setArmSetpoint(armSetpoint);
            }, _elevator, _arm);
    }
    
}
