package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public final class ElevatorCommands {
    private Elevator _elevator;

    public ElevatorCommands(Elevator elevator) {
        _elevator = elevator;
    }

    public Command runElevatorPID() {
        return Commands.run(() -> {
            _elevator.runMotorsWithPID();
        } );
    }

    public Command decrementElevatorPosition() {
        return new InstantCommand(
            () -> {
                _elevator.decrementElevatorPosition();
                
            } , _elevator);  
    }
  
    public Command incrementElevatorPosition(){
        return new InstantCommand(
           () -> {
            _elevator.incrementElevatorPosition();
    
           }, _elevator);
    } 

    public Command setElevatorSetpoint(ElevatorPosition setpoint) {
        return new InstantCommand(
            () -> {
            _elevator.setSetpoint(setpoint);
            }, _elevator);
    }
}

