package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberCommands {
    private Climber _climber;

    public ClimberCommands(Climber climber) {
        _climber = climber;
    }

    public Command runClimber(DoubleSupplier ySupplier) {
        return Commands.run(
                () -> {
                    _climber.setClimberSpeed(ySupplier.getAsDouble());
                },
                _climber);
    }
}