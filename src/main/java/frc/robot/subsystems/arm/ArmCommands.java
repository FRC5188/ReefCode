package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ArmCommands {
    private Arm _arm;

    public ArmCommands(Arm arm) {
        _arm = arm;
    }

    public Command intake() {
        return new StartEndCommand(
            () -> {
                _arm.setIntakeSpeed(0.5);
            }, 
            () -> {
                _arm.setIntakeSpeed(0);
            },
            _arm).until(() -> _arm.hasPiece());
    }
}
