package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm.ArmPosition;

public class ArmCommands {
    private Arm _arm;

    public ArmCommands(Arm arm) {
        _arm = arm;
    }
    public Command setArmPosition(ArmPosition setpoint) {
        return new InstantCommand(
            ()->{
                _arm.setArmSetpoint(setpoint);
            },
            _arm);
    }
}
