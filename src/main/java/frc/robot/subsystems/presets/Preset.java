package frc.robot.subsystems.presets;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.OverallPosition;

public class Preset {
    public enum ReefSide {
        Left,
        Right;
    }

    private OverallPosition _level = OverallPosition.Stow;
    private ReefSide _side = ReefSide.Right;
    private boolean _isPresetValid = false;

    private void setPreset(OverallPosition level, ReefSide side) {
        if (level == OverallPosition.Stow || level == OverallPosition.Loading || level == OverallPosition.L4_Score) {
            throw new RuntimeException("Invalid Preset Level/Position");
        }

        _level = level;
        _side = side;
        _isPresetValid = true;
    }

    public OverallPosition getLevel() {
        return _level;
    }

    public ReefSide getSide() {
        return _side;
    }

    public boolean isPresetValid() {
        return _isPresetValid;
    }

    public Command setPresetCommand(OverallPosition level, ReefSide side) {
        return new InstantCommand(
                () -> setPreset(level, side));
    }

    public Command resetPreset() {
        return new InstantCommand(
                () -> _isPresetValid = false);
    }
}