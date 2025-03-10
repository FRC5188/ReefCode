package frc.robot.subsystems.presets;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.OverallPosition;

public class Preset {
    public enum ReefSide {
        Left,
        Right;
    }

    private OverallPosition _level = OverallPosition.L1;
    private ReefSide _side = ReefSide.Right;
    private boolean _isLevelValid = false;
    private boolean _isSideValid = false;

    private void setPresetLevel(OverallPosition level) {
        if (level == OverallPosition.Stow || level == OverallPosition.Loading || level == OverallPosition.L4_Score) {
            throw new RuntimeException("Invalid Preset Level/Position");
        }

        _level = level;
        _isLevelValid = true;
    }

    private void setPresetSide(ReefSide side) {
        _side = side;
        _isSideValid = true;
    }

    public OverallPosition getLevel() {
        return _level;
    }

    public ReefSide getSide() {
        return _side;
    }

    public boolean isPresetValid() {
        return _isLevelValid && _isSideValid;
    }

    public Command setPresetLevelCommand(OverallPosition level) {
        return new InstantCommand(
                () -> setPresetLevel(level));
    }

    public Command setPresetSideCommand(ReefSide side) {
        return new InstantCommand(
                () -> setPresetSide(side));
    }

    public Command resetPreset() {
        return new InstantCommand(
                () -> {
                    _isLevelValid = false;
                    _isSideValid = false;
                });
    }
}