package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private ClimberIO _io;
    private ClimberIOInputsAutoLogged _inputs;

    public Climber(ClimberIO io) {
        _io = io;
        _inputs = new ClimberIOInputsAutoLogged();
    }

    public void setClimberSpeed(double speed) {
        _io.setClimberMotorSpeed(speed);
    }

    @Override
    public void periodic() {
        _io.updateInputs(_inputs);
        Logger.processInputs("Climber", _inputs);
    }
}