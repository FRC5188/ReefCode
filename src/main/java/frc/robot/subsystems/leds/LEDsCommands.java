package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.leds.LEDs;
//import frc.robot.subsystems.elevator.Elevator;

public class LEDsCommands {
  private LEDs _leds;

  public LEDsCommands(LEDs leds) {
    _leds = leds;
  }

  public Command intaking() {
      return new StartEndCommand(
        () -> {
          _leds.intaking();
        }, 
        () -> {
          _leds.reset();
        },
         _leds);
  }
  
  public Command elevatorOrArmIsMoving() {
    return new StartEndCommand(
      () -> {
        _leds.elevatorOrArmIsMoving();
      },
      () -> {
        _leds.reset();
      },
      _leds);
  }

  public Command elevatorAndArmAtSetpoints() {
    return new StartEndCommand(
      () -> {
        _leds.elevatorAndArmAtSetpoints();
      },
      () -> {
        _leds.reset();
      },
      _leds);
  }
}