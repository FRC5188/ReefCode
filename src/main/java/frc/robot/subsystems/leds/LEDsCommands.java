package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

//import frc.robot.subsystems.elevator.Elevator;

public class LEDsCommands {
  private LEDs _leds;

  public LEDsCommands(LEDs leds) {
    _leds = leds;
  }

  //Runs when intake button is pushed
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

  //Runs when we have just acquired a piece
  public Command hasPiece() {
    return new StartEndCommand(
      () -> {
        _leds.hasPiece();
      }, 
      () -> {
        _leds.reset();
      },
      _leds).withTimeout(3);
  }
  
  // Runs when we have no gamepiece and we are toggled to pick up coral
  public Command pickingUpCoral() {
    return new StartEndCommand(
      () -> {
        _leds.pickingUpCoral();
      }, 
      () -> {
        _leds.reset();
      },
      _leds);
  }

  // Runs when we have no gamepiece and we are toggled to pick up coral
  public Command pickingUpAlgae() {
    return new StartEndCommand(
      () -> {
        _leds.pickingUpAlgae();
      }, 
      () -> {
        _leds.reset();
      },
      _leds);
  }
  
  //We have a piece and hasPiece animation has already run
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

  //Runs when robot has finished climbing
  public Command robotHasClimbed() {
    return new StartEndCommand(
      () -> {
        _leds.robotHasClimbed();
      }, 
      () -> {
        _leds.reset();
      },
       _leds);
  }

  public Command disabledAnimation1() {
    return new StartEndCommand(
      () -> {
        _leds.disabledAnimation1();
      }, 
      () -> {
        _leds.reset();
      },
       _leds) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
       };
  }

}