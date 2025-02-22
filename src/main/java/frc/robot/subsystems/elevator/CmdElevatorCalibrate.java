// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class CmdElevatorCalibrate extends Command {

  private Elevator _elevator;
  private int _spikeCounter;

  public CmdElevatorCalibrate(Elevator elevator) {
    _elevator = elevator;
  }


  @Override
  public void initialize() {
    _elevator.runMotorsDown();
    _spikeCounter = 0;
  }


  @Override
  public void execute() {
    if (_elevator.isAboveCurrentLimit()) {
      _spikeCounter++;
    }
  }


  @Override
  public void end(boolean interrupted) {
    _elevator.resetEncoders();
    _elevator.stopMotors();
    _elevator.setIsCalibrated(true);
  }


  @Override
  public boolean isFinished() {
    return _spikeCounter >= 3;
  }
}
