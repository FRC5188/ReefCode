// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareConstants.CAN;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    switch (HardwareConstants.currentMode) {
      case REAL:
      /*
       * If you are really tight on ram you can comment out the logwriter
       * for the robot with the V1 rio. Keeo the publisher so you can view
       * the live data on advantage scope.
       */
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // new PowerDistribution(CAN.PDH_ID, ModuleType.kRev);
        break;

      case SIM:
        System.out.println("SIM!!!");
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    Logger.recordMetadata("ProjectName", "ReefCode"); // Set a metadata value


    /*********************************************************
     * 
     * README README README README README README
     * 
     * Commenting out the start logger function will save you enough
     * ram for the robot code to run. If you add leds, cimbers, etc, 
     * then you may need to save additional ram. The proposed solution
     * is to add a boolen similar to the isreal and isSim flags to check
     * if this is the v1 or v2 bot. If it is v1 with the roborio one, 
     * then dont start the logger or start leds, etc.
     * 
     * 
     * NOTE: adding the climber and elevator and intake back to the code was enough
     * to put it over the ram limit. so either more boolean checks will be needed
     * to enable and disable features or we need to trim down our code to use less
     * ram.
     * 
     * Garrett
     * 
     ***********************************************************/
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
                      
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
