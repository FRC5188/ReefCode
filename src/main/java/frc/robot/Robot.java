// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareConstants.CAN;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command _previousAutonomousCommand;
  private RobotContainer m_robotContainer;
  private Field2d _autonomousTrajectory = new Field2d();
  private List<Pose2d> _shownPaths = new ArrayList<>();

  public Robot() {
    CanBridge.runTCP();
    Logger.recordMetadata("ProjectName", "ReefCode"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(CAN.PDH_ID, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
                    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.startIdleAnimations();
        SmartDashboard.putData("Autonomous Selection Preview", this._autonomousTrajectory);
  }

  @Override
  public void disabledPeriodic() {
        // The currently selected command.
        this.m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (this.m_autonomousCommand != this._previousAutonomousCommand) {
          if (AutoBuilder.getAllAutoNames().contains(m_autonomousCommand.getName())) {
            this._shownPaths.clear();
            try {
              PathPlannerAuto.getPathGroupFromAutoFile(this.m_autonomousCommand.getName()).stream()
                  .forEach(
                      path -> {
                        this._shownPaths.addAll(path.getPathPoses());
                      }
                  );
            } catch (IOException e) {
              // TODO Auto-generated catch block
              e.printStackTrace();
            } catch (ParseException e) {
              // TODO Auto-generated catch block
              e.printStackTrace();
            }
              this._autonomousTrajectory.getObject("Trajectory").setPoses(this._shownPaths);
          }
        }
        this._previousAutonomousCommand = this.m_autonomousCommand;
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

    m_robotContainer.calibrateAndStartPIDs();
    m_robotContainer.startEnabledLEDs();
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
    m_robotContainer.calibrateAndStartPIDs();
    m_robotContainer.startEnabledLEDs();
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
