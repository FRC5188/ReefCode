// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareConstants.CAN;
import java.io.IOException;
import org.json.simple.parser.ParseException;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command m_lastAutonomousCommand;
  private List<Pose2d> m_pathsToShow = new ArrayList<Pose2d>();
  private Field2d m_autoTraj = new Field2d();
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter =
        new Translation2d(fieldLength / 2, fieldWidth / 2);

  private final RobotContainer m_robotContainer;

  public Robot() {

    switch (HardwareConstants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        new PowerDistribution(CAN.PDH_ID, ModuleType.kRev);
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
  }

  @Override
  public void disabledPeriodic() {
    {
        var m_alliance = DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
        // Get currently selected command

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // Check if is the same as the last one
        if (m_autonomousCommand != m_lastAutonomousCommand && m_autonomousCommand != null) {
            // Check if its contained in the list of our autos
            if (AutoBuilder.getAllAutoNames().contains(m_autonomousCommand.getName())) {
                // Clear the current path
                m_pathsToShow.clear();
                // Grabs all paths from the auto
                try {
                    for (PathPlannerPath path : PathPlannerAuto
                        .getPathGroupFromAutoFile(m_autonomousCommand.getName())) {
                        // Adds all poses to master list
                        m_pathsToShow.addAll(path.getPathPoses());
                    }
                } catch (IOException | ParseException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                // Check to see which alliance we are on Red Alliance
                if (m_alliance) {
                    for (int i = 0; i < m_pathsToShow.size(); i++) {
                        m_pathsToShow.set(i,
                            m_pathsToShow.get(i).rotateAround(fieldCenter, Rotation2d.k180deg));
                    }
                }
                // Displays all poses on Field2d widget
                m_autoTraj.getObject("traj").setPoses(m_pathsToShow);
            }
        }
        m_lastAutonomousCommand = m_autonomousCommand;

        if (!m_pathsToShow.isEmpty()) {
            var firstPose = m_pathsToShow.get(0);
            Logger.recordOutput("Alignment/StartPose", firstPose);
            SmartDashboard.putBoolean("Alignment/Translation",
                firstPose.getTranslation().getDistance(
                    m_robotContainer.getDrive().getPose().getTranslation()) <= Units
                        .inchesToMeters(1.5));
            SmartDashboard.putBoolean("Alignment/Rotation",
                firstPose.getRotation().minus(m_robotContainer.getDrive().getPose().getRotation())
                    .getDegrees() < 1);
        }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println(m_autonomousCommand);


    System.out.println("!!!!!!!!!!Autonomous init!!!!!!!!!!!");
    if (m_autonomousCommand != null) {
      System.out.println("FOUND A COMMAND");
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {} 

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
