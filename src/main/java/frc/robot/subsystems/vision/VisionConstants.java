// Code modified from AdvantageKit Vision Template (v4.0.1):
// https://github.com/Mechanical-Advantage/AdvantageKit/releases
//
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Camera names, must match names configured on coprocessor
  
  // New cameras for 2025 season are 5,6,7,8
  
  public static String camera5Name = "photoncamera_5";
  public static String camera6Name = "photoncamera_6";
  public static String camera7Name = "photoncamera_7";
  public static String camera8Name = "photoncamera_8";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // Note: 0.0254 multiplier converts inches to meters

  // Camera 5
  public static Transform3d robotToCamera5 =
      new Transform3d(-13.75*0.0254, -11.25*0.0254, 9.25*0.0254, new Rotation3d(0.0, -Math.PI/4, Math.PI));
  // Camera 6
  public static Transform3d robotToCamera6 =
      new Transform3d(-13.75*0.0254, 11.25*0.0254, 9.25*0.0254, new Rotation3d(0.0, 0, Math.PI));


  // Camera 7
  // Front right
  public static Transform3d robotToCamera7 =
      new Transform3d(14.125*0.0254, -8.5*0.0254, 7.75*0.0254, new Rotation3d(0.0, 0, 0));

  // Front Left
  // Camera 8
  public static Transform3d robotToCamera8 =
      new Transform3d(14.125*0.0254, 8.5*0.0254, 7.75*0.0254, new Rotation3d(0.0, 0, 0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 5
        1.0, // Camera 6
        1.0, // Camera 7
        1.0  // Camera 8
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // Ignore rotation data 
}
