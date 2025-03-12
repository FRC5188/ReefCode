// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.io.RealArmIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.io.RealClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.CmdElevatorCalibrate;
import frc.robot.subsystems.drive.io.GyroIO;
import frc.robot.subsystems.drive.io.GyroIOPigeon2;
import frc.robot.subsystems.drive.io.ModuleIO;
import frc.robot.subsystems.drive.io.ModuleIOSim;
import frc.robot.subsystems.drive.io.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsCommands;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.io.RealElevatorIO;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.OverallPosition;
import frc.robot.subsystems.presets.Preset;

public class RobotContainer {
  private final Drive drive;
  private final Elevator elevatorSubsystem = new Elevator(new RealElevatorIO());
  private final Arm armSubsystem = new Arm(new RealArmIO());
  private final LEDs LEDSubsystem = new LEDs();
  private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem);
  private final ArmCommands armCommands = new ArmCommands(armSubsystem);
  private final LEDsCommands LEDCommands = new LEDsCommands(LEDSubsystem);

  private final Climber climber = new Climber(new RealClimberIO());
  private final ClimberCommands ClimberCommands = new ClimberCommands(climber);

  private final MultiSubsystemCommands multiSubsystemCommands = new MultiSubsystemCommands(elevatorSubsystem,
      armSubsystem, elevatorCommands, armCommands);

  /** I am single :( 
   * hi zoe :3
  */
  private final Preset preset = new Preset();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
 
  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController climberstick = new CommandXboxController(1);

  private final GenericHID buttonbox1 = new GenericHID(1);
  private final JoystickButton L1Button = new JoystickButton(buttonbox1, 1);
  private final JoystickButton L2Button = new JoystickButton(buttonbox1, 2);
  private final JoystickButton StowButton = new JoystickButton(buttonbox1, 3);
  private final JoystickButton L3Button = new JoystickButton(buttonbox1, 4);
  private final JoystickButton L4Button = new JoystickButton(buttonbox1, 5);
  private final JoystickButton LoadingButton = new JoystickButton(buttonbox1, 6);
  private final JoystickButton intakeButton = new JoystickButton(buttonbox1, 7);
  private final JoystickButton L4_scoreButton = new JoystickButton(buttonbox1, 8);
  private final JoystickButton spitButton = new JoystickButton(buttonbox1, 9);
  private final JoystickButton gamepieceModeToggle = new JoystickButton(buttonbox1, 10);

  private final GenericHID buttonbox2 = new GenericHID(2);
  private final JoystickButton presetButton = new JoystickButton(buttonbox2, 2);
  private final JoystickButton incrementButton = new JoystickButton(buttonbox2, 4);
  private final JoystickButton decrementButton = new JoystickButton(buttonbox2, 7);

  // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {

    switch (HardwareConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        
        // vision =
        //         new Vision(
        //           drive::addVisionMeasurement,
        //           new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //           new VisionIOPhotonVision(camera1Name, robotToCamera1));   

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }
    

    // All AutoAligns for reef will align to Left position
    //TODO: Add AutoAligns to all the commands.

    // AutoAlignToReef + ScoreL1 (Move to L1, score)
    NamedCommands.registerCommand("L1",
        multiSubsystemCommands.scoreGamepieceAtPosition(OverallPosition.L1));
    
    // AutoAlignToReef + ScoreL2 (Move to L2, score)
    NamedCommands.registerCommand("L2",
        multiSubsystemCommands.scoreGamepieceAtPosition(OverallPosition.L2)); 

    // AutoAlignToReef + ScoreL3 (Move to L3, score)
    NamedCommands.registerCommand("L3",
        multiSubsystemCommands.scoreGamepieceAtPosition(OverallPosition.L3));
    
    // AutoAlignToReef + Move to L4 + Score
    NamedCommands.registerCommand("L4",
        multiSubsystemCommands.scoreGamepieceAtPosition(OverallPosition.L4));

    // AutoAlign to Intake + Intake
    NamedCommands.registerCommand("Intake",
        multiSubsystemCommands.loadGamepiece());

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));

    climber.setDefaultCommand(
      ClimberCommands.runClimber(
        () -> -climberstick.getLeftY()));

    
    // drivetrain.setDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    //climberstick.start().and(climberstick.y()).onTrue(getAutonomousCommand())

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drive.runOnce(() -> drive.seedFieldCentric()));
 
    // drive.registerTelemetry(logger::telemeterize);

    intakeButton.onTrue(multiSubsystemCommands.loadGamepiece().raceWith(LEDCommands.intaking()).andThen(LEDCommands.hasPiece()).andThen(LEDCommands.elevatorOrArmIsMoving()));
    spitButton.onTrue(armCommands.spit());

    L1Button.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L1));
    L2Button.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L2));
    StowButton.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.Stow));
    L3Button.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L3));
    L4Button.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L4));
    // LoadingButton.onTrue(multiSubsystemCommands.loadGamepiece());
    L4_scoreButton.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L4_Score));

    gamepieceModeToggle.whileTrue(multiSubsystemCommands.setGamepieceMode(GamepieceMode.ALGAE));
    gamepieceModeToggle.whileFalse(multiSubsystemCommands.setGamepieceMode(GamepieceMode.CORAL));

    // Runs the preset to score unless the preset is invalid.
    joystick.rightBumper().onTrue(
    multiSubsystemCommands.scoreGamepieceAtPosition(() -> preset.getLevel()).unless(()
    -> !preset.isPresetValid()));

    // Resets the preset when we don't have a piece.
    armSubsystem._hasPiece.onFalse(preset.resetPreset().andThen(LEDCommands.pickingUpCoral()));

    // Sets the level preset
    presetButton.and(L1Button).onTrue(preset.setPresetLevelCommand(OverallPosition.L1));
    presetButton.and(L2Button).onTrue(preset.setPresetLevelCommand(OverallPosition.L2));
    presetButton.and(L3Button).onTrue(preset.setPresetLevelCommand(OverallPosition.L3));
    presetButton.and(L4Button).onTrue(preset.setPresetLevelCommand(OverallPosition.L4));

    incrementButton.onTrue(elevatorCommands.incrementElevatorPosition());
    decrementButton.onTrue(elevatorCommands.decrementElevatorPosition());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void calibrateAndStartPIDs() {
    // PID commands: we only want one of them so start/stop works correctly
    Command elevatorPIDCommand = elevatorCommands.runElevatorPID();
    Command armPIDCommand = armCommands.runArmPID();
    // Start elevator pid
    if (elevatorSubsystem.isCalibrated()) {
      elevatorCommands.runElevatorPID();
      if (!CommandScheduler.getInstance().isScheduled(elevatorPIDCommand)) {
        CommandScheduler.getInstance().schedule(elevatorPIDCommand);
      }
    } else {
      Command calibCommand = new CmdElevatorCalibrate(elevatorSubsystem).andThen(elevatorPIDCommand);
      CommandScheduler.getInstance().schedule(calibCommand);
    }

    // Start arm pid
    if (!CommandScheduler.getInstance().isScheduled(armPIDCommand)) {
      CommandScheduler.getInstance().schedule(armPIDCommand);
    }

    // Set initial positions
    CommandScheduler.getInstance().schedule(elevatorCommands.setElevatorSetpoint(ElevatorPosition.Stow));
    CommandScheduler.getInstance().schedule(armCommands.setArmPosition(ArmPosition.Stow));

    CommandScheduler.getInstance().schedule(multiSubsystemCommands.setGamepieceMode(GamepieceMode.CORAL));
  }

  public void startIdleAnimations() {
    Command disabled1 = LEDCommands.disabledAnimation1();
    if (!CommandScheduler.getInstance().isScheduled(disabled1))
      CommandScheduler.getInstance().schedule(disabled1);
  }

  public void startEnabledLEDs() {
    Command initialLEDs = LEDCommands.pickingUpCoral();
    if (!CommandScheduler.getInstance().isScheduled(initialLEDs))
      CommandScheduler.getInstance().schedule(initialLEDs);
  }
}
