// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CurveConstants;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.splines.Spline;
import frc.robot.splines.SplineMetadata;
import frc.robot.splines.SplineSegmentFactory;
import frc.robot.splines.cubicsegments.hermitefactories.C2HermiteSegmentFactory;
import frc.robot.splines.linearsegments.LinearSegmentFactory;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // H! Auto Selector
  AutoSelector autoSelector;
  
  /** :3 Used as a reference for Robot to call periodically */
  public DataManager dataManager;

  // :3 controllers
  private JoyUtil primaryController = new JoyUtil(0);

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  public Photonvision photonvision;

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive =
    new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain, primaryController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      photonvision = new Photonvision();
    } catch (Exception e) {
      System.err.println("------------------------------------------------");
      System.err.println("Photonvision initialization failed: " + e);
      System.err.println("Failed to construct Photonvision, expect null exceptions...");
      System.err.println("------------------------------------------------");
    }

    // :3 constructs a DataManager instance using runtime-initialized RobotContainer members
    dataManager = new DataManager(this);

    // :3 set sensible default commands
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SplineSegmentFactory segment1 = new LinearSegmentFactory(new Translation2d(0, 0),
      new Translation2d(-0.5, 0));
    SplineSegmentFactory segment2 = new C2HermiteSegmentFactory(new Translation2d(-1.25, 0.5),
      new Translation2d(0, 1));
    SplineSegmentFactory segment3 = new C2HermiteSegmentFactory(new Translation2d(-0.8, 1),
      new Translation2d(0.6, 0));
    SplineSegmentFactory segment4 = new C2HermiteSegmentFactory(new Translation2d(-2, 0.4), 
      new Translation2d(-1, 0));
    SplineSegmentFactory segment5 = new C2HermiteSegmentFactory(new Translation2d(0, 0), 
      new Translation2d(1, 0));

    SplineMetadata rotationMetadata = new SplineMetadata();
    rotationMetadata.rotation.set(Rotation2d.fromDegrees(180));
    segment2.addMetadata(rotationMetadata);
    segment3.addMetadata(rotationMetadata);
    segment4.addMetadata(rotationMetadata);
    segment5.addMetadata(rotationMetadata);
      
    Spline spline = new Spline();
    spline.addSegment(segment1);
    spline.addSegment(segment2);
    spline.addSegment(segment3);
    spline.addSegment(segment4);
    spline.addSegment(segment5);
    spline.applyDropVelocities();

    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    PIDController thetaController = new PIDController(1.2, 0, 0);

    CommandSwerveFollowSpline splineCommand = new CommandSwerveFollowSpline(subsystemSwerveDrivetrain,
      spline, xController, yController, thetaController);

    primaryController.b().whileTrue(splineCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }
}
