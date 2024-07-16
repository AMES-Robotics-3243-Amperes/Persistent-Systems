// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Curves.Spline;
import frc.robot.Curves.CubicSegments.C0CubicBezierSegmentFactory;
import frc.robot.Curves.CubicSegments.C2HermiteSegmentFactory;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
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
      System.err.println("Photonvision initialization failed: " + e);
      System.err.println("Failed to construct Photonvision, expect null exceptions...");
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
    C0CubicBezierSegmentFactory segment1 = new C0CubicBezierSegmentFactory(new Translation2d(0, 0),
      new Translation2d(0.1, 0), new Translation2d(0.1, 0), new Translation2d(1, 0));
    C2HermiteSegmentFactory segment2 = new C2HermiteSegmentFactory(new Translation2d(1, 1),
      new Translation2d(-1, -1));
    C2HermiteSegmentFactory segment3 = new C2HermiteSegmentFactory(new Translation2d(0, 0),
      new Translation2d(-1, -1));
      
    Spline spline = new Spline();
    spline.addSegment(segment1);
    spline.addSegment(segment2);
    spline.addSegment(segment3);

    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);

    CommandSwerveFollowSpline splineCommand = new CommandSwerveFollowSpline(subsystemSwerveDrivetrain,
      spline, 1, xController, yController);

    primaryController.b().whileTrue(splineCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
