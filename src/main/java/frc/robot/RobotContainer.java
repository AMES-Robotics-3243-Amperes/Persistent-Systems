// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Curves.Spline;
import frc.robot.Curves.SplineSegment;
import frc.robot.Curves.CubicSegments.BezierSegment;
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

    Translation2d zero = new Translation2d(0, 0);
    Translation2d one = new Translation2d(1, 1);
    BezierSegment segment = new BezierSegment(zero, zero, one, one);
    BezierSegment segment2 = new BezierSegment(one, one, one.times(2), one.times(2));
    ArrayList<SplineSegment> list = new ArrayList<SplineSegment>();
    list.add(segment);
    list.add(segment2);
    Spline spline = new Spline(list);
    System.out.println(spline.sample(1.5));
    System.out.println(spline.derivative(1.5));
    System.out.println(spline.arcLength(1.5));
    System.out.println(spline.timeAtArcLength(Math.sqrt(2) * 3 / 2));

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
    SequentialCommandGroup drivetrainSysIdCommand = new SequentialCommandGroup(
      subsystemSwerveDrivetrain.sysIdQuasistatic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdQuasistatic(Direction.kReverse),
      subsystemSwerveDrivetrain.sysIdDynamic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdDynamic(Direction.kReverse)
    );
    primaryController.start().whileTrue(drivetrainSysIdCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      subsystemSwerveDrivetrain.sysIdQuasistatic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdQuasistatic(Direction.kReverse),
      subsystemSwerveDrivetrain.sysIdDynamic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdDynamic(Direction.kReverse)
    );
  }
}
