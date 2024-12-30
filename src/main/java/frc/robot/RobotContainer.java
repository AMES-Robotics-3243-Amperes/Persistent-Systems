// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.splines.tasks.PerformAtTask;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // H! Auto Selector
  AutoSelector autoSelector;

  // controllers
  private JoyUtil primaryController = new JoyUtil(0);

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive = new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain,
      primaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);

    // set sensible default commands
    setDefaultCommands();

    // configure the controller bindings
    configureBindings();
  }

  /**
   * Used to set default commands for subsystems.
   */
  private void setDefaultCommands() {
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private class MockShooterSubsystem extends SubsystemBase {
  }

  private class MockReadyShooterCommand extends Command {
    public MockReadyShooterCommand(MockShooterSubsystem subsystem) {
      addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
      return true;
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Shooter readied!");
    }
  }

  private class MockShootCommand extends Command {
    public MockShootCommand(MockShooterSubsystem subsystem) {
      addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
      return true;
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Shot!");
    }
  }

  /**
   * Used to configure controller bindings.
   */
  private void configureBindings() {
    PIDController xController = new PIDController(0.4, 0, 0.02);
    PIDController yController = new PIDController(0.4, 0, 0.02);
    PIDController thetaController = new PIDController(1.2, 0, 0);

    MockShooterSubsystem mockShooterSubsystem = new MockShooterSubsystem();

    CommandSwerveFollowSpline followCommand = PathFactory.newFactory()
        .addPoint(0, 0)
        .addPoint(1, 1)
        .addTask(new FinishByTask(2, 0, new MockReadyShooterCommand(mockShooterSubsystem)))
        .addTask(new PerformAtTask(3, -1, Rotation2d.fromDegrees(180), new MockShootCommand(mockShooterSubsystem)))
        .addPoint(0, 0)
        .finalRotation(new Rotation2d(0))
        .buildCommand(subsystemSwerveDrivetrain, xController, yController, thetaController);

    primaryController.a().whileTrue(followCommand);
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
