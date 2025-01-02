// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    Timer timer = new Timer();
    double time = 0;

    public MockReadyShooterCommand(MockShooterSubsystem subsystem) {
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      System.out.println("Readying!");
      Random random = new Random();
      time = 1 + 6 * random.nextDouble();
      timer.restart();
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Ready!");
    }
  }

  private class MockShootCommand extends Command {
    private Timer timer = new Timer();

    public MockShootCommand(MockShooterSubsystem subsystem) {
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      System.out.println("Firing!");
      timer.restart();
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Fire!");
    }
  }

  /**
   * Used to configure controller bindings.
   */
  private void configureBindings() {
    PIDController xController = new PIDController(1.2, 0, 0.1);
    PIDController yController = new PIDController(1.2, 0, 0.1);
    ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0.0,
        new Constraints(3 * Math.PI, 6 * Math.PI));

    MockShooterSubsystem mockShooterSubsystem = new MockShooterSubsystem();

    CommandSwerveFollowSpline followCommand = PathFactory.newFactory()
        .addTask(13, 0, new PerformAtTask(Rotation2d.fromDegrees(0), new InstantCommand()))
        .addTask(12, 0.5, new FinishByTask(new MockReadyShooterCommand(mockShooterSubsystem)))
        .addTask(12, 0.5, new PerformAtTask(Rotation2d.fromDegrees(180), new MockShootCommand(mockShooterSubsystem)))
        .addTask(13, 0, new PerformAtTask(Rotation2d.fromDegrees(0), new InstantCommand()))
        .addTask(12, 0, new FinishByTask(new MockReadyShooterCommand(mockShooterSubsystem)))
        .addTask(12, 0, new PerformAtTask(Rotation2d.fromDegrees(180), new MockShootCommand(mockShooterSubsystem)))
        .addTask(13, 0, new PerformAtTask(Rotation2d.fromDegrees(0), new InstantCommand()))
        .addTask(12, -0.5, new FinishByTask(new MockReadyShooterCommand(mockShooterSubsystem)))
        .addTask(12, -0.5, new PerformAtTask(Rotation2d.fromDegrees(180), new MockShootCommand(mockShooterSubsystem)))
        .addTask(13, 0, new PerformAtTask(Rotation2d.fromDegrees(0), new InstantCommand()))
        .addPoint(12, -0.5)
        .addPoint(11, 0)
        .addPoint(12, 0.5)
        .addPoint(13, 0)
        .buildCommand(subsystemSwerveDrivetrain, xController, yController, thetaController);

    primaryController.a().onTrue(followCommand);
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
