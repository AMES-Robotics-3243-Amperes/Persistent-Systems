// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.crypto.Data;

import org.photonvision.PhotonTargetSortMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.DataManager.Setpoint;
import frc.robot.commands.leds.CommandLedPatternCycle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveModulesForward;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.automatics.ScoreIntakeAutoCommand;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.commands.elevator.ElevatorZeroCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand.Position;
import frc.robot.commands.CommandSwerveGetOffset;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.PerformAtTask;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.Constants.PhotonvisionConstants;
import frc.robot.Constants.Positions;
import frc.robot.Constants.Setpoints;

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
  // H! Main shuffleboard tab
  ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // H! Auto Selector
  AutoSelector autoSelector = new AutoSelector(mainTab);

  // controllers
  private JoyUtil primaryController = new JoyUtil(0);
  private JoyUtil secondaryController = new JoyUtil(1);

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  public SubsystemLeds subsystemLeds = new SubsystemLeds();
  public SubsystemElevator subsystemElevator = new SubsystemElevator();
  public SubsystemClaw subsystemClaw = new SubsystemClaw();

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive = new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain,
      primaryController);
  private CommandLedPatternCycle commandLedPatternCycle = new CommandLedPatternCycle(subsystemLeds);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);

    // set sensible default commands
    setDefaultCommands();

    // H! Set all commands in auto selector
    setAutoCommands();

    // configure the controller bindings
    configureBindings();
  }

  /**
   * Used to set default commands for subsystems.
   */
  private void setDefaultCommands() {
    subsystemLeds.setDefaultCommand(commandLedPatternCycle);
    //subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
    autoSelector
      .add(new InstantCommand(() -> {System.out.println("\n\n\nPRINT COMMAND RUN!\n\n\n");}), "Print Command")
      .add(new InstantCommand(() -> {System.out.println("\n\n\nOTHER COMMAND RUN!\n\n\n");}), "Other Command");
  }

  /**
   * Used to configure controller bindings.
   */
  private void configureBindings() {
    PIDController xController = new PIDController(1.2, 0, 0.1);
    PIDController yController = new PIDController(1.2, 0, 0.1);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.7, 0, 0.0,
        new Constraints(3 * Math.PI, 6 * Math.PI));

    CommandSwerveFollowSpline followCommand = PathFactory.newFactory()
        .addPoint(0, 0)
        .addPoint(2, 0)
        .addPoint(2, 2)
        .addTask(0, 0, new PerformAtTask(Rotation2d.fromDegrees(180), new InstantCommand()))
        .buildCommand(subsystemSwerveDrivetrain, xController, yController, thetaController);

    // Utility Commands
    InstantCommand moveClawDepositL1 = new InstantCommand(
      () -> { subsystemClaw.setOutsidePosition(Setpoint.L1.angle); },
      subsystemClaw
    );

    InstantCommand moveClawDepositL23 = new InstantCommand(
      () -> { subsystemClaw.setOutsidePosition(Setpoint.L2.angle); },
      subsystemClaw
    );

    InstantCommand moveClawDepositL4 = new InstantCommand(
      () -> { subsystemClaw.setOutsidePosition(Setpoint.L4.angle); },
      subsystemClaw
    );

    IntakeClawCommand intakeClawCommand = new IntakeClawCommand(subsystemClaw, Setpoints.intakePower);
    IntakeClawCommand depositClawCommand = new IntakeClawCommand(subsystemClaw, -Setpoints.intakePower);

    // Keybindings will change
    primaryController.a().onTrue(followCommand);

    // Triggers for auto scoring routine for L1
    secondaryController.leftTrigger().and(secondaryController.a()).and(secondaryController.povLeft())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L1, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, Positions.tagOffset, intakeClawCommand
    ));
    secondaryController.leftTrigger().and(secondaryController.a()).and(secondaryController.povRight())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L1, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, -Positions.tagOffset, intakeClawCommand
    ));

    // Triggers for auto scoring routine for L2
    secondaryController.leftTrigger().and(secondaryController.b()).and(secondaryController.povLeft())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L2, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, Positions.tagOffset, intakeClawCommand
    ));
    secondaryController.leftTrigger().and(secondaryController.b()).and(secondaryController.povRight())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L2, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, -Positions.tagOffset, intakeClawCommand
    ));

    // Triggers for auto scoring routine for L3
    secondaryController.leftTrigger().and(secondaryController.x()).and(secondaryController.povLeft())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L3, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, Positions.tagOffset, intakeClawCommand
    ));
    secondaryController.leftTrigger().and(secondaryController.x()).and(secondaryController.povRight())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L3, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, -Positions.tagOffset, intakeClawCommand
    ));

    // Triggers for auto scoring routine for L4
    secondaryController.leftTrigger().and(secondaryController.y()).and(secondaryController.povLeft())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L4, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, Positions.tagOffset, intakeClawCommand
    ));
    secondaryController.leftTrigger().and(secondaryController.y()).and(secondaryController.povRight())
    .onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.Intake, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, -Positions.tagOffset, intakeClawCommand
    ));

    // Auto intaking from loading station (player will still half to adjust left or right)
    secondaryController.povUp().onTrue(new ScoreIntakeAutoCommand(
      subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L4, PhotonvisionConstants.photonUnits.get(0),
      DataManager.instance().robotPosition, 0, depositClawCommand
    ));

    // Manual intaking/depositing, elevator movement, reef setpoints
    secondaryController.leftBumper().onTrue(new IntakeClawCommand(subsystemClaw, frc.robot.Constants.Setpoints.intakePower));
    secondaryController.rightBumper().onTrue(new IntakeClawCommand(subsystemClaw, -frc.robot.Constants.Setpoints.intakePower));

    primaryController.povUp().whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity));
    primaryController.povDown().whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.downNudgeVelocity));

    primaryController.a().and(secondaryController.leftTrigger().negate()).onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L1.height),
      moveClawDepositL1
    ));

    primaryController.b().and(secondaryController.leftTrigger().negate()).onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L2.height),
      moveClawDepositL23
    ));

    primaryController.x().and(secondaryController.leftTrigger().negate()).onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L3.height),
      moveClawDepositL23
    ));

    primaryController.y().and(secondaryController.leftTrigger().negate()).onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L4.height),
      moveClawDepositL4
    ));

    mainTab.add("Zero Elevator", new ElevatorZeroCommand(subsystemElevator)).withWidget(BuiltInWidgets.kCommand);

    primaryController.x().toggleOnTrue(new CommandSwerveGetOffset(subsystemSwerveDrivetrain));
    primaryController.b().onTrue(new InstantCommand(commandSwerveTeleopDrive::toggleFieldRelative));

    SequentialCommandGroup drivetrainSysIdCommand = new SequentialCommandGroup(
      subsystemSwerveDrivetrain.sysIdDriveQuasistatic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdDriveQuasistatic(Direction.kReverse),
      subsystemSwerveDrivetrain.sysIdDriveDynamic(Direction.kForward),
      subsystemSwerveDrivetrain.sysIdDriveDynamic(Direction.kReverse)
    );
    
    primaryController.start().onTrue(drivetrainSysIdCommand);
    primaryController.back().whileTrue(new CommandSwerveModulesForward(subsystemSwerveDrivetrain));
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
