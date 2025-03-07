// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.DataManager.Setpoint;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.automatics.MoveToPositionUtility;
import frc.robot.commands.automatics.ScoreIntakeAutoCommand;
import frc.robot.commands.claw.DeployClawCommand;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.commands.elevator.ElevatorZeroCommand;
import frc.robot.commands.leds.CommandLedPatternCycle;
import frc.robot.splines.PathFactory;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.FieldConstants;

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

  // Path Factory for auto routine
  PathFactory pathFactory = PathFactory.newFactory();

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

    SmartDashboard.putData(subsystemClaw);

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
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
    autoSelector
        .add(
            MoveToPositionUtility.autoBuilder(
                FieldConstants.AutonomousPaths.bluePositionOne,
                pathFactory, subsystemClaw, subsystemElevator,
                subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.blueSetpointsOne),
            "Blue Position One");
  }

  /**
   * Used to configure controller bindings.
   * Do not remove any of the commented out code. Most of it is commented for
   * testing purposes.
   */
  private void configureBindings() {
    // Purely testing purposes
    secondaryController.leftBumper().onTrue(
        new InstantCommand(
            () -> {
              subsystemClaw.setOutsidePosition(Setpoint.IntakeLeft.angle);
            },
            subsystemClaw));

    secondaryController.rightBumper().onTrue(
      new InstantCommand(
          () -> {
            subsystemClaw.setOutsidePosition(Setpoint.L3Left.angle);
          },
          subsystemClaw));

    secondaryController.a().onTrue(new IntakeClawCommand(subsystemClaw, Setpoints.intakePower));

    secondaryController.b().onTrue(new DeployClawCommand(subsystemClaw, -Setpoints.intakePower));

    secondaryController.x().onTrue(
      new InstantCommand(
        () -> {
          subsystemClaw.setIntakePower(0);
        }
      ));

    // secondaryController.a().onTrue(new ParallelCommandGroup(
    //   new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L1Left.height),
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(Setpoint.L1Left.angle);
    //     },
    //     subsystemClaw
    //   )
    // ));

    // secondaryController.b().onTrue(new ParallelCommandGroup(
    //   new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L2Left.height),
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(Setpoint.L2Left.angle);
    //     },
    //     subsystemClaw
    //   )
    // ));

    // secondaryController.x().onTrue(new ParallelCommandGroup(
    //   new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L3Left.height),
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(Setpoint.L3Left.angle);
    //     },
    //     subsystemClaw
    //   )
    // ));

    // secondaryController.y().onTrue(new ParallelCommandGroup(
    //   new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L4Left.height),
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(Setpoint.L4Left.angle);
    //     },
    //     subsystemClaw
    //   )
    // ));

    // // Auto intaking from loading station (Left or Right selected by D-pad)
    // primaryController.a().and(primaryController.povLeft()).onTrue(
    //   new ScoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.IntakeLeft,
    //       DataManager.instance().robotPosition, 0, new IntakeClawCommand(subsystemClaw, -Setpoints.intakePower))
    // );

    // primaryController.a().and(primaryController.povRight()).onTrue(
    //   new ScoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.IntakeRight,
    //       DataManager.instance().robotPosition, 0, new IntakeClawCommand(subsystemClaw, -Setpoints.intakePower))
    // );

    // // Auto score in nearest L4 (Left or Right selected by D-pad)
    // primaryController.y().and(primaryController.povLeft()).onTrue(
    //   new ScoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L4Left,
    //       DataManager.instance().robotPosition, 0, new IntakeClawCommand(subsystemClaw, -Setpoints.intakePower))
    // );

    // primaryController.y().and(primaryController.povRight()).onTrue(
    //   new ScoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw, subsystemElevator, Setpoint.L4Right,
    //       DataManager.instance().robotPosition, 0, new IntakeClawCommand(subsystemClaw, -Setpoints.intakePower))
    // );

    // // Manual intaking/depositing, elevator movement, reef setpoints
    // secondaryController.leftBumper().onTrue(new IntakeClawCommand(subsystemClaw, frc.robot.Constants.Setpoints.intakePower));
    // secondaryController.rightBumper().onTrue(new IntakeClawCommand(subsystemClaw, -frc.robot.Constants.Setpoints.intakePower));

    // double leftY = secondaryController.getLeftY();
    // if (leftY > Elevator.manualThreshold) {
    //   new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity);
    // } else if (leftY < -Elevator.manualThreshold) {
    //   new ElevatorNudgeCommand(subsystemElevator, -Constants.Elevator.Control.upNudgeVelocity);
    // }

    // double rightY = secondaryController.getRightY();
    // if (rightY > DifferentialArm.manualThreshold) {
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(subsystemClaw.getPosition() + convertJoystickToPosition(-leftY));
    //     },
    //     subsystemClaw);
    // } else if (rightY < -DifferentialArm.manualThreshold) {
    //   new InstantCommand(
    //     () -> {
    //       subsystemClaw.setOutsidePosition(subsystemClaw.getPosition() - convertJoystickToPosition(leftY));
    //     },
    //     subsystemClaw);
    // }

    // primaryController.povUp().whileTrue(
    //   new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity)
    // );
    // primaryController.povDown().whileTrue(
    //   new ElevatorNudgeCommand(subsystemElevator, -Constants.Elevator.Control.upNudgeVelocity)
    // );

    mainTab.add("Zero Elevator", new ElevatorZeroCommand(subsystemElevator)).withWidget(BuiltInWidgets.kCommand);

    // primaryController.x().toggleOnTrue(new CommandSwerveGetOffset(subsystemSwerveDrivetrain));
    primaryController.b().onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));
  }

  public static double convertJoystickToPosition(double joystick) {
    return (DifferentialArm.encoderRange * joystick + DifferentialArm.encoderRange);
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
