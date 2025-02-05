// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.DataManager.Setpoint;
import frc.robot.PhotonUnit;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.subsystems.SubsystemClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInReefCommand extends SequentialCommandGroup {
  /** Creates a new ScoreInReefCommand. */
  public ScoreInReefCommand(
      SubsystemSwerveDrivetrain drivetrain, SubsystemClaw diffClaw, SubsystemElevator elevator,
      Setpoint reefPosition, PhotonUnit photonCamera, Supplier<Pose2d> odometry
    ) {
    addCommands(
      new SequentialCommandGroup(
        alignToClosestTag(photonCamera, drivetrain, odometry, diffClaw, elevator, reefPosition),
        new IntakeClawCommand(diffClaw, Setpoints.intakePower)
      )
    );
  }

  public static CommandSwerveFollowSpline alignToClosestTag(PhotonUnit photonUnit, SubsystemSwerveDrivetrain drivetrain,
  Supplier<Pose2d> odometryPose, SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint reefPosition) {
      // Get a list of the most recent tag positions
      List<PhotonUnit.Measurement> latestTags = photonUnit.getMeasurement();

      // Find the closest tag to the robot's current position
      Pose2d currentPose = odometryPose.get();
      Translation2d currentPosition = currentPose.getTranslation();
      double minDistance = Double.MAX_VALUE;
      PhotonUnit.Measurement closestTag = latestTags.get(0);

      for (PhotonUnit.Measurement position : latestTags) {
        double potentialMin = position.targetPosition.getDistance(currentPosition);
        if (potentialMin < minDistance) {
          minDistance = potentialMin;
          closestTag = position;
        }
      }

      // Calculate a target pose to move to
      // Double check if this is necessary
      Rotation2d flippedRotation = closestTag.pose.getRotation().plus(new Rotation2d(Math.PI));
      Pose2d angleTargetPose = new Pose2d(closestTag.pose.getTranslation(), flippedRotation);

      double distanceFromTag = 0.1;
      Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, 0.0), new Rotation2d());
      Pose2d targetPose = angleTargetPose.plus(offset);

      // PID controllers for the drivetrain
      PIDController xController = new PIDController(0.1, 0, 0);
      PIDController yController = new PIDController(0.1, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0, null);

      return PathFactory.newFactory().addTask(targetPose.getTranslation(),
      new FinishByTask(
        new ParallelCommandGroup(
          new ElevatorMoveToPositionCommand(elevator, reefPosition.height),
          new InstantCommand(
            () -> { diffClaw.setOutsidePosition(reefPosition.angle); },
            diffClaw
          )
        )
      ))
      .finalRotation(targetPose.getRotation())
      .interpolateFromStart(true).buildCommand(drivetrain, xController, yController, thetaController);
  }
}
