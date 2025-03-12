// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.DataManager.Setpoint;
import frc.robot.DataManager;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.commands.claw.DeployClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.splines.tasks.PerformAtTask;
import frc.robot.subsystems.SubsystemClaw;
import edu.wpi.first.math.Pair;

/**
 * Scores a coral in the reef automatically
 * 
 * @author Jasper Davidson
 */
public class ScoreIntakeAutoCommandBuilder {
  /** Creates a new ScoreInReefCommand. */
  public static Command scoreIntakeAutoCommand(
      SubsystemSwerveDrivetrain drivetrain, SubsystemClaw diffClaw, SubsystemElevator elevator,
      Setpoint reefPosition, double tagOffset, boolean scoring) {
    Command command = new ProxyCommand(() -> {
      // Find the closest tag to the robot's current position
      Pose2d currentPose = DataManager.instance().robotPosition.get();
      Translation2d currentPosition = currentPose.getTranslation();
      double minDistance = Double.MAX_VALUE;

      // Initialize to any tag --> It *will* get overran (I hope there's a tag closer
      // than Double.MAX_VALUE...)
      Pose2d closestTag = FieldConstants.blueReef1.toPose2d();

      Set<Integer> filterTagIDs;

      if (scoring) {
        filterTagIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
      } else {
        filterTagIDs = Set.of(1, 2, 12, 13);
      }

      for (AprilTag tag : FieldConstants.tagList) {
        if (!filterTagIDs.contains(tag.ID))
          continue;

        Pose2d tagPosition = tag.pose.toPose2d();
        double potentialMin = (tagPosition.getTranslation()).getDistance(currentPosition);

        if (potentialMin < minDistance) {
          minDistance = potentialMin;
          closestTag = tagPosition;
        }
      }

      PathFactory pathFactory = PathFactory.newFactory();

      pathFactory
          .interpolateFromStart(true)
          .interpolator(new LinearInterpolator())
          .taskDampen((remainingLength) -> 2.2 * remainingLength + 0.05);
      moveToPositionTaskBuilder(closestTag, pathFactory, diffClaw, elevator, reefPosition,
          tagOffset, scoring);

      return pathFactory.buildCommand(
          drivetrain,
          FollowConstants.xyController(),
          FollowConstants.xyController(),
          FollowConstants.thetaController());
    });
    command.addRequirements(drivetrain, diffClaw, elevator);
    return command;
  }

  public static void moveToPositionTaskBuilder(Pose2d tagPosition, PathFactory pathFactory,
      SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset, boolean scoring) {
    double distanceFromTag = 0.5;
    Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, tagOffset), new Rotation2d());
    Translation2d targetPosition = tagPosition.plus(offset).getTranslation();
    Rotation2d targetRotation = tagPosition.getRotation().plus(Rotation2d.fromDegrees(180));

    double intakePower = Setpoints.intakePower;
    if (!scoring) {
      intakePower = -Setpoints.intakePower;
    }

    pathFactory
        .addTask(targetPosition, new FinishByTask(new ElevatorMoveToPositionCommand(elevator, targetSetpoint.height)))
        .addTask(targetPosition, new FinishByTask(new InstantCommand(
            () -> {
              diffClaw.setOutsidePosition(targetSetpoint.angle);
            },
            diffClaw)))
        .addTask(targetPosition,
            new PerformAtTask(targetRotation, new DeployClawCommand(diffClaw, intakePower)));
  }

  public static Command buildAuto(List<Pose2d> targetPositions, PathFactory pathFactory,
      SubsystemClaw diffClaw, SubsystemElevator elevator, SubsystemSwerveDrivetrain drivetrain,
      List<Pair<Setpoint, Boolean>> targetSetpoints) {
    Iterator<Pose2d> positions = targetPositions.iterator();
    Iterator<Pair<Setpoint, Boolean>> setpoints = targetSetpoints.iterator();

    while (positions.hasNext() && setpoints.hasNext()) {
      Pair<Setpoint, Boolean> setpoint = setpoints.next();
      moveToPositionTaskBuilder(positions.next(), pathFactory, diffClaw, elevator, setpoint.getFirst(),
          setpoint.getFirst().offset, setpoint.getSecond());
    }

    return pathFactory.interpolateFromStart(true).buildCommand(
        drivetrain, FollowConstants.xyController(), FollowConstants.xyController(),
        FollowConstants.thetaController());
  }
}
