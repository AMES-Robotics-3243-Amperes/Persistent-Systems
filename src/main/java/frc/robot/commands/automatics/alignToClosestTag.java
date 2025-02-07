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
import frc.robot.DataManager.Setpoint;
import frc.robot.PhotonUnit;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class alignToClosestTag {
    public static CommandSwerveFollowSpline alignToTag(PhotonUnit photonUnit, SubsystemSwerveDrivetrain drivetrain,
    Supplier<Pose2d> odometryPose, SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetPosition) {
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
          new ElevatorMoveToPositionCommand(elevator, targetPosition.height),
          new InstantCommand(
            () -> { diffClaw.setOutsidePosition(targetPosition.angle); },
            diffClaw
          )
        )
      ))
      .finalRotation(targetPose.getRotation())
      .interpolateFromStart(true).buildCommand(drivetrain, xController, yController, thetaController);
  }
}
