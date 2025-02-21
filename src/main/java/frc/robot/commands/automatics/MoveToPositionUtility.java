package frc.robot.commands.automatics;

import java.util.Iterator;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.DataManager.DataManagerEntry;
import frc.robot.DataManager.Setpoint;
import frc.robot.Constants;
import frc.robot.PhotonUnit;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * Aligns the robot to the nearest april tag
 * Intended to be used while the robot is close to the april tag it wants to score at
 * 
 * @author Jasper Davidson
 */
public class MoveToPositionUtility {
  /**
   * Generates a command that encodes how to align the robot to a given april tag and reef level
   * 
   * @param photonUnit - photon unit on the robot
   * @param drivetrain - drivetrain the robot is using
   * @param odometryPose - current robot position on the field
   * @param diffClaw - claw the robot is using
   * @param elevator - elevator the robot is using
   * @param targetSetpoint - target reef level to move to (L1, L2, L3, L4); encodes arm rotation and elevator height
   * @return a command for the robot that encodes the spline/how to move along the spline, as well as move the elevator/arm
   */
    public static CommandSwerveFollowSpline alignToTag(PhotonUnit photonUnit, SubsystemSwerveDrivetrain drivetrain,
    DataManagerEntry<Pose2d> odometryPose, SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset) {
      // Get a list of the most recent tag positions
      List<PhotonUnit.Measurement> latestTags = photonUnit.getMeasurement();

      // Find the closest tag to the robot's current position
      Pose2d currentPose = odometryPose.get();
      Translation2d currentPosition = currentPose.getTranslation();
      double minDistance = Double.MAX_VALUE;
      // Refactor how we get this once April tags are added
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
      Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, tagOffset), new Rotation2d());
      Pose2d targetPose = angleTargetPose.plus(offset);

      // PID controllers for the drivetrain
      PIDController xController = new PIDController(0.1, 0, 0);
      PIDController yController = new PIDController(0.1, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0, null);

      PathFactory pathFactory = PathFactory.newFactory();

      moveToPositionTaskBuilder(null /* Once the April tags are loaded, we'll just pass in the matching Pose3d */, pathFactory, diffClaw, elevator, targetSetpoint, tagOffset);

      return pathFactory.finalRotation(targetPose.getRotation()).interpolateFromStart(true)
             .buildCommand(drivetrain, xController, yController, thetaController);

      // return PathFactory.newFactory().addTask(targetPose.getTranslation(),
      // new FinishByTask(
      //   new ParallelCommandGroup(
      //     new ElevatorMoveToPositionCommand(elevator, targetPosition.height),
      //     new InstantCommand(
      //       () -> { diffClaw.setOutsidePosition(targetPosition.angle); },
      //       diffClaw
      //     )
      //   )
      // ))
      // .finalRotation(targetPose.getRotation())
      // .interpolateFromStart(true).buildCommand(drivetrain, xController, yController, thetaController);
  }

//   public static void moveToPositionTask(Pose3d targetPosition, PathFactory pathFactory,
// SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset) {
//     moveToPositionTaskBuilder(targetPosition, tagOffset, elevator, diffClaw, pathFactory, targetSetpoint);
//   }

  public static void moveToPositionTaskBuilder(Pose2d targetPosition, PathFactory pathFactory,
  SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset) {
    Translation2d targetTranslation = new Translation2d(targetPosition.getX(), targetPosition.getY());
    Rotation2d targetRotation = targetPosition.getRotation();

    Rotation2d flippedRotation = targetRotation.plus(new Rotation2d(Math.PI));
    Pose2d angleTargetPose = new Pose2d(targetTranslation, flippedRotation);

    double distanceFromTag = 0.1;
    Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, tagOffset), new Rotation2d());
    Pose2d targetPose = angleTargetPose.plus(offset);

    pathFactory.addTask(targetPose.getTranslation(), new FinishByTask(
      new ParallelCommandGroup(
        new ElevatorMoveToPositionCommand(elevator, targetSetpoint.height),
        new InstantCommand(
          () -> { diffClaw.setOutsidePosition(targetSetpoint.angle); },
          diffClaw
        )
      )
    ));
  }

  public static CommandSwerveFollowSpline autoOne(List<Pose2d> targetPositions, PathFactory pathFactory,
  SubsystemClaw diffClaw, SubsystemElevator elevator, SubsystemSwerveDrivetrain drivetrain, List<Setpoint> targetSetpoints) {
    Iterator<Pose2d> positions = targetPositions.iterator();
    Iterator<Setpoint> setpoints = targetSetpoints.iterator();

    while (positions.hasNext() && setpoints.hasNext()) {
      moveToPositionTaskBuilder(positions.next(), pathFactory, diffClaw, elevator, setpoints.next(), Constants.Positions.tagOffset);
    }
    
    return pathFactory.interpolateFromStart(true).buildCommand(drivetrain, null, null, null);
  }
}
