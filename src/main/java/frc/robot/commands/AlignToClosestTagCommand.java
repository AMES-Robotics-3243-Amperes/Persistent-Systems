// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.opencv.photo.Photo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PhotonUnit;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToClosestTagCommand extends Command {
    private final PhotonUnit photonUnit;
    private SubsystemSwerveDrivetrain drivetrain;
    private Supplier<Pose2d> odometryPose;
  
    /** Creates a new AlignToClosestTagCommand. */
    public AlignToClosestTagCommand(PhotonUnit photonUnit, SubsystemSwerveDrivetrain drivetrain, Supplier<Pose2d> odometry) {
      this.photonUnit = photonUnit;
      this.drivetrain = drivetrain;
      this.odometryPose = odometry;
  
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // Get a list of the most recent tag positions
      List<PhotonUnit.Measurement> latestTags = photonUnit.getMeasurement();
  
      // Extract the tag positions from measurements into a Set
      // Set<Translation2d> tagPositions = new HashSet<Translation2d>();

      // for (PhotonUnit.Measurement tag : latestTags) {
      //   tagPositions.add(tag.targetPosition);
      // }

      // If there are no tags, abort the command
      if (latestTags.isEmpty()) {
        cancel();
      }

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
      Rotation2d flippedRotation = closestTag.pose.getRotation().plus(new Rotation2d(Math.PI));
      Pose2d angleTargetPose = new Pose2d(closestTag.pose.getTranslation(), flippedRotation);

      double distanceFromTag = 0.1;
      Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, 0.0), new Rotation2d());
      Pose2d targetPose = angleTargetPose.plus(offset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should stop when the drivetrain is facing the tag within angle and position tolerance
    return false;
  }
}
