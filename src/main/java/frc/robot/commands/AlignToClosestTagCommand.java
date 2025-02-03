// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PhotonUnit;
import frc.robot.splines.PathFactory;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToClosestTagCommand extends Command {
    private final PhotonUnit photonUnit;
    private SubsystemSwerveDrivetrain drivetrain;
    private Supplier<Pose2d> odometryPose;
    private Pose2d targetPose;
    private CommandSwerveFollowSpline driveToPosCommand;
  
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
      targetPose = angleTargetPose.plus(offset);

      PIDController xController = new PIDController(0.1, 0, 0);
      PIDController yController = new PIDController(0.1, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0, null);
      
      driveToPosCommand = PathFactory.newFactory().addPoint(targetPose.getTranslation()).finalRotation(targetPose.getRotation())
      .interpolateFromStart(true).buildCommand(drivetrain, xController, yController, thetaController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveToPosCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Drivetrain will no longer run since execute is no longer being called
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // SUGGESTION: add velocity tolerance as well H!
    // Should stop when the drivetrain is facing the tag within angle and position tolerance
    Pose2d currentPose = odometryPose.get();
    
    double xTolerance = 0.05; // Calibrate these and move to constants
    double yTolerance = 0.05;
    double thetaTolerance = 0.05;

    return (Math.abs(currentPose.getX() - targetPose.getX()) < xTolerance)
        && (Math.abs(currentPose.getY() - targetPose.getY()) < yTolerance)
        && (Math.abs(currentPose.getRotation().plus(targetPose.getRotation().unaryMinus()).getRadians()) < thetaTolerance);
    // The strange a + -b pattern for rotations is because .plus() returns a value bounded from pi to -pi,
    // sidestepping the fact that WPILib Rotations can be greater than 360 degrees. H!
  }
}
