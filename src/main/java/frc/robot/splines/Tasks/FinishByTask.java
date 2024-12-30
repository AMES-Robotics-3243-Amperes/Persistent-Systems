package frc.robot.splines.Tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class FinishByTask extends Task {
  public FinishByTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, targetRotation, rotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Command command) {
    super(targetTranslation, targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Rotation2d targetRotation, Command command) {
    super(targetTranslation, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Command command) {
    super(targetTranslation, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(double targetX, double targetY, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    super(new Translation2d(targetX, targetY), targetRotation, rotationTolerance, command);
  }

  public FinishByTask(double targetX, double targetY, Optional<Rotation2d> targetRotation, Command command) {
    super(new Translation2d(targetX, targetY), targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(double targetX, double targetY, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    super(new Translation2d(targetX, targetY), Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public FinishByTask(double targetX, double targetY, Rotation2d targetRotation, Command command) {
    super(new Translation2d(targetX, targetY), Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(double targetX, double targetY, Command command) {
    super(new Translation2d(targetX, targetY), Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(Pose2d targetPosition, Rotation2d rotationTolerance, Command command) {
    super(targetPosition.getTranslation(), Optional.of(targetPosition.getRotation()), rotationTolerance, command);
  }

  public FinishByTask(Pose2d targetPosition, Command command) {
    super(targetPosition.getTranslation(), Optional.of(targetPosition.getRotation()), TaskConstants.defaultRotationTolerance, command);
  }

  @Override
  protected double calculateStartLength(Spline spline, double targetLength) {
    return 0;
  }

  @Override
  protected double calculateEndLength(Spline spline, double targetLength) {
    return targetLength;
  }
}
