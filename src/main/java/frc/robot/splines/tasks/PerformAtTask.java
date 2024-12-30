package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class PerformAtTask extends Task {
  private double tolerance = 0;

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Optional<Rotation2d> targetRotation,
      Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, targetRotation, rotationTolerance, command);
    this.tolerance = positionTolerance;
  }

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Rotation2d targetRotation,
      Rotation2d rotationTolerance, Command command) {
    this(targetTranslation, positionTolerance, Optional.of(targetRotation), rotationTolerance, command);
  }

  public PerformAtTask(double x, double y, Rotation2d targetRotation, Command command) {
    this(new Translation2d(x, y), TaskConstants.defaultPositionTolerance, Optional.ofNullable(targetRotation),
        TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Pose2d targetPosition, Command command) {
    this(targetPosition.getTranslation(), TaskConstants.defaultPositionTolerance,
        Optional.of(targetPosition.getRotation()), TaskConstants.defaultRotationTolerance, command);
  }

  @Override
  protected double calculateStartLength(Spline spline, double targetLength) {
    return targetLength - tolerance;
  }

  @Override
  protected double calculateEndLength(Spline spline, double targetLength) {
    return targetLength + tolerance;
  }

}
