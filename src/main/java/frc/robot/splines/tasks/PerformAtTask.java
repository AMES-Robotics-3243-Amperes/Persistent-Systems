package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class PerformAtTask extends Task {
  private double tolerance = 0;

  public PerformAtTask(Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, double positionTolerance,
      Command command) {
    super(targetRotation, rotationTolerance, command);
    this.tolerance = positionTolerance;
  }

  public PerformAtTask(Rotation2d targetRotation, Rotation2d rotationTolerance, double positionTolerance,
      Command command) {
    this(Optional.of(targetRotation), rotationTolerance, positionTolerance, command);
  }

  public PerformAtTask(Rotation2d targetRotation, Command command) {
    this(Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance,
        TaskConstants.defaultPositionTolerance, command);
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
