package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class FinishByTask extends Task {
  public FinishByTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation,
      Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, targetRotation, rotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Rotation2d targetRotation, Command command) {
    this(targetTranslation, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(Translation2d targetTranslation, Command command) {
    this(targetTranslation, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public FinishByTask(double x, double y, Command command) {
    this(new Translation2d(x, y), Optional.empty(), TaskConstants.defaultRotationTolerance, command);
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
