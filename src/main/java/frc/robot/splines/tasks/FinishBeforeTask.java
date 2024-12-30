package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class FinishBeforeTask extends Task {
  private double buffer = 0;

  public FinishBeforeTask(Translation2d targetTranslation, double positionBuffer, Optional<Rotation2d> targetRotation,
      Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, targetRotation, rotationTolerance, command);
    this.buffer = positionBuffer;
  }

  public FinishBeforeTask(Translation2d targetTranslation, double positionBuffer, Rotation2d targetRotation,
      Command command) {
    this(targetTranslation, positionBuffer, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance,
        command);
  }

  public FinishBeforeTask(Translation2d targetTranslation, Command command) {
    this(targetTranslation, TaskConstants.defaultPositionBuffer, Optional.empty(),
        TaskConstants.defaultRotationTolerance, command);
  }

  public FinishBeforeTask(double x, double y, Command command) {
    this(new Translation2d(x, y), TaskConstants.defaultPositionBuffer, Optional.empty(),
        TaskConstants.defaultRotationTolerance, command);
  }

  @Override
  protected double calculateStartLength(Spline spline, double targetLength) {
    return 0;
  }

  @Override
  protected double calculateEndLength(Spline spline, double targetLength) {
    return targetLength - this.buffer;
  }
}
