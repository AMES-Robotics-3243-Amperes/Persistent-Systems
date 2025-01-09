package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class FinishBeforeTask extends Task {
  private double buffer = 0;

  public FinishBeforeTask(Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, double positionBuffer,
      Command command) {
    super(targetRotation, rotationTolerance, command);
    this.buffer = positionBuffer;
  }

  public FinishBeforeTask(double positionBuffer, Command command) {
    this(Optional.empty(), TaskConstants.defaultRotationTolerance, positionBuffer, command);
  }

  public FinishBeforeTask(Command command) {
    this(Optional.empty(), TaskConstants.defaultRotationTolerance, TaskConstants.defaultPositionBuffer, command);
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
