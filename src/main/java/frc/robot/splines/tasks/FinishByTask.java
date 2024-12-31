package frc.robot.splines.tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class FinishByTask extends Task {
  public FinishByTask(Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    super(targetRotation, rotationTolerance, command);
  }

  public FinishByTask(Command command) {
    this(Optional.empty(), TaskConstants.defaultRotationTolerance, command);
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
