package frc.robot.splines.Tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.splines.Spline;

public class PerformAtTask extends Task {
  private double tolerance = 0;

  public PerformAtTask(Translation2d targetTranslation, double tolerance, Optional<Rotation2d> targetRotation,
      Command command) {
    super(targetTranslation, targetRotation, command);
    this.tolerance = tolerance;
  }

  public PerformAtTask(Translation2d targetTranslation, double tolerance, Rotation2d targetRotation, Command command) {
    this(targetTranslation, tolerance, Optional.ofNullable(targetRotation), command);
  }

  public PerformAtTask(Translation2d targetTranslation, double tolerance, Command command) {
    this(targetTranslation, tolerance, Optional.empty(), command);
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
