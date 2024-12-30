package frc.robot.splines.Tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.splines.Spline;

public class FinishByTask extends Task {
  public FinishByTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Command command) {
    super(targetTranslation, targetRotation, command);
  }

  public FinishByTask(Translation2d targetTranslation, Rotation2d targetRotation, Command command) {
    this(targetTranslation, Optional.ofNullable(targetRotation), command);
  }

  public FinishByTask(Translation2d targetTranslation, Command command) {
    this(targetTranslation, Optional.empty(), command);
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
