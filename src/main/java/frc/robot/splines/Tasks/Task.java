package frc.robot.splines.Tasks;

import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.splines.Spline;

/**
 * Represents a command that should be run by the robot at a specific position
 * in the field while following a {@link Spline}.
 */
public abstract class Task {
  private double startLength = 0;
  private double endLength = 1;

  private final Translation2d targetTranslation;
  private final Optional<Rotation2d> targetRotation;
  private final Command command;

  private final Rotation2d rotationTolerance;

  private boolean completed = false;

  public Task(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance,
      Command command) {
    Objects.requireNonNull(targetTranslation, "targetTranslation cannot be null");
    Objects.requireNonNull(targetRotation, "targetRotation cannot be null");
    Objects.requireNonNull(rotationTolerance, "rotationTolerance cannot be null");
    Objects.requireNonNull(command, "command cannot be null");

    this.targetTranslation = targetTranslation;
    this.targetRotation = targetRotation;
    this.rotationTolerance = rotationTolerance;
    this.command = command.finallyDo(new Runnable() {
      @Override
      public void run() {
        completed = true;
      }
    });
  }

  public final void initialize(Spline spline, double targetLength) {
    completed = false;
    startLength = calculateStartLength(spline, targetLength);
    endLength = calculateEndLength(spline, targetLength);
  }

  protected abstract double calculateStartLength(Spline spline, double targetLength);

  protected abstract double calculateEndLength(Spline spline, double targetLength);

  /**
   * Updates the task's end length. This should only be used to stop tasks from
   * ending further along the path than tasks that proceed it; not to
   * fundamentally change the task itself.
   * 
   * @param newEndLength the new end length of the task
   */
  public final void setEndLength(double newEndLength) {
    this.endLength = newEndLength;
  }

  public final Translation2d getTargetTranslation() {
    return targetTranslation;
  }

  public final Optional<Rotation2d> getTargetRotation() {
    return targetRotation;
  };

  public final Set<Subsystem> getRequirements() {
    return command.getRequirements();
  }

  public final double getStartLength() {
    return this.startLength;
  }

  public final double getEndLength() {
    return this.endLength;
  }

  public final double getRemainingLength(double length) {
    return this.endLength - length;
  }

  public final boolean isValidRotation(Rotation2d rotation) {
    if (this.targetRotation.isEmpty()) {
      return true;
    }

    boolean directlyNear = MathUtil.isNear(this.targetRotation.get().getDegrees(), rotation.getDegrees(), this.rotationTolerance.getDegrees());
    boolean targetLoopNear = MathUtil.isNear(this.targetRotation.get().getDegrees() + 360, rotation.getDegrees(), this.rotationTolerance.getDegrees());
    boolean actualLoopNear = MathUtil.isNear(this.targetRotation.get().getDegrees(), rotation.getDegrees() + 360, this.rotationTolerance.getDegrees());
    return directlyNear || targetLoopNear || actualLoopNear;
  }

  public final boolean isUpcoming(double length) {
    return length <= endLength && !completed;
  }

  public final boolean isActive(double length) {
    return length >= startLength && !completed;
  }

  public final boolean isComplete() {
    return completed;
  }

  public final void runCommand() {
    if (!this.command.isScheduled()) {
      this.command.schedule();
    }
  }
}
