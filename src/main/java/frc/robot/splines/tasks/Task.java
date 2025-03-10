package frc.robot.splines.tasks;

import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.splines.Spline;

/**
 * Represents a command that should be run by the robot at a specific position
 * in the field while following a {@link Spline}.
 */
public abstract class Task {
  private double startLength = 0.0;
  private double endLength = 1.0;

  private final Optional<Rotation2d> targetRotation;
  private final Command command;

  private final Rotation2d rotationTolerance;

  private boolean completed = false;

  public Task(Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    Objects.requireNonNull(targetRotation, "targetRotation cannot be null");
    Objects.requireNonNull(rotationTolerance, "rotationTolerance cannot be null");
    Objects.requireNonNull(command, "command cannot be null");

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
    startLength = MathUtil.clamp(calculateStartLength(spline, targetLength), 0, spline.arcLength(1));
    endLength = MathUtil.clamp(calculateEndLength(spline, targetLength), 0, spline.arcLength(1));

    assert startLength <= endLength;
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

  public final Rotation2d getRotaitonTolerance() {
    return this.rotationTolerance;
  }

  public final boolean isValidRotation(Rotation2d rotation) {
    if (this.targetRotation.isEmpty()) {
      return true;
    }

    return Math
        .abs(MathUtil.angleModulus(this.targetRotation.get().getRadians() - rotation.getRadians())) <= rotationTolerance
            .getRadians();
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

  /**
   * Sets a task as completed. Should only be used for testing purposes.
   */
  public final void markCompleted() {
    completed = true;
  }

  public final void runCommand() {
    if (!this.command.isScheduled()) {
      this.command.schedule();
    }
  }
}
