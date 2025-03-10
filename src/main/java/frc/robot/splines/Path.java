package frc.robot.splines;

import java.util.List;
import java.util.ListIterator;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.Entry;
import frc.robot.splines.NumericalMethods.RealFunction;
import frc.robot.splines.tasks.Task;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * An arclength-parametrized path in 2D space for the robot to follow. Where a
 * {@link Spline} is simply a time-parameterized curve, a Path contains much
 * more utility and is a complete entity for a robot to follow.
 * Should generally be constructed using a
 * {@link frc.robot.splines.PathFactory}.
 */
public class Path {
  private Entry<Pose2d> positionEntry = null;
  private ControlPointList controlPoints = new ControlPointList();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = FollowConstants.defaultInterpolator;
  private RealFunction offsetDampen = FollowConstants::splineOffsetVelocityDampen;
  private RealFunction startDampen = FollowConstants::splineStartVelocityDampen;
  private RealFunction completeDampen = FollowConstants::splineCompleteVelocityDampen;
  private RealFunction taskDampen = FollowConstants::splineTaskVelocityDampen;
  private double maxAccelAfterTask = FollowConstants.maxAccelAfterTask;
  private double maxSpeed = FollowConstants.maxSpeed;
  private double maxCentrifugalAcceleration = FollowConstants.maxCentrifugalAcceleration;
  private boolean interpolateFromStart = FollowConstants.interpolateFromStart;

  public Spline spline; // TODO: private

  // the current parameterization is expensive to calculate, so we cache it
  private Optional<Double> currentParameterization = Optional.empty();
  private double currentLength = 0;
  private double previousAdvanceTimestamp = 0;
  private double previousVelocityTimestamp = 0;
  private double previousTaskSpeed = 0;

  public Path(Entry<Pose2d> positionEntry,
      ControlPointList points,
      Optional<Rotation2d> finalRotation,
      SplineInterpolator interpolator,
      RealFunction offsetDampen,
      RealFunction startDampen,
      RealFunction completeDampen,
      RealFunction taskDampen,
      double maxAccelAfterTask,
      double maxSpeed,
      double maxCentrifugalAcceleration,
      boolean interpolateFromStart) {
    Objects.requireNonNull(positionEntry, "positionEntry cannot be null");
    Objects.requireNonNull(points, "points cannot be null");
    Objects.requireNonNull(finalRotation, "finalRotation cannot be null");
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    Objects.requireNonNull(offsetDampen, "offsetDampen cannot be null");
    Objects.requireNonNull(startDampen, "startDampen cannot be null");
    Objects.requireNonNull(completeDampen, "completeDampen cannot be null");
    Objects.requireNonNull(taskDampen, "taskDampen cannot be null");
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");

    this.positionEntry = positionEntry;
    this.controlPoints = points;
    this.finalRotation = finalRotation;
    this.interpolator = interpolator;
    this.offsetDampen = offsetDampen;
    this.startDampen = startDampen;
    this.completeDampen = completeDampen;
    this.taskDampen = taskDampen;
    this.maxAccelAfterTask = maxAccelAfterTask;
    this.maxSpeed = maxSpeed;
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    this.interpolateFromStart = interpolateFromStart;

    this.initialize();
  }

  public double getLength() {
    return currentLength;
  }

  public Pose2d getCurrentPosition() {
    return positionEntry.get();
  }

  public double getParameterization() {
    if (currentParameterization.isEmpty()) {
      currentParameterization = Optional.of(spline.parameterizationAtArcLength(currentLength));
    }

    return currentParameterization.get();
  }

  public Translation2d getGoalPosition() {
    return spline.at(getParameterization());
  }

  public Optional<Rotation2d> getDesiredRotation() {
    Optional<Rotation2d> rotation = finalRotation;

    List<Task> upcomingTasks = controlPoints.getUpcomingTasks(currentLength);
    ListIterator<Task> it = upcomingTasks.listIterator();
    while (it.hasNext()) {
      Task nextTask = it.next();
      if (nextTask.getTargetRotation().isPresent()) {
        rotation = nextTask.getTargetRotation();
        break;
      }
    }

    return rotation;
  }

  public Rotation2d getMinimumRotationTolerance() {
    return controlPoints.getMinimumRotationTolerance();
  }

  public double getDesiredSpeed() {
    double offsetSpeed = maxSpeed
        * offsetDampen.sample(positionEntry.get().getTranslation().getDistance(getGoalPosition()));
    double completeSpeed = Math.min(offsetSpeed, completeDampen.sample(Math.abs(getLength() - spline.arcLength(1))));
    double startSpeed = Math.min(completeSpeed, startDampen.sample(currentLength));
    double centrifugalSpeed = Math.min(startSpeed,
        Math.sqrt(maxCentrifugalAcceleration / spline.curvature(getParameterization())));

    List<Task> upcomingTasks = controlPoints.getUpcomingTasks(currentLength);
    double taskSpeed = centrifugalSpeed;
    if (!upcomingTasks.isEmpty()) {
      taskSpeed = Math.min(centrifugalSpeed, taskDampen.sample(upcomingTasks.get(0).getRemainingLength(currentLength)));
    }

    double currentTimestamp = MathSharedStore.getTimestamp();
    taskSpeed = Math.min(taskSpeed,
        previousTaskSpeed + maxAccelAfterTask * Math.abs(currentTimestamp - previousVelocityTimestamp));
    previousVelocityTimestamp = currentTimestamp;
    previousTaskSpeed = taskSpeed;
    return taskSpeed;
  }

  public Translation2d getDesiredVelocity() {
    Translation2d splineDerivative = spline.derivative(getParameterization());
    return splineDerivative.times(getDesiredSpeed() / splineDerivative.getNorm());
  }

  public void initialize() {
    currentLength = 0;
    previousTaskSpeed = maxSpeed;
    previousVelocityTimestamp = MathSharedStore.getTimestamp();
    previousAdvanceTimestamp = MathSharedStore.getTimestamp();
    currentParameterization = Optional.empty();

    Optional<Translation2d> interpolateFromStartTranslation = interpolateFromStart
        ? Optional.of(getCurrentPosition().getTranslation())
        : Optional.empty();
    spline = interpolator
        .interpolatePoints(controlPoints.getInterpolationTranslations(interpolateFromStartTranslation));
    controlPoints.initializeTasks(spline, interpolateFromStart);
  }

  public void advance() {
    double newTimestamp = MathSharedStore.getTimestamp();
    double oldLength = currentLength;
    currentLength += (newTimestamp - previousAdvanceTimestamp) * getDesiredSpeed();
    currentParameterization = Optional.empty();
    previousAdvanceTimestamp = newTimestamp;

    if (!controlPoints.getUpcomingTasks(oldLength).isEmpty()) {
      currentLength = Math.min(currentLength, controlPoints.getUpcomingTasks(oldLength).get(0).getEndLength());
      currentLength = Math.min(currentLength, spline.arcLength(1));
    }

    startTasks();
  }

  /**
   * Starts any active tasks. <b> Already run by {@link #advance}. </b> This
   * method is separate from {@link #advance} only for testing purposes.
   */
  public void startTasks() {
    for (Task task : controlPoints.getActiveTasks(currentLength)) {
      if (task.isValidRotation(getCurrentPosition().getRotation())) {
        task.runCommand();
      }
    }
  }

  /**
   * Advances the spline to a specific arc length. Should primarily be used for
   * debug/testing purposes; does not interact well with tasks.
   * 
   * @param newLength the length to advance to
   */
  public void advanceTo(double newLength) {
    currentLength = newLength;
    currentParameterization = Optional.empty();
    previousAdvanceTimestamp = MathSharedStore.getTimestamp();
  }

  public boolean isComplete() {
    return currentLength > spline.arcLength(1);
  }
}
