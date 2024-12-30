package frc.robot.splines;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.ListIterator;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  private ArrayList<Pair<Translation2d, Optional<Task>>> points = new ArrayList<Pair<Translation2d, Optional<Task>>>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = FollowConstants.defaultInterpolator;
  private RealFunction offsetDampen = FollowConstants::splineOffsetVelocityDampen;
  private RealFunction startDampen = FollowConstants::splineStartVelocityDampen;
  private RealFunction completeDampen = FollowConstants::splineCompleteVelocityDampen;
  private RealFunction taskDampen = FollowConstants::splineTaskVelocityDampen;
  private double maxSpeed = FollowConstants.maxSpeed;
  private double maxCentrifugalAcceleration = FollowConstants.maxCentrifugalAcceleration;
  private boolean interpolateFromStart = FollowConstants.interpolateFromStart;

  private Spline spline;

  // the current parameterization is expensive to calculate, so we cache it
  private Optional<Double> currentParameterization = Optional.empty();
  private double currentLength = 0;
  private double previousTimestamp = 0;

  public Path(Entry<Pose2d> positionEntry,
      ArrayList<Pair<Translation2d, Optional<Task>>> points,
      Optional<Rotation2d> finalRotation,
      SplineInterpolator interpolator,
      RealFunction offsetDampen,
      RealFunction startDampen,
      RealFunction completeDampen,
      RealFunction taskDampen,
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
    Objects.requireNonNull(maxSpeed, "maxSpeed cannot be null");
    Objects.requireNonNull(maxCentrifugalAcceleration, "maxCentrifugalAcceleration cannot be null");
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");

    this.positionEntry = positionEntry;
    this.points = points;
    this.finalRotation = finalRotation;
    this.interpolator = interpolator;
    this.offsetDampen = offsetDampen;
    this.startDampen = startDampen;
    this.completeDampen = completeDampen;
    this.taskDampen = taskDampen;
    this.maxSpeed = maxSpeed;
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    this.interpolateFromStart = interpolateFromStart;

    // verify that tasks match the point they are zipped with. tasks could
    // theoretically be handled in a way that makes these checks irrelevant, but for
    // the purpose of simplicity, this sanity check will do.
    for (var point : points) {
      if (point.getSecond().isPresent()) {
        assert point.getSecond().get().getTargetTranslation() == point.getFirst()
            : "task target position does not match accompanying point";
      }
    }

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

    List<Task> upcomingTasks = getUpcomingTasks();
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

  public double getDesiredSpeed() {
    double offsetSpeed = maxSpeed
        * offsetDampen.sample(positionEntry.get().getTranslation().getDistance(getGoalPosition()));
    double completeSpeed = Math.min(offsetSpeed, completeDampen.sample(Math.abs(getLength() - spline.arcLength(1))));
    double startSpeed = Math.min(completeSpeed, startDampen.sample(currentLength));
    double centrifugalSpeed = Math.min(startSpeed,
        Math.sqrt(maxCentrifugalAcceleration / spline.curvature(getParameterization())));

    // TODO: smooth speed after a task ends
    List<Task> upcomingTasks = getUpcomingTasks();
    if (!upcomingTasks.isEmpty()) {
      return Math.min(centrifugalSpeed, taskDampen.sample(upcomingTasks.get(0).getRemainingLength(currentLength)));
    } else {
      return centrifugalSpeed;
    }
  }

  public Translation2d getDesiredVelocity() {
    Translation2d splineDerivative = spline.derivative(getParameterization());
    return splineDerivative.times(getDesiredSpeed() / splineDerivative.getNorm());
  }

  private List<Translation2d> getTranslations() {
    return points.stream().map(point -> point.getFirst()).toList();
  }

  private List<Task> getTasks() {
    return points.stream().filter(point -> point.getSecond().isPresent()).map(point -> point.getSecond().get())
        .toList();
  }

  private List<Task> getUpcomingTasks() {
    return getTasks().stream().filter(task -> task.isUpcoming(currentLength)).toList();
  }

  private List<Task> getActiveTasks() {
    return getTasks().stream().filter(task -> task.isActive(currentLength)).toList();
  }

  /**
   * Similar go {@link #getActiveTasks}, but if some subsystem has multiple active
   * tasks, only includes the first.
   * 
   * @return The next active tasks
   */
  private List<Task> getValidActiveTasks() {
    List<Task> activeTasks = getActiveTasks();
    List<Task> validActiveTasks = new ArrayList<Task>();
    HashSet<Subsystem> activeSubsystems = new HashSet<Subsystem>();
    for (Task task : activeTasks) {
      if (task.getRequirements().stream().anyMatch(subsystem -> activeSubsystems.contains(subsystem))) {
        continue;
      }

      activeSubsystems.addAll(task.getRequirements());
      validActiveTasks.add(task);
    }

    return validActiveTasks;
  }

  public void initialize() {
    currentLength = 0;
    previousTimestamp = MathSharedStore.getTimestamp();
    currentParameterization = Optional.empty();

    // construct the spline
    if (interpolateFromStart) {
      ArrayList<Translation2d> pointsWithStart = new ArrayList<Translation2d>();
      pointsWithStart.add(positionEntry.get().getTranslation());
      pointsWithStart.addAll(getTranslations());
      spline = interpolator.interpolatePoints(pointsWithStart);
    } else {
      spline = interpolator.interpolatePoints(getTranslations());
    }

    // initialize the tasks
    ListIterator<Task> it = getTasks().listIterator();
    while (it.hasNext()) {
      double index = (double) it.nextIndex();
      it.next().initialize(spline, spline.arcLength(index / (double) (points.size() - 1)));
    }

    // make sure that no task ends after a task that proceeds it. this allows us to
    // only deal with one task per subsystem at a time
    double earliestEnd = Double.MAX_VALUE;
    List<Task> tasks = getTasks();
    it = tasks.listIterator(tasks.size());
    while (it.hasPrevious()) {
      Task nextTask = it.previous();
      earliestEnd = Double.min(nextTask.getEndLength(), earliestEnd);

      nextTask.setEndLength(earliestEnd);
    }
  }

  public void advance() {
    double newTimestamp = MathSharedStore.getTimestamp();
    currentLength += (newTimestamp - previousTimestamp) * getDesiredSpeed();
    currentParameterization = Optional.empty();
    previousTimestamp = newTimestamp;

    startTasks();
  }

  /**
   * Starts any active tasks. <b> Already run by {@link #advance}. </b> This
   * method is separate from {@link #advance} only for testing purposes.
   */
  public void startTasks() {
    for (Task task : getValidActiveTasks()) {
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
    previousTimestamp = MathSharedStore.getTimestamp();
  }

  public boolean isComplete() {
    // TODO: should there be options to increase precision once the end of the
    // spline is reached?
    return currentLength >= spline.arcLength(1);
  }
}
