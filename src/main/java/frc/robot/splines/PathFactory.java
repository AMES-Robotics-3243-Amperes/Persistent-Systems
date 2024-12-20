package frc.robot.splines;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.DataManager;
import frc.robot.Entry;
import frc.robot.Constants.SplineConstants.PathFactoryDefaults;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * Used to build a {@link Path}. See {@link #newFactory PathFactory.newFactory()}.
 */
public class PathFactory {
  private Entry<Pose2d> positionEntry = null;
  private ArrayList<Translation2d> points = new ArrayList<Translation2d>();
  private ArrayList<Task> tasks = new ArrayList<Task>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = PathFactoryDefaults.defaultInterpolator;
  private double maxSpeed = PathFactoryDefaults.defaultMaxSpeed;
  private double maxCentrifugalAcceleration = PathFactoryDefaults.defaultMaxCentrifugalAcceleration;
  private boolean interpolateFromStart = PathFactoryDefaults.defaultInterpolateFromStart;

  /**
   * This constructor is private for the sake of enforcing
   * sane (i.e. one line) construction of {@link Path Paths}.
   */
  private PathFactory() {}

  /**
   * Constructs a new {@link PathFactory}. Construction and usage should
   * optimally take place in a single line of code. For example:
   * 
   * <pre>
   * {@code Path path = PathFactory.newFactory()
   *  .addTranslation(...)
   *  .finalRotation(...)
   *  .interpolator(...)
   *  .build(); }
   * </pre>
   */
  public static PathFactory newFactory() {
    return new PathFactory();
  }

  public PathFactory positionEntry(Entry<Pose2d> positionEntry) {
    Objects.requireNonNull(positionEntry, "positionEntry cannot be null");
    this.positionEntry = positionEntry;
    return this;
  }

  public PathFactory useRobotPositionEntry() {
    this.positionEntry = DataManager.instance().robotPosition;
    return this;
  }

  public PathFactory addPoint(Translation2d point) {
    Objects.requireNonNull(point, "point cannot be null");
    points.add(point);
    return this;
  }

  public PathFactory addPoints(List<Translation2d> points) {
    Objects.requireNonNull(points, "point cannot be null");
    points.addAll(points);
    return this;
  }

  public PathFactory addTask(Task task) {
    Objects.requireNonNull(task, "task cannot be null");
    tasks.add(task);
    return this;
  }

  public PathFactory finalRotation(Rotation2d rotation) {
    this.finalRotation = Optional.ofNullable(rotation);
    return this;
  }

  public PathFactory finalRotation(Optional<Rotation2d> rotation) {
    this.finalRotation = rotation;
    return this;
  }

  public PathFactory interpolator(SplineInterpolator interpolator) {
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    this.interpolator = interpolator;
    return this;
  }

  public PathFactory maxSpeed(double maxSpeed) {
    Objects.requireNonNull(maxSpeed, "maxSpeed cannot be null");
    this.maxSpeed = maxSpeed;
    return this;
  }

  public PathFactory maxCentrifugalAcceleration(double maxCentrifugalAcceleration) {
    Objects.requireNonNull(maxCentrifugalAcceleration, "maxCentrifugalAcceleration cannot be null");
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    return this;
  }

  public PathFactory interpolateFromStart(boolean interpolateFromStart) {
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");
    this.interpolateFromStart = interpolateFromStart;
    return this;
  }

  public Path build() {
    return new Path(positionEntry, points, tasks, finalRotation, interpolator, maxSpeed, maxCentrifugalAcceleration, interpolateFromStart);
  }
}
