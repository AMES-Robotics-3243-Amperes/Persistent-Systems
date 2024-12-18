package frc.robot.splines;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SplineConstants.PathFactoryDefaults;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * Used to build a {@link Path}. See {@link #newFactory PathFactory.newFactory()}.
 */
public class PathFactory {
  private ArrayList<Translation2d> points = new ArrayList<Translation2d>();
  private ArrayList<Task> tasks = new ArrayList<Task>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = PathFactoryDefaults.defaultInterpolator;
  private double maxVelocity = PathFactoryDefaults.defaultMaxVelocity;
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

  public void addPoint(Translation2d point) {
    Objects.requireNonNull(point, "point cannot be null");
    points.add(point);
  }

  public void addPoints(List<Translation2d> points) {
    Objects.requireNonNull(points, "point cannot be null");
    points.addAll(points);
  }

  public void addTask(Task task) {
    Objects.requireNonNull(task, "task cannot be null");
    tasks.add(task);
  }

  public void finalRotation(Rotation2d rotation) {
    this.finalRotation = Optional.ofNullable(rotation);
  }

  public void finalRotation(Optional<Rotation2d> rotation) {
    this.finalRotation = rotation;
  }

  public void interpolator(SplineInterpolator interpolator) {
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    this.interpolator = interpolator;
  }

  public void maxVelocity(double maxVelocity) {
    Objects.requireNonNull(maxVelocity, "maxVelocity cannot be null");
    this.maxVelocity = maxVelocity;
  }

  public void maxCentrifugalAcceleration(double maxCentrifugalAcceleration) {
    Objects.requireNonNull(maxCentrifugalAcceleration, "maxCentrifugalAcceleration cannot be null");
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
  }

  public void interpolateFromStart(boolean interpolateFromStart) {
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");
    this.interpolateFromStart = interpolateFromStart;
  }

  public Path build() {
    assert points.size() > 1 && (points.size() > 0 && interpolateFromStart): "path factory not given enough points";
    return new Path(points, tasks, finalRotation, interpolator, maxVelocity, maxCentrifugalAcceleration, interpolateFromStart);
  }
}
