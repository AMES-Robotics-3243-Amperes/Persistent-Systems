package frc.robot.splines;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.DataManager;
import frc.robot.Entry;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.splines.NumericalMethods.RealFunction;
import frc.robot.splines.interpolation.SplineInterpolator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * Used to build a {@link Path}. See {@link #newFactory
 * PathFactory.newFactory()}.
 */
public class PathFactory {
  private Optional<Entry<Pose2d>> positionEntry = Optional.empty();
  private ArrayList<Translation2d> points = new ArrayList<Translation2d>();
  private ArrayList<Task> tasks = new ArrayList<Task>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = FollowConstants.defaultInterpolator;
  private RealFunction offsetDampen = FollowConstants::splineOffsetVelocityDampen;
  private RealFunction completeDampen = FollowConstants::splineCompleteVelocityDampen;
  private double maxSpeed = FollowConstants.maxSpeed;
  private double maxCentrifugalAcceleration = FollowConstants.maxCentrifugalAcceleration;
  private boolean interpolateFromStart = FollowConstants.interpolateFromStart;

  /**
   * This constructor is private for the sake of enforcing
   * sane (i.e. one line) construction of {@link Path Paths}.
   */
  private PathFactory() {
  }

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
    this.positionEntry = Optional.of(positionEntry);
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
    Objects.requireNonNull(rotation, "rotation cannot be null");
    this.finalRotation = rotation;
    return this;
  }

  public PathFactory interpolator(SplineInterpolator interpolator) {
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    this.interpolator = interpolator;
    return this;
  }

  public PathFactory offsetDampen(RealFunction offsetDampen) {
    Objects.requireNonNull(offsetDampen, "offsetDampen cannot be null");
    this.offsetDampen = offsetDampen;
    return this;
  }

  public PathFactory completeDampen(RealFunction completeDampen) {
    Objects.requireNonNull(completeDampen, "completeDampen cannot be null");
    this.completeDampen = completeDampen;
    return this;
  }

  public PathFactory maxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  public PathFactory maxCentrifugalAcceleration(double maxCentrifugalAcceleration) {
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    return this;
  }

  public PathFactory interpolateFromStart(boolean interpolateFromStart) {
    this.interpolateFromStart = interpolateFromStart;
    return this;
  }

  public Path build() {
    // we delay setting the default position entry until now
    // since we don't want to assume DataManager.instance() exists
    // if we don't absolutely need to
    if (this.positionEntry.isEmpty()) {
      this.positionEntry = Optional.of(DataManager.instance().robotPosition);
    }

    return new Path(positionEntry.get(), points, tasks, finalRotation, interpolator, offsetDampen, completeDampen,
        maxSpeed, maxCentrifugalAcceleration, interpolateFromStart);
  }

  public CommandSwerveFollowSpline buildCommand(SubsystemSwerveDrivetrain subsystem, PIDController xController,
      PIDController yController, PIDController thetaController) {
    return new CommandSwerveFollowSpline(subsystem, this.build(), xController, yController, thetaController);
  }
}
