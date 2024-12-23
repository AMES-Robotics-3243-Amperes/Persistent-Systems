package frc.robot.splines;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.Constants.SplineConstants.PathFactoryDefaults;
import frc.robot.Entry;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * An arclength-parametrized path in 2D space for the robot to follow. Where a
 * {@link Spline} is simply a time-parameterized curve, a Path contains much
 * more utility and is a complete entity for a robot to follow.
 * Should generally be constructed using a {@link frc.robot.splines.PathFactory}.
 */
public class Path {
  private Entry<Pose2d> positionEntry = null;
  private ArrayList<Translation2d> points = new ArrayList<Translation2d>();
  @SuppressWarnings("unused") // TODO: implement tasks
  private ArrayList<Task> tasks = new ArrayList<Task>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = PathFactoryDefaults.defaultInterpolator;
  private double maxSpeed = PathFactoryDefaults.defaultMaxSpeed;
  private double maxCentrifugalAcceleration = PathFactoryDefaults.defaultMaxCentrifugalAcceleration;
  private boolean interpolateFromStart = PathFactoryDefaults.defaultInterpolateFromStart;

  private Spline spline;

  // the current parameterization is expensive to calculate, so we cache it
  private Optional<Double> currentParameterization = Optional.empty();
  private double currentLength = 0;
  private Timer timer = new Timer();

  public Path(Entry<Pose2d> positionEntry,
      ArrayList<Translation2d> points,
      ArrayList<Task> tasks,
      Optional<Rotation2d> finalRotation,
      SplineInterpolator interpolator,
      double maxSpeed,
      double maxCentrifugalAcceleration,
      boolean interpolateFromStart) {
    Objects.requireNonNull(positionEntry, "positionEntry cannot be null");
    Objects.requireNonNull(points, "points cannot be null");
    Objects.requireNonNull(tasks, "tasks cannot be null");
    Objects.requireNonNull(finalRotation, "finalRotation cannot be null");
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    Objects.requireNonNull(maxSpeed, "maxSpeed cannot be null");
    Objects.requireNonNull(maxCentrifugalAcceleration, "maxCentrifugalAcceleration cannot be null");
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");

    this.positionEntry = positionEntry;
    this.points = points;
    this.tasks = tasks;
    this.finalRotation = finalRotation;
    this.interpolator = interpolator;
    this.maxSpeed = maxSpeed;
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    this.interpolateFromStart = interpolateFromStart;

    this.initialize();
  }

  public double getLength() {
    return currentLength;
  }

  public Pose2d getRobotPosition() {
    // TODO: make a test that makes sure this works
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
    return finalRotation;
  }

  public double getDesiredSpeed() {
    double baseSpeed = maxSpeed *
      FollowConstants.splineOffsetVelocityDampen(positionEntry.get().getTranslation().getDistance(getGoalPosition()));
    double dampenSpeed = Math.min(baseSpeed, 
      FollowConstants.splineCompleteVelocityDampen(Math.abs(getLength() - spline.arcLength(1))) * baseSpeed);
    double maxCentrifugalSpeed = Math.sqrt(maxCentrifugalAcceleration / spline.curvature(getParameterization()));
    return Math.min(maxCentrifugalSpeed, dampenSpeed);
  }

  public Translation2d getDesiredVelocity() {
    Translation2d splineDerivative = spline.derivative(getParameterization());
    return splineDerivative.times(getDesiredSpeed() / splineDerivative.getNorm());
  }

  public void initialize() {
    currentLength = 0;
    timer.restart();

    if (!interpolateFromStart) {
      spline = interpolator.interpolatePoints(points);
      return;
    }

    ArrayList<Translation2d> pointsWithStart = new ArrayList<Translation2d>();
    pointsWithStart.add(positionEntry.get().getTranslation());
    pointsWithStart.addAll(points);
    spline = interpolator.interpolatePoints(pointsWithStart);
  }

  public void advance() {
    currentLength += timer.get() * getDesiredSpeed();
    currentParameterization = Optional.empty();
    timer.restart();
  }

  public void advanceTo(double newLength) {
    currentLength = newLength;
    currentParameterization = Optional.empty();
    timer.restart();
  }

  public boolean isComplete() {
    // TODO: should there be options to increase precision once the end of the spline is reached?
    return currentLength >= spline.arcLength(1);
  }
}
