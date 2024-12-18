package frc.robot.splines;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DataManager;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.Constants.SplineConstants.PathFactoryDefaults;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * An arclength-parametrized path in 2D space for the robot to follow. Where a
 * {@link Spline} is simply a time-parameterized curve, a Path contains much
 * more utility and is a complete entity for a robot to follow.
 * Should generally be constructed using a {@link frc.robot.splines.PathFactory}.
 */
public class Path {
  private ArrayList<Translation2d> points = new ArrayList<Translation2d>();
  @SuppressWarnings("unused") // TODO: implement tasks
  private ArrayList<Task> tasks = new ArrayList<Task>();
  private Optional<Rotation2d> finalRotation = Optional.empty();
  private SplineInterpolator interpolator = PathFactoryDefaults.defaultInterpolator;
  private double maxVelocity = PathFactoryDefaults.defaultMaxVelocity;
  private double maxCentrifugalAcceleration = PathFactoryDefaults.defaultMaxCentrifugalAcceleration;
  private boolean interpolateFromStart = PathFactoryDefaults.defaultInterpolateFromStart;

  private Spline spline;
  private double currentLength = 0;
  private Timer timer = new Timer();

  public Path(ArrayList<Translation2d> points,
      ArrayList<Task> tasks,
      Optional<Rotation2d> finalRotation,
      SplineInterpolator interpolator,
      double maxVelocity,
      double maxCentrifugalAcceleration,
      boolean interpolateFromStart) {
    Objects.requireNonNull(points, "points cannot be null");
    Objects.requireNonNull(tasks, "tasks cannot be null");
    Objects.requireNonNull(finalRotation, "finalRotation cannot be null");
    Objects.requireNonNull(interpolator, "interpolator cannot be null");
    Objects.requireNonNull(maxVelocity, "maxVelocity cannot be null");
    Objects.requireNonNull(maxCentrifugalAcceleration, "maxCentrifugalAcceleration cannot be null");
    Objects.requireNonNull(interpolateFromStart, "interpolateFromStart cannot be null");

    this.points = points;
    this.tasks = tasks;
    this.finalRotation = finalRotation;
    this.interpolator = interpolator;
    this.maxVelocity = maxVelocity;
    this.maxCentrifugalAcceleration = maxCentrifugalAcceleration;
    this.interpolateFromStart = interpolateFromStart;
  }

  public double getLength() {
    return currentLength;
  }

  public Translation2d getGoalPosition() {
    double parameterization = spline.parameterizationAtArcLength(currentLength);
    return spline.at(parameterization);
  }

  public Optional<Rotation2d> getDesiredRotation() {
    return finalRotation;
  }

  public double getDesiredSpeed() {
    double velocity = maxVelocity *
      FollowConstants.splineOffsetVelocityDampen(DataManager.instance().robotPosition.get().getTranslation().getDistance(getGoalPosition()));
    return Math.min(Math.sqrt(maxCentrifugalAcceleration / spline.curvature(spline.parameterizationAtArcLength(currentLength))), velocity);
  }

  public Translation2d getDesiredVelocity() {
    double parameterization = spline.parameterizationAtArcLength(currentLength);
    Translation2d splineDerivative = spline.derivative(parameterization);
    return splineDerivative.times(getDesiredSpeed() / splineDerivative.getNorm());
  }

  public void initialize() {
    currentLength = 0;
    timer.restart();

    if (!interpolateFromStart) {
      spline = interpolator.interpolatePoints(points);
      return;
    }

    ArrayList<Translation2d> pointsWithStart = points;
    pointsWithStart.add(DataManager.instance().robotPosition.get().getTranslation());
    pointsWithStart.addAll(points);
  }

  public void advance() {
    currentLength += timer.get() * getDesiredSpeed();
    timer.restart();
  }

  public boolean isComplete() {
    // TODO: should there be options to increase precision once the end of the spline is reached?
    return currentLength >= spline.arcLength(1);
  }
}
