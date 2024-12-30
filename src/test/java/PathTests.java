import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Entry;
import frc.robot.splines.Path;
import frc.robot.splines.PathFactory;
import frc.robot.splines.Spline;
import frc.robot.splines.interpolation.SplineInterpolator;

public class PathTests {
  final double sqrtTwo = Math.sqrt(2);

  public class MockInterpolator implements SplineInterpolator {
    public int parameterizationAtArcLengthCalls = 0;

    @Override
    public Spline interpolatePoints(List<Translation2d> points) {
      return new Spline() {

        @Override
        public Translation2d at(double t) {
          return new Translation2d(t, t);
        }

        @Override
        public Translation2d derivative(double t) {
          return new Translation2d(1, 1);
        }

        @Override
        public double curvature(double t) {
          return t;
        }

        @Override
        public double arcLength(double t) {
          return t * sqrtTwo;
        }

        @Override
        public double parameterizationAtArcLength(double length) {
          parameterizationAtArcLengthCalls++;
          return length / sqrtTwo;
        }

      };
    }

  }

  public class MockInterpolatorTwo implements SplineInterpolator {
    public int parameterizationAtArcLengthCalls = 0;

    @Override
    public Spline interpolatePoints(List<Translation2d> points) {
      return new Spline() {

        @Override
        public Translation2d at(double t) {
          return points.get(0);
        }

        @Override
        public Translation2d derivative(double t) {
          return new Translation2d();
        }

        @Override
        public double curvature(double t) {
          return 0;
        }

      };
    }

  }

  public class MockEntry extends Entry<Pose2d> {
    Pose2d pose = new Pose2d();

    @Override
    public Pose2d get() {
      return pose;
    }

    @Override
    public void set(Pose2d newPose) {
      pose = newPose;
    }
  }

  @Test
  public void pathCorrectlySamplesSpline() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
        .positionEntry(entry)
        .interpolator(new MockInterpolator())
        .maxSpeed(Double.MAX_VALUE)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    assertEquals(new Translation2d(0, 0), path.getGoalPosition());

    path.advanceTo(0.5 * sqrtTwo);
    assertEquals(new Translation2d(0.5, 0.5), path.getGoalPosition());

    path.advanceTo(sqrtTwo);
    assertEquals(new Translation2d(1, 1), path.getGoalPosition());
  }

  @Test
  public void pathCorrectlyReportsEntry() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
        .positionEntry(entry)
        .interpolator(new MockInterpolator())
        .maxSpeed(Double.MAX_VALUE)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    entry.set(new Pose2d());
    assertEquals(new Pose2d(), path.getCurrentPosition());

    entry.set(new Pose2d(new Translation2d(0, 4), new Rotation2d(3)));
    assertEquals(new Pose2d(new Translation2d(0, 4), new Rotation2d(3)), path.getCurrentPosition());

    entry.set(new Pose2d(new Translation2d(3, -2), new Rotation2d(-1)));
    assertEquals(new Pose2d(new Translation2d(3, -2), new Rotation2d(-1)), path.getCurrentPosition());
  }

  @Test
  public void pathSetsSpeedToMaxVelocity() {
    Path path = PathFactory.newFactory()
        .positionEntry(new MockEntry())
        .interpolator(new MockInterpolator())
        .maxSpeed(1)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());

    path.advanceTo(0.5 * sqrtTwo);
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());
    assertEquals(1, path.getDesiredSpeed());

    path.advanceTo(sqrtTwo);
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());
  }

  @Test
  public void pathCorrectlyCapsCentrifugalAcceleration() {
    Path path = PathFactory.newFactory()
        .positionEntry(new MockEntry())
        .interpolator(new MockInterpolator())
        .maxSpeed(Double.MAX_VALUE)
        .maxCentrifugalAcceleration(1)
        .offsetDampen(x -> 1)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    assert Double.MAX_VALUE / 2 < path.getDesiredVelocity().getX();
    assert Double.MAX_VALUE / 2 < path.getDesiredVelocity().getY();

    path.advanceTo(0.5 * sqrtTwo);
    assertEquals(new Translation2d(1, 1), path.getDesiredVelocity());

    path.advanceTo(sqrtTwo);
    assertEquals(new Translation2d(1 / sqrtTwo, 1 / sqrtTwo), path.getDesiredVelocity());
  }

  @Test
  public void deviatingFromGoalLimitsSpeed() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
        .positionEntry(entry)
        .interpolator(new MockInterpolator())
        .maxSpeed(1)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1 / (x + 1))
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    entry.set(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    assertEquals(new Translation2d(1 / sqrtTwo, 1 / sqrtTwo), path.getDesiredVelocity());

    path.advanceTo(0.5 * sqrtTwo);
    entry.set(new Pose2d(new Translation2d(0.25, 0.25), new Rotation2d()));
    double component = 1 / (sqrtTwo * (1 + 1 / Math.sqrt(8)));
    assertEquals(new Translation2d(component, component), path.getDesiredVelocity());

    path.advanceTo(sqrtTwo);
    entry.set(new Pose2d(new Translation2d(1.5, 1.5), new Rotation2d()));
    component = 1 / (sqrtTwo * (1 + 1 / sqrtTwo));
    assertEquals(new Translation2d(component, component), path.getDesiredVelocity());
  }

  @Test
  public void endingPathLimitsSpeed() {
    Path path = PathFactory.newFactory()
        .positionEntry(new MockEntry())
        .interpolator(new MockInterpolator())
        .maxSpeed(1)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1)
        .completeDampen(x -> x)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    path.initialize();
    assertEquals(new Translation2d(1 / sqrtTwo, 1 / sqrtTwo), path.getDesiredVelocity());

    path.advanceTo(0.5 * sqrtTwo);
    assertEquals(new Translation2d(0.5, 0.5), path.getDesiredVelocity());

    path.advanceTo(sqrtTwo);
    assertEquals(new Translation2d(0, 0), path.getDesiredVelocity());
  }

  @Test
  public void parameterizationComputationKeptToMinimum() {
    MockInterpolator interpolator = new MockInterpolator();
    Path path = PathFactory.newFactory()
        .positionEntry(new MockEntry())
        .interpolator(interpolator)
        .maxSpeed(Double.MAX_VALUE)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> 1)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(false)
        .build();

    assertEquals(0, interpolator.parameterizationAtArcLengthCalls);

    path.initialize();
    path.getParameterization();
    path.getGoalPosition();
    path.getDesiredRotation();
    path.getDesiredSpeed();
    assert 1 >= interpolator.parameterizationAtArcLengthCalls;

    path.advanceTo(0.5 * sqrtTwo);
    path.getGoalPosition();
    path.getDesiredVelocity();
    path.getDesiredVelocity();
    path.getGoalPosition();
    assert 2 >= interpolator.parameterizationAtArcLengthCalls;

    path.advanceTo(sqrtTwo);
    path.getDesiredVelocity();
    path.getParameterization();
    path.getGoalPosition();
    path.getParameterization();
    assert 3 >= interpolator.parameterizationAtArcLengthCalls;
  }

  @Test
  public void interpolateFromStart() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
        .positionEntry(entry)
        .interpolator(new MockInterpolatorTwo())
        .maxSpeed(Double.MAX_VALUE)
        .maxCentrifugalAcceleration(Double.MAX_VALUE)
        .offsetDampen(x -> Double.MAX_VALUE)
        .completeDampen(x -> Double.MAX_VALUE)
        .taskDampen(x -> Double.MAX_VALUE)
        .interpolateFromStart(true)
        .build();

    Translation2d[] goalPositions = { new Translation2d(1, 1), new Translation2d(-1, 2), new Translation2d(1.5, -3) };

    for (Translation2d goalPosition : goalPositions) {
      entry.set(new Pose2d(goalPosition, new Rotation2d()));
      path.initialize();
      assertEquals(path.getGoalPosition(), goalPosition);
    }
  }
}
