import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Entry;
import frc.robot.Constants.SplineConstants.FollowConstants;
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
        public double parameterizationAtArcLength(double length) {
          parameterizationAtArcLengthCalls++;
          return length / sqrtTwo;
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
      .build();

    path.initialize();
    assertEquals(new Translation2d(0, 0), path.getGoalPosition());

    path.advanceTo(0.5 * sqrtTwo);
    assertEquals(new Translation2d(0.5, 0.5), path.getGoalPosition());
  
    path.advanceTo(sqrtTwo);
    assertEquals(new Translation2d(1, 1), path.getGoalPosition());
  }

  @Test
  public void pathSetsSpeedToMaxVelocity() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
      .positionEntry(entry)
      .interpolator(new MockInterpolator())
      .maxSpeed(1)
      .maxCentrifugalAcceleration(Double.MAX_VALUE)
      .build();

    path.initialize();
    entry.set(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());

    path.advanceTo(0.5 * sqrtTwo);
    entry.set(new Pose2d(new Translation2d(0.5, 0.5), new Rotation2d()));
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());
    assertEquals(1, path.getDesiredSpeed());
  
    path.advanceTo(sqrtTwo);
    entry.set(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    assertEquals(new Translation2d(sqrtTwo / 2, sqrtTwo / 2), path.getDesiredVelocity());
  }

  @Test
  public void pathCorrectlyCapsCentrifugalAcceleration() {
    MockEntry entry = new MockEntry();
    Path path = PathFactory.newFactory()
      .positionEntry(entry)
      .interpolator(new MockInterpolator())
      .maxSpeed(Double.MAX_VALUE)
      .maxCentrifugalAcceleration(1)
      .build();
    
    path.initialize();
    entry.set(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    assert Double.MAX_VALUE / 2 < path.getDesiredVelocity().getX();
    assert Double.MAX_VALUE / 2 < path.getDesiredVelocity().getY();

    path.advanceTo(0.5 * sqrtTwo);
    entry.set(new Pose2d(new Translation2d(0.5, 0.5), new Rotation2d()));
    assertEquals(new Translation2d(1, 1), path.getDesiredVelocity());

    path.advanceTo(sqrtTwo);
    entry.set(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
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
      .build();
    
    path.initialize();
    entry.set(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    assertEquals(new Translation2d(1 / sqrtTwo, 1 / sqrtTwo), path.getDesiredVelocity());

    path.advanceTo(0.5 * sqrtTwo);
    entry.set(new Pose2d(new Translation2d(0.25, 0.25), new Rotation2d()));
    double component = FollowConstants.splineOffsetVelocityDampen(1 / Math.sqrt(8)) / sqrtTwo;
    assertEquals(new Translation2d(component, component), path.getDesiredVelocity());

    path.advanceTo(sqrtTwo);
    entry.set(new Pose2d(new Translation2d(1.5, 1.5), new Rotation2d()));
    component = FollowConstants.splineOffsetVelocityDampen(1 / Math.sqrt(2)) / sqrtTwo;
    assertEquals(new Translation2d(component, component), path.getDesiredVelocity());
  }

  @Test
  public void parameterizationComputationKeptToMinimum() {
    MockInterpolator interpolator = new MockInterpolator();
    Path path = PathFactory.newFactory()
      .positionEntry(new MockEntry())
      .interpolator(interpolator)
      .maxSpeed(Double.MAX_VALUE)
      .maxCentrifugalAcceleration(Double.MAX_VALUE)
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
}
