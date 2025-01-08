package frc.robot.splines.linearsegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;

/**
 * Linear interpolation between two points to form a {@link SplineSegment}.
 * 
 * @author :3
 */
public class LinearSegment extends SplineSegment {
  private Translation2d P0;
  private Translation2d P1;

  /**
   * Creates a linear segment from two points. @author :3
   */
  public LinearSegment(Translation2d P0, Translation2d P1) {
    this.P0 = P0;
    this.P1 = P1;
  }

  @Override
  public Translation2d sample(double t) {
    return P0.interpolate(P1, t);
  }

  @Override
  public Translation2d derivative(double t) {
    return P1.minus(P0);
  }

  @Override
  public double curvature(double t) {
    return 0;
  }

  @Override
  public double arcLength(double t) {
    return P1.minus(P0).times(t).getNorm();
  }

  @Override
  public double timeAtArcLength(double arcLength) {
    return arcLength / arcLength(1);
  }

  @Override
  public double timeAtArcLength(double arcLength, double _initialGuess) {
    // since there's a simple expression for the arc length, the guess is irrelevant
    return timeAtArcLength(arcLength);
  }
}
