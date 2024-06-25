package frc.robot.Curves;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CurveConstants;

/**
 * A single segment of a {@link Spline}.
 * 
 * @author :3
 */
public abstract class SplineSegment {
  /**
   * Samples the segment at a specific parameterization.
   * 
   * @param t The parameterization to sample at; should be between 0 and 1
   * @return The segment at the parameterization
   * 
   * @author :3
   */
  public abstract Translation2d sample(double t);

  /**
   * Samples the segment's derivative at a specific parameterization.
   * The derivative is not normalized.
   * 
   * @param t The parameterization to sample at; should be between 0 and 1
   * @return The derivative at the parameterization, with respect to t
   * 
   * @author :3
   */
  public abstract Translation2d derivative(double t);

  /**
   * Samples the segments's total arc length at a specific parameterization.
   * Namely, it finds the integral of the derivative's magnitude from 0 to t.
   * 
   * @param t The parameterization to sample at; should be between 0 and 1
   * @return The arc length at the parameterization
   * 
   * @author :3
   */
  public abstract double arcLength(double t);

  /**
   * Finds the segment's total arc length.
   * Is more efficient than arcLength(1) and/or cached for some types of segments.
   * 
   * @return arcLength(1)
   * 
   * @author :3
   */
  public abstract double totalArcLength();

  /**
   * Given some arc length L, finds the parameterization t at which arcLength(t) = L.
   * 
   * @param arcLength The arcLength to find the parameterization of
   * @return The parameterization that has arc length of the input
   * 
   * @author :3
   */
  public double timeAtArcLength(double arcLength) {
    // set the initial guess as if the function is linear
    return timeAtArcLength(arcLength, arcLength / totalArcLength());
  }

  /**
   * Works the same as {@link timeAtArcLength}, but uses an explicit
   * first guess for potentially more accurate results.
   * 
   * @param arcLength The arcLength to find the parameterization of
   * @param initialGuess The initial guess of the final parameterization
   * @return The parameterization that has arc length of the input
   * 
   * @author :3
   */
  public double timeAtArcLength(double arcLength, double initialGuess) {
    // apply newton's method to approximate the zero of
    // f(t) = arcLength(t) - length, where f'(t) is of course
    // the magnitude of the curve's derivative
    double guess = initialGuess;
    for (int i = 0; i < CurveConstants.newtonIterations; i++) {
      guess = guess - (arcLength(guess) - arcLength) / derivative(guess).getNorm();
    }

    return guess;
  }
}
