package frc.robot.Curves;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CurveConstants;

public abstract class SplineSegment {
  public abstract Translation2d sample(double t);
  public abstract Translation2d derivative(double t);
  public abstract double arcLength(double t);

  public double timeAtArcLength(double length) {
    // set the initial guess as if the function is linear
    return timeAtArcLength(length, length);
  }

  public double timeAtArcLength(double length, double initialGuess) {
    // apply newton's method to approximate the zero of
    // f(t) = arcLength(t) - length and f'(t) is of course
    // the magnitude of the curve's derivative
    double guess = initialGuess;
    for (int i = 0; i < CurveConstants.newtonIterations; i++) {
      guess = guess - (arcLength(guess) - length) / derivative(guess).getNorm();
    }

    return guess;
  }
}
