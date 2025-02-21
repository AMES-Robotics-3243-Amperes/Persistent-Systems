package frc.robot.splines;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SplineConstants.NumericalConstants;
import frc.robot.splines.NumericalMethods.DifferentiableFunction;

/**
 * A curve in 2D space. For usage in {@link Path Paths}.
 */
public interface Spline {
  public Translation2d at(double t);

  public Translation2d derivative(double t);

  public double curvature(double t);

  public default double arcLength(double t) {
    return NumericalMethods.compositeGaussianQuadrature(x -> derivative(x).getNorm(), 0, t,
        NumericalConstants.compositeGaussianQuadratureIntervals);
  }

  public default double parameterizationAtArcLength(double length) {
    return NumericalMethods.newtonRaphsonBounded(new DifferentiableFunction() {

      @Override
      public double sample(double x) {
        return arcLength(x) - length;
      }

      @Override
      public double firstDerivative(double x) {
        return derivative(x).getNorm();
      }

    }, length / arcLength(1), 0, 1, NumericalConstants.newtonRaphsonIterations);
  }
}
