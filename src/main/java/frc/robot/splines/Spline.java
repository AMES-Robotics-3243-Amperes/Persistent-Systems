package frc.robot.splines;

import edu.wpi.first.math.MathUtil;
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

  // TODO: tests
  public default double arcLength(double t) {
    return NumericalMethods.compositeGaussianQuadrature(x -> derivative(x).getNorm(), 0, t,
        NumericalConstants.newtonRaphsonIterations);
  }

  public default double parameterizationAtArcLength(double length) {
    return MathUtil.clamp(NumericalMethods.newtonRaphson(new DifferentiableFunction() {

      @Override
      public double sample(double x) {
        return arcLength(MathUtil.clamp(x, 0, 1)) - length;
      }

      @Override
      public double firstDerivative(double x) {
        return derivative(MathUtil.clamp(x, 0, 1)).getNorm();
      }

    }, length / arcLength(1), NumericalConstants.newtonRaphsonIterations), 0, 1);
  }
}
