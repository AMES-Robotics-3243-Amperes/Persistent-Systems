package frc.robot.splines.interpolation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SplineConstants.NumericalConstants;
import frc.robot.splines.NumericalMethods;
import frc.robot.splines.Spline;

public class CubicInterpolator implements SplineInterpolator {
  private List<CubicPolynomial> polynomials;

  private class CubicPolynomial {
    // a is the degree 3 term, b is the degree 2 term, and so on
    Translation2d a, b, c, d;

    public CubicPolynomial(Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
      this.a = a;
      this.b = b;
      this.c = c;
      this.d = d;
    }

    public Translation2d at(double t) {
      return a.times(t * t * t).plus(b.times(t * t)).plus(c.times(t)).plus(d);
    }

    public Translation2d derivative(double t) {
      return a.times(3 * t * t).plus(b.times(2 * t)).plus(c);
    }

    public double curvature(double t) {
      Translation2d derivative = derivative(t);
      Translation2d second_derivative = a.times(6 * t).plus(b.times(2));

      double x1 = derivative.getX();
      double y1 = derivative.getY();
      double x2 = second_derivative.getX();
      double y2 = second_derivative.getY();

      double numerator = x1 * y2 - y1 * x2;
      double denominator_base = derivative.getNorm();
      double denominator = denominator_base * denominator_base * denominator_base;

      return Math.abs(numerator) / denominator;
    }
  }

  public CubicInterpolator() {
  }

  /**
   * An internal helper function that generates a 1D natural spline.
   * 
   * @param samples the function samples (for our use case, the x or y coordinates
   *                of interpolated points)
   * @return an {@link List} containing lists for the a, b, c, and d
   *         coefficients. there will be a.size() - 1 useful entries; the last
   *         entry is trash and remains only to keep this function as simple as
   *         possible
   */
  private List<List<Double>> interpolateAxis(List<Double> samples) {
    // see https://www.math.ntnu.no/emner/TMA4215/2008h/cubicsplines.pdf
    // for a technical explanation (or just trust the unit tests smhing my head)

    double h = 1.0 / (samples.size() - 1);
    ArrayList<Double> alphas = new ArrayList<Double>();
    for (int i = 1; i < samples.size() - 1; i++) {
      alphas.add((samples.get(i + 1) - 2 * samples.get(i) + samples.get(i - 1)) * (3 / h));
    }

    List<Double> l = new ArrayList<Double>(Collections.nCopies(samples.size(), 1.0));
    List<Double> mu = new ArrayList<Double>(Collections.nCopies(samples.size(), 0.0));
    List<Double> z = new ArrayList<Double>(Collections.nCopies(samples.size(), 0.0));
    for (int i = 1; i < samples.size() - 1; i++) {
      l.set(i, 4 * h - h * mu.get(i - 1));
      mu.set(i, h / l.get(i));
      z.set(i, (alphas.get(i - 1) - h * z.get(i - 1)) / l.get(i));
    }

    List<Double> c = new ArrayList<Double>(Collections.nCopies(samples.size(), 0.0));
    List<Double> b = new ArrayList<Double>(Collections.nCopies(samples.size(), 0.0));
    List<Double> a = new ArrayList<Double>(Collections.nCopies(samples.size(), 0.0));
    for (int i = samples.size() - 2; i >= 0; i--) {
      b.set(i, z.get(i) - mu.get(i) * b.get(i + 1));
      c.set(i, (samples.get(i + 1) - samples.get(i)) / h - h * (b.get(i + 1) + 2 * b.get(i)) / 3.0);
      a.set(i, (b.get(i + 1) - b.get(i)) / (3 * h));
    }

    return List.of(a, b, c, samples);
  }

  @Override
  public Spline interpolatePoints(List<Translation2d> points) {
    assert points.size() > 1 : "cubic interpolation requires at least two points";

    // generate the spline
    List<Double> xs = points.stream().map(Translation2d::getX).collect(Collectors.toList());
    List<Double> ys = points.stream().map(Translation2d::getY).collect(Collectors.toList());
    List<List<Double>> xCoefficients = interpolateAxis(xs);
    List<List<Double>> yCoefficients = interpolateAxis(ys);

    polynomials = new ArrayList<CubicPolynomial>();
    for (int i = 0; i < points.size() - 1; i++) {
      Translation2d a = new Translation2d(xCoefficients.get(0).get(i), yCoefficients.get(0).get(i));
      Translation2d b = new Translation2d(xCoefficients.get(1).get(i), yCoefficients.get(1).get(i));
      Translation2d c = new Translation2d(xCoefficients.get(2).get(i), yCoefficients.get(2).get(i));
      Translation2d d = new Translation2d(xCoefficients.get(3).get(i), yCoefficients.get(3).get(i));
      polynomials.add(new CubicPolynomial(a, b, c, d));
    }

    // all cubic interpolation entails is multiplying our input by
    // the number of points we have (which makes each whole number
    // input correspond to a unique segment) and then just work with
    // the cubic segment between points[index] and points[index + 1]
    return new Spline() {

      @Override
      public Translation2d at(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a cubic spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        double h = t - (double) index / (double) (points.size() - 1);
        return polynomials.get(index).at(h);
      }

      @Override
      public Translation2d derivative(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a cubic spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        double h = t - (double) index / (double) (points.size() - 1);
        return polynomials.get(index).derivative(h);
      }

      @Override
      public double curvature(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a cubic spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        double h = t - (double) index / (double) (points.size() - 1);
        return polynomials.get(index).curvature(h);
      }

      public double arcLength(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a cubic spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        double integral = 0.0;
        for (int i = 0; i < index; i++) {
          final CubicPolynomial polynomial = polynomials.get(i);
          integral += NumericalMethods.compositeGaussianQuadrature(x -> polynomial.derivative(x).getNorm(),
              0, 1.0 / (points.size() - 1), NumericalConstants.compositeGaussianQuadratureIntervals);
        }

        final CubicPolynomial polynomial = polynomials.get(index);
        integral += NumericalMethods.compositeGaussianQuadrature(x -> polynomial.derivative(x).getNorm(), 0,
            t - (double) index / (double) (points.size() - 1), NumericalConstants.compositeGaussianQuadratureIntervals);
        return integral;
      }
      
    };
  }
}
