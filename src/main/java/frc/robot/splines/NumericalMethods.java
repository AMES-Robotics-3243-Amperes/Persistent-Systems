package frc.robot.splines;

import edu.wpi.first.math.MathUtil;

public class NumericalMethods {
  public interface RealFunction {
    double sample(double x);
  }

  public interface DifferentiableFunction extends RealFunction {
    double firstDerivative(double x);
  }

  /**
   * Approximates a zero of a function using the Newton-Raphson method.
   * 
   * @param function     the function to approximate the zero of
   * @param initialGuess an initial guess of a zero - the closer the better, but
   *                     (most) guesses will be strictly improved
   * @param iterations   the number of iterations to run - the more the better,
   *                     but keep it realistic (2-3 is usually enough)
   * @return an approximate zero of the function
   */
  public static double newtonRaphson(DifferentiableFunction function, double initialGuess, int iterations) {
    assert iterations > 0 : "newton-raphson requires at least one iteration";

    double guess = initialGuess;
    for (int i = 0; i < iterations; i++) {
      guess = guess - (function.sample(guess) / function.firstDerivative(guess));
    }

    return guess;
  }

  /**
   * Approximates a zero of a function using the Newton-Raphson method.
   * 
   * @param function     the function to approximate the zero of
   * @param initialGuess an initial guess of a zero - the closer the better, but
   *                     (most) guesses will be strictly improved
   * @param lowerBound   a lower bound for the output
   * @param upperBound   a upper bound for the output
   * @param iterations   the number of iterations to run - the more the better,
   *                     but keep it realistic (2-3 is usually enough)
   * @return an approximate zero of the function
   */
  public static double newtonRaphsonBounded(DifferentiableFunction function, double initialGuess, double lowerBound,
      double upperBound, int iterations) {
    assert iterations > 0 : "newton-raphson requires at least one iteration";

    double guess = MathUtil.clamp(initialGuess, lowerBound, upperBound);
    for (int i = 0; i < iterations; i++) {
      guess = MathUtil.clamp(guess - (function.sample(guess) / function.firstDerivative(guess)), lowerBound,
          upperBound);
    }

    return guess;
  }

  /**
   * Approximates an integral using five-point Gaussian Quadrature.
   * 
   * @param function   the function to approximate the integral of
   * @param leftBound  left/lower bound of the integral
   * @param rightBound right/upper bound of the integral
   * @return the approximate integral of the function
   */
  public static double gaussianQuadrature(RealFunction function, double leftBound, double rightBound) {
    // gaussian quadrature works by intelligently choosing points on our function
    // and integrating
    // the polynomial that interpolates between them. in practice, implementing it
    // just involves
    // translating some simple formulas to code. for more information, see
    // https://en.wikipedia.org/wiki/Gaussian_quadrature
    double midpoint = (leftBound + rightBound) / 2;
    double scaleFactor = (rightBound - leftBound) / 2;

    double termOne = 0.568889 * function.sample(midpoint);
    double termTwo = 0.478629 * function.sample(scaleFactor * 0.538469 + midpoint);
    double termThree = 0.478629 * function.sample(-scaleFactor * 0.538469 + midpoint);
    double termFour = 0.236927 * function.sample(scaleFactor * 0.90618 + midpoint);
    double termFive = 0.236927 * function.sample(-scaleFactor * 0.90618 + midpoint);

    return scaleFactor * (termOne + termTwo + termThree + termFour + termFive);
  }

  /**
   * Approximates an integral using Composite Gaussian Quadrature. Gaussian
   * Quadrature is usually enough on its own, but Composite Gaussian Quadrature
   * further increases accuracy by using standard Gaussian Quadrature repeatedly
   * over sub-intervals of the function.
   * 
   * @param function   the function to approximate the integral of
   * @param leftBound  left/lower bound of the integral
   * @param rightBound right/upper bound of the integral
   * @param intervals  the number of intervals to use in the approximation
   * @return the approximate integral of the function
   */
  public static double compositeGaussianQuadrature(RealFunction function, double leftBound, double rightBound,
      int intervals) {
    double subIntervalLength = (rightBound - leftBound) / intervals;

    double totalIntegral = 0;
    for (int i = 0; i < intervals; i++) {
      totalIntegral += gaussianQuadrature(function, leftBound + i * subIntervalLength,
          leftBound + (i + 1) * subIntervalLength);
    }

    return totalIntegral;
  }
}
