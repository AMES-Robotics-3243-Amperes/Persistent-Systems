package frc.robot.splines.cubicsegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;

/**
 * {@link SplineSegment SplineSegments} that can be expressed as third-degree polynomial.
 * <a href="https://www.youtube.com/watch?v=jvPPXbo87ds">This video</a>
 * is a particularly good first introduction to cubic splines.
 * 
 * @author :3
 */
public class CubicSegment extends SplineSegment {
  private Translation2d degreeZeroTerm;
  private Translation2d degreeOneTerm;
  private Translation2d degreeTwoTerm;
  private Translation2d degreeThreeTerm;
  private double totalArcLength;

  /**
   * Creates a cubic segment given the coefficients of all 4 terms.
   * 
   * @author :3
   */
  public CubicSegment(Translation2d degreeZeroTerm,
      Translation2d degreeOneTerm,
      Translation2d degreeTwoTerm,
      Translation2d degreeThreeTerm) {
    this.degreeZeroTerm = degreeZeroTerm;
    this.degreeOneTerm = degreeOneTerm;
    this.degreeTwoTerm = degreeTwoTerm;
    this.degreeThreeTerm = degreeThreeTerm;
    totalArcLength = arcLength(1);
  }

  @Override
  public Translation2d sample(double t) {
    return degreeZeroTerm
      .plus(degreeOneTerm.times(t))
      .plus(degreeTwoTerm.times(t * t))
      .plus(degreeThreeTerm.times(t * t * t));
  }

  @Override
  public Translation2d derivative(double t) {
    return degreeOneTerm
      .plus(degreeTwoTerm.times(2 * t))
      .plus(degreeThreeTerm.times(3 * t * t));
  }

  @Override
  public double curvature(double t) {
    Translation2d derivative = derivative(t);
    Translation2d second_derivative = degreeTwoTerm.times(2)
      .plus(degreeThreeTerm.times(6 * t));

    double x1 = derivative.getX();
    double y1 = derivative.getY();
    double x2 = second_derivative.getX();
    double y2 = second_derivative.getY();

    double numerator = x1 * y2 - y1 * x2;
    double denominator_base = derivative.getNorm();
    double denominator = denominator_base * denominator_base * denominator_base;
  
    return Math.abs(numerator) / denominator;
  }

  @Override
  public double arcLength(double t) {
    /*
     * Uses gaussian quadrature with n = 5. (This is computationally
     * cheap enough and *extremely* accurate. That's important, as this eventually
     * gets fed into newton-raphson, adding yet another source of error).
     * 
     * See https://en.wikipedia.org/wiki/Gaussian_quadrature for more.
     */
    double termOne = 0.568889 * derivative(t / 2).getNorm();
    double termTwo = 0.478629 * derivative(t * 0.538469 / 2 + t / 2).getNorm();
    double termThree = 0.478629 * derivative(-t * 0.538469 / 2 + t / 2).getNorm();
    double termFour = 0.236927 * derivative(t * 0.90618 / 2 + t / 2).getNorm();
    double termFive = 0.236927 * derivative(-t * 0.90618 / 2 + t / 2).getNorm();

    return (t / 2) * (termOne + termTwo + termThree + termFour + termFive);
  }

  @Override
  public double totalArcLength() {
    return totalArcLength;
  }
}
