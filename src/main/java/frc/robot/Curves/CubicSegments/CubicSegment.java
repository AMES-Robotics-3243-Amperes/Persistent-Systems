package frc.robot.Curves.CubicSegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Curves.SplineSegment;

public class CubicSegment extends SplineSegment {
  public Translation2d degreeZeroTerm;
  public Translation2d degreeOneTerm;
  public Translation2d degreeTwoTerm;
  public Translation2d degreeThreeTerm;

  public CubicSegment(Translation2d degreeZeroTerm,
      Translation2d degreeOneTerm,
      Translation2d degreeTwoTerm,
      Translation2d degreeThreeTerm) {
    this.degreeZeroTerm = degreeZeroTerm;
    this.degreeOneTerm = degreeOneTerm;
    this.degreeTwoTerm = degreeTwoTerm;
    this.degreeThreeTerm = degreeThreeTerm;
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
  public double arcLength(double t) {
    // uses gaussian quadrature with n = 5. (computing 5 square
    // roots since we only have one curve to do this for)
    double termOne = 0.568889 * derivative(t / 2).getNorm();
    double termTwo = 0.478629 * derivative(t * 0.538469 / 2 + t / 2).getNorm();
    double termThree = 0.478629 * derivative(-t * 0.538469 / 2 + t / 2).getNorm();
    double termFour = 0.236927 * derivative(t * 0.90618 / 2 + t / 2).getNorm();
    double termFive = 0.236927 * derivative(-t * 0.90618 / 2 + t / 2).getNorm();

    return (t / 2) * (termOne + termTwo + termThree + termFour + termFive);
  }
}
