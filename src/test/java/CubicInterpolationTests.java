
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;
import frc.robot.splines.interpolation.CubicInterpolator;

public class CubicInterpolationTests {
  // used as a small distance while traversing the spline doing sanity checks
  private final double h = 0.005;

  @Test
  public void sample() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(2, 0));
    points.add(new Translation2d(-1, -5.5));
    points.add(new Translation2d(2, -0.5));

    CubicInterpolator interpolator = new CubicInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    AssertHelpers.assertEquals(new Translation2d(0, 0), spline.at(0), 1e-10);
    AssertHelpers.assertEquals(new Translation2d(1, 1), spline.at(0.25), 1e-10);
    AssertHelpers.assertEquals(new Translation2d(2, 0), spline.at(0.5), 1e-10);
    AssertHelpers.assertEquals(new Translation2d(-1, -5.5), spline.at(0.75), 1e-10);
    AssertHelpers.assertEquals(new Translation2d(2, -0.5), spline.at(1), 1e-10);

    // ensure continuity
    for (int i = 0; i < ((int) (1.0 / h)) - 1; i++) {
      AssertHelpers.assertEquals(spline.at(i * h), spline.at(i * h + h), 5 * h * spline.arcLength(1));
    }
  }

  @Test
  public void derivative() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(-1, 3));
    points.add(new Translation2d(0.5, 0));
    points.add(new Translation2d(-3, 2));
    points.add(new Translation2d(4, 3));

    CubicInterpolator interpolator = new CubicInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    // ensure forward-difference roughly agrees with the derivative
    for (int i = 0; i < ((int) (1.0 / h)) - 2; i++) {
      // this tolerance is a rough estimate of the forward-difference error
      // bound. we estimate the second derivative from the spline itself since
      // we cannot yet trust the accuracy of the derivative function
      double tolerance = spline.at(i * h + 2 * h).minus(spline.at(i * h + h).times(2)).plus(spline.at(i * h)).div(h * h)
          .getNorm() * h * 0.6;
      AssertHelpers.assertEquals(spline.at(i * h + h).minus(spline.at(i * h)).div(h), spline.derivative(i * h + h),
          tolerance);
    }
  }

  @Test
  public void arcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(0.5, 2));
    points.add(new Translation2d(-8, 6));
    points.add(new Translation2d(4, 2));
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(0, 1));

    CubicInterpolator interpolator = new CubicInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    double estimate = 0;
    for (int i = 1; i < ((int) (1 / h)); i++) {
      estimate += spline.derivative(i * h - h).plus(spline.derivative(i * h)).getNorm() * h / 2;
      assertEquals(estimate, spline.arcLength(i * h), h * 1e+1);
    }
  }

  @Test
  public void parameterizationAtArcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(11, 1));
    points.add(new Translation2d(13, 0));
    points.add(new Translation2d(12, -0.5));
    points.add(new Translation2d(13, -0.5));
    points.add(new Translation2d(12, -0.5));

    CubicInterpolator interpolator = new CubicInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    for (int i = 0; i < ((int) (1 / h)); i++) {
      assertEquals(i * h, spline.parameterizationAtArcLength(spline.arcLength(i * h)), 1e-2);
    }
  }

  @Test
  public void curvature() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(1, 2));
    points.add(new Translation2d(1, 3));
    points.add(new Translation2d(-2, 4));
    points.add(new Translation2d(-3, -1.5));

    CubicInterpolator interpolator = new CubicInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    // ensure forward-difference roughly agrees with the curvature
    for (int i = 0; i < ((int) (1.0 / h)) - 1; i++) {
      Translation2d unitOne = spline.derivative(i * h);
      Translation2d unitTwo = spline.derivative(i * h + h);
      unitOne = unitOne.div(unitOne.getNorm());
      unitTwo = unitTwo.div(unitTwo.getNorm());

      assertEquals(unitTwo.minus(unitOne).div(spline.arcLength(i * h + h) - spline.arcLength(i * h)).getNorm(),
          spline.curvature(i * h), h * 2e+1);
    }
  }
}
