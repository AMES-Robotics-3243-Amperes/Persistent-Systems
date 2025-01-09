import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;
import frc.robot.splines.interpolation.LinearInterpolator;

public class LinearInterpolationTests {
  @Test
  public void sample() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(5, 2));
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(-2, -1));
    points.add(new Translation2d(0, 1));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    assertEquals(new Translation2d(0, 0), spline.at(0));
    assertEquals(new Translation2d(5.0 / 2.0, 1), spline.at(0.125));
    assertEquals(new Translation2d(5, 2), spline.at(0.25));
    assertEquals(new Translation2d(-1, -1.0 / 2.0), spline.at(0.625));
    assertEquals(new Translation2d(-2, -1), spline.at(0.75));
    assertEquals(new Translation2d(0, 1), spline.at(1));
  }

  @Test
  public void derivative() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(-1, 3));
    points.add(new Translation2d(0, 0));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    assertEquals(new Translation2d(-1, 3), spline.derivative(0));
    assertEquals(new Translation2d(-1, 3), spline.derivative(0.2));
    AssertHelpers.assertEqualsOr(new Translation2d(-1, 3), new Translation2d(1, -3), spline.derivative(0.5), 1e-5);
    assertEquals(new Translation2d(1, -3), spline.derivative(0.7));
    assertEquals(new Translation2d(1, -3), spline.derivative(1));
  }

  @Test
  public void arcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(0, 1));
    points.add(new Translation2d(0, 0));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    assertEquals(0, spline.arcLength(0));
    assertEquals(0.5, spline.arcLength(0.125));
    assertEquals(1, spline.arcLength(0.25));
    assertEquals(2.2, spline.arcLength(0.55));
    assertEquals(4, spline.arcLength(1));
  }

  @Test
  public void parameterizationAtArcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(-1, 2));
    points.add(new Translation2d(0, 0));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePointsChecked(points);

    double sqrtTwo = Math.sqrt(2);
    double sqrtFive = Math.sqrt(5);

    assertEquals(0, spline.parameterizationAtArcLength(0));
    assertEquals(1 / (sqrtTwo * 3), spline.parameterizationAtArcLength(1));
    assertEquals(1.0 / 3.0, spline.parameterizationAtArcLength(sqrtTwo));
    assertEquals(2.0 / 3.0, spline.parameterizationAtArcLength(sqrtTwo + sqrtFive));
    assertEquals(2.5 / 3.0, spline.parameterizationAtArcLength(sqrtTwo + (3.0 / 2.0) * sqrtFive));
    assertEquals(1, spline.parameterizationAtArcLength(sqrtTwo + 2 * sqrtFive));
  }
}
