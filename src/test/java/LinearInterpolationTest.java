import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;
import frc.robot.splines.interpolation.LinearInterpolator;

public class LinearInterpolationTest {
  @Test
  public void arcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(0, 1));
    points.add(new Translation2d(0, 0));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePoints(points);

    assertEquals(spline.arcLength(0), 0);
    assertEquals(spline.arcLength(0.125), 0.5);
    assertEquals(spline.arcLength(0.25), 1);
    assertEquals(spline.arcLength(0.55), 2.2);
    assertEquals(spline.arcLength(1), 4);
  }

  @Test
  public void parameterizationAtArcLength() {
    ArrayList<Translation2d> points = new ArrayList<Translation2d>();
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(-1, 2));
    points.add(new Translation2d(0, 0));

    LinearInterpolator interpolator = new LinearInterpolator();
    Spline spline = interpolator.interpolatePoints(points);

    double sqrtTwo = Math.sqrt(2);
    double sqrtFive = Math.sqrt(5);

    assertEquals(spline.parameterizationAtArcLength(0), 0);
    assertEquals(spline.parameterizationAtArcLength(1), 1 / (sqrtTwo * 3));
    assertEquals(spline.parameterizationAtArcLength(sqrtTwo), 1.0 / 3.0);
    assertEquals(spline.parameterizationAtArcLength(sqrtTwo + sqrtFive), 2.0 / 3.0);
    assertEquals(spline.parameterizationAtArcLength(sqrtTwo + (3.0 / 2.0) * sqrtFive), 2.5 / 3.0);
    assertEquals(spline.parameterizationAtArcLength(sqrtTwo + 2 * sqrtFive), 1);
  }
}
