import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;

public class DefaultSplineTests {
  double sqrtTwo = Math.sqrt(2);

  private class MockSpline implements Spline {

    @Override
    public Translation2d at(double t) {
      return new Translation2d(t * t + t, t * t + t);
    }

    @Override
    public Translation2d derivative(double t) {
      return new Translation2d(2 * t + 1, 2 * t + 1);
    }

    @Override
    public double curvature(double t) {
      return 0;
    }

  }

  @Test
  public void arcLength() {
    MockSpline spline = new MockSpline();

    assertEquals(0, spline.arcLength(0));
    assertEquals(sqrtTwo * 0.1 * 1.1, spline.arcLength(0.1), 1e-4);
    assertEquals(sqrtTwo * 0.5 * 1.5, spline.arcLength(0.5), 1e-4);
    assertEquals(sqrtTwo * 0.7 * 1.7, spline.arcLength(0.7), 1e-4);
    assertEquals(sqrtTwo * 2, spline.arcLength(1), 1e-4);
  }

  @Test
  public void parameterizationAtArcLength() {
    MockSpline spline = new MockSpline();

    assertEquals(0, spline.parameterizationAtArcLength(0), 1e-4);
    assertEquals(0.2, spline.arcLength(spline.parameterizationAtArcLength(0.2)), 1e-4);
    assertEquals(0.43, spline.arcLength(spline.parameterizationAtArcLength(0.43)), 1e-4);
    assertEquals(0.8, spline.arcLength(spline.parameterizationAtArcLength(0.8)), 1e-4);
    assertEquals(1, spline.arcLength(spline.parameterizationAtArcLength(1)), 1e-4);
    assertEquals(1 + sqrtTwo, spline.arcLength(spline.parameterizationAtArcLength(1 + sqrtTwo)), 1e-4);
    assertEquals(sqrtTwo * 2, spline.arcLength(spline.parameterizationAtArcLength(sqrtTwo * 2)), 1e-4);
  }
}
