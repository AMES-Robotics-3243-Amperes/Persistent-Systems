package frc.robot.Curves.CubicSegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Curves.SplineSegment;
import frc.robot.Curves.SplineSegmentFactory;

/**
 * A factory that creates C2 continuous spline segments.
 * 
 * @author :3
 */
public class C2HermiteSegmentFactory extends SplineSegmentFactory {
  private Translation2d P1;
  private Translation2d V1;

  /**
   * Takes in the final velocity and control point. The first velocity and control point
   * are extrapolated from the previous segment. @author :3
   */
  public C2HermiteSegmentFactory(Translation2d P1, Translation2d V1) {
    this.P1 = P1;
    this.V1 = V1;
  }

  @Override
  public SplineSegment build(SplineSegment previousSegment) {
    if (previousSegment == null)
      throw new UnsupportedOperationException("C2HermiteSegmentFactory cannot be first segment");
    
    Translation2d P0 = previousSegment.sample(1);
    Translation2d V0 = previousSegment.derivative(1);
    return new BezierSegment(P0, P0.plus(V0.div(3)), P1.minus(V1.div(3)), P1);
  }
}
