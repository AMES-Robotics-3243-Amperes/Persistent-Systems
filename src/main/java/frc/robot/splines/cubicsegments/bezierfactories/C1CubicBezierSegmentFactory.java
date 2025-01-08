package frc.robot.splines.cubicsegments.bezierfactories;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;
import frc.robot.splines.SplineSegmentFactory;
import frc.robot.splines.cubicsegments.BezierSegment;

/**
 * A factory that creates C1 continuous spline segments.
 * 
 * @author :3
 */
public class C1CubicBezierSegmentFactory extends SplineSegmentFactory {
  private Translation2d P1;
  private Translation2d P2;
  private Translation2d P3;

  /**
   * Takes in the final three control points. The first control point
   * is extrapolated from the previous segment. @author :3
   */
  public C1CubicBezierSegmentFactory(Translation2d P1,
      Translation2d P2,
      Translation2d P3) {
    this.P1 = P1;
    this.P2 = P2;
    this.P3 = P3;
  }

  @Override
  public SplineSegment buildOverride(SplineSegment previousSegment) {
    if (previousSegment == null)
      throw new UnsupportedOperationException("C1CubicBezierSegmentFactory cannot be first segment");
    
    Translation2d P0 = previousSegment.sample(1);
    return new BezierSegment(P0, P1, P2, P3);
  }
}
