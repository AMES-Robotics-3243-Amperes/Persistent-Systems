package frc.robot.Curves.CubicSegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Curves.SplineSegment;
import frc.robot.Curves.SplineSegmentFactory;

public class C1CubicBezierSegmentFactory extends SplineSegmentFactory {
  public Translation2d P1;
  public Translation2d P2;
  public Translation2d P3;

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
  public SplineSegment build(SplineSegment previousSegment) {
    if (previousSegment == null)
      throw new UnsupportedOperationException("C1Continuous Bezier Factory cannot be first segment");
    
    Translation2d P0 = previousSegment.sample(1);
    return new BezierSegment(P0, P1, P2, P3);
  }
}
