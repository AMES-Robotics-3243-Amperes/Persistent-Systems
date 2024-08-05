package frc.robot.splines.cubicsegments.bezierfactories;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;
import frc.robot.splines.SplineSegmentFactory;
import frc.robot.splines.cubicsegments.BezierSegment;

/**
 * A factory that creates C0 continuous spline segments.
 * 
 * @author :3
 */
public class C0CubicBezierSegmentFactory extends SplineSegmentFactory {
  private Translation2d P0;
  private Translation2d P1;
  private Translation2d P2;
  private Translation2d P3;

  /**
   * Creates a bezier segment from four control points. @author :3
   */
  public C0CubicBezierSegmentFactory(Translation2d P0,
      Translation2d P1,
      Translation2d P2,
      Translation2d P3) {
    this.P0 = P0;
    this.P1 = P1;
    this.P2 = P2;
    this.P3 = P3;
  }

  @Override
  public SplineSegment buildOverride(SplineSegment previousSegment) {
    return new BezierSegment(P0, P1, P2, P3);
  }
}
