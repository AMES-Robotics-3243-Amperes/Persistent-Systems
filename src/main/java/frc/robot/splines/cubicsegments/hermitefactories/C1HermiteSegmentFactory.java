package frc.robot.splines.cubicsegments.hermitefactories;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;
import frc.robot.splines.SplineSegmentFactory;
import frc.robot.splines.cubicsegments.BezierSegment;

/**
 * A factory that creates C1 continuous spline segments.
 * 
 * @author :3
 */
public class C1HermiteSegmentFactory extends SplineSegmentFactory {
  private Translation2d V0;
  private Translation2d P1;
  private Translation2d V1;

  /**
   * Takes in both velocities and the final control point. The first control point
   * is extrapolated from the previous segment. @author :3
   */
  public C1HermiteSegmentFactory(Translation2d V0,
      Translation2d P1,
      Translation2d V1) {
    this.V0 = V0;
    this.P1 = P1;
    this.V1 = V1;
  }

  @Override
  public SplineSegment build(SplineSegment previousSegment) {
    if (previousSegment == null)
      throw new UnsupportedOperationException("C1HermiteSegmentFactory cannot be first segment");
    
    Translation2d P0 = previousSegment.sample(1);
    return new BezierSegment(P0, P0.plus(V0.div(3)), P1.minus(V1.div(3)), P1);
  }
}
