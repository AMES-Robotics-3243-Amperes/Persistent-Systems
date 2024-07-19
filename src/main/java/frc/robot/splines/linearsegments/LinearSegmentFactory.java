package frc.robot.splines.linearsegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;
import frc.robot.splines.SplineSegmentFactory;

/**
 * A factory that creates {@link LinearSegment LinearSegments}.
 * 
 * @author :3
 */
public class LinearSegmentFactory extends SplineSegmentFactory {
  private Translation2d P0;
  private Translation2d P1;

  /**
   * Directly creates a linear segment from the given points.
   */
  public LinearSegmentFactory(Translation2d P0, Translation2d P1) {
    this.P0 = P0;
    this.P1 = P1;
  }

  @Override
  public SplineSegment build(SplineSegment _previousSegment) {
    return new LinearSegment(P0, P1);
  }
  
}
