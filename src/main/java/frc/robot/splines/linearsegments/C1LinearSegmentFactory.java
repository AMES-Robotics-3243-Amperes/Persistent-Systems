package frc.robot.splines.linearsegments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.SplineSegment;
import frc.robot.splines.SplineSegmentFactory;

/**
 * A factory that creates C1 Continuous {@link LinearSegment LinearSegments}.
 * 
 * @author :3
 */
public class C1LinearSegmentFactory extends SplineSegmentFactory {
  private Translation2d P1;

  /**
   * Directly creates a linear segment from the given points.
   */
  public C1LinearSegmentFactory(Translation2d P1) {
    this.P1 = P1;
  }

  @Override
  public SplineSegment buildOverride(SplineSegment previousSegment) {
    if (previousSegment == null)
      throw new UnsupportedOperationException("C1HermiteSegmentFactory cannot be first segment");

    return new LinearSegment(previousSegment.sample(1), P1);
  }
  
}
