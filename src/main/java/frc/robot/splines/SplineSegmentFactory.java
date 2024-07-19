package frc.robot.splines;

/**
 * Used in the construction of {@link SplineSegment SplineSegments} and
 * {@link Spline Splines}. Takes in some information, and is able to build
 * a new spline given a previous spline. See specific implementations
 * for more information.
 * 
 * @author :3
 */
public abstract class SplineSegmentFactory {
  /**
   * Builds a new {@link SplineSegment}, given a previous segment.
   * 
   * @param previousSegment The segment to build a new segment off of
   * @return A new segment
   * 
   * @author :3
   */
  public abstract SplineSegment build(SplineSegment previousSegment);
}
