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
  private SplineMetadata metadata = new SplineMetadata();

  /**
   * Builds a new {@link SplineSegment}, given a previous segment.
   * 
   * @param previousSegment The segment to build a new segment off of
   * @return A new segment
   * 
   * @author :3
   */
  protected abstract SplineSegment buildOverride(SplineSegment previousSegment);

  /**
   * Builds a new {@link SplineSegment}, given a previous segment. Adds metadata.
   * 
   * @param previousSegment The segment to build a new segment off of
   * @return A new segment
   * 
   * @author :3
   */
  public final SplineSegment build(SplineSegment previousSegment) {
    SplineSegment segment = buildOverride(previousSegment);
    segment.setMetadata(metadata);
    return segment;
  }

  /**
   * Adds metadata to the factory, which will add it to the segment on build.
   * 
   * @author :3
   */
  public void addMetadata(SplineMetadata metadata) {
    this.metadata = metadata;
  }
}
