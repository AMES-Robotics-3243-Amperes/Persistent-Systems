package frc.robot.Curves;

import java.util.ArrayList;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A series of {@link SplineSegment SplineSegments} forming a
 * spline for a robot to follow.
 *
 * <p> The recommended way to construct splines is by using the
 * {@link addSegment} method, as it allows the spline to guarantee
 * it maintains the level of continuity provided by factories.
 * 
 * @author :3
 */
public class Spline {
  private ArrayList<SplineSegment> segments;

  /**
   * Creates a spline from an initial array of {@link SplineSegment SplineSegments}.
   * It's not recommended to provide more than perhaps one initial segment;
   * read {@link Spline} documentation for more information on adding segments.
   * 
   * @param segments The initial segments of the curve
   * 
   * @author :3
   */
  public Spline(ArrayList<SplineSegment> segments) {
    this.segments = new ArrayList<SplineSegment>();
    segments.forEach((segment) -> this.segments.add(segment));
  }

  /**
   * Creates a new, empty spline.
   * Read {@link Spline} documentation for information on adding segments.
   * 
   * @author :3
   */
  public Spline() {
    this(new ArrayList<SplineSegment>());
  }

  /**
   * Extends the spline by taking {@link SplineSegmentFactory SplineSegmentFactories},
   * providing the previous segment (or null in none exists) to factory.build(),
   * and appending the new {@link SplineSegment} to the spline.
   * 
   * @param factory The spline factory to construct the new segment from
   * 
   * @author :3
   */
  public void addSegment(SplineSegmentFactory factory) {
    SplineSegment segment = factory.build(segments.size() >= 1 ? segments.get(segments.size() - 1) : null);
    segments.add(segment);
  }

  /**
   * Samples the spline at a specific parameterization.
   * 
   * @param t The parameterization to sample at; every 1.0 is a new segment
   * @return The spline at the parameterization
   * 
   * @author :3
   */
  public Translation2d sample(double t) {
    int index = (int) t;
    SplineSegment segment = segments.get(index);
    return segment.sample(t - Math.floor(t));
  }

  /**
   * Samples the spline's derivative at a specific parameterization.
   * The derivative is not normalized.
   * 
   * @param t The parameterization to sample at; every 1.0 is a new segment
   * @return The derivative at the parameterization, with respect to t
   * 
   * @author :3
   */
  public Translation2d derivative(double t) {
    int index = (int) t;
    SplineSegment segment = segments.get(index);
    return segment.derivative(t - Math.floor(t));
  }

  /**
   * Samples the spline's total arc length at a specific parameterization.
   * Namely, it finds the integral of the derivative's magnitude from 0 to t.
   * 
   * @param t The parameterization to sample at; every 1.0 is a new segment
   * @return The arc length at the parameterization
   * 
   * @author :3
   */
  public double arcLength(double t) {
    double length = 0;
    ListIterator<SplineSegment> iterator = segments.listIterator();

    while (iterator.hasNext()) {
      if (t - 1 >= (double) iterator.nextIndex()) {
        length += iterator.next().arcLength(1);
      } else {
        length += iterator.next().arcLength(t - Math.floor(t));
        break;
      }
    }

    return length;
  }

  /**
   * Given some arc length L, finds the parameterization t at which arcLength(t) = L.
   * 
   * @param arcLength The arcLength to find the parameterization of
   * @return The parameterization that has arc length of the input
   * 
   * @author :3
   */
  public double timeAtArcLength(double arcLength) {
    double totalLength = 0;
    ListIterator<SplineSegment> iterator = segments.listIterator();

    while (iterator.hasNext()) {
      SplineSegment nextSegment = iterator.next();
      if (arcLength >= (double) totalLength + nextSegment.totalArcLength()) {
        totalLength += nextSegment.totalArcLength();
      } else {
        return iterator.previousIndex() + nextSegment.timeAtArcLength(arcLength - totalLength);
      }
    }

    return (double) segments.size();
  }
}
