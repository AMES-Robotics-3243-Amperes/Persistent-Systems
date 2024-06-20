package frc.robot.Curves;

import java.util.ArrayList;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Translation2d;

public class Spline {
  private ArrayList<SplineSegment> segments;

  public Spline(ArrayList<SplineSegment> segments) {
    this.segments = new ArrayList<SplineSegment>();
    segments.forEach((segment) -> this.segments.add(segment));
  }

  public Spline() {
    this(new ArrayList<SplineSegment>());
  }

  public void addSegment(SplineSegmentFactory factory) {
    SplineSegment segment = factory.build(segments.size() >= 1 ? segments.get(segments.size() - 1) : null);
    segments.add(segment);
  }

  public Translation2d sample(double t) {
    int index = (int) t;
    SplineSegment segment = segments.get(index);
    return segment.sample(t - Math.floor(t));
  }

  public Translation2d derivative(double t) {
    int index = (int) t;
    SplineSegment segment = segments.get(index);
    return segment.derivative(t - Math.floor(t));
  }

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
