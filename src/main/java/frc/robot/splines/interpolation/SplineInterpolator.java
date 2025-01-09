package frc.robot.splines.interpolation;

import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;

public interface SplineInterpolator {
  public Spline interpolatePoints(List<Translation2d> points);

  /**
   * Ensures points are distributed evenly along the path. Should mainly be used
   * for testing purposes; the interpolation can be assumed to be fine if the
   * checks never fail in testing.
   */
  public default Spline interpolatePointsChecked(List<Translation2d> points) {
    Spline spline = interpolatePoints(points);

    for (ListIterator<Translation2d> it = points.listIterator(); it.hasNext();) {
      assert spline.at((double) it.nextIndex() / (double) (points.size() - 1)).getDistance(it.next()) < 1e-4;
    }

    return spline;
  }
}
