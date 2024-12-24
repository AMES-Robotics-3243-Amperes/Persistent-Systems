package frc.robot.splines.interpolation;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;

public class LinearInterpolator implements SplineInterpolator {
  public LinearInterpolator() {
  }

  @Override
  public Spline interpolatePoints(List<Translation2d> points) {
    assert points.size() > 1 : "linear interpolation requires at least two points";

    // all linear interpolation entails is multiplying our input by
    // the number of points we have (which makes each whole number
    // input correspond to a unique segment) and then just work with
    // the segment between points[index] and points[index + 1]
    return new Spline() {

      @Override
      public Translation2d at(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a linearly interpolated spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        return points.get(index).interpolate(points.get(index + 1), adjustedSample - index);
      }

      @Override
      public Translation2d derivative(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a linearly interpolated spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        return points.get(index + 1).minus(points.get(index));
      }

      @Override
      public double curvature(double t) {
        return 0;
      }

      @Override
      public double arcLength(double t) {
        assert t <= 1 && t >= 0 : "cannot sample a linearly interpolated spline at points not in [0, 1]";

        double adjustedSample = t * (points.size() - 1);
        int index = (int) Math.floor(adjustedSample);

        // ensure t = 1 works as expected
        if (index == points.size() - 1)
          index -= 1;

        double totalLength = 0;
        for (int i = 0; i < index; i++) {
          totalLength += points.get(i).getDistance(points.get(i + 1));
        }

        totalLength += points.get(index).getDistance(points.get(index + 1)) * (adjustedSample - index);
        return totalLength;
      }

      @Override
      public double parameterizationAtArcLength(double length) {
        // this works by traversing each segment and subtracting the length of that
        // segment from the total length. once a certain segment would put the length
        // at a negative value, we know that the parameterization we're looking for
        // is somewhere on that segment
        int lastIndex = points.size() - 2;
        for (int i = 0; i < points.size() - 2; i++) {
          double remainingLength = length - points.get(i).getDistance(points.get(i + 1));
          if (remainingLength < 0) {
            lastIndex = i;
            break;
          }

          length = remainingLength;
        }

        return (lastIndex + length / points.get(lastIndex).getDistance(points.get(lastIndex + 1)))
            / (points.size() - 1);
      }
    };
  }
}
