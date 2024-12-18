package frc.robot.splines.interpolation;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.splines.Spline;

public interface SplineInterpolator {
  public Spline interpolatePoints(List<Translation2d> points);
}
