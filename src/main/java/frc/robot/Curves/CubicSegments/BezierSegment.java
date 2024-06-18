package frc.robot.Curves.CubicSegments;

import edu.wpi.first.math.geometry.Translation2d;

public class BezierSegment extends CubicSegment {
  public BezierSegment(Translation2d P0,
      Translation2d P1,
      Translation2d P2,
      Translation2d P3) {
    super(P0,
      P0.times(-3).plus(P1.times(3)),
      P0.times(3).plus(P1.times(-6)).plus(P2.times(3)),
      P0.times(-1).plus(P1.times(3)).plus(P2.times(-3)).plus(P3));
  }
}
