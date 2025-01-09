package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * For interfacing with an IMU, intended to make switching
 * between IMUs as painless as possible.
 * 
 * @author :3
 */
public interface IMU {
  /** Returns a Rotation2d of rotation on the field @author :3 */
  Rotation2d getRotation();

  /**
   * Returns a Rotation2d of rotation on the field; angle between -pi and pi
   * 
   * @author :3
   */
  default Rotation2d getRotationModulus() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(getRotation().getRadians()));
  }
}
