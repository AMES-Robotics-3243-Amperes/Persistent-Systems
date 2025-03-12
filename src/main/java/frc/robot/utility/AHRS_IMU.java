package frc.robot.utility;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class AHRS_IMU implements IMU {
  private AHRS ahrs = new AHRS(NavXComType.kMXP_SPI);

  @Override
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(-ahrs.getYaw());
  }
}
