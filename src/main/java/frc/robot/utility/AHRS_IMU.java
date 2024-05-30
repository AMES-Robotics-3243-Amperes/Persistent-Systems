package frc.robot.utility;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class AHRS_IMU implements IMU {
    private AHRS ahrs = new AHRS();

    @Override
    public Rotation2d getRotation() {
        return ahrs.getRotation2d();
    }
}
