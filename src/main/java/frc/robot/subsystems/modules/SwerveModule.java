package frc.robot.subsystems.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;

public interface SwerveModule {
    public void setDesiredState(SwerveModuleState state);

    public void setDesiredRotation(Rotation2d roation);

    public SwerveModulePosition getPosition();

    public void update();

    public void driveVoltage(double voltage);

    public void driveLog(MotorLog motorLog);
}
