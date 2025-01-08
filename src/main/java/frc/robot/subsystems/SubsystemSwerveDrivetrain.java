package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SwerveConstants.DriveTrainConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;

public class SubsystemSwerveDrivetrain extends SubsystemBase {
  private final SubsystemSwerveModule m_frontLeft = new SubsystemSwerveModule(
    DriveTrainConstants.IDs.kFrontLeftDrivingCanId,
    DriveTrainConstants.IDs.kFrontLeftTurningCanId, DriveTrainConstants.ModuleOffsets.kFrontLeftOffset);

  private final SubsystemSwerveModule m_frontRight = new SubsystemSwerveModule(
    DriveTrainConstants.IDs.kFrontRightDrivingCanId,
    DriveTrainConstants.IDs.kFrontRightTurningCanId, DriveTrainConstants.ModuleOffsets.kFrontRightOffset);

  private final SubsystemSwerveModule m_rearLeft = new SubsystemSwerveModule(
    DriveTrainConstants.IDs.kRearLeftDrivingCanId,
    DriveTrainConstants.IDs.kRearLeftTurningCanId, DriveTrainConstants.ModuleOffsets.kBackLeftOffset);

  private final SubsystemSwerveModule m_rearRight = new SubsystemSwerveModule(
    DriveTrainConstants.IDs.kRearRightDrivingCanId,
    DriveTrainConstants.IDs.kRearRightTurningCanId, DriveTrainConstants.ModuleOffsets.kBackRightOffset);

  public SubsystemSwerveDrivetrain() {}

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxObtainableModuleSpeed);

    // :3 set the desired states
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set the swerve modules' desired rotations. Does not optimize rotations.
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  public void setModuleRotations(Rotation2d[] desiredRotations) {
    // :3 set the desired states
    m_frontLeft.setDesiredRotation(desiredRotations[0]);
    m_frontRight.setDesiredRotation(desiredRotations[1]);
    m_rearLeft.setDesiredRotation(desiredRotations[2]);
    m_rearRight.setDesiredRotation(desiredRotations[3]);
  }

  /**
   * Used for pose estimation.
   * 
   * @author :3
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_rearLeft.getPosition(), m_rearRight.getPosition()
    };
  }

  @Override
  public void periodic() {
    m_frontLeft.update();
    m_frontRight.update();
    m_rearLeft.update();
    m_rearRight.update();
  }

  public SysIdRoutine driveRoutine = new SysIdRoutine(
    new Config(BaseUnits.VoltageUnit.of(0.75).per(BaseUnits.TimeUnit),
      BaseUnits.VoltageUnit.of(3),
      BaseUnits.TimeUnit.of(8),
      null),
    new Mechanism(
      this::sysIdDrive, 
      this::sysIdDriveLog, 
      this
    )
  );

  /**
   * Sets the modules' drive voltages to a specific value
   * 
   * @param voltage the value to set the drive voltages to
   */
  public void sysIdDrive(Voltage voltage) {
    m_frontLeft.driveVoltage(voltage.baseUnitMagnitude());
    m_frontRight.driveVoltage(voltage.baseUnitMagnitude());
    m_rearLeft.driveVoltage(voltage.baseUnitMagnitude());
    m_rearRight.driveVoltage(voltage.baseUnitMagnitude());
  }

  /**
   * Logs the motors' info. For use with SysID.
   * 
   * @param log The {@link SysIdRoutineLog} to log to
   */
  public void sysIdDriveLog(SysIdRoutineLog log) {
    m_frontLeft.driveLog(log.motor("front_left_drive"));
    m_frontRight.driveLog(log.motor("front_right_drive"));
    m_rearLeft.driveLog(log.motor("rear_left_drive"));
    m_rearRight.driveLog(log.motor("rear_right_drive"));
  }

  /**
   * @param direction The direction for a quasistatic routine to run
   * @return A quasistatic routine
   */
  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction);
  }

  /**
   * @param direction The direction for a dynamic routine to run
   * @return A dynamic routine
   */
  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }
}
