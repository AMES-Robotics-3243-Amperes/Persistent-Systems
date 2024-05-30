package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
