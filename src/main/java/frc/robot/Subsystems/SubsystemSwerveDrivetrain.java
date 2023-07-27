// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.OperationNotSupportedException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataManager;
import frc.robot.IMUWrapper;
import frc.robot.Constants.DriveTrain.DriveConstants;

/**
 * Subsystem for a swerve drivetrain
 * 
 * @author :3
 */
public class SubsystemSwerveDrivetrain extends SubsystemBase {

  // :3 create swerve modules
  private final SubsystemSwerveModule m_frontLeft = new SubsystemSwerveModule(DriveConstants.IDs.kFrontLeftDrivingCanId,
    DriveConstants.IDs.kFrontLeftTurningCanId, DriveConstants.ModuleOffsets.kFrontLeftOffset);

  private final SubsystemSwerveModule m_frontRight = new SubsystemSwerveModule(DriveConstants.IDs.kFrontRightDrivingCanId,
    DriveConstants.IDs.kFrontRightTurningCanId, DriveConstants.ModuleOffsets.kFrontRightOffset);

  private final SubsystemSwerveModule m_rearLeft = new SubsystemSwerveModule(DriveConstants.IDs.kRearLeftDrivingCanId,
    DriveConstants.IDs.kRearLeftTurningCanId, DriveConstants.ModuleOffsets.kBackLeftOffset);

  private final SubsystemSwerveModule m_rearRight = new SubsystemSwerveModule(DriveConstants.IDs.kRearRightDrivingCanId,
    DriveConstants.IDs.kRearRightTurningCanId, DriveConstants.ModuleOffsets.kBackRightOffset);

  // :3 imu
  private final IMUWrapper m_imuWrapper = new IMUWrapper();

  /** :3 odometry for tracking robot pose */
  SwerveDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   * 
   * @author :3
   */
  public SubsystemSwerveDrivetrain() {
    // :3 initialize odometry
    m_odometry = new SwerveDriveOdometry(DriveConstants.ChassisKinematics.kDriveKinematics,
      DataManager.currentRobotPose.get().getRotation().toRotation2d(), getModulePositions());

    // :3 reset module driving encoders
    resetEncoders();
  }

  // :> Test

  @Override
  public void periodic() {
    // :3 update odometry and feed that information into DataManager
    m_odometry.update(m_imuWrapper.getYaw(), getModulePositions());
    DataManager.currentRobotPose.updateWithOdometry(m_odometry.getPoseMeters());
  }

  /**
   * Drives the robot with raw speeds
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotation      angular rate of the robot in radians
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  private void driveWithRawSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // :3 adjust the inputs if field relative is true
    // TODO: move field relative out of here
    SwerveModuleState[] swerveModuleStates = DriveConstants.ChassisKinematics.kDriveKinematics.toSwerveModuleStates(
      fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the speeds the robot should drive at.
   * Nulls are okay and will be used as 0's.
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotationSpeed angular rate of the robot in radians
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void drive(Double xSpeed, Double ySpeed, Double rotationSpeed, Boolean fieldRelative) {
    // TODO: store speeds somewhere and drive in periodic
    driveWithRawSpeeds(xSpeed, ySpeed, rotationSpeed, fieldRelative);
  }

  /**
   * drive the robot with a rotation setpoint
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotation      or goal of field relative driving
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void updateSetpoint(double xSpeed, double ySpeed, Rotation2d rotation, boolean fieldRelative) 
      throws OperationNotSupportedException {
    // TODO: actually write this
    throw new OperationNotSupportedException();
  }

  /**
   * Set wheels into an x position to prevent movement
   * 
   * @author :3
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // :3 desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDrivingSpeedDamper);

    // :3 set the desired states
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the {@link SubsystemSwerveModule}s' driving encoders
   * 
   * @author :3
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoder();
    m_rearLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_rearRight.resetEncoder();
  }

  /**
   * @return the positions of the swerve modules
   * 
   * @author :3
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_rearLeft.getPosition(), m_rearRight.getPosition()
    };
  }

  /**
   * Private helper function
   * 
   * @return the robot's heading
   * 
   * @author :3
   */
  private Rotation2d getHeading() {
    return DataManager.currentRobotPose.get().getRotation().toRotation2d();
  }

  /**
   * @return if the motors are at safe temperatures
   * 
   * @author :3
   */
  public boolean getMotorsOkTemperature() {
    return !(m_frontLeft.isTooHot() || m_frontRight.isTooHot() || m_rearLeft.isTooHot() || m_rearRight.isTooHot());
  }
}