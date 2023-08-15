// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataManager;
import frc.robot.IMUWrapper;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.utility.TranslationRateLimiter;

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

  // :3 driving speeds/information
  private Translation2d m_speeds;
  private Boolean m_drivingFieldRelative;
  private Double m_turnSpeedRadians;

  private Translation2d m_fieldSetpoint;
  private Rotation2d m_rotationSetpoint;

  // :3 rotation pid and rate limit
  private ProfiledPIDController m_rotationPidController = AutoConstants.kTurningPID;
  private SlewRateLimiter m_roatationLimiter =
    new SlewRateLimiter(DriveConstants.kMaxRotationAcceleration, -DriveConstants.kMaxRotationAcceleration, 0);

  // :3 driving rate limiter
  private TranslationRateLimiter m_drivingRateLimiter =
    new TranslationRateLimiter(new Translation2d(), DriveConstants.kMaxDrivingAcceleration);

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

  @Override
  public void periodic() {
    // :3 update odometry and feed that information into DataManager
    m_odometry.update(m_imuWrapper.getYaw(), getModulePositions());
    DataManager.currentRobotPose.updateWithOdometry(m_odometry.getPoseMeters());

    // :3 initialize speeds
    double rawXSpeed = 0.0;
    double rawYSpeed = 0.0;
    double rawRotationSpeed = 0.0;
    boolean fieldRelative = false;

    // :3 initialize movement speeds
    if (m_fieldSetpoint != null) {
      // TODO: write setpoint code
    } else if (m_speeds != null && m_drivingFieldRelative != null) {
      rawXSpeed = m_speeds.getX();
      rawYSpeed = m_speeds.getY();
      fieldRelative = m_drivingFieldRelative;
    }

    // :3 initialize rotation speeds
    if (m_rotationSetpoint != null) {
      // TODO: write setpoint code
    } else if (m_turnSpeedRadians != null) {
      rawRotationSpeed = m_turnSpeedRadians;
    }

    // :3 convert to field relative speeds
    double rawFieldRelativeXSpeed;
    double rawFieldRelativeYSpeed;
    if (!fieldRelative) {
      rawFieldRelativeXSpeed = rawXSpeed * getHeading().getCos() - rawYSpeed * getHeading().getSin();
      rawFieldRelativeYSpeed = rawXSpeed * getHeading().getSin() + rawYSpeed * getHeading().getSin();
    } else {
      rawFieldRelativeXSpeed = rawXSpeed;
      rawFieldRelativeYSpeed = rawYSpeed;
    }
    
    // :3 rate limit
    rawRotationSpeed = m_roatationLimiter.calculate(rawRotationSpeed);
    Translation2d rawFieldRelativeSpeeds = 
      m_drivingRateLimiter.calculate(new Translation2d(rawFieldRelativeXSpeed, rawFieldRelativeYSpeed));
    rawFieldRelativeXSpeed = rawFieldRelativeSpeeds.getX();
    rawFieldRelativeYSpeed = rawFieldRelativeSpeeds.getY();

    // :3 reset pids
    if (m_rotationSetpoint == null) {
      m_rotationPidController.reset(getHeading().getRadians(), rawRotationSpeed);
    }
    if (m_fieldSetpoint == null) {
      // TODO: reset driving pid
    }

    // :3 get module states
    SwerveModuleState[] swerveModuleStates = DriveConstants.ChassisKinematics.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(rawFieldRelativeXSpeed, rawFieldRelativeYSpeed, rawRotationSpeed, getHeading()));

    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the speeds the robot should drive at.
   * Nulls are okay and will be used as 0's.
   *
   * @param speeds a vector representing the speeds of the robot
   * @param rotationSpeed angular rate of the robot in radians
   * @param drivingFieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void driveWithSpeeds(Translation2d speeds, Double rotationSpeed, Boolean drivingFieldRelative) {
    m_speeds = speeds;
    m_turnSpeedRadians = rotationSpeed;
    m_drivingFieldRelative = drivingFieldRelative;

    // :3 if something's being controlled with speeds, null the setpoints
    if (speeds != null) {
      m_fieldSetpoint = null;
    }

    if (rotationSpeed != null) {
      m_rotationSetpoint = null;
    }
  }

  /**
   * drive the robot with setpoints
   *
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param rotation or goal of field relative driving
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void driveWithSetpoint(Translation2d fieldSetpoint, Rotation2d rotationSetpoint) {
    m_fieldSetpoint = fieldSetpoint;
    m_rotationSetpoint = rotationSetpoint;

    // :3 if something's being controlled with setpoints, null the speeds
    if (fieldSetpoint != null) {
      m_speeds = null;
    }

    if (rotationSetpoint != null) {
      m_turnSpeedRadians = null;
    }
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxObtainableModuleSpeed);

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