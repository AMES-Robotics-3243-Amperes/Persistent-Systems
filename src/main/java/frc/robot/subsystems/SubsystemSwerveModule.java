// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.PIDF;

/**
 * Represents a single module of an {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class SubsystemSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final SparkMaxConfig m_driveConfig;
  private final SparkMaxConfig m_turnConfig;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SimpleMotorFeedforward m_drivingFeedforwardController;
  private final PIDController m_drivingPIDController;
  private double m_drivingVelocitySetpoint = 0;

  private final SparkClosedLoopController m_turningPIDController;

  private final Rotation2d m_wheelOffset;

  /**
   * Constructs a {@link SubsystemSwerveModule}
   * 
   * @param drivingCANId the id of the {@link CANSparkMax} for driving
   * @param turningCANId the id of the {@link CANSparkMax} for turning
   * @param wheelOffset  the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public SubsystemSwerveModule(int drivingCANId, int turningCANId, Rotation2d wheelOffset) {
    // :3 initialize spark maxes
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // sio2 setup the motor configurations
    m_driveConfig = new SparkMaxConfig();
    m_turnConfig = new SparkMaxConfig();

    // sio2 apply idle modes and current limits
    m_driveConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turnConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
    m_driveConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turnConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // sio2 apply position/velocity conversion factors and other encoder configurations
    m_driveConfig.encoder.positionConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderPositionFactor);
    m_driveConfig.encoder.velocityConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderVelocityFactor);

    m_turnConfig.encoder.positionConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderPositionFactor);
    m_turnConfig.encoder.velocityConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderVelocityFactor);
    m_driveConfig.encoder.inverted(ModuleConstants.PhysicalProperties.kTurningEncoderInverted);

    // sio2 configure the pids
    m_turnConfig.closedLoop.pidf(
      ModuleConstants.PIDF.kTurningP,
      ModuleConstants.PIDF.kTurningI,
      ModuleConstants.PIDF.kTurningD,
      ModuleConstants.PIDF.kTurningFF
    );

    m_turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_turnConfig.closedLoop.outputRange(ModuleConstants.PIDF.kTurningMinOutput, ModuleConstants.PIDF.kTurningMaxOutput);

    // sio2 enable pid wrap on 0 to 2 pi, as the wheels rotate freely
    m_turnConfig.closedLoop.positionWrappingEnabled(true);
    m_turnConfig.closedLoop.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turnConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // sio2 set the uwu average and measurement period
    m_driveConfig.encoder.uvwAverageDepth(8);
    m_driveConfig.encoder.uvwMeasurementPeriod(8);
    m_turnConfig.encoder.uvwAverageDepth(8);

    // sio2 configure the motors
    m_drivingSparkMax.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // :3 setup encoders
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    // :3 setup pid and feedforward controllers
    m_drivingFeedforwardController = new SimpleMotorFeedforward(PIDF.kDrivingKs, PIDF.kDrivingKv, PIDF.kDrivingKv);
    m_drivingPIDController = new PIDController(PIDF.kDrivingP, PIDF.kDrivingI, PIDF.kDrivingD);

    // m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // :3 apply position and velocity conversion factors
    // m_drivingEncoder.setPositionConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderPositionFactor);
    // m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderVelocityFactor);
    // m_turningEncoder.setPositionConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderPositionFactor);
    // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderVelocityFactor);

    // :3 invert the turning encoder
    // m_turningEncoder.setInverted(ModuleConstants.PhysicalProperties.kTurningEncoderInverted);

    // :3 enable pid wrap on 0 to 2 pi, as the wheels rotate freely
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    // m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // :3 set p, i, and d terms for turning
    // m_turningPIDController.setP(ModuleConstants.PIDF.kTurningP);
    // m_turningPIDController.setI(ModuleConstants.PIDF.kTurningI);
    // m_turningPIDController.setD(ModuleConstants.PIDF.kTurningD);
    // m_turningPIDController.setFF(ModuleConstants.PIDF.kTurningFF);
    // m_turningPIDController.setOutputRange(ModuleConstants.PIDF.kTurningMinOutput, ModuleConstants.PIDF.kTurningMaxOutput);

    // :3 set idle modes and current limits
    // m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    // m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    // m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // m_drivingEncoder.setAverageDepth(8);
    // m_drivingEncoder.setMeasurementPeriod(8);
    // m_turningEncoder.setAverageDepth(8);

    // :3 reset driving encoder
    resetEncoder();

    // :3 set wheel offset
    m_wheelOffset = wheelOffset;
  }

  /**
   * Sets the desired state of the module
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // :3 apply wheel angular offset
    SwerveModuleState offsetState = new SwerveModuleState();

    offsetState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    offsetState.angle = desiredState.angle.plus(m_wheelOffset);

    // :3 optimize state to avoid turning more than 90 degrees
    Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getPosition());
    SwerveModuleState optimizedState = SwerveModuleState.optimize(offsetState, currentAngle);

    // :3 command driving
    m_drivingVelocitySetpoint = optimizedState.speedMetersPerSecond;
    m_turningPIDController.setReference(optimizedState.angle.getRadians(), SparkMax.ControlType.kPosition);
  }

  /**
   * Sets the desired state of the module. Does not optimize states.
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    Rotation2d offsetRotation = desiredRotation.plus(m_wheelOffset);
    m_turningPIDController.setReference(offsetRotation.getRadians(), SparkMax.ControlType.kPosition);
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double drivingPIDOutput = m_drivingPIDController.calculate(m_drivingEncoder.getVelocity(), m_drivingVelocitySetpoint);
    double drivingFFOutput = m_drivingFeedforwardController.calculate(m_drivingVelocitySetpoint);
    m_drivingSparkMax.setVoltage(drivingPIDOutput + drivingFFOutput);
  }

  /**
   * Resets the module's driving encoder
   * 
   * @author :3
   */
  public void resetEncoder() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * @return the {@link SwerveModulePosition} of the module
   * 
   * @author :3
   */
  public SwerveModulePosition getPosition() {
    double position = m_drivingEncoder.getPosition();
    Rotation2d rotation = new Rotation2d(m_turningEncoder.getPosition() - m_wheelOffset.getRadians());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * @return Gets the output motor current of an inputted motor
   * 
   * @author :>
   */
  public double getMotorOutputCurrent() {
    return m_drivingSparkMax.getOutputCurrent();
  }

  /**
   * Drives the module with the specified voltage for the drive motor,
   * and sets the target rotation for the pivot motor to 0.
   * 
   * @param voltage The voltage to set the drive motor to
   */
  public void driveVoltage(double voltage) {
    m_drivingSparkMax.setVoltage(voltage);
    setDesiredRotation(new Rotation2d(0));
  }

  /**
   * Logs all info for this motor. For use with SysID.
   * 
   * @param motorLog The {@link MotorLog} to log with
   */
  public void driveLog(MotorLog motorLog) {
    motorLog.current(Units.Amp.of(m_drivingSparkMax.getOutputCurrent()));
    motorLog.linearPosition(Units.Meters.of(m_drivingEncoder.getPosition()));
    motorLog.linearVelocity(Units.MetersPerSecond.of(m_drivingEncoder.getVelocity()));
    motorLog.voltage(Units.Volts.of(m_drivingSparkMax.getBusVoltage() * m_drivingSparkMax.getAppliedOutput()));
  }
}