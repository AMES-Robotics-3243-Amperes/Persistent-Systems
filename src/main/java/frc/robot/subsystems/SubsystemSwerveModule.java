// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
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

    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // :3 setup pid and feedforward controllers
    m_drivingFeedforwardController = new SimpleMotorFeedforward(PIDF.kDrivingKs, PIDF.kDrivingKv, PIDF.kDrivingKv);
    m_drivingPIDController = new PIDController(PIDF.kDrivingP, PIDF.kDrivingI, PIDF.kDrivingD);

    // :3 setup turning encoder feedback sensor
    turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // :3 apply position and velocity conversion factors
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderPositionFactor);
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderVelocityFactor);
    turningConfig.encoder.positionConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderPositionFactor);
    turningConfig.encoder.velocityConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderVelocityFactor);

    // :3 invert the turning encoder
    turningConfig.inverted(ModuleConstants.PhysicalProperties.kTurningEncoderInverted);

    // :3 enable pid wrap on 0 to 2 pi, as the wheels rotate freely
    turningConfig.closedLoop.positionWrappingEnabled(true);
    turningConfig.closedLoop.positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput,
        ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // :3 configure turning pid
    turningConfig.closedLoop.pidf(ModuleConstants.PIDF.kTurningP, ModuleConstants.PIDF.kTurningI,
        ModuleConstants.PIDF.kTurningD, ModuleConstants.PIDF.kTurningFF);
    turningConfig.closedLoop.outputRange(ModuleConstants.PIDF.kTurningMinOutput,
        ModuleConstants.PIDF.kTurningMaxOutput);

    // :3 set idle modes and current limits
    drivingConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    drivingConfig.encoder.uvwAverageDepth(4);
    drivingConfig.encoder.quadratureMeasurementPeriod(8);
    turningConfig.encoder.uvwAverageDepth(8);

    // finish up configuration
    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

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
    offsetState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // :3 command driving
    m_drivingVelocitySetpoint = offsetState.speedMetersPerSecond;
    m_turningPIDController.setReference(offsetState.angle.getRadians(), ControlType.kPosition);
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
    m_turningPIDController.setReference(offsetRotation.getRadians(), ControlType.kPosition);
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double drivingPIDOutput = m_drivingPIDController.calculate(m_drivingEncoder.getVelocity(),
        m_drivingVelocitySetpoint);
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
    motorLog.current(Units.Amps.of(m_drivingSparkMax.getOutputCurrent()));
    motorLog.linearPosition(Units.Meters.of(m_drivingEncoder.getPosition()));
    motorLog.linearVelocity(Units.MetersPerSecond.of(m_drivingEncoder.getVelocity()));
    motorLog.voltage(Units.Volts.of(m_drivingSparkMax.getBusVoltage() * m_drivingSparkMax.getAppliedOutput()));
  }
}