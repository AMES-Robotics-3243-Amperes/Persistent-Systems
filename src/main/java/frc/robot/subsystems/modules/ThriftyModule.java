// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.SwerveConstants.ModuleConstants.PIDF;

/**
 * Represents a single module of a {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class ThriftyModule implements SwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_azimuthSparkMax;

  private final AnalogEncoder m_azimuthEncoder;

  private final SimpleMotorFeedforward m_drivingFeedforwardController;
  private final PIDController m_drivingPIDController;
  private final PIDController m_azimuthPIDController;
  private LinearVelocity m_drivingVelocitySetpoint = LinearVelocity.ofRelativeUnits(0, Units.MetersPerSecond);

  private final Rotation2d m_azimuthOffset;

  private boolean doingSysID = false;

  // TODO: constants!
  private final double wheelRadiusMeters = edu.wpi.first.math.util.Units.inchesToMeters(2);
  private final double drivingConversionFactor = 1 / 5.14;

  /**
   * Constructs a {@link ThriftyModule}
   * 
   * @param drivingCANId the id of the {@link CANSparkMax} for driving
   * @param azimuthCANId the id of the {@link CANSparkMax} for azimuth control
   * @param azimuthOffset  the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public ThriftyModule(int drivingCANId, int azimuthCANId, int encoderID, Rotation2d azimuthOffset) {
    // :3 initialize spark maxes
    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_azimuthSparkMax = new SparkMax(azimuthCANId, MotorType.kBrushless);

    // get controllers and encoders
    m_drivingFeedforwardController = new SimpleMotorFeedforward(PIDF.kDrivingKs, PIDF.kDrivingKv, PIDF.kDrivingKv);
    m_drivingPIDController = new PIDController(PIDF.kDrivingP, PIDF.kDrivingI, PIDF.kDrivingD);
    
    m_azimuthEncoder = new AnalogEncoder(encoderID);
    m_azimuthPIDController = new PIDController(PIDF.kAzimuthP, PIDF.kAzimuthI, PIDF.kAzimuthD);
    m_azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // configure motors
    TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    SparkMaxConfig azimuthConfig = new SparkMaxConfig();

    drivingConfig.CurrentLimits
      .withSupplyCurrentLimit(70)
      .withSupplyCurrentLimitEnable(true);
    drivingConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake);
    drivingConfig.Feedback
      .withSensorToMechanismRatio(drivingConversionFactor)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    drivingConfig.MotionMagic
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(400)) // TODO: constants!
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(400));
    drivingConfig.Slot0
      .withKS(0)
      .withKV(0.1)
      .withKA(0)
      .withKP(0)
      .withKI(0)
      .withKD(0);

    azimuthConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

    // finish up configuration
    m_azimuthSparkMax.clearFaults();
    m_azimuthSparkMax.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drivingTalonFX.clearStickyFaults();
    m_drivingTalonFX.getConfigurator().apply(drivingConfig);

    m_drivingTalonFX.getVelocity().setUpdateFrequency(Frequency.ofRelativeUnits(51, Units.Hertz));
    m_drivingTalonFX.getPosition().setUpdateFrequency(Frequency.ofRelativeUnits(51, Units.Hertz));
    m_drivingTalonFX.getSupplyCurrent().setUpdateFrequency(Frequency.ofRelativeUnits(51, Units.Hertz));
    m_drivingTalonFX.getSupplyVoltage().setUpdateFrequency(Frequency.ofRelativeUnits(51, Units.Hertz));
    m_drivingTalonFX.optimizeBusUtilization();

    // set wheel offset
    m_azimuthOffset = azimuthOffset;
  }

  /**
   * Returns the azimuth encoder value as a {@link Rotaiton2d}.
   */
  private Rotation2d azimuthPosition() {
    return Rotation2d.fromRotations(m_azimuthEncoder.get());
  }

  /**
   * Returns the driving velocity as a {@link LinearVelocity}
   */
  private LinearVelocity drivingVelocity() {
    double radiansPerSecond = m_drivingTalonFX.getVelocity().getValue().times(drivingConversionFactor).in(Units.RadiansPerSecond);
    return LinearVelocity.ofRelativeUnits(radiansPerSecond * wheelRadiusMeters, Units.MetersPerSecond);
  }

  /**
   * Returns the driving position as a {@link Distance}
   */
  private Distance linearDrivingPosition() {
    double radians = m_drivingTalonFX.getPosition().getValue().times(drivingConversionFactor).in(Units.Radians);
    return Distance.ofRelativeUnits(radians * wheelRadiusMeters, Units.Meters);
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
    offsetState.angle = desiredState.angle.plus(m_azimuthOffset);
    offsetState.optimize(azimuthPosition());

    AngularVelocity desiredAngularVelocity = RadiansPerSecond.of(offsetState.speedMetersPerSecond / wheelRadiusMeters);
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    m_drivingTalonFX.setControl(request.withVelocity(desiredAngularVelocity));
  
    m_azimuthPIDController.setSetpoint(offsetState.angle.getRadians());
  }

  /**
   * Sets the desired state of the module. Does not optimize states.
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    Rotation2d offsetRotation = desiredRotation.plus(m_azimuthOffset);
    m_azimuthPIDController.setSetpoint(offsetRotation.getRadians());
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double desiredSpeedMetersPerSecond = m_drivingVelocitySetpoint.in(Units.MetersPerSecond)
      * Math.cos(m_azimuthPIDController.getSetpoint() - azimuthPosition().getRadians());

    double drivingPIDOutput = m_drivingPIDController.calculate(drivingVelocity().in(Units.MetersPerSecond),
        desiredSpeedMetersPerSecond);
    double drivingFFOutput = m_drivingFeedforwardController.calculate(desiredSpeedMetersPerSecond);
    
    double azimuthOutput = m_azimuthPIDController.calculate(azimuthPosition().getRadians());

    if (!doingSysID) m_drivingTalonFX.setVoltage(drivingPIDOutput + drivingFFOutput);
    m_azimuthSparkMax.setVoltage(azimuthOutput);
  }

  /**
   * @return the {@link SwerveModulePosition} of the module
   * 
   * @author :3
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(linearDrivingPosition().in(Units.Meters), azimuthPosition().minus(m_azimuthOffset));
  }

  /**
   * @return the {@link SwerveModulePosition} of the module without the offset included
   * 
   * @author :3
   */
  public SwerveModulePosition getAbsolutePosition() {
    return new SwerveModulePosition(linearDrivingPosition().in(Units.Meters), azimuthPosition());
  }

  /**
   * @return Gets the output motor current of an inputted motor
   * 
   * @author :>
   */
  public double getMotorOutputCurrent() {
    return m_drivingTalonFX.getSupplyCurrent().getValue().in(Units.Amps);
  }

  /**
   * Drives the module with the specified voltage for the drive motor,
   * and sets the target rotation for the pivot motor to 0.
   * 
   * @param voltage The voltage to set the drive motor to
   */
  public void driveVoltage(double voltage) {
    m_drivingTalonFX.setVoltage(voltage);
    setDesiredRotation(new Rotation2d(0));
  }

  public void setSysID(boolean doingSysID) {
    this.doingSysID = doingSysID;
  }
}