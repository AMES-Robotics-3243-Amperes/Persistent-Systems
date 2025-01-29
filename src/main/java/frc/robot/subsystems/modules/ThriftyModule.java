// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import frc.robot.Constants.SwerveConstants.ModuleConstants.PIDF;

/**
 * Represents a single module of an {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class ThriftyModule implements SwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_turningSparkMax;

  private final AnalogEncoder m_turningEncoder;

  private final SimpleMotorFeedforward m_drivingFeedforwardController;
  private final PIDController m_turningPIDController;
  private final PIDController m_drivingPIDController;
  private double m_drivingVelocitySetpoint = 0;

  private final Rotation2d m_wheelOffset;

  double turningFactor = 2 * Math.PI * 25;
  double id = 0;

  /**
   * Constructs a {@link ThriftyModule}
   * 
   * @param drivingCANId the id of the {@link CANSparkMax} for driving
   * @param turningCANId the id of the {@link CANSparkMax} for turning
   * @param wheelOffset  the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public ThriftyModule(int drivingCANId, int turningCANId, int encoderID, Rotation2d wheelOffset) {
    id = encoderID;

    // :3 initialize spark maxes
    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // :3 setup pid and feedforward controllers
    m_drivingFeedforwardController = new SimpleMotorFeedforward(PIDF.kDrivingKs, PIDF.kDrivingKv, PIDF.kDrivingKv);
    m_drivingPIDController = new PIDController(PIDF.kDrivingP, PIDF.kDrivingI, PIDF.kDrivingD);

    drivingConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    drivingConfig.CurrentLimits.withSupplyCurrentLimit(70).withSupplyCurrentLimitEnable(true);

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
    //turningConfig.alternateEncoder
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
    //    .inverted(true)
    //    .positionConversionFactor(turningFactor) // radians
    //    .velocityConversionFactor(turningFactor / 60.0); // radians per second
    //turningConfig.closedLoop
    //    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    //    // These are example gains you may need to them for your own robot!
    //    .pid(PIDF.kTurningP, PIDF.kTurningI, PIDF.kTurningD)
    //    .outputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
    //    .positionWrappingEnabled(true)
    //    .positionWrappingInputRange(0, turningFactor);

    m_turningEncoder = new AnalogEncoder(encoderID);
    m_turningPIDController = new PIDController(PIDF.kTurningP, PIDF.kTurningI, PIDF.kTurningD);
    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

    // finish up configuration
    m_drivingTalonFX.getConfigurator().apply(drivingConfig);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    offsetState.optimize(new Rotation2d(m_turningEncoder.get()));

    // :3 command driving
    m_drivingVelocitySetpoint = offsetState.speedMetersPerSecond;
    m_turningPIDController.setSetpoint(offsetState.angle.getRadians());
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
    m_turningPIDController.setSetpoint(offsetRotation.getRadians());
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double drivingPIDOutput = m_drivingPIDController.calculate(m_drivingTalonFX.getVelocity().getValueAsDouble() / 5.79,
        m_drivingVelocitySetpoint);
    double drivingFFOutput = m_drivingFeedforwardController.calculate(m_drivingVelocitySetpoint);
    m_drivingTalonFX.setVoltage(drivingPIDOutput + drivingFFOutput);

    SmartDashboard.putNumber("module" + id, m_turningEncoder.get());
    
    double turningOutput = m_turningPIDController.calculate(m_turningEncoder.get() * turningFactor);
    m_turningSparkMax.setVoltage(turningOutput);
  }

  /**
   * @return the {@link SwerveModulePosition} of the module
   * 
   * @author :3
   */
  public SwerveModulePosition getPosition() {
    double position = m_drivingTalonFX.getPosition().getValueAsDouble();
    Rotation2d rotation = new Rotation2d(m_turningEncoder.get() * turningFactor - m_wheelOffset.getRadians());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * @return Gets the output motor current of an inputted motor
   * 
   * @author :>
   */
  public double getMotorOutputCurrent() {
    return m_drivingTalonFX.getSupplyCurrent().getValueAsDouble();
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

  /**
   * Logs all info for this motor. For use with SysID.
   * 
   * @param motorLog The {@link MotorLog} to log with
   */
  public void driveLog(MotorLog motorLog) {
    motorLog.current(Units.Amps.of(m_drivingTalonFX.getSupplyCurrent().getValueAsDouble()));
    motorLog.linearPosition(Units.Meters.of(m_drivingTalonFX.getPosition().getValueAsDouble()));
    motorLog.linearVelocity(Units.MetersPerSecond.of(m_drivingTalonFX.getVelocity().getValueAsDouble()));
    motorLog.voltage(Units.Volts.of(m_drivingTalonFX.getMotorVoltage().getValueAsDouble()));
  }
}