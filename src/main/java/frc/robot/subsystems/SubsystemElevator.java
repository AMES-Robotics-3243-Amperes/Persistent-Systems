// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Elevator.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SubsystemElevator extends SubsystemBase {

  // H! Uses the parent class for spark maxes and spark flexes, so that they can be easily swapped. REV is the best (:
  private SparkBase motorLeader;
  private SparkBase motorFollower;

  private RelativeEncoder encoderLeader;
  @SuppressWarnings("unused")
  private RelativeEncoder encoderFollower;

  private SparkClosedLoopController controlLoop;

  /** Creates a new SubsystemElevator. */
  public SubsystemElevator() {
    motorLeader = new SparkFlex(Motors.leaderCanId, MotorType.kBrushless);
    motorFollower = new SparkFlex(Motors.followerCanId, MotorType.kBrushless);

    // Create configuration
    SparkBaseConfig leaderConfig = new SparkFlexConfig();
    SparkBaseConfig followerConfig = new SparkFlexConfig();

    leaderConfig.encoder
      .positionConversionFactor(Motors.positionConversionRatio)
      .velocityConversionFactor(Motors.positionConversionRatio);
    
    followerConfig.encoder
      .positionConversionFactor(Motors.positionConversionRatio)
      .velocityConversionFactor(Motors.positionConversionRatio);
    
    leaderConfig.closedLoop
      .pidf(Motors.P, Motors.I, Motors.D, Motors.FF);

    followerConfig.follow(motorLeader);

    // Apply configuration
    motorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set encoders
    encoderLeader = motorLeader.getEncoder();
    encoderFollower = motorFollower.getEncoder();

    // Set PID loop
    controlLoop = motorLeader.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ### Public API used by commands ###

  public void setPosition(double position) {
    position = clamp(minPos, maxPos, position);
    controlLoop.setReference(position, ControlType.kPosition);
  }

  public double getPosition() {
    return encoderLeader.getPosition();
  }

  public double getVelocity() {
    return encoderLeader.getVelocity();
  }

  private static double clamp(double min, double max, double x) {
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
  }
}
