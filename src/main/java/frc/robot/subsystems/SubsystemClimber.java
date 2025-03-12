// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemClimber extends SubsystemBase {
  /** Creates a new SubsystemClimber. */
  SparkMax climberMotor = new SparkMax(21, MotorType.kBrushless);

  public SubsystemClimber() {
  }

 public void setVoltage(double volts){
   climberMotor.setVoltage(volts);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
