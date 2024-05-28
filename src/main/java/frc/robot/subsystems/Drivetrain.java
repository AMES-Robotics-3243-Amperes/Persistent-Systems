// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  SwerveModule frontRightModule;
  SwerveModule rearRightModule;
  SwerveModule frontLeftModule;
  SwerveModule rearLeftModule;

  SwerveDriveKinematics driveKinematics;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontRightModule = new SwerveModule(
      new CANSparkMax(0, MotorType.kBrushless),
      new CANSparkMax(10, MotorType.kBrushless),
      0
    );
    rearRightModule = new SwerveModule(
      new CANSparkMax(1, MotorType.kBrushless),
      new CANSparkMax(11, MotorType.kBrushless),
      0.25
    );
    frontLeftModule = new SwerveModule(
      new CANSparkMax(2, MotorType.kBrushless),
      new CANSparkMax(12, MotorType.kBrushless),
      0.75
    );
    rearLeftModule = new SwerveModule(
      new CANSparkMax(3, MotorType.kBrushless),
      new CANSparkMax(13, MotorType.kBrushless),
      0.5
    );

    driveKinematics = new SwerveDriveKinematics(
      new Translation2d(),
      new Translation2d(),
      new Translation2d(),
      new Translation2d()
    );
  }
  

  public void setStates(SwerveModuleState frontRightState, SwerveModuleState rearRightState, SwerveModuleState frontLeftState, SwerveModuleState rearLeftState) {
    frontRightModule.setState(frontRightState);
    rearRightModule.setState(rearRightState);
    frontLeftModule.setState(frontLeftState);
    rearLeftModule.setState(rearLeftState);
  }


  public void setVelocities(double x, double y, double theta) {
    driveKinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta));
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontRightModule.periodic();
    rearRightModule.periodic();
    frontLeftModule.periodic();
    rearLeftModule.periodic();
  }
}
