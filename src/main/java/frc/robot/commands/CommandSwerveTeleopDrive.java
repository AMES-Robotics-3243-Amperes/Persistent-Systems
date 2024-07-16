// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.Constants.SwerveConstants.ControlConstants;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveTeleopDrive extends Command {

  // :3 subsystem
  private final SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;

  // :3 driver joyutil
  private final JoyUtil controller;

  // :3 teleop driving should be reversed depending on field side
  private boolean reverse = false;

  /**
   * Creates a new SwerveTeleopCommand.
   * 
   * @author :3
   */
  public CommandSwerveTeleopDrive(SubsystemSwerveDrivetrain subsystem, JoyUtil controller) {
    subsystemSwerveDrivetrain = subsystem;
    this.controller = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY() * (reverse ? -1 : 1) * ControlConstants.movingSpeed;
    double ySpeed = -controller.getLeftX() * (reverse ? -1 : 1) * ControlConstants.movingSpeed;
    Translation2d speeds = new Translation2d(xSpeed, ySpeed);
    speeds = speeds.rotateBy(DataManager.instance().robotPosition.get().getRotation().times(-1));

    double controllerRightX = controller.getRightX();
    double rotationSpeed = -controllerRightX * ControlConstants.rotationSpeed;

    // :3 drive with those speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    subsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    subsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}