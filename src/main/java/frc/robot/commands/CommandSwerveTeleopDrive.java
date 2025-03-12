// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private boolean fieldRelative = true;
  private boolean redAlliance = false;

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

  public void toggleFieldRelative() {
    this.fieldRelative = !this.fieldRelative;
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      redAlliance = (alliance.get() == Alliance.Red);
    }
  }

  @Override
  public void execute() {
    Translation2d speeds = controller.getLeftAxis().times(ControlConstants.movingSpeed).times(reverse ? 1 : -1);
    speeds = new Translation2d(speeds.getY(), speeds.getX()); // :3 convert to robot coordinates

    if (fieldRelative && redAlliance)
      speeds = speeds.rotateBy(Rotation2d.fromDegrees(180));
    if (fieldRelative)
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