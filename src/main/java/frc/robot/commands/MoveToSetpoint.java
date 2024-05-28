// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataManager;
import frc.robot.subsystems.Drivetrain;

public class MoveToSetpoint extends Command {
  protected Drivetrain drivetrain;
  protected State targetState;
  protected static HolonomicDriveController driveController = 
  new HolonomicDriveController(
    new PIDController(0, 0, 0), 
    new PIDController(0, 0, 0), 
    new ProfiledPIDController(0, 0, 0, new Constraints(1, 1))
  );


  public MoveToSetpoint(Drivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveController.calculate(DataManager.currentRobotPose.get().toPose2d(), new State(), null);
    drivetrain.setVelocities(0, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
