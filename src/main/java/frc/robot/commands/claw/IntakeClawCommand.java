// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Setpoints.LevelAngles;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemClaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeClawCommand extends Command {
  private SubsystemClaw differentialArm;
  private double power;

  public IntakeClawCommand(SubsystemClaw differentialArm, double power /*, Ultrasonic rangeFinder */) {
    this.differentialArm = differentialArm;
    this.power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    differentialArm.setIntakePower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    differentialArm.resetFilter();
    differentialArm.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true when ultrasonic sensor detects decrease in distance, switch is triggered, etc.
    if (differentialArm.targetPivotPosition > LevelAngles.Intake - DifferentialArm.positionDelta && differentialArm.targetPivotPosition < LevelAngles.Intake + DifferentialArm.positionDelta) {
      if (differentialArm.smoothedCurrentDifference > DifferentialArm.currentDifferenceThreshold) {
        return true;
      }
    }

    return false;
  }
}
