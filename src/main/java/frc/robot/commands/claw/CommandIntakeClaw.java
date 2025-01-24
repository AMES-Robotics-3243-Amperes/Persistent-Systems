// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemClaw.SetpointDiffArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandIntakeClaw extends Command {
  private SubsystemClaw differentialArm;
  private SetpointDiffArm setpoint;

  public CommandIntakeClaw(SubsystemClaw differentialArm, SetpointDiffArm setpoint /*, Ultrasonic rangeFinder */) {
    this.differentialArm = differentialArm;
    this.setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Setting intake");
    differentialArm.setIntakePower(setpoint.power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // differentialArm.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true when ultrasonic sensor detects decrease in distance, switch is triggered, etc.
    return true;
  }
}
