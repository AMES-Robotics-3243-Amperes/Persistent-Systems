// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemClaw.SetpointDiffArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandTeleopClaw extends Command {
  private SubsystemClaw differentialClaw;
  private JoyUtil controller;
  private boolean aButtonWasPressed = false;
  private boolean bButtonWasPressed = false;

  /** Creates a new CommandTeleopClaw. */
  public CommandTeleopClaw(SubsystemClaw differentialClaw, JoyUtil controller) {
    this.controller = controller;
    this.differentialClaw = differentialClaw;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new CommandMoveClaw(differentialClaw, SetpointDiffArm.Starting));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If statements to control setpoint movements (CONTROLLER BUTTONS ARE PLACEHOLDERS RIGHT NOW)
    if (controller.getAButton()) {
      if (!aButtonWasPressed) {
        Commands.parallel(new CommandMoveClaw(differentialClaw, SetpointDiffArm.Intake),
        new CommandIntakeClaw(differentialClaw, SetpointDiffArm.Intake)).schedule();
      }
      aButtonWasPressed = true;
    } else {
      aButtonWasPressed = false;
    }
    
    if (controller.getBButton()) {
      if (!bButtonWasPressed) {
        Commands.parallel(new CommandMoveClaw(differentialClaw, SetpointDiffArm.Intake),
        new CommandIntakeClaw(differentialClaw, SetpointDiffArm.Intake)).schedule();
      }
      bButtonWasPressed = true;
    } else {
      bButtonWasPressed = false;
    }
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
