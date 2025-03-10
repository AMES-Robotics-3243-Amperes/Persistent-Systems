// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandClimber extends Command {
  /** Creates a new CommandClimber. */

  private JoyUtil controller;
  private SubsystemClimber climber;

  public CommandClimber(JoyUtil controller, SubsystemClimber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.climber = climber;
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(controller.getPOVUp()){
      climber.setVoltage(0.2);
    }
    else if(controller.getPOVDown()){
      climber.setVoltage(-0.2);
    }
    else{
      climber.setVoltage(0);
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
