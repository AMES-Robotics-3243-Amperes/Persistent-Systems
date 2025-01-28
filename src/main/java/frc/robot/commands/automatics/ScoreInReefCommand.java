// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.DataManager.Setpoint;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.subsystems.SubsystemClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInReefCommand extends SequentialCommandGroup {
  /** Creates a new ScoreInReefCommand. */
  public ScoreInReefCommand(SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint reefPosition) {
    addCommands(
      // Make a parallel command with driving to align with april tag and raise elevator
      new ElevatorMoveToPositionCommand(elevator, reefPosition.height),
      new IntakeClawCommand(diffClaw, reefPosition.angle)
    );
  }
}
