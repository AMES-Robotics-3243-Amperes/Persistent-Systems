// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.DataManager.Setpoint;
import frc.robot.PhotonUnit;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.subsystems.SubsystemClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInReefCommand extends SequentialCommandGroup {
  /** Creates a new ScoreInReefCommand. */
  public ScoreInReefCommand(
      SubsystemSwerveDrivetrain drivetrain, SubsystemClaw diffClaw, SubsystemElevator elevator,
      Setpoint reefPosition, PhotonUnit photonCamera, Supplier<Pose2d> odometry
    ) {
    addCommands(
      new SequentialCommandGroup(
        alignToClosestTag.alignToTag(photonCamera, drivetrain, odometry, diffClaw, elevator, reefPosition),
        new IntakeClawCommand(diffClaw, Setpoints.intakePower)
      )
    );
  }
}
