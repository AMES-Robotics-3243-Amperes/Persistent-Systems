// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.automatics;

// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.SubsystemElevator;
// import frc.robot.subsystems.SubsystemSwerveDrivetrain;
// import frc.robot.DataManager.DataManagerEntry;
// import frc.robot.DataManager.Setpoint;
// import frc.robot.DataManager;
// import frc.robot.PhotonUnit;
// import frc.robot.Constants.Setpoints;
// import frc.robot.commands.claw.IntakeClawCommand;
// import frc.robot.subsystems.SubsystemClaw;

// /**
//  * Scores a coral in the reef automatically
//  * 
//  * @author Jasper Davidson
//  */
// public class ScoreIntakeAutoCommand extends SequentialCommandGroup {
//   /** Creates a new ScoreInReefCommand. */
//   public ScoreIntakeAutoCommand(
//       SubsystemSwerveDrivetrain drivetrain, SubsystemClaw diffClaw, SubsystemElevator elevator,
//       Setpoint reefPosition, PhotonUnit photonCamera, DataManagerEntry<Pose2d> odometry, double tagOffset, IntakeClawCommand clawCommand
//     ) {
//     addCommands(
//       new SequentialCommandGroup(
//         MoveToPositionUtility.alignToTag(photonCamera, drivetrain, odometry, diffClaw, elevator, reefPosition, tagOffset),
//         clawCommand
//       )
//     );
//   }
// }
