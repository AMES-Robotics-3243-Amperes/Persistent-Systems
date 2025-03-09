package frc.robot.commands.automatics;

import java.util.Iterator;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DataManager.DataManagerEntry;
import frc.robot.DataManager.Setpoint;
import frc.robot.Constants;
import frc.robot.PhotonUnit;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand.Position;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.FinishByTask;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.CommandWrapper;

/**
 * Aligns the robot to the nearest april tag
 * Intended to be used while the robot is close to the april tag it wants to
 * score at
 * 
 * @author Jasper Davidson
 */
public class MoveToPositionUtility {


    public static class AlignToTagCommand extends SequentialCommandGroup {
        // This has to be broken up into two parts: an instant command which
        // does all the logic to generate a command, and the actual command itself, 
        // which is generated and entered by the first command. So that the command
        // can be added at runtime, the second command is a command wrapper.
        // This means the command can be created and swapped in at runtime.
        private CommandWrapper resultantCommand = new CommandWrapper(new InstantCommand());
        // False now -> will throw a null pointer if no command is set before this is run.

        /**
         * Generates a command that encodes how to align the robot to a given april tag
         * and reef level
         * 
         * @param photonUnit     - photon unit on the robot
         * @param drivetrain     - drivetrain the robot is using
         * @param odometryPose   - current robot position on the field
         * @param diffClaw       - claw the robot is using
         * @param elevator       - elevator the robot is using
         * @param targetSetpoint - target reef level to move to (L1, L2, L3, L4);
         *                       encodes arm rotation and elevator height
         * @return a command for the robot that encodes the spline/how to move along the
         *         spline, as well as move the elevator/arm
         */
        public AlignToTagCommand(SubsystemSwerveDrivetrain drivetrain,
                DataManagerEntry<Pose2d> odometryPose, SubsystemClaw diffClaw, SubsystemElevator elevator,
                Setpoint targetSetpoint, /*
                                        * Maybe abstract this away? If I have time I'll make it so it searches to find
                                        * which offset to use
                                        */ double tagOffset) {
            
            addCommands
            (
                new InstantCommand(() -> {
                    // Find the closest tag to the robot's current position
                    Pose2d currentPose = odometryPose.get();
                    Translation2d currentPosition = currentPose.getTranslation();
                    double minDistance = Double.MAX_VALUE;

                    // Initialize to any tag --> It *will* get overran (I hope there's a tag closer
                    // than Double.MAX_VALUE...)
                    Pose2d closestTag = FieldConstants.blueReef1.toPose2d();

                    for (AprilTag tag : FieldConstants.tagList) {
                        Pose2d tagPosition = tag.pose.toPose2d();
                        double potentialMin = (tagPosition.getTranslation()).getDistance(currentPosition);

                        if (potentialMin < minDistance) {
                            minDistance = potentialMin;
                            closestTag = tagPosition;
                        }
                    }

                    Pose2d targetPose = closestTag;

                    PathFactory pathFactory = PathFactory.newFactory();

                    moveToPositionTaskBuilder(targetPose, pathFactory, diffClaw, elevator, targetSetpoint, tagOffset);
                    pathFactory
                        .finalRotation(targetPose.getRotation())
                        .interpolateFromStart(true);
                    
                    
                    resultantCommand.setBackingCommand( pathFactory.buildCommand
                    (
                        drivetrain, 
                        FollowConstants.xyController(), 
                        FollowConstants.xyController(), 
                        FollowConstants.thetaController()
                    ));
                }),
                resultantCommand
            );
            
            
        }
    }
    

    public static void moveToPositionTaskBuilder(Pose2d targetPosition, PathFactory pathFactory,
            SubsystemClaw diffClaw, SubsystemElevator elevator, Setpoint targetSetpoint, double tagOffset) {
        Translation2d targetTranslation = new Translation2d(targetPosition.getX(), targetPosition.getY());
        Rotation2d targetRotation = targetPosition.getRotation();

        Rotation2d flippedRotation = targetRotation.plus(new Rotation2d(Math.PI));
        Pose2d angleTargetPose = new Pose2d(targetTranslation, flippedRotation);

        double distanceFromTag = 0.1;
        Transform2d offset = new Transform2d(new Translation2d(distanceFromTag, tagOffset), new Rotation2d());
        Pose2d targetPose = angleTargetPose.plus(offset);

        pathFactory.addTask(targetPose.getTranslation(), new FinishByTask(
                new ParallelCommandGroup(
                        new ElevatorMoveToPositionCommand(elevator, targetSetpoint.height),
                        new InstantCommand(
                                () -> {
                                    diffClaw.setOutsidePosition(targetSetpoint.angle);
                                },
                                diffClaw))));
    }

    public static CommandSwerveFollowSpline autoBuilder(List<Pose2d> targetPositions, PathFactory pathFactory,
            SubsystemClaw diffClaw, SubsystemElevator elevator, SubsystemSwerveDrivetrain drivetrain,
            List<Setpoint> targetSetpoints) {
        Iterator<Pose2d> positions = targetPositions.iterator();
        Iterator<Setpoint> setpoints = targetSetpoints.iterator();

        while (positions.hasNext() && setpoints.hasNext()) {
            Setpoint setpoint = setpoints.next();
            moveToPositionTaskBuilder(positions.next(), pathFactory, diffClaw, elevator, setpoints.next(),
                    setpoint.offset);
        }

        return pathFactory.interpolateFromStart(true).buildCommand(
                drivetrain, FollowConstants.xyController(), FollowConstants.xyController(),
                FollowConstants.thetaController());
    }
}
