package frc.robot.splines.Tasks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SplineConstants.TaskConstants;
import frc.robot.splines.Spline;

public class PerformAtTask extends Task {
  private double tolerance = 0;

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    super(targetTranslation, targetRotation, rotationTolerance, command);
    this.tolerance = positionTolerance;
  }

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Optional<Rotation2d> targetRotation, Command command) {
    this(targetTranslation, positionTolerance, targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    this(targetTranslation, positionTolerance, Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Rotation2d targetRotation, Command command) {
    this(targetTranslation, positionTolerance, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, double positionTolerance, Command command) {
    this(targetTranslation, positionTolerance, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, double positionTolerance, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    this(new Translation2d(targetX, targetY), positionTolerance, targetRotation, rotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, double positionTolerance, Optional<Rotation2d> targetRotation, Command command) {
    this(new Translation2d(targetX, targetY), positionTolerance, targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, double positionTolerance, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    this(new Translation2d(targetX, targetY), positionTolerance, Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, double positionTolerance, Rotation2d targetRotation, Command command) {
    this(new Translation2d(targetX, targetY), positionTolerance, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, double positionTolerance, Command command) {
    this(new Translation2d(targetX, targetY), positionTolerance, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Pose2d targetPosition, double positionTolerance, Rotation2d rotationTolerance, Command command) {
    this(targetPosition.getTranslation(), positionTolerance, Optional.of(targetPosition.getRotation()), rotationTolerance, command);
  }

  public PerformAtTask(Pose2d targetPosition, double positionTolerance, Command command) {
    this(targetPosition.getTranslation(), positionTolerance, Optional.of(targetPosition.getRotation()), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    this(targetTranslation, TaskConstants.defaultPerformAtTaskTolerance, targetRotation, rotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, Optional<Rotation2d> targetRotation, Command command) {
    this(targetTranslation, TaskConstants.defaultPerformAtTaskTolerance, targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    this(targetTranslation, TaskConstants.defaultPerformAtTaskTolerance, Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, Rotation2d targetRotation, Command command) {
    this(targetTranslation, TaskConstants.defaultPerformAtTaskTolerance, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Translation2d targetTranslation, Command command) {
    this(targetTranslation, TaskConstants.defaultPerformAtTaskTolerance, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, Optional<Rotation2d> targetRotation, Rotation2d rotationTolerance, Command command) {
    this(new Translation2d(targetX, targetY), TaskConstants.defaultPerformAtTaskTolerance, targetRotation, rotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, Optional<Rotation2d> targetRotation, Command command) {
    this(new Translation2d(targetX, targetY), TaskConstants.defaultPerformAtTaskTolerance, targetRotation, TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, Rotation2d targetRotation, Rotation2d rotationTolerance, Command command) {
    this(new Translation2d(targetX, targetY), TaskConstants.defaultPerformAtTaskTolerance, Optional.ofNullable(targetRotation), rotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, Rotation2d targetRotation, Command command) {
    this(new Translation2d(targetX, targetY), TaskConstants.defaultPerformAtTaskTolerance, Optional.ofNullable(targetRotation), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(double targetX, double targetY, Command command) {
    this(new Translation2d(targetX, targetY), TaskConstants.defaultPerformAtTaskTolerance, Optional.empty(), TaskConstants.defaultRotationTolerance, command);
  }

  public PerformAtTask(Pose2d targetPosition, Rotation2d rotationTolerance, Command command) {
    this(targetPosition.getTranslation(), TaskConstants.defaultPerformAtTaskTolerance, Optional.of(targetPosition.getRotation()), rotationTolerance, command);
  }

  public PerformAtTask(Pose2d targetPosition, Command command) {
    this(targetPosition.getTranslation(), TaskConstants.defaultPerformAtTaskTolerance, Optional.of(targetPosition.getRotation()), TaskConstants.defaultRotationTolerance, command);
  }

  @Override
  protected double calculateStartLength(Spline spline, double targetLength) {
    return targetLength - tolerance;
  }

  @Override
  protected double calculateEndLength(Spline spline, double targetLength) {
    return targetLength + tolerance;
  }

}
