package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CurveConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.splines.Spline;
import frc.robot.splines.SplineSegment;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveFollowSpline extends Command {
  private SubsystemSwerveDrivetrain drivetrain;
  private Timer timer = new Timer();

  private Spline spline;
  private double currentArcLength;
  private double previousParameterization;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  public CommandSwerveFollowSpline(SubsystemSwerveDrivetrain drivetrain,
      Spline spline,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    this.spline = spline;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.drivetrain = drivetrain;

    currentArcLength = 0;
    previousParameterization = 0;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    currentArcLength = 0;
    previousParameterization = 0;

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    xController.reset();
    yController.reset();
    thetaController.reset();

    timer.restart();
  }

  @Override
  public void execute() {
    Translation2d robotPosition = DataManager.instance().robotPosition.get().getTranslation();
    Rotation2d robotRotation = DataManager.instance().robotPosition.get().getRotation();

    double currentParameterization =
      spline.timeAtArcLength(currentArcLength, previousParameterization);
    Translation2d splineVelocity = spline.derivative(currentParameterization);
    Translation2d goalPosition = spline.sample(currentParameterization);
    SplineSegment segment = spline.segment(currentParameterization);

    // :3 in order to prevent falling catastrophically behind, slow progression
    // along the spline when the robot finds itself far off of the curve.
    double velocity = segment.metadata().velocity.get(currentParameterization - Math.floor(currentParameterization))
      * CurveConstants.splineOffsetVelocityDampen(robotPosition.getDistance(goalPosition));
    velocity = Math.min(Math.sqrt(CurveConstants.maxCentrifugalAcceleration / spline.curvature(currentParameterization)), velocity);

    double xValue = xController.calculate(robotPosition.getX() - goalPosition.getX());
    double yValue = yController.calculate(robotPosition.getY() - goalPosition.getY());
    Translation2d pidAdjustment = new Translation2d(xValue, yValue);
    
    Translation2d splineVelocityAdjusted = splineVelocity.times(velocity / splineVelocity.getNorm());
    Translation2d targetRobotVelocity = splineVelocityAdjusted.plus(pidAdjustment);

    double rotationSpeed = 0;
    if (segment.metadata().rotation.active()) {
      rotationSpeed = thetaController.calculate(robotRotation.getRadians(),
        segment.metadata().rotation.get(currentParameterization - Math.floor(currentParameterization)).getRadians());
    }

    Translation2d speeds = targetRobotVelocity.rotateBy(robotRotation.times(-1));
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);

    previousParameterization = currentParameterization;
    currentArcLength += velocity * timer.get();
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return currentArcLength >= spline.totalArcLength();
  }
}
