package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CurveConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.splines.Spline;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveFollowSpline extends Command {
  private SubsystemSwerveDrivetrain drivetrain;
  private Timer timer = new Timer();

  private Spline spline;
  private double velocity;
  private double currentArcLength;
  private double previousArcLength;

  private PIDController xController;
  private PIDController yController;

  public CommandSwerveFollowSpline(SubsystemSwerveDrivetrain drivetrain,
      Spline spline,
      double velocity,
      PIDController xController,
      PIDController yController) {
    this.spline = spline;
    this.velocity = velocity;
    this.xController = xController;
    this.yController = yController;
    this.drivetrain = drivetrain;

    currentArcLength = 0;
    previousArcLength = 0;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    currentArcLength = 0;
    previousArcLength = 0;

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    xController.reset();
    yController.reset();

    timer.restart();
  }

  @Override
  public void execute() {
    Translation2d robotPosition = DataManager.instance().robotPosition.get().getTranslation();

    double currentParameterization =
      spline.timeAtArcLength(currentArcLength, previousArcLength + velocity * CurveConstants.halfLoopTime);
    Translation2d splineVelocity = spline.derivative(currentParameterization);
    Translation2d goalPosition = spline.sample(currentParameterization);

    // :3 in order to prevent falling catastrophically behind, slow progression
    // along the spline when the robot finds itself far off of the curve.
    double trueVelocity = velocity * CurveConstants.splineOffsetVelocityDampen(robotPosition.getDistance(goalPosition));

    double xValue = xController.calculate(robotPosition.getX() - goalPosition.getX());
    double yValue = yController.calculate(robotPosition.getY() - goalPosition.getY());
    Translation2d pidAdjustment = new Translation2d(xValue, yValue);
    
    Translation2d splineVelocityAdjusted = splineVelocity.times(trueVelocity / splineVelocity.getNorm());
    Translation2d targetRobotVelocity = splineVelocityAdjusted.plus(pidAdjustment);

    Translation2d speeds = targetRobotVelocity.rotateBy(DataManager.instance().robotPosition.get().getRotation().times(-1));
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), 0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);

    previousArcLength = currentArcLength;
    currentArcLength += trueVelocity * timer.get();
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return currentArcLength >= spline.totalArcLength();
  }
}
