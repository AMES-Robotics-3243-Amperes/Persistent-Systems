package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.splines.Path;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveFollowSpline extends Command {
  private SubsystemSwerveDrivetrain drivetrain;
  private Path path;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  public CommandSwerveFollowSpline(SubsystemSwerveDrivetrain drivetrain,
      Path path,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    this.path = path;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    path.initialize();
  }

  @Override
  public void execute() {
    Translation2d robotPosition = DataManager.instance().robotPosition.get().getTranslation();
    Rotation2d robotRotation = DataManager.instance().robotPosition.get().getRotation();
    Translation2d goal = path.getGoalPosition();

    double xValue = xController.calculate(robotPosition.getX() - goal.getX());
    double yValue = yController.calculate(robotPosition.getY() - goal.getY());
    Translation2d pidAdjustment = new Translation2d(xValue, yValue);

    double rotationSpeed = 0;
    if (path.getDesiredRotation().isPresent()) {
      rotationSpeed = thetaController.calculate(robotRotation.getRadians(), path.getDesiredRotation().get().getRadians());
    }

    Translation2d speeds = path.getDesiredVelocity().plus(pidAdjustment).rotateBy(robotRotation.times(-1));
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);

    path.advance();
  }

  @Override
  public boolean isFinished() {
    return path.isComplete();
  }
}
