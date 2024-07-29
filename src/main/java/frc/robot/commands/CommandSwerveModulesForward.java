package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveModulesForward extends Command {
  SubsystemSwerveDrivetrain drivetrain;
  Timer timer = new Timer();

  public CommandSwerveModulesForward(SubsystemSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    Rotation2d rotation = new Rotation2d(0);
    Rotation2d[] rotations = {rotation, rotation, rotation, rotation};
    drivetrain.setModuleRotations(rotations);
    timer.restart();
  }

  @Override
  public void execute() {
    Rotation2d rotation = new Rotation2d(0);
    Rotation2d[] rotations = {rotation, rotation, rotation, rotation};
    drivetrain.setModuleRotations(rotations);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1.5;
  }

  @Override
  public void end(boolean interrupted) {
    Rotation2d rotation = new Rotation2d(0);
    Rotation2d[] rotations = {rotation, rotation, rotation, rotation};
    drivetrain.setModuleRotations(rotations);
  }
}
