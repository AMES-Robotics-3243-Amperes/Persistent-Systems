package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveXWheels extends Command {
  SubsystemSwerveDrivetrain drivetrain;
  public CommandSwerveXWheels(SubsystemSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    SwerveModuleState[] states = { new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)) };
    
    drivetrain.setModuleStates(states);
  }
}
