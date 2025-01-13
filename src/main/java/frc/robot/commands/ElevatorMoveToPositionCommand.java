// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemElevator;
import static frc.robot.Constants.Elevator.MoveToPosition.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveToPositionCommand extends Command {

  private SubsystemElevator elevator;
  private double target;
  private double deltaP;
  private double deltaV;

  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, double target, double deltaP, double deltaV) {
    this.elevator = elevator;
    this.target = target;
    this.deltaP = deltaP;
    this.deltaV = deltaV;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  /** Creates a new ClimberMoveToPositionCommand. */
  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, double target) {
    this(elevator, target, defaultDeltaP, defaultDeltaV);
  }

  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, Positions target) {
    this(elevator, target.position, defaultDeltaP, defaultDeltaV);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      Math.abs(elevator.getPosition() - target) < deltaP &&
      Math.abs(elevator.getVelocity()) < deltaV
    ;
  }

  public enum Positions {
    Starting(0.0),
    Loading(1.0),
    L1(DataManager.ElevatorSetpoint.L1.position),
    L2(DataManager.ElevatorSetpoint.L2.position),
    L3(DataManager.ElevatorSetpoint.L3.position),
    L4(DataManager.ElevatorSetpoint.L4.position);

    public final double position;

    private Positions(double position) {
      this.position = position;
    }
  }
}
