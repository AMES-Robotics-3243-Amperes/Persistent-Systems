// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemLeds.Mode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandLedPatternCycle extends Command {
  private SubsystemLeds leds;
  private int counter = 1000;
  private SubsystemLeds.Mode mode = Mode.Error;
  /** Creates a new CommandLedPatternCycle. */
  public CommandLedPatternCycle(SubsystemLeds leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    //SmartDashboard.putNumber("LED Counter", counter);
    if (counter > 50 * 8) {
      counter = 0;
      //System.out.println("\n\nLeds now displaying: " + mode + "\n\n");
      leds.setState(mode);
      mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
