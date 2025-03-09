package frc.robot.utility;

import java.util.Set;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Provides a wrapper for a command, so that the command
 * can be changed at runtime. This kinda breaks a bunch of
 * guarantees commands are supposed to have, so use this with
 * care.
 */
public class CommandWrapper extends Command {
  private Command backingCommand = null;

  public CommandWrapper() {}

  public CommandWrapper(Command command) {
    backingCommand = command;
  }

  public Command getBackingCommand() { return backingCommand; }

  public void setBackingCommand(Command command) { backingCommand = command; }

  @Override
  public final void initialize() {
    backingCommand.initialize();
  }

  @Override
  public final void execute() {
    backingCommand.execute();
  }

  @Override
  public final void end(boolean interrupted) {
    backingCommand.end(interrupted);
  }

  @Override
  public final boolean isFinished() {
    return backingCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return backingCommand.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return backingCommand.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return backingCommand.getRequirements();
  }
}
