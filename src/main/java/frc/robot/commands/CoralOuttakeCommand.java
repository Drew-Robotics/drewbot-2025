package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;

public class CoralOuttakeCommand extends Command {
  public CoralOuttakeCommand() {
    addRequirements(subsystems.coralIntake);
  }

  @Override
  public void initialize() {
    subsystems.coralIntake.setState(CoralIntakeState.Outtake);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.coralIntake.setState(CoralIntakeState.Rest);
  }

  @Override
  public boolean isFinished() {
    return !subsystems.coralIntake.hasPiece();
  }
}
