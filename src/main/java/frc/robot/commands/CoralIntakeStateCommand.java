package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;

public class CoralIntakeStateCommand extends Command {
  CoralIntakeState m_intakeState;

  public CoralIntakeStateCommand(CoralIntakeState intakeState) {
    m_intakeState = intakeState;

    addRequirements(subsystems.coralIntake);
  }

  @Override
  public void initialize() {
    subsystems.coralIntake.setState(m_intakeState);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.coralIntake.setState(CoralIntakeState.Rest);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
