package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeStates;

public class CoralIntakeStateCommand extends Command {
  CoralIntakeStates m_intakeState;

  public CoralIntakeStateCommand(CoralIntakeStates intakeState) {
    m_intakeState = intakeState;

    addRequirements(subsystems.coralIntake);
  }

  @Override
  public void initialize() {
    CoralState state = subsystems.coralStateManager.getState().setIntakeState(m_intakeState);
    subsystems.coralStateManager.setState(state);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    CoralState state = subsystems.coralStateManager.getState().setIntakeState(CoralIntakeStates.Rest);
    subsystems.coralStateManager.setState(state);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
