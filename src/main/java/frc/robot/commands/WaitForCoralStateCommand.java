package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.subsystems.coral.CoralState;

public class WaitForCoralStateCommand extends Command {
  private final CoralState m_targetState;

  public WaitForCoralStateCommand(CoralState targetState) {
    m_targetState = targetState;
    addRequirements(
      subsystems.elevator,
      subsystems.coralArm
    );
  }

  @Override
  public void initialize() {
    subsystems.elevator.setState(m_targetState);
    subsystems.coralArm.setState(m_targetState);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    System.out.println("elevator at state : " + subsystems.elevator.atState());
    System.out.println("coralArm at state : " + subsystems.coralArm.atState());
    return subsystems.elevator.atState() && subsystems.coralArm.atState();
  }
}
