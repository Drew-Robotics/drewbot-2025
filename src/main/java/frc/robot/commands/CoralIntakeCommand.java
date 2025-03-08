package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;

public class CoralIntakeCommand extends Command {
  CoralIntakeState m_intakeState;

  public CoralIntakeCommand() {
    addRequirements(subsystems.coralIntake, subsystems.coralArm, subsystems.elevator);
  }

  @Override
  public void initialize() {
    subsystems.coralIntake.setState(m_intakeState);
    new SetCoralStateCommand(CoralStates.kStation).schedule();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.coralIntake.setState(CoralIntakeState.Hold);
    
  }

  @Override
  public boolean isFinished() {
    return subsystems.coralIntake.hasPiece();
  }
}
