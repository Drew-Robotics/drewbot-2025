package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;

public class CoralIntakeCommand extends Command {
  private CoralState m_restState = CoralStates.kRest;

  public CoralIntakeCommand(CoralState restState) {
    m_restState = restState;
    addRequirements(subsystems.coralIntake);
  }

  public CoralIntakeCommand() {
    addRequirements(subsystems.coralIntake);
  }

  @Override
  public void initialize() {
    // System.out.println("set coral instake intake");
    subsystems.coralIntake.setState(CoralIntakeState.Intake);
    new SetCoralStateCommand(CoralStates.kStation).schedule();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("end " + (subsystems.elevator.getState() != CoralStates.kStation) + ", " + (subsystems.coralArm.getState() != CoralStates.kStation));
    subsystems.coralIntake.setState(CoralIntakeState.Hold);
    new SetCoralStateCommand(m_restState).schedule();
    // System.out.println("end " + interrupted);
    // new SetCoralStateCommand(CoralStates.kRest).schedule();
  }

  @Override
  public boolean isFinished() {    
    // System.out.println(subsystems.coralIntake.hasPiece() || 
    // subsystems.elevator.getState() != CoralStates.kStation ||
    // subsystems.coralArm.getState() != CoralStates.kStation);
    // System.out.println("note " + subsystems.coralIntake.hasPiece());
    // System.out.println("ele " + subsystems.elevator.getState().getName());
    // System.out.println("arm " + subsystems.coralArm.getState().getName());
    return subsystems.coralIntake.hasPiece() || 
      subsystems.elevator.getState() != CoralStates.kStation ||
      subsystems.coralArm.getState() != CoralStates.kStation;
  }
}
