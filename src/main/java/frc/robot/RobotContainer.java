//     .----.   @   @
//    / .-"-.`.  \v/
//    | | '\ \ \_/ )
//   ,-\ `-.' /.'  /
// '---`----'----'   <- finny snail

package frc.robot;

import java.util.concurrent.Flow.Subscriber;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.RemoveAlgaeCommand;
import frc.robot.commands.ScoreSetupCommand;
import frc.robot.commands.SetCoralStateCommand;
import frc.robot.commands.drivecommands.TagHeadingAlignDriveCommand;
import frc.robot.commands.drivecommands.DriveCommand;
import frc.robot.commands.drivecommands.TurnToAngleCommand;
import frc.robot.constants.CoralStates;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.algae.AlgaeArmSubsystem;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.coral.CoralArmSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.coral.ElevatorSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ReefSide.ReefBranch;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  public final class subsystems {
    public static final DriveSubsystem drive = DriveSubsystem.getInstance();
    public static final VisionSubsystem vision = VisionSubsystem.getInstance();

    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static final CoralArmSubsystem coralArm = CoralArmSubsystem.getInstance();
    public static final CoralIntakeSubsystem coralIntake = CoralIntakeSubsystem.getInstance();
    
    public static final AlgaeArmSubsystem algaeArm = AlgaeArmSubsystem.getInstance();
    public static final AlgaeIntakeSubsystem algaeIntake = AlgaeIntakeSubsystem.getInstance();
  }

  private static CoralState m_operatorCoralState = CoralStates.kL2;
  private static CoralState m_operatorAlgaeRMState = CoralStates.kAlgaeL2;
  private static ReefBranch m_operatorReefBranch = ReefBranch.Left;

  public final class controllers {
    public static final DriverController driver = DriverController.getInstance();
    public static final OperatorController operator = OperatorController.getInstance();
  }

  private static RobotContainer m_instance;
  public static RobotContainer getInstance() {
    if (m_instance == null)
      m_instance = new RobotContainer();
    return m_instance;
  }

  protected RobotContainer() {
    subsystems.drive.pathPlannerConfig();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("defaultAuto", AutoBuilder.buildAuto("defaultAuto"));

    SmartDashboard.putData("Set Elevator Zero", new InstantCommand(subsystems.elevator::setEncoderZero));
    SmartDashboard.putData("Set Algae Arm Zero", new InstantCommand(subsystems.algaeArm::setEncoderZero));
    configureDriverBindings();
    configureOperatorBindings();

    SmartDashboard.putData("Reveal Sequence",
      new SetCoralStateCommand(CoralStates.kStation)
        .alongWith(new InstantCommand(() -> subsystems.coralIntake.setState(CoralIntakeState.Intake)))
        .andThen(new WaitCommand(0.5))
        .andThen(
          new SetCoralStateCommand(CoralStates.kRest)
            .alongWith(new InstantCommand(() -> subsystems.coralIntake.setState(CoralIntakeState.Rest))))
        .andThen(
          new AlgaeIntakeCommand().withDeadline(
            new WaitCommand(0.5)
              .andThen(new SetCoralStateCommand(CoralStates.kL4))
              .alongWith(new InstantCommand(() -> subsystems.coralIntake.setState(CoralIntakeState.Outtake)))
              .andThen(new WaitCommand(1))
          )
        )
        .andThen(new SetCoralStateCommand(CoralStates.kRest)
          .alongWith(
            new InstantCommand(() -> subsystems.coralIntake.setState(CoralIntakeState.Rest))
          )
        )
    );
  }

  private Command turnToAngleCommand(Rotation2d rot) {
    return new TurnToAngleCommand(
      controllers.driver::getXVelocity,
      controllers.driver::getYVelocity,
      rot
    );
  }

  private void configureDriverBindings() {
    subsystems.drive.setDefaultCommand(
      new DriveCommand(
        controllers.driver::getXVelocity, 
        controllers.driver::getYVelocity, 
        controllers.driver::getRotationalVelocity
      )
    );

    // controllers.driver.getToggleAutoRotate().toggleOnTrue(
    //   new TagHeadingAlignDriveCommand(
    //     controllers.driver::getXVelocity,
    //     controllers.driver::getYVelocity
    //   )
    // );

    controllers.driver.getScore().onTrue(
      new CoralOuttakeCommand().withTimeout(1)
      .andThen(new SetCoralStateCommand(CoralStates.kRest).withTimeout(1))
    );

    controllers.driver.getScoreSetup().whileTrue(
      new ScoreSetupCommand(() -> RobotContainer.m_operatorCoralState, () -> RobotContainer.m_operatorReefBranch)
    );
    
    controllers.driver.getRemoveAlgae().onTrue(
      new RemoveAlgaeCommand(() -> RobotContainer.m_operatorAlgaeRMState)
    );

    controllers.driver.getTurnTo0Button().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(0)));
    controllers.driver.getTurnTo90Button().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(90)));
    controllers.driver.getTurnTo180Button().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(180)));
    controllers.driver.getTurnTo270Button().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(270)));

    controllers.driver.getTurnToFrontLeftSide().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(-60)));
    controllers.driver.getTurnToFrontRightSide().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(60)));
    controllers.driver.getTurnToBackLeftSide().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(180 +60)));
    controllers.driver.getTurnToBackRightSide().whileTrue(turnToAngleCommand(Rotation2d.fromDegrees(180 -60)));
    
    controllers.driver.getSetStateStation().onTrue(
      new CoralIntakeCommand()
    );
  }

  private void configureOperatorBindings() {
    controllers.operator.getSetStateL1().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL1)
    );

    controllers.operator.getSetStateL2().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL2)
    );

    controllers.operator.getSetStateL3().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL3)
    );

    controllers.operator.getSetStateL4().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL4)
    );

    // controllers.operator.getSetStateL1().onTrue(
    //   new SetCoralStateCommand(CoralStates.kL1)
    // );

    // controllers.operator.getSetStateL2().onTrue(
    //   new SetCoralStateCommand(CoralStates.kL2)
    // );

    // controllers.operator.getSetStateL3().onTrue(
    //   new SetCoralStateCommand(CoralStates.kL3)
    // );

    // controllers.operator.getSetStateL4().onTrue(
    //   new SetCoralStateCommand(CoralStates.kL4)
    // );

    controllers.operator.getSetStateRMAlgaeL2().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorAlgaeRMState = CoralStates.kAlgaeL2)
    );

    controllers.operator.getSetStateRMAlgaeL3().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorAlgaeRMState = CoralStates.kAlgaeL3)
    );

    controllers.operator.getSetBranchLeft().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorReefBranch = ReefBranch.Left)
    );

    controllers.operator.getSetBranchRight().onTrue(
      new InstantCommand(() -> RobotContainer.m_operatorReefBranch = ReefBranch.Right)
    );

    controllers.operator.getCoralOuttake().onTrue(
      new CoralOuttakeCommand()
    );

    controllers.operator.getAlgaeIntake().whileTrue(new AlgaeIntakeCommand());

    controllers.operator.getStow().onTrue(
      new SetCoralStateCommand(CoralStates.kRest).andThen(
        new InstantCommand(() -> {
            if (subsystems.coralIntake.hasPiece()){
              subsystems.coralIntake.setState(CoralIntakeState.Hold);
            } else {
              subsystems.coralIntake.setState(CoralIntakeState.Rest);
            }
          }, subsystems.coralIntake
        )
      )
    );

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public String getAutonomousName() {
    StringTopic activeAutoTopic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Chooser/active");
    String activeAutoName = activeAutoTopic.getEntry("defaultAuto", PubSubOption.sendAll(false)).get();
  
    return activeAutoName;
  }
}
