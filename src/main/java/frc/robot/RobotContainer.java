//     .----.   @   @
//    / .-"-.`.  \v/
//    | | '\ \ \_/ )
//   ,-\ `-.' /.'  /
// '---`----'----'   <- finny snail

package frc.robot;

import java.util.Comparator;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.Flow.Subscriber;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.RemoveAlgaeCommand;
import frc.robot.commands.SetCoralStateCommand;
import frc.robot.commands.drivecommands.TagHeadingAlignDriveCommand;
import frc.robot.commands.drivecommands.AutoAlignDriveCommand;
import frc.robot.commands.drivecommands.DriveCommand;
import frc.robot.commands.drivecommands.TurnToAngleCommand;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CoralStates;
import frc.robot.constants.DriveAutoConstants;
import frc.robot.constants.ReefSides;
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
import frc.robot.subsystems.drive.ReefSide;
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

  private Supplier<Command> m_scoreCommand = () ->
    new CoralOuttakeCommand().withTimeout(1)
    .andThen(new SetCoralStateCommand(CoralStates.kRest));

  private Supplier<Command> m_scoreCommandL3Rest = () ->
  new CoralOuttakeCommand().withTimeout(0.3)
  .andThen(new SetCoralStateCommand(CoralStates.kL3));

  private Supplier<Command> m_setLeftCommand = () ->
    new InstantCommand(() -> RobotContainer.m_operatorReefBranch = ReefBranch.Left);

  private Supplier<Command> m_setRightCommand = () ->
    new InstantCommand(() -> RobotContainer.m_operatorReefBranch = ReefBranch.Right);

  private Supplier<Command> m_setL1Command = () -> 
    new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL1);
  private Supplier<Command> m_setL2Command = () -> 
    new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL2);
  private Supplier<Command> m_setL3Command = () -> 
    new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL3);
  private Supplier<Command> m_setL4Command = () -> 
    new InstantCommand(() -> RobotContainer.m_operatorCoralState = CoralStates.kL4);

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
    SmartDashboard.putData("Set Elevator Zero", new InstantCommand(subsystems.elevator::setEncoderZero));
    SmartDashboard.putData("Set Algae Arm Zero", new InstantCommand(subsystems.algaeArm::setEncoderZero));
    configureDriverBindings();
    configureOperatorBindings();

    // Named commands for pathplanner

    NamedCommands.registerCommand("coralIntake", 
      Commands.defer(() -> new AutoAlignDriveCommand(DriveAutoConstants.kFeedStationPose, DriveAutoConstants.kAutoFeedAlignVel)
        .withDeadline(new CoralIntakeCommand(CoralStates.kL3)), Set.of() // runs until we get a piece
      )
    );

    // NamedCommands.registerCommand("scoreCoral", m_scoreCommand.get());

    NamedCommands.registerCommand("scoreCoralFirst", 
      autoAlignCommand(DriveAutoConstants.kAutoCoralAlignVel).withTimeout(1.1)
      .andThen(Commands.defer(m_scoreCommandL3Rest, Set.of()))
    );

    NamedCommands.registerCommand("scoreCoral", 
      autoAlignCommand(DriveAutoConstants.kAutoCoralAlignVel).withTimeout(1.35)
      .andThen(Commands.defer(m_scoreCommandL3Rest, Set.of()))
    );

    NamedCommands.registerCommand("setLeftBranch", m_setLeftCommand.get());
    NamedCommands.registerCommand("setRightBranch", m_setRightCommand.get());

    NamedCommands.registerCommand("setL1", m_setL1Command.get());
    NamedCommands.registerCommand("setL2", m_setL2Command.get());
    NamedCommands.registerCommand("setL3", m_setL3Command.get());
    NamedCommands.registerCommand("setL4", m_setL4Command.get());

    NamedCommands.registerCommand("setL3Force", Commands.defer(
      (Supplier<Command>) () -> {
        return new SetCoralStateCommand(CoralStates.kL3);
      }, Set.of()
    ));
    

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

    // runs forever and logs all our stored states
    // new FunctionalCommand(() -> {}, 
    // () -> {
    //   for (CoralState coralState : CoralStates.kLoggedStates) {
    //     SmartDashboard.putBoolean(
    //       coralState.getName(), 
    //       (m_operatorCoralState == coralState) || (m_operatorAlgaeRMState == coralState)
    //     );
    //   }
    // }
    // , i -> {}, () -> false).schedule();

    subsystems.drive.pathPlannerConfig();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("defaultAuto", AutoBuilder.buildAuto("defaultAuto"));
  }

  // public static Command algaeIntakeZeroFixCommand() {
  //   return Commands.defer(
  //   () -> new InstantCommand(
  //       () -> subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmCorrectAngle)
  //     )
  //     .andThen(new WaitCommand(0.2))
  //     .andThen(new InstantCommand(
  //       subsystems.algaeArm::setEncoderZero
  //     )), Set.of()
  //   );
  // }

  public Command autoAlignCommand(){
    return Commands.defer(this::getAutoAlignCommand, Set.of());
  }

  public Command autoAlignCommand(LinearVelocity maxVel){
    return Commands.defer(() -> getAutoAlignCommand(maxVel), Set.of());
  }

  public Command getAutoAlignCommand(){
    return getAutoAlignCommand(null);
  }

  public Command getAutoAlignCommand(LinearVelocity maxVel){
    CoralState coralState = m_operatorCoralState;
    ReefBranch reefBranch = m_operatorReefBranch;
    Distance pieceDisp = subsystems.coralIntake.getPieceDispFromCenter();

    Pose2d startPose = subsystems.drive.getPose();

    Optional<ReefSide> reefSideOp = ReefSides.kReefSides
      .stream()
      .sorted(Comparator.comparingDouble(
        side -> {
          Transform2d dist = side.getEndPose(reefBranch).minus(startPose);
          return dist.getX()*dist.getX() + dist.getY()*dist.getY();
        }
      ))
      .findFirst();

    ReefSide reefSide = reefSideOp.orElse(null);

    if (reefSide == null) return Commands.none();
    if (!subsystems.coralIntake.hasPiece()) return Commands.none();

    Pose2d targetPose = subsystems.drive.getReefTargetPose(coralState, reefSide, reefBranch, pieceDisp);

    System.out.println("Auto driving to tag : " + reefSide.kTagID);
    if (maxVel != null){
      return new SetCoralStateCommand(coralState).andThen(
        new AutoAlignDriveCommand(targetPose, maxVel)
      );
    }

    return new SetCoralStateCommand(coralState).andThen(
      new AutoAlignDriveCommand(targetPose)
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
      m_scoreCommand.get()
    );

    controllers.driver.getScoreSetup().whileTrue(autoAlignCommand(DriveAutoConstants.kTeleopCoralAlignVel));
    
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
    controllers.operator.getSetStateL1().onTrue(m_setL1Command.get());
    controllers.operator.getSetStateL2().onTrue(m_setL2Command.get());
    controllers.operator.getSetStateL3().onTrue(m_setL3Command.get());
    controllers.operator.getSetStateL4().onTrue(m_setL4Command.get());

    controllers.operator.getClimberUp().onTrue(new SetCoralStateCommand(CoralStates.kClimberUp));
    controllers.operator.getClimberHold().onTrue(new SetCoralStateCommand(CoralStates.kClimberHold));
    controllers.operator.getClimberHoldArmUp().onTrue(new SetCoralStateCommand(CoralStates.kClimberHoldArmUp));

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

    controllers.operator.getSetBranchLeft().onTrue(m_setLeftCommand.get());
    controllers.operator.getSetBranchRight().onTrue(m_setRightCommand.get());

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

    controllers.operator.getAlgaeIntake().whileTrue(new AlgaeIntakeCommand());
    controllers.operator.getAlgaeOuttake().whileTrue(new AlgaeOuttakeCommand());

    controllers.operator.getDebugCoralEject().onTrue(
      new SetCoralStateCommand(
          CoralStates.kL3
      ).andThen(
        new InstantCommand(
          () -> subsystems.coralIntake.setState(CoralIntakeState.DebugEject)
        )
      )
    );

    // controllers.operator.getDebugAlgaeIntakeBack().onTrue(
    //   RobotContainer.algaeIntakeZeroFixCommand()
    // );
  }

  public Command getAutonomousCommand() {
    // return Commands.defer(
    //   () -> new ScoreSetupCommand(
    //       () -> RobotContainer.m_operatorCoralState, () -> RobotContainer.m_operatorReefBranch
    //     ).withTimeout(2)
    //     .andThen(new CoralOuttakeCommand().withTimeout(1))
    //     .andThen(new SetCoralStateCommand(CoralStates.kRest))
    //   , Set.of());
    return autoChooser.getSelected().beforeStarting(subsystems.drive::resetGyroYaw, subsystems.drive);
  }

  public String getAutonomousName() {
    StringTopic activeAutoTopic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Chooser/active");
    String activeAutoName = activeAutoTopic.getEntry("defaultAuto", PubSubOption.sendAll(false)).get();
  
    return activeAutoName;
  }

  // gotta do what you gotta do
  public void loggingPeriodicCalledInElevator() {
    SmartDashboard.putBoolean("Aligned Left", m_operatorReefBranch == ReefBranch.Left);
    SmartDashboard.putBoolean("Aligned Right", m_operatorReefBranch == ReefBranch.Right);

    for (CoralState coralState : CoralStates.kLoggedStates) {
      SmartDashboard.putBoolean(
        coralState.getName(), 
        (m_operatorCoralState == coralState) || (m_operatorAlgaeRMState == coralState)
      );
    }
  }
}
