//     .----.   @   @
//    / .-"-.`.  \v/
//    | | '\ \ \_/ )
//   ,-\ `-.' /.'  /
// '---`----'----'   <- finny snail

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringTopic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CoralIntakeStateCommand;
import frc.robot.commands.drivecommands.AutoAlignDriveCommand;
import frc.robot.commands.drivecommands.DriveCommand;
import frc.robot.commands.drivecommands.TurnToAngleCommand;
import frc.robot.constants.CoralStates;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.algae.AlgaeArmSubsystem;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algae.AlgaeSensorSubsystem;
import frc.robot.subsystems.coral.CoralArmSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.coral.CoralStateManager;
import frc.robot.subsystems.coral.ElevatorSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeStates;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  public final class subsystems {
    public static final DriveSubsystem drive = DriveSubsystem.getInstance();
    public static final VisionSubsystem vision = VisionSubsystem.getInstance();

    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static final CoralArmSubsystem coralArm = CoralArmSubsystem.getInstance();
    public static final CoralIntakeSubsystem coralIntake = CoralIntakeSubsystem.getInstance();
    public static final CoralStateManager coralStateManager = CoralStateManager.getInstance();

    public static final AlgaeArmSubsystem algaeArm = AlgaeArmSubsystem.getInstance();
    public static final AlgaeIntakeSubsystem algaeIntake = AlgaeIntakeSubsystem.getInstance();
    public static final AlgaeSensorSubsystem algaeSensor = AlgaeSensorSubsystem.getInstance();
  }

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
    autoChooser.setDefaultOption("DefaultAuto", AutoBuilder.buildAuto("DefaultAuto"));


    
    SmartDashboard.putData("Set Elevator Zero", new InstantCommand(subsystems.elevator::setZero));
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    subsystems.drive.setDefaultCommand(
      new DriveCommand(
        controllers.driver::getXVelocity, 
        controllers.driver::getYVelocity, 
        controllers.driver::getRotationalVelocity
      )
    );

    controllers.driver.getToggleAutoRotate().toggleOnTrue(
      new AutoAlignDriveCommand(
        controllers.driver::getXVelocity,
        controllers.driver::getYVelocity
      )
    );

    // TODO: add more turn to angle commands.

    controllers.driver.getTurnToZeroButton().onTrue(
      new TurnToAngleCommand(
        controllers.driver::getXVelocity,
        controllers.driver::getYVelocity,
        Rotation2d.fromDegrees(0)
      )
    );
  }

  private void configureOperatorBindings() {
    controllers.operator.getSetStateRest().onTrue(
      subsystems.coralStateManager.getSetStateCommand(CoralStates.kRest)
    );

    controllers.operator.getSetStateL1().onTrue(
      subsystems.coralStateManager.getSetStateCommand(CoralStates.kL1)
    );

    controllers.operator.getSetStateL2().onTrue(
      subsystems.coralStateManager.getSetStateCommand(CoralStates.kL2)
    );

    controllers.operator.getSetStateL3().onTrue(
      subsystems.coralStateManager.getSetStateCommand(CoralStates.kL3)
    );

    controllers.operator.getSetStateL4().onTrue(
      subsystems.coralStateManager.getSetStateCommand(CoralStates.kL4)
    );

    controllers.operator.getCoralIntake().onTrue(
      new CoralIntakeStateCommand(CoralIntakeStates.Intake)
    );

    controllers.operator.getCoralOuttake().onTrue(
      new CoralIntakeStateCommand(CoralIntakeStates.Outtake)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // public String getAutonomousName() {
  //   StringTopic activeAutoTopic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Chooser/active");
  //   String activeAutoName = activeAutoTopic.getEntry("Super Cool Fast Awesome Auto", PubSubOption.sendAll(false)).get();
  
  //   return activeAutoName;
  // }
}
