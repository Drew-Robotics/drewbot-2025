package frc.robot.commands.drivecommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.DriveConstants;

public class AutoAlignDriveCommand extends TurnToAngleCommand {

    private Pose2d m_targetPose;
    private LinearVelocity m_maxVelocity = DriveConstants.DrivingPID.kMaxVel;


    private StructPublisher<Pose2d> m_targetPoseStructPublisher = 
        NetworkTableInstance.getDefault().getStructTopic("AutoAlignTargetPose", Pose2d.struct).publish();

    // private ProfiledPIDController m_xController = new ProfiledPIDController(
    private PIDController m_xController = new PIDController(
        DriveConstants.DrivingPID.kP,
        DriveConstants.DrivingPID.kI,
        DriveConstants.DrivingPID.kD
        // new Constraints(
        //     DriveConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond),
        //     DriveConstants.DrivingPID.kMaxAccel.in(Units.MetersPerSecondPerSecond)
        // )
    );
    // private ProfiledPIDController m_yController = new ProfiledPIDController(
    private PIDController m_yController = new PIDController(
        DriveConstants.DrivingPID.kP,
        DriveConstants.DrivingPID.kI,
        DriveConstants.DrivingPID.kD
        // new Constraints(
        //     DriveConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond),
        //     DriveConstants.DrivingPID.kMaxAccel.in(Units.MetersPerSecondPerSecond)
        // )
    );

    public AutoAlignDriveCommand(Pose2d target, LinearVelocity maxVel) {
        this(target);
        m_maxVelocity = maxVel;
    } 

    public AutoAlignDriveCommand(Pose2d targetPose) {
        super(targetPose);

        m_targetPose = targetPose;
        m_setAngle = m_targetPose::getRotation;

        m_xController.setTolerance(DriveConstants.kPositionTolerance.in(Units.Meters));
        m_yController.setTolerance(DriveConstants.kPositionTolerance.in(Units.Meters));

        addRequirements(subsystems.drive);
        // subsystems.drive.drive(0, 0, 0);
        m_targetPoseStructPublisher.accept(targetPose);
        subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmAlignAngle);
    }

    public LinearVelocity calculateX() {
        double out = m_xController.calculate(
            subsystems.drive.getPose().getX(),
            m_targetPose.getX()
        );

        // System.out.println(out);

        return Units.MetersPerSecond.of(out);
    }

    public LinearVelocity calculateY() {
        double out = m_yController.calculate(
            subsystems.drive.getPose().getY(),
            m_targetPose.getY()
        );

        return Units.MetersPerSecond.of(out);
        
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double maxVel = m_maxVelocity.in(Units.MetersPerSecond);

        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        double xVel = Math.max(Math.min(calculateX().in(Units.MetersPerSecond), maxVel), - maxVel);
        double yVel = Math.max(Math.min(calculateY().in(Units.MetersPerSecond), maxVel), - maxVel);


        // chassisSpeeds = subsystems.drive.fieldOrientChassisSpeeds(chassisSpeeds);
        // chassisSpeeds.omegaRadiansPerSecond = radiansPerSecond;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        Rotation2d setAngle = m_setAngle.get();

        if (alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red) {
                xVel = -xVel;
                yVel = -yVel;
                setAngle = Rotation2d.fromDegrees(setAngle.getDegrees() + 180);
            }
        }
        

        // double radiansPerSecond = 
        ChassisSpeeds chassisSpeeds = 
            subsystems.drive.getChassisSpeedOnRotationControl(
                xVel/maxVel, yVel/maxVel, setAngle
            );//.omegaRadiansPerSecond;

        // chassisSpeeds.omegaRadiansPerSecond = radiansPerSecond;
        
        // redundant but just in case

        chassisSpeeds.vxMetersPerSecond /= DriveConstants.MaxVels.kTranslationalVelocity.in(MetersPerSecond);
        chassisSpeeds.vyMetersPerSecond /= DriveConstants.MaxVels.kTranslationalVelocity.in(MetersPerSecond);

        chassisSpeeds.vxMetersPerSecond = Math.max(Math.min(chassisSpeeds.vxMetersPerSecond, 1), - 1);
        chassisSpeeds.vyMetersPerSecond = Math.max(Math.min(chassisSpeeds.vyMetersPerSecond, 1), - 1);

        chassisSpeeds.vxMetersPerSecond *= maxVel;
        chassisSpeeds.vyMetersPerSecond *= maxVel;

        // System.out.print("x " + radiansPerSecond + " | t ");
        // System.out.print(subsystems.drive.getPose().getX() + " | m ");
        // System.out.println(m_targetPose.getX());
        // System.out.println("Auto aligning");

        // chassisSpeeds = subsystems.drive.fieldOrientChassisSpeeds(chassisSpeeds);
        subsystems.drive.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        subsystems.drive.setChassisSpeeds(new ChassisSpeeds(0,0,0));
        // System.out.println("auto align end, to rest state");
        subsystems.algaeArm.toRestState();
    }

    @Override
    public boolean isFinished() {
        return false;
        // System.out.println(m_xController.atSetpoint());
        // System.out.println(m_yController.atSetpoint());
        // System.out.println(subsystems.drive.atRotationSetpoint());
        // return m_xController.atSetpoint() && m_yController.atSetpoint() && subsystems.drive.atRotationSetpoint();
    }
}
