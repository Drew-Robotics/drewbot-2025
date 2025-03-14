package frc.robot.commands.drivecommands;

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
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveConstants;

public class AutoAlignDriveCommand extends TurnToAngleCommand {

    private Pose2d m_targetPose;

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
    public AutoAlignDriveCommand(Pose2d targetPose) {
        super(targetPose);

        m_targetPose = targetPose;
        m_setAngle = m_targetPose::getRotation;

        m_xController.setTolerance(DriveConstants.kPositionTolerance.in(Units.Meters));
        m_yController.setTolerance(DriveConstants.kPositionTolerance.in(Units.Meters));

        addRequirements(subsystems.drive);
        subsystems.drive.drive(0, 0, 0);
        m_targetPoseStructPublisher.accept(targetPose);
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
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        chassisSpeeds.vxMetersPerSecond = calculateX().in(Units.MetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = calculateY().in(Units.MetersPerSecond);
        chassisSpeeds = subsystems.drive.fieldOrientChassisSpeeds(chassisSpeeds);

        // chassisSpeeds = subsystems.drive.fieldOrientChassisSpeeds(chassisSpeeds);
        // chassisSpeeds.omegaRadiansPerSecond = radiansPerSecond;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        Rotation2d setAngle = m_setAngle.get();

        if (alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red) {
                chassisSpeeds.vxMetersPerSecond = -chassisSpeeds.vxMetersPerSecond;
                chassisSpeeds.vyMetersPerSecond = -chassisSpeeds.vyMetersPerSecond;
                setAngle = Rotation2d.fromDegrees(setAngle.getDegrees() + 180);
            }
        }

        double radiansPerSecond = 
            subsystems.drive.getChassisSpeedOnRotationControl(
                0, 0, setAngle
            ).omegaRadiansPerSecond;

        chassisSpeeds.omegaRadiansPerSecond = radiansPerSecond;
        

        // System.out.print("x " + radiansPerSecond + " | t ");
        // System.out.print(subsystems.drive.getPose().getX() + " | m ");
        // System.out.println(m_targetPose.getX());

        double maxSpeed = DriveConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond);

        chassisSpeeds.vxMetersPerSecond = Math.max(
            Math.min(chassisSpeeds.vxMetersPerSecond, maxSpeed), - maxSpeed
        );

        chassisSpeeds.vyMetersPerSecond = Math.max(
            Math.min(chassisSpeeds.vyMetersPerSecond, maxSpeed), - maxSpeed
        );

        

        subsystems.drive.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        subsystems.drive.setChassisSpeeds(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return false;
        // return m_xController.atSetpoint() && m_yController.atSetpoint() && subsystems.drive.atRotationSetpoint();
    }
}
