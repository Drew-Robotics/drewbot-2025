package frc.robot.commands.drivecommands;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveAutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.MaxVels;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoAlignDriveCommand extends TurnToAngleCommand {

    private Pose2d m_targetPose;

    private ProfiledPIDController m_xController = new ProfiledPIDController(
        DriveConstants.DrivingPID.kP,
        DriveConstants.DrivingPID.kI,
        DriveConstants.DrivingPID.kD,
        new Constraints(
            DriveConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond),
            DriveConstants.DrivingPID.kMaxAccel.in(Units.MetersPerSecondPerSecond)
        )
    );
    private ProfiledPIDController m_yController = new ProfiledPIDController(
        DriveConstants.DrivingPID.kP,
        DriveConstants.DrivingPID.kI,
        DriveConstants.DrivingPID.kD,
        new Constraints(
            DriveConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond),
            DriveConstants.DrivingPID.kMaxAccel.in(Units.MetersPerSecondPerSecond)
        )
    );
    public AutoAlignDriveCommand(Pose2d targetPose) {
        super(targetPose);

        m_targetPose = targetPose;

        m_setAngle = m_targetPose::getRotation;
        m_targetPose = targetPose;

        m_xController.setTolerance(DriveConstants.kPositionTolerance.in(Units.Meters));
        m_yController.setTolerance(0.01);

        addRequirements(subsystems.drive);
    }

    public LinearVelocity calculateX() {
        double out = m_xController.calculate(
            subsystems.drive.getPose().getX(),
            m_targetPose.getX()
        );

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
        double radiansPerSecond = 
        subsystems.drive.getChassisSpeedOnRotationControl(
            0, 0, m_setAngle.get()
        ).omegaRadiansPerSecond;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        chassisSpeeds.vxMetersPerSecond = calculateX().in(Units.MetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = calculateY().in(Units.MetersPerSecond);

        chassisSpeeds = subsystems.drive.fieldOrientChassisSpeeds(chassisSpeeds);
        chassisSpeeds.omegaRadiansPerSecond = radiansPerSecond;

        double maxSpeed = DriveAutoConstants.DrivingPID.kMaxVel.in(Units.MetersPerSecond);

        chassisSpeeds.vxMetersPerSecond = Math.max(
            Math.min(chassisSpeeds.vxMetersPerSecond, maxSpeed), - maxSpeed
        );

        chassisSpeeds.vyMetersPerSecond = Math.max(
            Math.min(chassisSpeeds.vyMetersPerSecond, maxSpeed), - maxSpeed
        );

        // System.out.print(chassisSpeeds.vxMetersPerSecond + " | ");
        // System.out.print(subsystems.drive.getPose().getX() + " | ");
        // System.out.println(m_targetPose.getX());

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
