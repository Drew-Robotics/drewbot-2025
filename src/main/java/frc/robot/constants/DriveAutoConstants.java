package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

public class DriveAutoConstants {
  // from pathplanner
  public static final Pose2d kFeedStationPose = new Pose2d(1.2, 7, Rotation2d.fromDegrees(130));
  public static final LinearVelocity kTeleopCoralAlignVel = MetersPerSecond.of(0.6);
  public static final LinearVelocity kAutoCoralAlignVel = MetersPerSecond.of(1.15);
  public static final LinearVelocity kAutoFeedAlignVel = MetersPerSecond.of(2);

  public static final LinearVelocity kMaxVelocity = Units.MetersPerSecond.of(4.5);
  public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(3);
  public static final AngularVelocity kMaxAngularVelocity = Units.DegreesPerSecond.of(180);
  public static final AngularAcceleration kMaxAngularAcceleration  = Units.DegreesPerSecondPerSecond.of(360);

  public static final List<Integer> kAutoAlignTagsIDs = List.of(0, 1, 2, 3);
  public static final List<Integer> kAutoDriveTagsIDs = List.of(0, 1, 2, 3);

  /**
   * This is the distance that the robot's center should be from the edge of the reef while scoring.
   */
  public static final Distance kRobotDistanceFromCenter = Units.Inches.of(17);

  /**
   * This is the distance from the center of the side of a reef (between both branches), to the center of any of the branches
   */
  public static final Distance kReefBranchDistanceFromCenter = Units.Inches.of(13.25 * 0.5);

  /**
   * This is the distance from the center of the branch to where the path finder should end.
   * This creates a new pose that goes this distance out (perpendicular) to the branch, which will be where the path finding ends.
   * The distance needs to be enough so that the robot can auto rotate to the side.
   * (edit) I'm pretty sure we dont need this.
   */
  // public static final Distance kReefBranchDistanceToPathingStart = Units.Meters.of(2);
  
  

  public static class AutoNames {
  }

  public static final PathConstraints kPathingConstraints = new PathConstraints(
    kMaxVelocity, kMaxAcceleration, kMaxAngularVelocity, kMaxAngularAcceleration
  );


  public static final PPHolonomicDriveController autoDriveController = new PPHolonomicDriveController(
    DriveAutoConstants.DrivingPID.pidConstants, DriveAutoConstants.TurningPID.pidConstants
  );

  public static final ModuleConfig moduleConfig = new ModuleConfig(
    DriveConstants.SwerveCalculations.kWheelDiameter.times(0.5), // radius
    Units.MetersPerSecond.of(5.65), // max measured speed
    DriveConstants.SwerveCalculations.kWheelCoefficientOfFriction,
    DCMotor.getNeoVortex(1), 
    DriveConstants.SwerveCalculations.kDrivingMotorReduction,
    DriveConstants.kDrivingMotorCurrentLimit,
    1
  );

  public static final RobotConfig robotConfig = new RobotConfig(
    Units.Pounds.of(110),
    Units.KilogramSquareMeters.of(4),
    moduleConfig,
    DriveConstants.kDriveKinematics.getModules()
  );

  public static final class DrivingPID {
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    // TODO : probably lower since we dont need to be so speedy
    public static final LinearVelocity kMaxVel = kMaxVelocity;
    public static final LinearAcceleration kMaxAccel = kMaxAcceleration;

    public static final PIDConstants pidConstants = new PIDConstants(
      kP, kI, kD
    );
  }
  
  public static final class TurningPID {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final PIDConstants pidConstants = new PIDConstants(
      kP, kI, kD
    );
  }
}
