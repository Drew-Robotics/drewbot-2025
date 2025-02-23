package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

public class DriveAutoConstants {
  
  public static final LinearVelocity kMaxVelocity = Units.MetersPerSecond.of(4.5);
  public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(9);
  public static final AngularVelocity kMaxAngularVelocity = Units.DegreesPerSecond.of(180);
  public static final AngularAcceleration kMaxAngularAcceleration  = Units.DegreesPerSecondPerSecond.of(360);

  /**
   * This is the distance that the robot's center should be from the edge of the reef while scoring.
   */
  public static final Distance kRobotDistanceFromCenter = Units.Inches.of(30); // TODO : set this to something correct (probably not 30)

  /**
   * This is the distance from the center of the side of a reef (between both branches), to the center of any of the branches
   */
  public static final Distance kReefBranchDistanceFromCenter = Units.Inches.of(15); // TODO : this as well

  /**
   * This is the distance from the center of the branch to where the path finder should end.
   * This creates a new pose that goes this distance out (perpendicular) to the branch, which will be where the path finding ends.
   * The distance needs to be enough so that the robot can auto rotate to the side.
   * (edit) I'm pretty sure we dont need this.
   */
  // public static final Distance kReefBranchDistanceToPathingStart = Units.Meters.of(2);
  
  

  public static class AutoNames {
    public static final String kPathingToStation = "Pathing to Station";
  }

  public static final PathConstraints kPathingConstraints = new PathConstraints(
    kMaxVelocity, kMaxAcceleration, kMaxAngularVelocity, kMaxAngularAcceleration
  );

  public static final PPHolonomicDriveController autoDriveController = new PPHolonomicDriveController(
    DriveAutoConstants.DrivingPID.pidConstants, DriveAutoConstants.TurningPID.pidConstants
  );

  public static final ModuleConfig moduleConfig = new ModuleConfig(
    DriveConstants.SwerveCalculations.kWheelDiameter.times(0.5), //radius
    kMaxVelocity,
    DriveConstants.SwerveCalculations.kWheelCoefficientOfFriction,
    DCMotor.getNeoVortex(1), 
    DriveConstants.SwerveCalculations.kDrivingMotorReduction,
    DriveConstants.kDrivingMotorCurrentLimit,
    1
  );

  public static final RobotConfig robotConfig = new RobotConfig(
    Units.Pounds.of(74.088),
    Units.KilogramSquareMeters.of(1),
    moduleConfig,
    DriveConstants.kDriveKinematics.getModules()
  );

  public static final class DrivingPID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final PIDConstants pidConstants = new PIDConstants(
      kP, kI, kD
    );
  }
  
  public static final class TurningPID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final PIDConstants pidConstants = new PIDConstants(
      kP, kI, kD
    );
  }
}
