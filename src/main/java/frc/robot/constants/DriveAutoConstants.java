package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveAutoConstants {
  
  public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(4.5);

  public static final PPHolonomicDriveController autoDriveController = new PPHolonomicDriveController(
    new PIDConstants(DriveAutoConstants.DrivingPID.kP, DriveAutoConstants.DrivingPID.kI, DriveAutoConstants.DrivingPID.kD), // Translation PID constants
    new PIDConstants(DriveAutoConstants.TurningPID.kP, DriveAutoConstants.TurningPID.kI, DriveAutoConstants.TurningPID.kD) // Rotation PID constants
  );


  public static final ModuleConfig moduleConfig = new ModuleConfig(
    DriveConstants.SwerveCalculations.kWheelDiameter.times(0.5), //radius
    kMaxSpeed,
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
      kP,
      kI,
      kD
    );
  }
  
  public static final class TurningPID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final PIDConstants pidConstants = new PIDConstants(
      kP,
      kI,
      kD
    );
  }
}
