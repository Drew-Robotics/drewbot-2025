package frc.robot.constants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class AutoConstants {
  
  public static final Measure<Velocity<Distance>> kMaxSpeed = Units.MetersPerSecond.of(4.5);

  public static final class DrivingPID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
  }
  
  public static final class TurningPID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
