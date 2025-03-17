package frc.robot.subsystems.leds;

import java.util.ArrayList;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ledTools.LEDManager;
import frc.lib.ledTools.Strip;
import frc.lib.ledTools.animation.Animation;
import frc.lib.ledTools.animation.AnimationLoop;
import frc.lib.ledTools.animation.AnimationState;
import frc.lib.ledTools.finals.LEDColor;
import frc.robot.constants.OtherConstants;
import frc.robot.subsystems.leds.Animations.BeamBreak;

public class LEDSubsystem extends SubsystemBase {

  // --- FIELDS AND CONSTRUCTORS ---

  private CANdle m_candle;
  private Strip m_strip;
  private LEDManager m_leds;

  private final int m_resolution = 31;

  public enum LEDSubsystemState {
    SinWaves,
    BeamBreak
  }

  private LEDSubsystemState m_LEDSubsystemState = LEDSubsystemState.SinWaves;

  private static LEDSubsystem m_instance;

  public static LEDSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new LEDSubsystem();
    }
    return m_instance;
  }

  private LEDSubsystem() {
    m_candle = new CANdle(OtherConstants.kCandleCanID, "rio");
    m_strip = new Strip(m_candle, 8, m_resolution);

    m_leds = new LEDManager(m_strip);

    ArrayList<Animation> sinWavesAnimations = new ArrayList<>();

    int i = 100;

    sinWavesAnimations.add(getSin(0.5, 10, new LEDColor(10, 10, 10, 100)));
    sinWavesAnimations.add(getSin(0.3, 5, new LEDColor(i, 0, 0)));
    sinWavesAnimations.add(getSin(0.1, 3, new LEDColor(i, i, 0)));

    m_leds.addState(
      new AnimationState(
        () -> m_LEDSubsystemState == LEDSubsystemState.SinWaves,
        sinWavesAnimations
      )
    );


    m_leds.addState(
      new AnimationState(
        () -> m_LEDSubsystemState == LEDSubsystemState.BeamBreak,
        new BeamBreak(() -> 1, 2)
      )
    );

  }

  private Animation getSin(double size, double speed, LEDColor color) {
    return new AnimationLoop(speed, m_resolution) {
      public LEDColor getLEDColor(double timeProgress, double ledProgress, Strip stripOut) {
        int res = stripOut.resolution;
        double p;

        p = Math.sin(timeProgress * 2 * Math.PI);
        p = (p + 1) / 2;
        p = p * ((double) res - size * (double) res) / (double) res;

        if (Math.abs(p - ledProgress) < size) {
          return color;
        }
        return new LEDColor();
      }
    };
  }

  // --- GENERIC FUNCTIONS ---

  @Override
  public void periodic() {

    SmartDashboard.putString("LEDSubsystemState", m_LEDSubsystemState.toString());

    m_leds.checkStates();
  }

  // --- PRIVATE FUNCTIONS ---

  // --- PUBLIC FUNCTIONS ---

  public void setLEDSubsystemState(LEDSubsystemState ledSubsystemState) {
    m_LEDSubsystemState = ledSubsystemState;
  }
}
