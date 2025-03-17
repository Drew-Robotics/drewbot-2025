package frc.lib.ledTools.finals;

import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.function.IntSupplier;

public final class LEDInput {

  public final double time;
  public final double ledProgress;

  public HashMap<String, IntSupplier> inputs;


  public LEDInput(double ledProgress) {
    this(ledProgress, null);
  }

  public LEDInput(double ledProgress, HashMap<String, IntSupplier> inputs) {
    time = Timer.getFPGATimestamp();
    this.ledProgress = Math.abs(ledProgress % 1); // clamp from 0 to 1
    this.inputs = inputs;
  }
}
