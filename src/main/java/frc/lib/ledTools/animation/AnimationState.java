package frc.lib.ledTools.animation;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import frc.lib.ledTools.Strip;
import frc.lib.ledTools.finals.LEDColor;
import frc.lib.ledTools.finals.StripState;

public class AnimationState {

  public final BooleanSupplier isActive;
  public final Function<Strip, StripState> stripFunction;

  public AnimationState(BooleanSupplier isActive, LEDColor color) {
    this.isActive = isActive;
    stripFunction = strip -> new StripState(color);
  }

  public AnimationState(BooleanSupplier isActive, Supplier<StripState> stripSub) {
    this.isActive = isActive;
    stripFunction = strip -> stripSub.get();
  }

  public AnimationState(BooleanSupplier isActive, Animation animation) {
    this.isActive = isActive;
    this.stripFunction = strip -> animation.getStrip(strip);
  }

  public AnimationState(BooleanSupplier isActive, ArrayList<Animation> animations) {
    this.isActive = isActive;
    this.stripFunction = strip -> {
      StripState stateOut = new StripState(new LEDColor());
      for (Animation animation : animations) {
        stateOut = stateOut.add(animation.getStrip(strip));
      }
      return stateOut;
    };
  }

  public AnimationState(BooleanSupplier isActive, Function<Strip, StripState> stripFunction) {
    this.isActive = isActive;
    this.stripFunction = stripFunction;
  }

  public boolean isActive() {
    return isActive.getAsBoolean();
  }

  public StripState getStrip(Strip strip) {
    return stripFunction.apply(strip);
  }
}
