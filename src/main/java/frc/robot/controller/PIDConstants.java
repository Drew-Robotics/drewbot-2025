package frc.robot.controller;

import java.util.Optional;

public class PIDConstants {
    public final double kP;
    public final double kI;
    public final double kD;
    public final Optional<Double> kFF;

    public PIDConstants(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kFF = Optional.of(kFF);
    }

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kFF = Optional.empty();
    }
}
