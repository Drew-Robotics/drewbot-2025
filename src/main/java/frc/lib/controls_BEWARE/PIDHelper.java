package frc.lib.controls_BEWARE;

import com.revrobotics.spark.config.SparkFlexConfig;

public class PIDHelper {

    public static <T extends PIDConstants> void applyPIDConstants(SparkFlexConfig config) {
        config.closedLoop.pid(
            T.kP,
            T.kI,
            T.kD
        );
    }

    public static <T extends PIDConstantsFF> void applyPIDFFConstants(SparkFlexConfig config) {
        config.closedLoop.pidf(
            T.kP,
            T.kI,
            T.kD,
            T.kFF
        );
    }
}
