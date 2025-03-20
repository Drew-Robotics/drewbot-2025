package frc.robot.constants;

import java.util.List;

import frc.robot.subsystems.drive.ReefSide;

public class ReefSides {
    public static final ReefSide kFrontBlue = new ReefSide(18, false);
    public static final ReefSide kFrontLeftBlue = new ReefSide(19, false);
    public static final ReefSide kFrontRightBlue = new ReefSide(17, false);

    public static final ReefSide kBackBlue = new ReefSide(21, true);
    public static final ReefSide kBackLeftBlue = new ReefSide(20, true);
    public static final ReefSide kBackRightBlue = new ReefSide(22, true);


    public static final ReefSide kFrontRed = new ReefSide(7, false);
    public static final ReefSide kFrontLeftRed = new ReefSide(6, false);
    public static final ReefSide kFrontRightRed = new ReefSide(8, false);

    public static final ReefSide kBackRed = new ReefSide(10, true);
    public static final ReefSide kBackLeftRed = new ReefSide(11, true);
    public static final ReefSide kBackRightRed = new ReefSide(9, true);


    public static final List<ReefSide> kReefSides = List.of(
        kFrontBlue, kFrontLeftBlue, kFrontRightBlue,
        kBackBlue, kBackLeftBlue, kBackRightBlue,

        kFrontRed, kFrontLeftRed, kFrontRightRed,
        kBackRed, kBackLeftRed, kBackRightRed
    );
}
