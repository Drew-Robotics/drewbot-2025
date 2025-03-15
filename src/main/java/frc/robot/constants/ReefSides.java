package frc.robot.constants;

import java.util.List;

import frc.robot.subsystems.drive.ReefSide;

public class ReefSides {
    public static final ReefSide kFrontBlue = new ReefSide(18);
    public static final ReefSide kFrontLeftBlue = new ReefSide(19);
    public static final ReefSide kFrontRightBlue = new ReefSide(17);

    public static final ReefSide kBackBlue = new ReefSide(21);
    public static final ReefSide kBackLeftBlue = new ReefSide(20);
    public static final ReefSide kBackRightBlue = new ReefSide(22);


    public static final ReefSide kFrontRed = new ReefSide(7);
    public static final ReefSide kFrontLeftRed = new ReefSide(6);
    public static final ReefSide kFrontRightRed = new ReefSide(8);

    public static final ReefSide kBackRed = new ReefSide(10);
    public static final ReefSide kBackLeftRed = new ReefSide(11);
    public static final ReefSide kBackRightRed = new ReefSide(9);


    public static final List<ReefSide> kReefSides = List.of(
        kFrontBlue, kFrontLeftBlue, kFrontRightBlue,
        kBackBlue, kBackLeftBlue, kBackRightBlue,

        kFrontRed, kFrontLeftRed, kFrontRightRed,
        kBackRed, kBackLeftRed, kBackRightRed
    );
}
