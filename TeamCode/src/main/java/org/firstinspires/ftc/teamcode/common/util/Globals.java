package org.firstinspires.ftc.teamcode.WIP.common.util;

import org.apache.commons.math3.geometry.partitioning.Side;

public class Globals {
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = true;
    public static boolean USING_DASHBOARD = false;

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void stopScoring(){
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

}