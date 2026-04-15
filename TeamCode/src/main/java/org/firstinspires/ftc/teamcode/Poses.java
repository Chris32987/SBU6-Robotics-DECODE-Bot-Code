package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import dev.nextftc.extensions.pedro.PedroComponent;

public class Poses {
    public enum AllianceColor {
        BLUE, RED
    }

    public static AllianceColor CurrentAlliance = AllianceColor.BLUE;
    public static Pose Goal = new Pose(0,144);
    //public static Pose AutoEnd = new Pose(15.2,119,Math.toRadians(324));
    public static Pose  HumanPlayerZone = new Pose (136.8125, 6.09375,  Math.toRadians(0));
    public static double TurretEnd = -391;

    public static final long instanceId = System.nanoTime();

    public static void SetAlliance(AllianceColor alliance){
        TurretEnd = -391;
        if(alliance == AllianceColor.BLUE){
            CurrentAlliance = AllianceColor.BLUE;
            Goal = new Pose(0,144);
            AUTO_END_POSE = new Pose (15.2,119,Math.toRadians(324));
            HumanPlayerZone = new Pose (136.8125, 6.09375,  Math.toRadians(180));

        }

        else{
            CurrentAlliance = AllianceColor.RED;
            Goal = new Pose(0,144).mirror();
            AUTO_END_POSE = new Pose (15.2,119,Math.toRadians(324)).mirror();
            HumanPlayerZone = new Pose (136.8125, 6.09375, Math.toRadians(180)).mirror();
        }
    }
    public static Pose AUTO_END_POSE = new Pose(17.5, 120, Math.toRadians(324));

    public static double AUTO_END_X = 17.5;
    public static double AUTO_END_Y = 120;
    public static double AUTO_END_HEADING = Math.toRadians(324);
}
