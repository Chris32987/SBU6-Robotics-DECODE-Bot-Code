package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Poses {
    public enum AllianceColor {
        BLUE, RED
    }

    public static AllianceColor CurrentAlliance = AllianceColor.BLUE;
    public static Pose Goal = new Pose(0,144);
    public static Pose AutoEnd = new Pose(15,119.2,Math.toRadians(324));

    public static double TurretEnd = -391;

    public static final long instanceId = System.nanoTime();

    public static void SetAlliance(AllianceColor alliance){
        TurretEnd = -391;
        if(alliance == AllianceColor.BLUE){
            CurrentAlliance = AllianceColor.BLUE;
            Goal = new Pose(10,135);
            AutoEnd = new Pose (15,119.2,Math.toRadians(324));
        }
        else{
            CurrentAlliance = AllianceColor.RED;
            Goal = new Pose(134, 135);
            AutoEnd = new Pose (15,119.2,Math.toRadians(324)).mirror();
        }
    }
}
