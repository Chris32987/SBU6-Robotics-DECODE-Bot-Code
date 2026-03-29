package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.SixBall;
import org.firstinspires.ftc.teamcode.Poses;
@Autonomous (name = "blue 6 ball")
public class blue6ball extends SixBall {
    public blue6ball (){ super(Poses.AllianceColor.BLUE);}
}
