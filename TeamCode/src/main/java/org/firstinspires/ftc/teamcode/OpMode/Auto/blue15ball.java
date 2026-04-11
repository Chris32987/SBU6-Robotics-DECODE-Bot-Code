package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.FifteenBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="15 Ball Autonomous - BLUE")
public class blue15ball extends FifteenBall {
    public blue15ball() { super(Poses.AllianceColor.BLUE); }
}
