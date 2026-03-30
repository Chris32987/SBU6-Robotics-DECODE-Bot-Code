package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.EighteenBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="18 Ball Autonomous - BLUE")
public class blue18ball extends EighteenBall {
    public blue18ball() { super(Poses.AllianceColor.BLUE); }
}
