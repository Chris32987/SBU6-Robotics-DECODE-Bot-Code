package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.EighteenBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="18 Ball Autonomous - red")
public class red18ball extends EighteenBall {
    public red18ball() { super(Poses.AllianceColor.RED); }
}
