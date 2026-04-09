package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.NineBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="9 Ball Autonomous - RED")
public class red9ball extends NineBall {
    public red9ball() {super(Poses.AllianceColor.RED); }
}
