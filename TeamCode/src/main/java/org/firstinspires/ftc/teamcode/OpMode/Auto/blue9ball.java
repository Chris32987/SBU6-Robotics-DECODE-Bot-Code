package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.NineBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="9 Ball Autonomous - BLUE")
public class blue9ball extends NineBall {
    public blue9ball() {super(Poses.AllianceColor.BLUE); }
}
