package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.TwelveBall;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="12 Ball Autonomous - BLUE")
public class blue12ball extends TwelveBall {
    public blue12ball() {super(Poses.AllianceColor.BLUE); }
}
