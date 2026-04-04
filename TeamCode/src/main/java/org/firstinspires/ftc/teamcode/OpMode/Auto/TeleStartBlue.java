package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.TeleStart;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="BlueTeleStart")
public class TeleStartBlue extends TeleStart {
    public TeleStartBlue() { super(Poses.AllianceColor.BLUE); }
}
