package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.OpMode.Auto.Paths.TeleStart;
import org.firstinspires.ftc.teamcode.Poses;

@Autonomous(name="RedTeleStart")
public class TeleStartRed extends TeleStart {
    public TeleStartRed() { super(Poses.AllianceColor.RED); }
}
