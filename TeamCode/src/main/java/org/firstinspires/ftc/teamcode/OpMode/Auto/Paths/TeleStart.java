package org.firstinspires.ftc.teamcode.OpMode.Auto.Paths;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

public abstract class TeleStart extends NextFTCOpMode {
    protected final Poses.AllianceColor alliance;

    public TeleStart(Poses.AllianceColor alliance) {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose = new Pose(15.2, 119, Math.toRadians(90));

    private void InitPoses() {
        if (alliance == Poses.AllianceColor.RED) {
            StartPose = StartPose.mirror();
        }
    }

    @Override
    public void onInit() {
        Poses.SetAlliance(alliance);
        InitPoses();
        PedroComponent.follower().setStartingPose(StartPose);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance", Poses.CurrentAlliance);
        telemetry.update();
    }
}