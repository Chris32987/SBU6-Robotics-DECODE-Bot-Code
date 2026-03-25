package org.firstinspires.ftc.teamcode.OpMode.Auto.Paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class SixBall extends NextFTCOpMode { // constant over every auto
    protected final Poses.AllianceColor alliance;
    private final double ShootTime = 0.8; //seconds
    public SixBall(Poses.AllianceColor alliance) {
        addComponents(

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose = new Pose(15,119.2, Math.toRadians(324)); //make poses for every point
    private Pose ScorePose = new Pose(50.4,84.2, Math.toRadians(270));
    private Pose MiddleSpikePose = new Pose(15.2,60, Math.toRadians(180));
    private Pose MiddleSpikeControl = new Pose(58.8,57.7);
    private Pose EndScorePose = new Pose(62.8,98);

    private void InitPoses() {
        if (alliance == Poses.AllianceColor.RED) {
            StartPose = StartPose.mirror();
            ScorePose = ScorePose.mirror();
            MiddleSpikeControl = MiddleSpikeControl.mirror();
            MiddleSpikePose = MiddleSpikePose.mirror();
            EndScorePose = EndScorePose.mirror();
        }
    }

    private PathChain ScorePreload, IntakeMiddleSpike, ScoreMiddleSpike; //actually making paths
    private void BuildPaths() {
        ScorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(StartPose, ScorePose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), ScorePose.getHeading())
                .build();
        IntakeMiddleSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScorePose, MiddleSpikeControl, MiddleSpikePose))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), MiddleSpikePose.getHeading())
                .build();
        ScoreMiddleSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(MiddleSpikePose, EndScorePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();
    }


}