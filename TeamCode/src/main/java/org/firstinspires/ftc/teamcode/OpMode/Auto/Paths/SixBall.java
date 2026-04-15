package org.firstinspires.ftc.teamcode.OpMode.Auto.Paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
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
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose = new Pose(15.2, 119, Math.toRadians(324)); //make poses for every point
    private Pose ScorePose = new Pose(50.4, 84.2, Math.toRadians(270));
    private Pose MiddleSpikePose = new Pose(15.2, 60, Math.toRadians(180));
    private Pose MiddleSpikeControl = new Pose(58.8, 57.7);
    private Pose EndScorePose = new Pose(62.8, 98);

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

    private Command ShootArtifacts() {
        return new SequentialGroup(
                new SequentialGroup(
                        Intake.INSTANCE.GateOpen,
                        Intake.INSTANCE.intakeSpin,
                        new Delay(ShootTime)
                ),
                new ParallelGroup(
                        Intake.INSTANCE.GateClose,
                        Intake.INSTANCE.intakeOff
                )
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new ParallelGroup( //drives to first point while spinning flywheel and turning on tracking
                        new FollowPath(ScorePreload),
                        Shooter.INSTANCE.FlywheelOn,
                        new SequentialGroup(
                                new Delay(0.4),
                                Turret.INSTANCE.TrackingOn
                        )
                ),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                //intake middle spike
                Intake.INSTANCE.intakeSpin,
                new FollowPath(IntakeMiddleSpike),

                //scoring middle spike
                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(ScoreMiddleSpike),

                ShootArtifacts()
        );
    }
    @Override
    public void onInit() {
        Poses.SetAlliance(alliance);
        InitPoses();
        BuildPaths();
        PedroComponent.follower().setStartingPose(StartPose);
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
    @Override
    public void onUpdate() {
        Pose robotPose = PedroComponent.follower().getPose();
        if (robotPose.distanceFrom(Poses.AUTO_END_POSE) < 5) {
            Poses.AUTO_END_POSE = new Pose(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
            Poses.AUTO_END_X = robotPose.getX();
            Poses.AUTO_END_Y = robotPose.getY();
            Poses.AUTO_END_HEADING = robotPose.getHeading();
        }

        Poses.TurretEnd = Turret.INSTANCE.GetTurretPosition();

        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", Poses.CurrentAlliance);
        telemetry.addData("Goal Pose", Poses.Goal);
        telemetry.addData("Log", Poses.instanceId);
        telemetry.update();
        drawOnlyCurrent();
    }

    @Override
    public void onStop() {
        Turret.INSTANCE.TrackingOff.schedule();
        Shooter.INSTANCE.FlywheelOff.schedule();
    }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(PedroComponent.follower().getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}

