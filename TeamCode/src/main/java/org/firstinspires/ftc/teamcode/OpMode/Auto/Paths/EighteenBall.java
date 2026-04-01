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

public abstract class EighteenBall extends NextFTCOpMode {
    protected final Poses.AllianceColor alliance;
    private final double ShootTime = 0.8;

    public EighteenBall(Poses.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose = new Pose(15.2, 119, Math.toRadians(324));

    // preload
    private Pose ScorePreloadPose = new Pose(52.095, 93.356, Math.toRadians(-74));

    // middle spike
    private Pose IntakeMiddleSpikeControl = new Pose(62.969, 55.892);
    private Pose IntakeMiddleSpikePose = new Pose(13.434, 59.800, Math.toRadians(0));
    private Pose ScoreMiddleSpikePose = new Pose(60.031, 76.434, Math.toRadians(0));

    // gate intake run 1
    private Pose ApproachGateIntake1Control = new Pose(31.717, 69.619);
    private Pose ApproachGateIntake1Pose = new Pose(11.403, 69.732, Math.toRadians(0));
    private Pose GateIntake1Control = new Pose(14.510, 60.203);
    private Pose GateIntake1Pose = new Pose(4.247, 55.800, Math.toRadians(90));
    private Pose ScoreGateIntake1Pose = new Pose(60.939, 76.434, Math.toRadians(0));

    // gate intake run 2
    private Pose ApproachGateIntake2Control = new Pose(31.578, 69.564);
    private Pose ApproachGateIntake2Pose = new Pose(11.349, 69.529, Math.toRadians(0));
    private Pose GateIntake2Control = new Pose(14.195, 60.108);
    private Pose GateIntake2Pose = new Pose(4.207, 56.105, Math.toRadians(90));
    private Pose ScoreGateIntake2Pose = new Pose(57.854, 84.861, Math.toRadians(0));

    // close spike
    private Pose IntakeCloseSpikePose = new Pose(14.831, 83.695, Math.toRadians(0));
    private Pose ScoreCloseSpikePose = new Pose(60.671, 76.502, Math.toRadians(0));

    // far spike
    private Pose ApproachFarSpikeControl = new Pose(60.529, 32.102);
    private Pose IntakeFarSpikePose = new Pose(13.424, 35.634, Math.toRadians(0));
    private Pose ScoreFarSpikePose = new Pose(55.614, 118.512, Math.toRadians(0));

    private void InitPoses() {
        if (alliance == Poses.AllianceColor.RED) {
            StartPose = StartPose.mirror();
            ScorePreloadPose = ScorePreloadPose.mirror();
            IntakeMiddleSpikeControl = IntakeMiddleSpikeControl.mirror();
            IntakeMiddleSpikePose = IntakeMiddleSpikePose.mirror();
            ScoreMiddleSpikePose = ScoreMiddleSpikePose.mirror();
            ApproachGateIntake1Control = ApproachGateIntake1Control.mirror();
            ApproachGateIntake1Pose = ApproachGateIntake1Pose.mirror();
            GateIntake1Control = GateIntake1Control.mirror();
            GateIntake1Pose = GateIntake1Pose.mirror();
            ScoreGateIntake1Pose = ScoreGateIntake1Pose.mirror();
            ApproachGateIntake2Control = ApproachGateIntake2Control.mirror();
            ApproachGateIntake2Pose = ApproachGateIntake2Pose.mirror();
            GateIntake2Control = GateIntake2Control.mirror();
            GateIntake2Pose = GateIntake2Pose.mirror();
            ScoreGateIntake2Pose = ScoreGateIntake2Pose.mirror();
            IntakeCloseSpikePose = IntakeCloseSpikePose.mirror();
            ScoreCloseSpikePose = ScoreCloseSpikePose.mirror();
            ApproachFarSpikeControl = ApproachFarSpikeControl.mirror();
            IntakeFarSpikePose = IntakeFarSpikePose.mirror();
            ScoreFarSpikePose = ScoreFarSpikePose.mirror();
        }
    }

    private PathChain scorePreload;
    private PathChain intakeMiddleSpike, scoreMiddleSpike;
    private PathChain approachGateIntake1, gateIntake1, scoreGateIntake1;
    private PathChain approachGateIntake2, gateIntake2, scoreGateIntake2;
    private PathChain intakeCloseSpike, scoreCloseSpike;
    private PathChain approachFarSpike, scoreFarSpike;

    private void BuildPaths() {
        // drive to preload score position
        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(StartPose, ScorePreloadPose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), ScorePreloadPose.getHeading())
                .build();

        // sweep out to collect the middle spike ball
        intakeMiddleSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScorePreloadPose, IntakeMiddleSpikeControl, IntakeMiddleSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        // drive back to score the middle spike, reversed
        scoreMiddleSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(IntakeMiddleSpikePose, ScoreMiddleSpikePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        // reposition to get in line for the gate
        approachGateIntake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScoreMiddleSpikePose, ApproachGateIntake1Control, ApproachGateIntake1Pose))
                .setTangentHeadingInterpolation()
                .build();

        // run through the gate to collect balls
        gateIntake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ApproachGateIntake1Pose, GateIntake1Control, GateIntake1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(-180), GateIntake1Pose.getHeading())
                .build();

        // drive back to score gate intake run 1, reversed
        scoreGateIntake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(GateIntake1Pose, ScoreGateIntake1Pose))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        // reposition to line up for the gate again
        approachGateIntake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScoreGateIntake1Pose, ApproachGateIntake2Control, ApproachGateIntake2Pose))
                .setTangentHeadingInterpolation()
                .build();

        // run through the gate again to collect more balls
        gateIntake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ApproachGateIntake2Pose, GateIntake2Control, GateIntake2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(-180), GateIntake2Pose.getHeading())
                .build();

        // drive back to score gate intake run 2
        scoreGateIntake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(GateIntake2Pose, ScoreGateIntake2Pose))
                .setTangentHeadingInterpolation()
                .build();

        // straight line grab of the close spike ball
        intakeCloseSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(ScoreGateIntake2Pose, IntakeCloseSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        // drive back to score the close spike
        scoreCloseSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(IntakeCloseSpikePose, ScoreCloseSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        // swing around to collect the far spike ball
        approachFarSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScoreCloseSpikePose, ApproachFarSpikeControl, IntakeFarSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        // drive to final score position
        scoreFarSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(IntakeFarSpikePose, ScoreFarSpikePose))
                .setTangentHeadingInterpolation()
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

                // score preload: flywheel spins up, turret starts tracking after 0.4s
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Shooter.INSTANCE.FlywheelOn,
                        new SequentialGroup(
                                new Delay(0.4),
                                Turret.INSTANCE.TrackingOn
                        )
                ),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // middle spike
                Intake.INSTANCE.intakeSpin,
                new FollowPath(intakeMiddleSpike),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreMiddleSpike),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // gate intake run 1
                new FollowPath(approachGateIntake1),

                Intake.INSTANCE.intakeSpin,
                new FollowPath(gateIntake1),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreGateIntake1),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // gate intake run 2
                new FollowPath(approachGateIntake2),

                Intake.INSTANCE.intakeSpin,
                new FollowPath(gateIntake2),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreGateIntake2),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // close spike
                Intake.INSTANCE.intakeSpin,
                new FollowPath(intakeCloseSpike),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreCloseSpike),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // far spike
                Intake.INSTANCE.intakeSpin,
                new FollowPath(approachFarSpike),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreFarSpike),
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
        Poses.AutoEnd = robotPose;
        Poses.TurretEnd = Turret.INSTANCE.GetTurretPosition();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", Poses.CurrentAlliance);
        telemetry.addData("Goal Pose", Poses.Goal);
        telemetry.addData("AutoEndThink", Poses.AutoEnd);
        telemetry.update();
        drawOnlyCurrent();
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
