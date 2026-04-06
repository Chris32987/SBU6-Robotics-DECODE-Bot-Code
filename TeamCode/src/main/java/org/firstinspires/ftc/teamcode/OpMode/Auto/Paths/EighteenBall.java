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

    private Pose StartPose = new Pose(15, 119.2, Math.toRadians(324));

    // preload
    private Pose ScorePreloadPose = new Pose(52.095, 93.356, Math.toRadians(-74));

    // middle spike
    private Pose IntakeMiddleSpikeControl = new Pose(60.0,  65.0);
    private Pose IntakeMiddleSpikePose =  new Pose(9.136,  60.152);
    private Pose ScoreMiddleSpikePose = new Pose(60.031, 76.434, Math.toRadians(180));

    // gate intake run 1
    private Pose ApproachGateIntake1Control = new Pose(2.559, 73.856);
    private Pose ApproachGateIntake1Control2 = new Pose(52, 57.23);
    private Pose ApproachGateIntake1Pose = new Pose(40, 69.732, Math.toRadians(90));
    private Pose GateIntake1Control = new Pose(14.510, 60.203);
    private Pose GateIntake1Pose = new Pose(4.247, 53, Math.toRadians(100));
    private Pose ScoreGateIntake1Pose = new Pose(60.939, 76.434, Math.toRadians(0));

    // gate intake run 2
    private Pose ApproachGateIntake2Control = new Pose(2.559, 73.856);
    private Pose ApproachGateIntake2Control2 = new Pose(24.817, 53.331);
    private Pose ApproachGateIntake2Pose = new Pose(40, 69.732, Math.toRadians(180));
    private Pose GateIntake2Control = new Pose(14.510, 60.203);
    private Pose GateIntake2Pose = new Pose(4.247, 55.800, Math.toRadians(90));
    private Pose ScoreGateIntake2Pose = new Pose(60.939, 76.434, Math.toRadians(0));

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
            ApproachGateIntake1Control2 = ApproachGateIntake1Control2.mirror();
            ApproachGateIntake2Control2 = ApproachGateIntake2Control2.mirror();
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
                .addPath(new BezierCurve(ScoreMiddleSpikePose, ApproachGateIntake1Control, ApproachGateIntake1Control2, GateIntake1Pose))
                .setLinearHeadingInterpolation(ScoreMiddleSpikePose.getHeading(), GateIntake1Pose.getHeading())
                .build();

        // drive back to score gate intake run 1, reversed
        scoreGateIntake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(GateIntake1Pose,ScoreMiddleSpikePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        // straight line grab of the close spike ball
        intakeCloseSpike = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(ScoreMiddleSpikePose, IntakeCloseSpikePose))
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
                Intake.INSTANCE.intakeSpin,
                new FollowPath(approachGateIntake1),
                new Delay(1.5),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreGateIntake1),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                // gate intake run 1
                Intake.INSTANCE.intakeSpin,
                new FollowPath(approachGateIntake1),
                new Delay(1.5),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(scoreGateIntake1),
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
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff


        );
    }

    @Override
    public void onInit() {
        Poses.SetAlliance(alliance);
        InitPoses();
        BuildPaths();
        PedroComponent.follower().setStartingPose(StartPose);
        Turret.INSTANCE.SetTurretPosition(-391).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        Pose robotPose = PedroComponent.follower().getPose();
        Poses.AUTO_END_POSE = robotPose;
        Poses.AUTO_END_X = robotPose.getX();
        Poses.AUTO_END_Y = robotPose.getY();
        Poses.AUTO_END_HEADING = robotPose.getHeading();
        Poses.TurretEnd = Turret.INSTANCE.GetTurretPosition();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Alliance", Poses.CurrentAlliance);
        telemetry.addData("Goal Pose", Poses.Goal);
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
