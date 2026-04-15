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

public abstract class TwelveBall extends NextFTCOpMode {
    protected final Poses.AllianceColor alliance;
    private final double ShootTime = 0.8;

    public TwelveBall(Poses.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose     = new Pose(15.2,      119,   Math.toRadians(324));
    private Pose ScorePose     = new Pose(32.976,  108.539, Math.toRadians(270));
    private Pose Spike1Control = new Pose(70,  90);
    private Pose Spike1Pose    = new Pose(12.411,  83.929,  Math.toRadians(170));
    private Pose ScorePose2    = new Pose(55.802,  84.189,  Math.toRadians(-86));
    private Pose Spike2Control = new Pose(58.002,  57.245);
    private Pose Spike2Pose    = new Pose(9.136,  60.152,  Math.toRadians(180));
    private Pose ScorePose3    = new Pose(64.626,  75.066,  Math.toRadians(-102));
    private Pose Spike3Control = new Pose(58.002,  32.733);
    private Pose Spike3Pose    = new Pose(10.387,  35.427);
    private Pose EndScorePose  = new Pose(58.870,  111.051);

    private void InitPoses() {
        if (alliance == Poses.AllianceColor.RED) {
            StartPose = StartPose.mirror();
            ScorePose     = ScorePose.mirror();
            Spike1Control = Spike1Control.mirror();
            Spike1Pose    = Spike1Pose.mirror();
            ScorePose2    = ScorePose2.mirror();
            Spike2Control = Spike2Control.mirror();
            Spike2Pose    = Spike2Pose.mirror();
            ScorePose3    = ScorePose3.mirror();
            Spike3Control = Spike3Control.mirror();
            Spike3Pose    = Spike3Pose.mirror();
            EndScorePose  = EndScorePose.mirror();
        }
    }

    private PathChain ScorePreload, IntakeSpike1, ScoreSpike1, IntakeSpike2, ScoreSpike2, IntakeSpike3, ScoreSpike3;

    private void BuildPaths() {
        ScorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(StartPose, ScorePose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), ScorePose.getHeading())
                .build();
        IntakeSpike1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScorePose, Spike1Control, Spike1Pose))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), Spike1Pose.getHeading())
                .build();
        ScoreSpike1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Spike1Pose, ScorePose2))
                .setLinearHeadingInterpolation(Spike1Pose.getHeading(), ScorePose2.getHeading())
                .build();
        IntakeSpike2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScorePose2, Spike2Control, Spike2Pose))
                .setTangentHeadingInterpolation()
                .build();
        ScoreSpike2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Spike2Pose, ScorePose3))
                .setLinearHeadingInterpolation(Spike2Pose.getHeading(), ScorePose3.getHeading())
                .build();
        IntakeSpike3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ScorePose3, Spike3Control, Spike3Pose))
                .setTangentHeadingInterpolation()
                .build();
        ScoreSpike3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Spike3Pose, EndScorePose))
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

                Intake.INSTANCE.intakeSpin,
                new FollowPath(IntakeSpike1),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(ScoreSpike1),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                Intake.INSTANCE.intakeSpin,
                new FollowPath(IntakeSpike2),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(ScoreSpike2),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                Intake.INSTANCE.intakeSpin,
                new FollowPath(IntakeSpike3),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(ScoreSpike3),
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
        if (robotPose.distanceFrom(Poses.AutoEnd) < 5) {
            Poses.AUTO_END_POSE = robotPose;
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