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

public abstract class NineBall extends NextFTCOpMode {
    protected final Poses.AllianceColor alliance;
    private final double ShootTime = 0.8;

    public NineBall(Poses.AllianceColor alliance) {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
        this.alliance = alliance;
    }

    private Pose StartPose        = new Pose(15.2,      119,      Math.toRadians(90));
    private Pose Score1Pose       = new Pose(52.475,    83.471,   Math.toRadians(180));
    private Pose Intake1Pose      = new Pose(15.356,    83.403);
    private Pose Score2Pose       = new Pose(52.746,    83.105,   Math.toRadians(-80));
    private Pose Intake2Control   = new Pose(60,        65);
    private Pose Intake2Pose      = new Pose(15.105,    58.695);
    private Pose Score3Pose       = new Pose(56.434,    116.007);

    private void InitPoses() {
        if (alliance == Poses.AllianceColor.RED) {
            StartPose      = StartPose.mirror();
            Score1Pose     = Score1Pose.mirror();
            Intake1Pose    = Intake1Pose.mirror();
            Score2Pose     = Score2Pose.mirror();
            Intake2Control = Intake2Control.mirror();
            Intake2Pose    = Intake2Pose.mirror();
            Score3Pose     = Score3Pose.mirror();
        }
    }

    private PathChain Path1, Path2, Path3, Path4, Path5;

    private void BuildPaths() {
        Path1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(StartPose, Score1Pose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), Score1Pose.getHeading())
                .build();
        Path2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Score1Pose, Intake1Pose))
                .setTangentHeadingInterpolation()
                .build();
        Path3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Intake1Pose, Score2Pose))
                .setLinearHeadingInterpolation(Intake1Pose.getHeading(), Score2Pose.getHeading())
                .setReversed()
                .build();
        Path4 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(Score2Pose, Intake2Control, Intake2Pose))
                .setTangentHeadingInterpolation()
                .build();
        Path5 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(Intake2Pose, Score3Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
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
                new ParallelGroup( // drive to score 1 while spinning flywheel and turning on tracking
                        new FollowPath(Path1),
                        Shooter.INSTANCE.FlywheelOn,
                        new SequentialGroup(
                                new Delay(0.4),
                                Turret.INSTANCE.TrackingOn
                        )
                ),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                Intake.INSTANCE.intakeSpin,
                new FollowPath(Path2),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(Path3),
                ShootArtifacts(),
                Shooter.INSTANCE.FlywheelOff,

                Intake.INSTANCE.intakeSpin,
                new FollowPath(Path4),
                Intake.INSTANCE.intakeOff,

                Shooter.INSTANCE.FlywheelOn,
                new FollowPath(Path5),
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