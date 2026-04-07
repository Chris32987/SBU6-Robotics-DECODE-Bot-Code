package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor2").floatMode(),
            new MotorEx("flywheelMotor1").reversed().floatMode()
    );

    private final ServoEx HoodServoRight = new ServoEx("HoodServoRight");
    private final ServoEx HoodServoLeft = new ServoEx("HoodServoLeft");

    private final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(51.3696, 57.0557, 70.3733, 74.1295, 80.2525, 94.6509, 99.6189, 103.065),
            Arrays.asList(1300.0, 1360.0, 1420.0, 1440.0, 1480.0, 1620.0, 1680.0, 1740.0)
    ).createLUT();

    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(51.3696, 57.0557, 70.3733, 74.1295, 80.2525, 94.6509, 99.6189, 103.065),
            Arrays.asList(0.64, 0.60, 0.58, 0.58, 0.56, 0.54, 0.52, 0.54)
    ).createLUT();


    private boolean spinFlywheel = false;
    public boolean AtSpeed = false;


    ControlSystem controller = ControlSystem.builder()
            .velPid(0.005,0,0)
            .basicFF(0.00037,0,0.09)
            .build();

    public Command FlywheelOn = new InstantCommand(() -> spinFlywheel = true);
    public Command FlywheelOff = new InstantCommand(() -> spinFlywheel = false);

    @Override
    public void periodic() {
        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(Poses.Goal);
        double TargetVelocity = velocityLUT.get(distance);
        AtSpeed = Math.abs(flywheelMotors.getVelocity() - TargetVelocity) < 50;

        HoodServoRight.setPosition(hoodLUT.get(distance));
        HoodServoLeft.setPosition(1 - hoodLUT.get(distance));

        controller.setGoal(new KineticState(0, TargetVelocity, 0));

        flywheelMotors.setPower(!spinFlywheel ? 0 : controller.calculate(flywheelMotors.getState().times(-1)));

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity() * -1);
        ActiveOpMode.telemetry().addData("Flywheel Target", TargetVelocity);
        ActiveOpMode.telemetry().addData("Hood Position", HoodServoRight.getPosition());
        ActiveOpMode.telemetry().addData("Distance to Goal", distance);
        ActiveOpMode.telemetry().addData("Goal Pose", Poses.Goal);

        TelemetryPacket packet = new TelemetryPacket();
        //PedroComponent.follower().telemetryDebug(packet);
        packet.put("Flywheel Speed", flywheelMotors.getVelocity() * -1);
        packet.put("Flywheel Target", TargetVelocity);
        packet.put("Hood Position", HoodServoRight.getPosition());
        packet.put("Distance to Goal", distance);
        packet.put("At Speed", AtSpeed);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }}
