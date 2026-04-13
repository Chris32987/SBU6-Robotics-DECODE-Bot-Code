package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private final MotorEx TurretMotor = new MotorEx("TurretMotor").brakeMode().zeroed().reversed();
    private boolean EnableTracking    = false;
    private boolean wasAimed          = false;
    private boolean trackingOffPending = false;
    private GoBildaPrismDriver leds;
    private final com.qualcomm.robotcore.util.ElapsedTime trackingOffTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    private double CalculateTurretPos(Pose CurrentGoalPos) {
        Pose RobotPose = PedroComponent.follower().getPose();

        double dx = CurrentGoalPos.getX() - RobotPose.getX();
        double dy = CurrentGoalPos.getY() - RobotPose.getY();
        double rawDelta = Math.atan2(dy, dx) - (RobotPose.getHeading() + Math.PI);

        double AngleRad = Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));

        double CalcGoalTicks = (AngleRad / (2 * Math.PI)) * 145.1 * (97.0 / 18.0);
        Aimed = Math.abs(CalcGoalTicks - TurretMotor.getCurrentPosition() * -1) < 15;
        return Math.max(-350, Math.min(CalcGoalTicks, 435));
    }

    public Boolean Aimed = false;

    public Command TrackingOn = new InstantCommand(() -> {
        EnableTracking = true;
        trackingOffPending = false;
    });
    public Command TrackingOff = new InstantCommand(() -> {
        EnableTracking = false;
        trackingOffPending = true;
        trackingOffTimer.reset();
    });
    public Command SetTurretPosition(double pos) {
        return new InstantCommand(() -> TurretMotor.setCurrentPosition(pos));
    }

    public double GetTurretPosition() { return TurretMotor.getCurrentPosition(); }

    ControlSystem controller = ControlSystem.builder()
            .posPid(0.014, 0, 0.0008)
            .build();

    @Override
    public void initialize() {
        leds = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "leds");
        leds.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_1,
                new PrismAnimations.Solid(Color.TRANSPARENT)
        );
    }

    @Override
    public void periodic() {
        double target = CalculateTurretPos(Poses.Goal);
        controller.setGoal(new KineticState(target, 0, 0));
        TurretMotor.setPower(!EnableTracking ? 0 : controller.calculate(TurretMotor.getState().times(-1)));

        if (Aimed && !wasAimed) {
            leds.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_1,
                    new PrismAnimations.Solid(Color.RED)
            );
        } else if (!Aimed && wasAimed) {
            leds.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_1,
                    new PrismAnimations.Solid(Color.TRANSPARENT)
            );
        }
        wasAimed = Aimed;

        if (trackingOffPending && trackingOffTimer.seconds() >= 1.0) {
            leds.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_1,
                    new PrismAnimations.Solid(Color.TRANSPARENT)
            );
            trackingOffPending = false;
        }

        ActiveOpMode.telemetry().addData("TurretTarget", target);
        ActiveOpMode.telemetry().addData("TurretPos", TurretMotor.getCurrentPosition() * -1);
    }
}