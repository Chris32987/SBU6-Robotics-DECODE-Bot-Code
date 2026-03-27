package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Poses;

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
    private final MotorEx TurretMotor = new MotorEx("TurretMotor").brakeMode().zeroed();
    private boolean EnableTracking = false;
    private double CalculateTurretPos(Pose CurrentGoalPos){
        Pose RobotPose = PedroComponent.follower().getPose();
        double dx = CurrentGoalPos.getX() - RobotPose.getX();
        double dy = CurrentGoalPos.getY() - RobotPose.getY();
        double rawDelta = Math.atan2(dy, dx) + RobotPose.getHeading();

        double AngleRad = Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));

        double CalcGoalTicks = (AngleRad / (2 * Math.PI)) * 145.1 * (97.0/18.0);
        return Math.max(-150, Math.min(CalcGoalTicks, 150));
    }
    public Command TrackingOn = new InstantCommand(() -> EnableTracking = true);
    public Command TrackingOff = new InstantCommand(() -> EnableTracking = false);
    public Command SetTurretPosition(double pos) {
        return new InstantCommand(() -> TurretMotor.setCurrentPosition(pos));
    }
    ControlSystem controller = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .build();

    @Override
    public void periodic() {
        double target = CalculateTurretPos(Poses.Goal);
        controller.setGoal(new KineticState(target, 0, 0));
        TurretMotor.setPower(!EnableTracking ? 0 : controller.calculate(TurretMotor.getState().times(-1))); //w claude
        ActiveOpMode.telemetry().addData("TurretTarget", target);
        ActiveOpMode.telemetry().addData("TurretPos", TurretMotor.getCurrentPosition());
        ActiveOpMode.telemetry().addData("TurretPower", controller.calculate(TurretMotor.getState().times(-1)));
    }

}

