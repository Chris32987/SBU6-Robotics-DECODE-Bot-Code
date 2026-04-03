package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TuningShooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Tuning Flywheel")
public class TuningOpmode extends NextFTCOpMode {
    public TuningOpmode() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, TuningShooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    public static double hoodPos = 0.4;

    @Override public void onInit() {
        PedroComponent.follower().setPose(Poses.AutoEnd);
        Turret.INSTANCE.TrackingOff.schedule();
        Turret.INSTANCE.SetTurretPosition(Poses.TurretEnd).schedule();
    }
    @Override

    public void onWaitForStart() {
        telemetry.addData("AutoEndPos", Poses.AutoEnd);
        telemetry.addData("CurrentPos", PedroComponent.follower().getPose());
        telemetry.addData("Log", Poses.instanceId);
        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled;

        if (Poses.CurrentAlliance == Poses.AllianceColor.BLUE) {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }
        else {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX().negate(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }

        driverControlled.schedule();

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(new ParallelGroup(
                        TuningShooter.INSTANCE.moveHoodByValue(0.02 + hoodPos),
                        new InstantCommand(() -> hoodPos += 0.02)
                ));

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(new ParallelGroup(
                        TuningShooter.INSTANCE.moveHoodByValue(-0.02 + hoodPos),
                        new InstantCommand(() -> hoodPos -= 0.02)
                ));

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(TuningShooter.INSTANCE.moveFlywheelByValue(20));

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(TuningShooter.INSTANCE.moveFlywheelByValue(-20));

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.TrackingOn)
                .whenBecomesFalse(Turret.INSTANCE.TrackingOff);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.intakeSpin)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(TuningShooter.INSTANCE.turnOn)
                .whenBecomesFalse(TuningShooter.INSTANCE.turnOff);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.05)
                .whenBecomesTrue(new SequentialGroup(
                        new ParallelGroup(
                                Intake.INSTANCE.GateOpen,
                                TuningShooter.INSTANCE.turnOn
                        ),
                        new Delay(0.5),
                        Intake.INSTANCE.intakeSpin
                ))
                .whenBecomesFalse( new ParallelGroup(
                        Intake.INSTANCE.intakeOff,
                        TuningShooter.INSTANCE.turnOff,
                        Intake.INSTANCE.GateClose
                ));
    }

    @Override
    public void onUpdate() {
        TuningShooter.INSTANCE.HoodServoRight.setPosition(hoodPos);
        TuningShooter.INSTANCE.HoodServoLeft.setPosition(1-hoodPos);
        Pose robotPose = PedroComponent.follower().getPose();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());
        telemetry.addData("Distance", robotPose.distanceFrom(Poses.Goal));
        telemetry.update();
    }
}
