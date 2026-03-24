package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE,),
                new PedroComponent(Constants::createFollower)
        );
    }
    @Override public void onInit() { }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.intakeSpin)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeReverse)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().rightTrigger().greaterThan(0.05)
                .whenBecomesTrue(Shooter.INSTANCE.FlywheelOn)
                .whenBecomesFalse(Shooter.INSTANCE.FlywheelOff);

    }
    @Override public void onUpdate() { }
    @Override public void onStop() { }


}
