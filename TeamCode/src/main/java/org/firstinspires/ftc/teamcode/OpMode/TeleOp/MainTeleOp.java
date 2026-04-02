package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

    }

    private PedroDriverControlled driverControlled;

    @Override public void onInit() {
        PedroComponent.follower().setPose(Poses.AutoEnd);
        Turret.INSTANCE.TrackingOff.schedule();
        Turret.INSTANCE.SetTurretPosition(Poses.TurretEnd).schedule();
    }
    @Override public void onWaitForStart() {
        telemetry.addData("AutoEndPos", Poses.AutoEnd);
        telemetry.addData("CurrentPos", PedroComponent.follower().getPose());
        telemetry.addData("Log", Poses.instanceId);
        telemetry.update();

    }
    @Override public void onStartButtonPressed() {

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
        //Turret.INSTANCE.TrackingOn.schedule();
        Gamepads.gamepad1().leftTrigger().greaterThan(0.05)
                .whenBecomesTrue(Intake.INSTANCE.intakeSpin)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.05)
                .whenBecomesTrue(new SequentialGroup(
                        new ParallelGroup(
                                Intake.INSTANCE.GateOpen,
                                Shooter.INSTANCE.FlywheelOn
                        ),
                        new ParallelRaceGroup(
                                new Delay(1),
                                new WaitUntil(() -> Shooter.INSTANCE.AtSpeed)
                        ),
                        Intake.INSTANCE.intakeSpin
                ))
                .whenBecomesFalse( new ParallelGroup(
                        Intake.INSTANCE.intakeOff,
                        Shooter.INSTANCE.FlywheelOff,
                        Intake.INSTANCE.GateClose
                ));

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.TrackingOn)
                .whenBecomesFalse(Turret.INSTANCE.TrackingOff);
    }
    @Override public void onUpdate() {
        driverControlled.setScalar(0.25);

        Pose robotPose = PedroComponent.follower().getPose();
        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Robot Heading", robotPose.getHeading());

        telemetry.update(); // LOOK HERE
    }
    @Override public void onStop() { }


}
