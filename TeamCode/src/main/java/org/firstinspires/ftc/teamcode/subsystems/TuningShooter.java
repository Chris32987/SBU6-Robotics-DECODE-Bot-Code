package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class TuningShooter implements Subsystem {
    public static final TuningShooter INSTANCE = new TuningShooter();
    private TuningShooter() { }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor2").floatMode(),
            new MotorEx("flywheelMotor1").reversed().floatMode()
    );

    public ServoEx HoodServoRight = new ServoEx("HoodServoRight");
    public ServoEx HoodServoLeft = new ServoEx("HoodServoLeft");

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.003,0,0)
            .basicFF(0.00038,0,0.09)
            .build();

    double power;
    double flywheelTarget;
    private boolean spinFlywheel = false;

    public Command moveFlywheelByValue(double increment) {
        return new InstantCommand(() -> {
            flywheelTarget += increment;
            controller.setGoal(new KineticState(0, flywheelTarget, 0));
        });
    }

    public Command moveHoodByValue(double newPosition) {
        return new InstantCommand(() -> {
            HoodServoRight.setPosition(newPosition);
            HoodServoLeft.setPosition(1 - newPosition);
        });
    }

    public Command turnOn = new InstantCommand(() -> spinFlywheel = true);
    public Command turnOff = new InstantCommand(() -> spinFlywheel = false);

    public double hoodPosition = 0.4;

    @Override
    public void initialize() {
        moveHoodByValue(hoodPosition).schedule();
        flywheelTarget = 0;
    }

    @Override
    public void periodic() {
        controller.setGoal(new KineticState(0, flywheelTarget, 0));
        if (spinFlywheel) power = controller.calculate(flywheelMotors.getState().times(-1));
        else power = 0;

        hoodPosition = HoodServoRight.getPosition();

        flywheelMotors.setPower(power);

        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelMotors.getVelocity() * -1);
        ActiveOpMode.telemetry().addData("Flywheel Target", flywheelTarget);
        ActiveOpMode.telemetry().addData("Applied Power", power);
        ActiveOpMode.telemetry().addData("Hood Position", HoodServoRight.getPosition());
    }
}