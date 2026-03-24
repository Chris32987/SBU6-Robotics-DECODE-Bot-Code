package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.Arrays;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").floatMode(),
            new MotorEx("flywheelMotor2").reversed().floatMode()
    );

    private final ServoEx HoodServoRight = new ServoEx("HoodServoRight");
    private final ServoEx HoodServoLeft = new ServoEx("HoodServoLeft");

    private final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(40.0, 50.0, 55.0, 70.0, 65.0, 70.0, 80.0, 90.0),
            Arrays.asList(1250.0, 1250.0, 1350.0, 1400.0, 1450.0, 1450.0, 1470.0, 1500.0)
    ).createLUT();

    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(40.0, 50.0, 55.0, 70.0, 65.0, 70.0, 80.0, 90.0),
            Arrays.asList(0.32, 0.30, 0.24, 0.20, 0.34, 0.20, 0.26, 0.18)
    ).createLUT();

    private boolean spinFlywheel = false;


            ControlSystem controller = ControlSystem.builder()
            .velPid(0, 0, 0)
            .basicFF(0, 0, 0)
            .build();

    public Command FlywheelOn = new InstantCommand(() -> spinFlywheel = true);
    public Command FlywheelOff = new InstantCommand(() -> spinFlywheel = false);

    @Override
    public void periodic() {
        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(new Pose(0,144));

        HoodServoRight.setPosition(hoodLUT.get(distance));
        HoodServoLeft.setPosition(1 - hoodLUT.get(distance));

        controller.setGoal(new KineticState(0, velocityLUT.get(distance), 0));

        flywheelMotors.setPower(!spinFlywheel ? 0 : controller.calculate(flywheelMotors.getState()));
}}
