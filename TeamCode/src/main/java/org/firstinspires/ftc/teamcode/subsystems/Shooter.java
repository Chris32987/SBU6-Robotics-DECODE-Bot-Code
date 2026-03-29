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
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").reversed().floatMode(),
            new MotorEx("flywheelMotor2").floatMode()
    );

    private final ServoEx HoodServoRight = new ServoEx("HoodServoRight");
    private final ServoEx HoodServoLeft = new ServoEx("HoodServoLeft");

    private final InterpLUT velocityLUT = new InterpLUT( // SORT THEM SO THAT THE FIRST ARRAY IS IN ORDER AND REARRANGE THE ARRAY POINTS SO THAT THEY MATCH UP
            Arrays.asList(20.5429, 13.3075, 43.4068, 54.2784, 60.5693),
            Arrays.asList(1080.0, 1040.0, 1140.0, 1360.0, 1260.0)
    ).createLUT();

    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(20.5429, 13.3075, 43.4068, 54.2784, 60.5693),
            Arrays.asList(0.64, 0.64, 0.60, 0.54, 0.56)
    ).createLUT();

    private boolean spinFlywheel = false;
    public boolean AtSpeed = false;


    ControlSystem controller = ControlSystem.builder()
            .velPid(0, 0, 0)
            .basicFF(0, 0, 0.001)
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

        flywheelMotors.setPower(!spinFlywheel ? 0 : controller.calculate(flywheelMotors.getState()));

}}
