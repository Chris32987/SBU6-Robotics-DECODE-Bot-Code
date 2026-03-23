package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.Arrays;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("flywheelMotor1").floatMode(),
            new MotorEx("flywheelMotor2").reversed().floatMode()
    );
    private final InterpLUT velocityLUT = new InterpLUT(
            Arrays.asList(40, 50, 55, 70, 65, 70, 80, 90),
            Arrays.asList(1250.0, 1250.0, 1350.0, 1400.0, 1450.0, 1450.0, 1470.0, 1500.0)
    ).createLUT();

    private final InterpLUT hoodLUT = new InterpLUT(
            Arrays.asList(40, 50, 55, 70, 65, 70, 80, 90),
            Arrays.asList(0.32, 0.30, 0.24, 0.20, 0.34, 0.20, 0.26, 0.18)
    ).createLUT();

}