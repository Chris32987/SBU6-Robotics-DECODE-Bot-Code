package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    private final MotorEx IntakeMotor = new MotorEx("IntakeMotor").brakeMode();
    private final ServoEx GateServo = new ServoEx("GateServo");
    public Command intakeSpin = new SetPower(IntakeMotor,1).requires(IntakeMotor);
    public Command intakeOff = new SetPower(IntakeMotor,0).requires(IntakeMotor);
    public Command GateOpen = new SetPosition(GateServo,0.65);
    public Command GateClose = new SetPosition(GateServo,1);
}
