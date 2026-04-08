package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    private final MotorEx IntakeMotor = new MotorEx("IntakeMotor").brakeMode();
    private final ServoEx GateServo = new ServoEx("GateServo");

    public Command intakeSpin = new SetPower(IntakeMotor, 1).requires(IntakeMotor);
    public Command intakeOff = new SetPower(IntakeMotor, 0.2).requires(IntakeMotor);
    public Command intakeReverse = new SetPower(IntakeMotor, -1).requires(IntakeMotor);
    public Command GateOpen = new SetPosition(GateServo, 0.65);
    public Command GateClose = new SetPosition(GateServo, 1);

    private DigitalChannel LaserLeft;
    private DigitalChannel LaserRight;
    private GoBildaPrismDriver leds;

    public static double INITIAL_WAIT_TIME = 0.3;
    private static final double BLINK_DURATION_MS = 2000;

    private boolean BallPresent = false;
    private boolean CurrentlyTiming = false;
    private boolean animationPlayed = false;
    private boolean solidSet = false;
    private final ElapsedTime Timer = new ElapsedTime();
    private final ElapsedTime blinkTimer = new ElapsedTime();

    public boolean HasThreeBalls = false;

    @Override
    public void initialize() {
        LaserLeft = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LaserLeft");
        LaserRight = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LaserRight");
        LaserLeft.setMode(DigitalChannel.Mode.INPUT);
        LaserRight.setMode(DigitalChannel.Mode.INPUT);
        leds = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "leds");
        leds.clearAllAnimations();
    }

    @Override
    public void periodic() {
        BallPresent = LaserLeft.getState() || LaserRight.getState();

        if (BallPresent) {
            if (CurrentlyTiming) {
                HasThreeBalls = Timer.seconds() >= INITIAL_WAIT_TIME;
            } else {
                Timer.reset();
                CurrentlyTiming = true;
            }
        } else {
            CurrentlyTiming = false;
            HasThreeBalls = false;
        }

        if (HasThreeBalls) {
            if (!animationPlayed) {
                leds.clearAllAnimations();
                leds.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Blink(Color.GREEN, Color.TRANSPARENT, 1000, 500)
                );
                blinkTimer.reset();
                animationPlayed = true;
            } else if (!solidSet && blinkTimer.milliseconds() > BLINK_DURATION_MS) {
                leds.clearAllAnimations();
                leds.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(Color.GREEN)
                );
                solidSet = true;
            }
        } else {
            if (animationPlayed) {
                leds.clearAllAnimations();
                animationPlayed = false;
                solidSet = false;
            }
        }

        ActiveOpMode.telemetry().addData("BallPresent", BallPresent);
        ActiveOpMode.telemetry().addData("HasThreeBalls", HasThreeBalls);
        ActiveOpMode.telemetry().addData("Timer", Timer.seconds());
    }
}