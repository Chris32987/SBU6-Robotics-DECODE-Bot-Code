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
import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    private final MotorEx IntakeMotor = new MotorEx("IntakeMotor").brakeMode();
    private final ServoEx GateServo = new ServoEx("GateServo");

    public Command intakeSpin    = new SetPower(IntakeMotor, 1).requires(IntakeMotor);
    public Command intakeOff     = new SetPower(IntakeMotor, 0.2).requires(IntakeMotor);
    public Command intakeReverse = new SetPower(IntakeMotor, -1).requires(IntakeMotor);
    public Command GateOpen      = new SetPosition(GateServo, 0.65);
    public Command GateClose     = new SetPosition(GateServo, 1);

    private DigitalChannel LaserLeft;
    private DigitalChannel LaserRight;
    private GoBildaPrismDriver leds;

    public static double INITIAL_WAIT_TIME = 0.3;

    private static final int    BLINK_PERIOD_MS = 400;
    private static final int    BLINK_ON_MS     = 200;
    private static final double TWO_BLINKS_MS   = 800;

    private enum LedState {
        OFF,
        TWO_FLASHES,
        SOLID
    }

    private boolean BallPresent     = false;
    private boolean CurrentlyTiming = false;
    private LedState ledState       = LedState.OFF;
    private final ElapsedTime Timer      = new ElapsedTime();
    private final ElapsedTime blinkTimer = new ElapsedTime();

    public boolean HasThreeBalls = false;

    private Color getAllianceColor() {
        return Poses.CurrentAlliance == Poses.AllianceColor.BLUE ? Color.BLUE : Color.GREEN;
    }

    @Override
    public void initialize() {
        LaserLeft  = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LaserLeft");
        LaserRight = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LaserRight");
        LaserLeft.setMode(DigitalChannel.Mode.INPUT);
        LaserRight.setMode(DigitalChannel.Mode.INPUT);
        leds = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "leds");
        leds.clearAllAnimations();
    }

    private void setLedState(LedState newState) {
        if (newState == ledState) return;
        ledState = newState;
        leds.clearAllAnimations();
        blinkTimer.reset();
        switch (ledState) {
            case TWO_FLASHES:
                leds.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Blink(getAllianceColor(), Color.TRANSPARENT, BLINK_PERIOD_MS, BLINK_ON_MS)
                );
                break;
            case SOLID:
                leds.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(getAllianceColor())
                );
                break;
            case OFF:
                break;
        }
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
            HasThreeBalls   = false;
        }

        switch (ledState) {
            case OFF:
                if (HasThreeBalls) setLedState(LedState.TWO_FLASHES);
                break;
            case TWO_FLASHES:
                if (!HasThreeBalls)                                  setLedState(LedState.OFF);
                else if (blinkTimer.milliseconds() > TWO_BLINKS_MS) setLedState(LedState.SOLID);
                break;
            case SOLID:
                if (!HasThreeBalls) setLedState(LedState.OFF);
                break;
        }

        ActiveOpMode.telemetry().addData("BallPresent",  BallPresent);
        ActiveOpMode.telemetry().addData("HasThreeBalls", HasThreeBalls);
        ActiveOpMode.telemetry().addData("LedState",     ledState);
        ActiveOpMode.telemetry().addData("Timer",        Timer.seconds());
    }
}