package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Flywheel {

    private static final double FLYWHEEL_POWER_MAX  = 1.0;
    private static final double FLYWHEEL_POWER_MIN  = 0.1;
    private static final double FLYWHEEL_POWER_STEP = 0.05;

    private final DcMotor       flywheel;
    private final Gamepad       gamepad;
    private final ControlProfile profile;

    private double  flywheelPower = 0.65;
    private boolean flywheelOn    = false;

    // Edge-detection compartido
    private boolean prevCross    = false;
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;

    public Flywheel(HardwareMap hardwareMap, Gamepad gamepad, ControlProfile profile) {
        this.flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        this.gamepad  = gamepad;
        this.profile  = profile;
    }

    public void init() {
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);
    }

    public void update() {
        switch (profile) {
            case DRIVER_A:
                updateDriverA();
                break;
            case DRIVER_B:
                updateDriverB();
                break;
        }
        updatePowerAdjust(); // dpad ↑↓ igual para ambos
    }

    /** X = toggle ON/OFF */
    private void updateDriverA() {
        boolean currentCross = gamepad.cross;
        if (currentCross && !prevCross) {
            flywheelOn = !flywheelOn;
            flywheel.setPower(flywheelOn ? flywheelPower : 0.0);
        }
        prevCross = currentCross;
    }

    /** L2 (mantener) = ON  |  soltar = OFF */
    private void updateDriverB() {
        boolean shouldRun = gamepad.left_trigger > 0.5;
        if (shouldRun != flywheelOn) {          // solo actúa cuando cambia
            flywheelOn = shouldRun;
            flywheel.setPower(flywheelOn ? flywheelPower : 0.0);
        }
        prevCross = gamepad.cross;              // sin uso, pero consistente
    }

    private void updatePowerAdjust() {
        boolean currentUp   = gamepad.dpad_up;
        boolean currentDown = gamepad.dpad_down;

        if (currentUp && !prevDpadUp) {
            flywheelPower = Math.min(flywheelPower + FLYWHEEL_POWER_STEP, FLYWHEEL_POWER_MAX);
            flywheelPower = Math.round(flywheelPower * 100.0) / 100.0;
            if (flywheelOn) flywheel.setPower(flywheelPower);
        }
        if (currentDown && !prevDpadDown) {
            flywheelPower = Math.max(flywheelPower - FLYWHEEL_POWER_STEP, FLYWHEEL_POWER_MIN);
            flywheelPower = Math.round(flywheelPower * 100.0) / 100.0;
            if (flywheelOn) flywheel.setPower(flywheelPower);
        }
        prevDpadUp   = currentUp;
        prevDpadDown = currentDown;
    }

    public boolean isOn()    { return flywheelOn; }
    public double getPower() { return flywheelPower; }
}