package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Flywheel {

    private static final double FLYWHEEL_POWER_MAX  = 1.0;
    private static final double FLYWHEEL_POWER_MIN  = 0.1;
    private static final double FLYWHEEL_POWER_STEP = 0.05;

    private final DcMotor flywheel;
    private final Gamepad gamepad;

    private double  flywheelPower = 0.75;
    private boolean flywheelOn    = false;
    private boolean prevCross     = false;
    private boolean prevDpadUp    = false;
    private boolean prevDpadDown  = false;

    public Flywheel(HardwareMap hardwareMap, Gamepad gamepad) {
        this.flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        this.gamepad = gamepad;
    }

    public void init() {
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);
    }

    public void update() {
        boolean currentCross    = gamepad.cross;
        boolean currentDpadUp   = gamepad.dpad_up;
        boolean currentDpadDown = gamepad.dpad_down;

        if (currentCross && !prevCross) {
            flywheelOn = !flywheelOn;
            flywheel.setPower(flywheelOn ? flywheelPower : 0.0);
        }

        if (currentDpadUp && !prevDpadUp) {
            flywheelPower = Math.min(flywheelPower + FLYWHEEL_POWER_STEP, FLYWHEEL_POWER_MAX);
            flywheelPower = Math.round(flywheelPower * 100.0) / 100.0;
            if (flywheelOn) flywheel.setPower(flywheelPower);
        }

        if (currentDpadDown && !prevDpadDown) {
            flywheelPower = Math.max(flywheelPower - FLYWHEEL_POWER_STEP, FLYWHEEL_POWER_MIN);
            flywheelPower = Math.round(flywheelPower * 100.0) / 100.0;
            if (flywheelOn) flywheel.setPower(flywheelPower);
        }

        prevCross    = currentCross;
        prevDpadUp   = currentDpadUp;
        prevDpadDown = currentDpadDown;
    }

    public boolean isOn()    { return flywheelOn; }
    public double getPower() { return flywheelPower; }
}
