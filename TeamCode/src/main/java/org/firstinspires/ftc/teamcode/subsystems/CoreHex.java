package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CoreHex {

    private static final double COREHEX_POWER = 0.8;

    private final DcMotorEx coreHex;
    private final Gamepad gamepad;

    public CoreHex(HardwareMap hardwareMap, Gamepad gamepad) {
        this.coreHex = hardwareMap.get(DcMotorEx.class, "coreHex");
        this.gamepad = gamepad;
    }

    public void init() {
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        coreHex.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);
    }

    public void update() {
        if (gamepad.right_trigger > 0.5) {
            coreHex.setPower(COREHEX_POWER);
        } else if (gamepad.left_trigger > 0.5) {
            coreHex.setPower(-COREHEX_POWER);
        } else {
            coreHex.setPower(0);
        }
    }

    public void setPower(double power) {
        coreHex.setPower(power);
    }

    public boolean isRunningForward() { return gamepad.right_trigger > 0.5; }
    public boolean isRunningReverse() { return gamepad.left_trigger > 0.5; }
}
