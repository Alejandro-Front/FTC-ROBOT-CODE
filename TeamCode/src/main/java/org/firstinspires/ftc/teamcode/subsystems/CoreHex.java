package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CoreHex {

    private static final double COREHEX_POWER = 0.8;

    private final DcMotorEx     coreHex;
    private final Gamepad       gamepad;
    private final ControlProfile profile;

    private boolean prevCross = false;

    /** Constructor TeleOp — con gamepad y perfil */
    public CoreHex(HardwareMap hardwareMap, Gamepad gamepad, ControlProfile profile) {
        this.coreHex = hardwareMap.get(DcMotorEx.class, "coreHex");
        this.gamepad = gamepad;
        this.profile = profile;
    }

    /** Constructor Autónomo — sin gamepad */
    public CoreHex(HardwareMap hardwareMap) {
        this.coreHex = hardwareMap.get(DcMotorEx.class, "coreHex");
        this.gamepad = null;
        this.profile = null;
    }

    public void init() {
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        coreHex.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);
    }

    public void update() {
        if (gamepad == null) return;
        switch (profile) {
            case DRIVER_A: updateDriverA(); break;
            case DRIVER_B: updateDriverB(); break;
        }
    }

    private void updateDriverA() {
        if (gamepad.right_trigger > 0.5) {
            coreHex.setPower(COREHEX_POWER);
        } else if (gamepad.left_trigger > 0.5) {
            coreHex.setPower(-COREHEX_POWER);
        } else {
            coreHex.setPower(0);
        }
        prevCross = gamepad.cross;
    }

    private void updateDriverB() {
        if (gamepad.cross) {
            coreHex.setPower(COREHEX_POWER);
        } else {
            coreHex.setPower(0);
        }
        prevCross = gamepad.cross;
    }

    public void setPower(double power) { coreHex.setPower(power); }

    public boolean isRunningForward() {
        if (gamepad == null) return false;  // ← fix autónomo
        return profile == ControlProfile.DRIVER_A
                ? gamepad.right_trigger > 0.5
                : gamepad.cross;
    }

    public boolean isRunningReverse() {
        if (gamepad == null) return false;  // ← fix autónomo
        return profile == ControlProfile.DRIVER_A
                && gamepad.left_trigger > 0.5;
    }
}