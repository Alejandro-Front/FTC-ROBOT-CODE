package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private static final double SPEED_SLOW   = 0.4;
    private static final double SPEED_NORMAL = 0.7;
    private static final double SPEED_TURBO  = 1.0;
    private static final double DRIVE_KP     = 0.002;
    private static final double SLEW_RATE    = 0.05;
    private static final double TURN_DEADBAND = 0.05;

    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    private int    lastLeftEncoder  = 0;
    private int    lastRightEncoder = 0;
    private double driveCorrection  = 0.0;
    private double lastLeftPower    = 0.0;
    private double lastRightPower   = 0.0;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        this.rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void init() {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        double speedMultiplier;
        if      (gamepad.left_bumper)  speedMultiplier = SPEED_SLOW;
        else if (gamepad.right_bumper) speedMultiplier = SPEED_TURBO;
        else                           speedMultiplier = SPEED_NORMAL;

        double forward = gamepad.left_stick_y;
        double turn    = gamepad.right_stick_x;

        if (Math.abs(turn) < TURN_DEADBAND) {
            int leftEnc  = leftDrive.getCurrentPosition();
            int rightEnc = rightDrive.getCurrentPosition();

            driveCorrection = (leftEnc - lastLeftEncoder - (rightEnc - lastRightEncoder)) * DRIVE_KP;
            lastLeftEncoder  = leftEnc;
            lastRightEncoder = rightEnc;
        } else {
            driveCorrection = 0.0;
        }

        double leftPower  = forward - turn + driveCorrection;
        double rightPower = forward + turn - driveCorrection;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower  /= maxPower;
            rightPower /= maxPower;
        }

        leftPower  = applySlewRate(leftPower  * speedMultiplier, lastLeftPower);
        rightPower = applySlewRate(rightPower * speedMultiplier, lastRightPower);

        lastLeftPower  = leftPower;
        lastRightPower = rightPower;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private double applySlewRate(double target, double current) {
        double delta = Math.max(-SLEW_RATE, Math.min(SLEW_RATE, target - current));
        return current + delta;
    }

    public void setPower(double left, double right) {
        lastLeftPower  = left;
        lastRightPower = right;
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public double getLastLeftPower()  { return lastLeftPower; }
    public double getLastRightPower() { return lastRightPower; }
}
