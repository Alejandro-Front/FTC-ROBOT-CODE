package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Autónomo — Alianza ROJA
 *
 * Secuencia:
 *  1. Espera que el sensor touch confirme posición inicial (robot pegado a la pared)
 *  2. Enciende CoreHex (recolector) durante todo el recorrido
 *  3. Avanza 1 metro en línea recta usando encoders + corrección IMU
 *  4. Gira 20° a la DERECHA usando IMU
 *  5. Apaga CoreHex
 */
@Autonomous(name = "Auto ROJA", group = "Autonomous")
@Disabled
public class AutoAlianzaRoja extends LinearOpMode {

    private DcMotor     leftDrive;
    private DcMotor     rightDrive;
    private DcMotorEx   coreHex;
    private IMU         imu;
    private TouchSensor touchSensor;

    private static final double TICKS_PER_REV    = 1120.0;
    private static final double WHEEL_DIAMETER_M = 0.090;
    private static final double TICKS_PER_METER  =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_M);

    private static final double DRIVE_POWER      = 0.5;
    private static final double TURN_POWER       = 0.4;
    private static final double COREHEX_POWER    = 0.8;

    private static final double TARGET_DISTANCE_M = 1.0;
    // DERECHA = heading negativo en FTC
    private static final double TARGET_ANGLE_DEG  = -20.0;

    private static final double HEADING_KP         = 0.02;
    private static final double TURN_TOLERANCE_DEG = 1.5;

    @Override
    public void runOpMode() {
        initHardware();
        waitForTouchAndStart();
        runSequence();
    }

    private void initHardware() {
        leftDrive   = hardwareMap.get(DcMotor.class,     "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class,     "rightDrive");
        coreHex     = hardwareMap.get(DcMotorEx.class,   "coreHex");
        imu         = hardwareMap.get(IMU.class,          "imu");
        touchSensor = hardwareMap.get(TouchSensor.class,  "touchSensor");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        telemetry.addLine("Auto ROJA — Hardware OK");
        telemetry.addLine("Esperando sensor touch (robot en pared)...");
        telemetry.update();
    }

    private void waitForTouchAndStart() {
        while (!isStarted() && !isStopRequested()) {
            boolean pressed = touchSensor.isPressed();
            telemetry.addLine("=== Auto ROJA — Pre-inicio ===");
            telemetry.addData("Sensor touch", pressed ? "✓ PRESIONADO (listo)" : "✗ NO presionado");
            telemetry.addLine(pressed ? "Esperando señal de inicio..." : "⚠ Pega el robot a la pared");
            telemetry.update();
            sleep(50);
        }
        imu.resetYaw();
        resetEncoders();
    }

    private void runSequence() {
        if (!opModeIsActive()) return;

        coreHex.setPower(COREHEX_POWER);
        telemetry.addLine("Recolector encendido");
        telemetry.update();

        driveDistance(TARGET_DISTANCE_M, DRIVE_POWER);
        turnToHeading(TARGET_ANGLE_DEG, TURN_POWER);   // -20° = derecha

        coreHex.setPower(0);
        stopDrive();

        telemetry.addLine("✓ Secuencia ROJA completa");
        telemetry.update();

        while (opModeIsActive()) { sleep(100); }
    }

    private void driveDistance(double meters, double power) {
        int targetTicks = (int) (meters * TICKS_PER_METER);
        resetEncoders();

        while (opModeIsActive()) {
            int leftTicks  = Math.abs(leftDrive.getCurrentPosition());
            int rightTicks = Math.abs(rightDrive.getCurrentPosition());
            int avgTicks   = (leftTicks + rightTicks) / 2;

            if (avgTicks >= targetTicks) break;

            double heading    = getHeadingDeg();
            double correction = heading * HEADING_KP;

            double leftPower  = power - correction;
            double rightPower = power + correction;

            double remaining = 1.0 - ((double) avgTicks / targetTicks);
            if (remaining < 0.20) {
                double scale = Math.max(0.25, remaining / 0.20);
                leftPower  *= scale;
                rightPower *= scale;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Avance", "%d / %d ticks  (%.2f m)",
                    avgTicks, targetTicks, avgTicks / TICKS_PER_METER);
            telemetry.addData("Heading", "%.1f°", heading);
            telemetry.update();
        }
        stopDrive();
        sleep(200);
    }

    private void turnToHeading(double targetDeg, double power) {
        while (opModeIsActive()) {
            double error = targetDeg - getHeadingDeg();

            while (error >  180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) <= TURN_TOLERANCE_DEG) break;

            double turnSpeed = Math.max(power * 0.3,
                    Math.min(power, Math.abs(error) / 20.0 * power));
            double sign = Math.signum(error);

            leftDrive.setPower(-sign * turnSpeed);
            rightDrive.setPower( sign * turnSpeed);

            telemetry.addData("Giro ROJA (der)", "error %.1f°  speed %.2f", error, turnSpeed);
            telemetry.update();
        }
        stopDrive();
        sleep(200);
    }

    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}