package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TestMotores — Prueba de 3 motores
 *
 * Controles:
 * ┌──────────────────┬─────────────────────────────────────────┐
 * │ R2 (right trigger)│ Motor derecho (rightDrive)             │
 * │ L2 (left trigger) │ Motor izquierdo (leftDrive)            │
 * │ Y / Triangle      │ CoreHex hacia adelante                 │
 * │ A / Cross         │ CoreHex hacia atrás                    │
 * └──────────────────┴─────────────────────────────────────────┘
 */
@TeleOp(name = "TestMotores", group = "TeleOp")
public class TestMotores extends LinearOpMode {

    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private DcMotorEx coreHex;

    private static final double COREHEX_POWER = 0.8;

    @Override
    public void runOpMode() {
        // Init hardware
        leftDrive  = hardwareMap.get(DcMotor.class,   "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,   "rightDrive");
        coreHex    = hardwareMap.get(DcMotorEx.class, "coreHex");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);

        telemetry.addLine("TestMotores listo — presiona Play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Motores directos con gatillos ---
            double rightPower = gamepad1.right_trigger; // R2
            double leftPower  = gamepad1.left_trigger;  // L2

            rightDrive.setPower(rightPower);
            leftDrive.setPower(-leftPower);

            // --- CoreHex con letras ---
            if (gamepad1.triangle) {           // Y en Xbox / Triangle en PS
                coreHex.setPower(COREHEX_POWER);
            } else if (gamepad1.cross) {       // A en Xbox / Cross en PS
                coreHex.setPower(-COREHEX_POWER);
            } else {
                coreHex.setPower(0);
            }

            // --- Telemetría ---
            telemetry.addLine("=== TestMotores ===");
            telemetry.addData("rightDrive (R2)", "%.2f", rightPower);
            telemetry.addData("leftDrive  (L2)", "%.2f", leftPower);
            telemetry.addData("CoreHex",
                    gamepad1.triangle ? "ADELANTE (Y)" :
                            gamepad1.cross    ? "ATRÁS (A)"    : "DETENIDO");
            telemetry.update();
        }
    }
}