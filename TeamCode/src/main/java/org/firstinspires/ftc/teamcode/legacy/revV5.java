package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * StarterBot TeleOp — revV5
 *
 * Mapa de controles:
 * ┌──────────────────┬─────────────────────────────────────────────────┐
 * │ Left stick Y     │ Avanzar / retroceder                            │
 * │ Right stick X    │ Girar                                           │
 * │ L1               │ Velocidad LENTA  (0.4×)                         │
 * │ R1               │ Velocidad TURBO  (1.0×)                         │
 * │ sin bumper       │ Velocidad NORMAL (0.7×)                         │
 * ├──────────────────┼─────────────────────────────────────────────────┤
 * │ R2 (mantener)    │ CoreHex gira normal (alimentador)               │
 * │ L2 (mantener)    │ CoreHex reversa (desatorar)                     │
 * │ R2 + L2          │ R2 tiene prioridad                              │
 * │ soltados         │ CoreHex se detiene                              │
 * ├──────────────────┼─────────────────────────────────────────────────┤
 * │ A/Cross (toggle) │ Disparador: 1er press enciende / 2do apaga      │
 * └──────────────────┴─────────────────────────────────────────────────┘
 */
@TeleOp(name = "v5", group = "TeleOp")
@Disabled
public class revV5 extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor   flywheel;
    private DcMotorEx coreHex;
    private DcMotor   leftDrive;
    private DcMotor   rightDrive;

    // -------------------------------------------------------------------------
    // Conducción
    // -------------------------------------------------------------------------
    private static final double SPEED_SLOW   = 0.4;
    private static final double SPEED_NORMAL = 0.7;
    private static final double SPEED_TURBO  = 1.0;
    private static final double DRIVE_KP     = 0.002;
    private static final double SLEW_RATE    = 0.05;

    private int    lastLeftEncoder  = 0;
    private int    lastRightEncoder = 0;
    private double driveCorrection  = 0.0;
    private double lastLeftPower    = 0.0;
    private double lastRightPower   = 0.0;

    // -------------------------------------------------------------------------
    // CoreHex
    // -------------------------------------------------------------------------
    private static final double COREHEX_POWER = 0.8;

    // -------------------------------------------------------------------------
    // Disparador (flywheel) — toggle con A/Cross
    // -------------------------------------------------------------------------
    private static final double FLYWHEEL_POWER = 1.0;
    private boolean flywheelOn = false;
    private boolean prevCross  = false;

    // =========================================================================
    // PUNTO DE ENTRADA
    // =========================================================================
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        doTeleOp();
    }

    // =========================================================================
    // INICIALIZACIÓN
    // =========================================================================
    private void initHardware() {
        flywheel   = hardwareMap.get(DcMotor.class,   "flywheel");
        coreHex    = hardwareMap.get(DcMotorEx.class, "coreHex");
        leftDrive  = hardwareMap.get(DcMotor.class,   "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,   "rightDrive");

        // Disparador — FORWARD, flota al parar
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        // Drivetrain
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CoreHex
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);

        telemetry.addLine("revV5 — Hardware inicializado, listo");
        telemetry.update();
    }

    // =========================================================================
    // TELEOP — Loop principal
    // =========================================================================
    private void doTeleOp() {
        while (opModeIsActive()) {
            arcadeDrive();
            controlCoreHex();
            controlFlywheel();
            updateTelemetry();
        }
    }

    // =========================================================================
    // CONDUCCIÓN — Arcade Drive
    // L1 = lento | R1 = turbo | sin bumper = normal
    // =========================================================================
    private void arcadeDrive() {
        double speedMultiplier;
        if      (gamepad1.left_bumper)  speedMultiplier = SPEED_SLOW;
        else if (gamepad1.right_bumper) speedMultiplier = SPEED_TURBO;
        else                            speedMultiplier = SPEED_NORMAL;

        double forward = gamepad1.left_stick_y;
        double turn    = gamepad1.right_stick_x;

        // Corrección de encoders en línea recta
        if (Math.abs(turn) < 0.05) {
            int leftEnc  = leftDrive.getCurrentPosition();
            int rightEnc = rightDrive.getCurrentPosition();

            driveCorrection  = (leftEnc - lastLeftEncoder - (rightEnc - lastRightEncoder)) * DRIVE_KP;
            lastLeftEncoder  = leftEnc;
            lastRightEncoder = rightEnc;
        } else {
            driveCorrection = 0.0;
        }

        double leftPower  = forward - turn + driveCorrection;
        double rightPower = forward + turn - driveCorrection;

        // Normalización
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

    // =========================================================================
    // COREHEX
    //   R2 presionado          → gira normal   (prioridad)
    //   L2 presionado (sin R2) → gira en reversa (desatorar)
    //   ambos soltados         → para
    // =========================================================================
    private void controlCoreHex() {
        if (gamepad1.right_trigger > 0.5) {
            coreHex.setPower(COREHEX_POWER);          // normal — R2 tiene prioridad
        } else if (gamepad1.left_trigger > 0.5) {
            coreHex.setPower(-COREHEX_POWER);         // reversa — desatorar
        } else {
            coreHex.setPower(0);
        }
    }

    // =========================================================================
    // DISPARADOR — Toggle con A/Cross
    //   1er press → enciende a potencia máxima
    //   2do press → apaga
    // =========================================================================
    private void controlFlywheel() {
        boolean currentCross = gamepad1.cross;

        if (currentCross && !prevCross) {
            flywheelOn = !flywheelOn;
            flywheel.setPower(flywheelOn ? FLYWHEEL_POWER : 0.0);
        }

        prevCross = currentCross;
    }

    // =========================================================================
    // TELEMETRÍA
    // =========================================================================
    private void updateTelemetry() {
        telemetry.addLine("=== revV5 ===");

        String speed = gamepad1.left_bumper  ? "LENTA  (L1)" :
                gamepad1.right_bumper ? "TURBO  (R1)" : "NORMAL";
        telemetry.addData("Velocidad",   speed);
        telemetry.addData("Drive L / R", "%.2f / %.2f", lastLeftPower, lastRightPower);

        telemetry.addLine("---");
        String coreHexStatus;
        if (gamepad1.right_trigger > 0.5)     coreHexStatus = "NORMAL (R2)";
        else if (gamepad1.left_trigger > 0.5) coreHexStatus = "REVERSA — desatorando (L2)";
        else                                  coreHexStatus = "DETENIDO";
        telemetry.addData("CoreHex",    coreHexStatus);
        telemetry.addData("Disparador", flywheelOn ? "ENCENDIDO (A)" : "APAGADO");

        telemetry.update();
    }
}