package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**
 * StarterBot TeleOp — revV4
 *
 * Mapa de controles:
 * ┌──────────────────┬─────────────────────────────────────────────────┐
 * │ Left stick Y     │ Avanzar / retroceder                            │
 * │ Right stick X    │ Girar                                           │
 * │ L1               │ Velocidad LENTA  (0.4×)                         │
 * │ R1               │ Velocidad TURBO  (1.0×)                         │
 * │ sin bumper       │ Velocidad NORMAL (0.7×)                         │
 * ├──────────────────┼─────────────────────────────────────────────────┤
 * │ R2 (presionado)  │ Flywheel sube gradual hasta POWER_TARGET        │
 * │ R2 (soltado)     │ Flywheel baja gradual hasta 0                   │
 * │ L2 (presionado)  │ CoreHex gira (alimenta pelotas)                 │
 * │ L2 (soltado)     │ CoreHex para                                    │
 * │ Square           │ Flywheel potencia máxima fija                   │
 * │ Options          │ Flywheel reversa (limpiar atascos)              │
 * ├──────────────────┼─────────────────────────────────────────────────┤
 * │ Triangle toggle  │ Servo acomodador: gira / para                   │
 * └──────────────────┴─────────────────────────────────────────────────┘
 *
 * Variables configurables clave:
 *   FLYWHEEL_RAMP_RATE  → qué tan gradual sube/baja el flywheel
 *   POWER_TARGET        → potencia objetivo del flywheel con R2
 *   COREHEX_POWER       → potencia de giro del CoreHex
 *   SERVO_RUN           → dirección del servo (0.7 normal, 0.3 si gira al revés)
 */
@TeleOp(name = "v4", group = "TeleOp")
public class revV4 extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor   flywheel;
    private DcMotorEx coreHex;
    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private CRServo gateServo;

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
    // Flywheel
    // -------------------------------------------------------------------------
    /** Potencia objetivo al mantener R2 presionado */
    private double powerTarget = 0.85;

    /** Potencia máxima (square) */
    private static final double POWER_MAX = 1.00;

    /**
     * Gradualidad de subida/bajada del flywheel por ciclo (~20 ms).
     *   0.005 → muy gradual (~3.4 s)
     *   0.01  → moderado   (~1.7 s)  ← actual
     *   0.02  → rápido     (~0.85 s)
     */
    private static final double FLYWHEEL_RAMP_RATE = 0.01;

    private double flywheelPower = 0.0;

    // -------------------------------------------------------------------------
    // CoreHex — giro continuo controlado por L2
    // -------------------------------------------------------------------------
    /** Potencia de giro continuo del CoreHex al alimentar */
    private static final double COREHEX_POWER = 0.8;

    // -------------------------------------------------------------------------
    // Servo acomodador (Servo normal 0.0–1.0)
    //   0.5 = detenido
    //   0.7 = gira hacia flywheel  (cambiar a 0.3 si gira al revés)
    // -------------------------------------------------------------------------
    private static final double SERVO_STOP = 0.0;
    private static final double SERVO_RUN  = 1.0; // prueba -1.0 si gira al revés

    private boolean servoToggleOn  = false;
    private boolean prevTriangle   = false;

    private boolean prevDpadDown = false;

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
        gateServo = hardwareMap.get(CRServo.class, "servo");

        // Flywheel
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Drivetrain
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CoreHex — giro continuo
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);

        // Servo — detenido al inicio
        gateServo.setPower(SERVO_STOP);

        telemetry.addLine("revV4 — Hardware inicializado, listo");
        telemetry.update();
    }

    // =========================================================================
    // TELEOP — Loop principal
    // =========================================================================
    private void doTeleOp() {
        while (opModeIsActive()) {
            arcadeDrive();
            controlFlywheel();
            controlCoreHex();
            setPowerObjet();
            controlServo();
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
    // FLYWHEEL — Rampa gradual con R2
    //   R2 presionado → sube hasta POWER_TARGET
    //   R2 soltado    → baja hasta 0
    //   square        → POWER_MAX fijo
    //   options       → reversa para atascos
    // =========================================================================
    private void controlFlywheel() {
        if (gamepad1.options) {
            flywheelPower = -0.5;
        } else if (gamepad1.square) {
            flywheelPower = POWER_MAX;
        } else if (gamepad1.right_trigger > 0.5) {
            flywheelPower = Math.min(flywheelPower + FLYWHEEL_RAMP_RATE, powerTarget);
        } else {
            flywheelPower = Math.max(flywheelPower - FLYWHEEL_RAMP_RATE, 0.0);
        }

        flywheel.setPower(flywheelPower);
    }
    //=========================================================================
    // SETPOWER

    private boolean prevDpadUp = false;

    private void setPowerObjet() {
        boolean down = gamepad1.dpad_down;
        boolean up   = gamepad1.dpad_up;

        if (down && !prevDpadDown) {
            powerTarget -= 0.05;
        }

        if (up && !prevDpadUp) {
            powerTarget += 0.05;
        }

        powerTarget = Math.max(0, Math.min(1.0, powerTarget));

        prevDpadDown = down;
        prevDpadUp   = up;
    }


    // =========================================================================
    // COREHEX — Momentáneo con L2
    //   L2 presionado → gira a COREHEX_POWER
    //   L2 soltado    → para
    // =========================================================================
    private void controlCoreHex() {
        if (gamepad1.left_trigger > 0.5) {
            coreHex.setPower(COREHEX_POWER);
        } else {
            coreHex.setPower(0);
        }
    }

    // =========================================================================
    // SERVO ACOMODADOR — Toggle con Triangle
    //   Primer press → gira (SERVO_RUN)
    //   Segundo press → para (SERVO_STOP)
    // =========================================================================
    private void controlServo() {
        if (gamepad1.triangle) {
            gateServo.setPower(1.0);
            servoToggleOn = true;
        } else if (gamepad1.cross) {
            gateServo.setPower(-1.0);
            servoToggleOn = true;
        } else {
            servoToggleOn = false;
            gateServo.setPower(0.0);
        }
    }

    // =========================================================================
    // TELEMETRÍA
    // =========================================================================
    private void updateTelemetry() {
        telemetry.addLine("=== revV4 ===");

        String speed = gamepad1.left_bumper  ? "LENTA  (L1)" :
                gamepad1.right_bumper ? "TURBO  (R1)" : "NORMAL";
        telemetry.addData("Velocidad",        speed);
        telemetry.addData("Drive L / R",      "%.2f / %.2f", lastLeftPower, lastRightPower);

        telemetry.addLine("---");
        telemetry.addData("Flywheel",         "%.3f / %.3f objetivo", flywheelPower, powerTarget);
        telemetry.addData("CoreHex",          gamepad1.left_trigger > 0.5 ? "GIRANDO (L2)" : "DETENIDO");
        telemetry.addData("Servo acomodador", servoToggleOn ? "GIRANDO" : "DETENIDO");

        telemetry.update();
    }
}