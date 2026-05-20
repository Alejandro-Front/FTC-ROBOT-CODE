package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * StarterBot TeleOp — rev
 *
 * Mapa de controles completo:
 * ┌─────────────────┬──────────────────────────────────────────┐
 * │ Left stick Y    │ Avanzar / retroceder                     │
 * │ Right stick X   │ Girar                                    │
 * │ L1 (left_bumper)│ Velocidad LENTA (0.4×)                   │
 * │ R1 (right_bump) │ Velocidad TURBO (1.0×)                   │
 * │ Sin bumper      │ Velocidad NORMAL (0.7×)                  │
 * ├─────────────────┼──────────────────────────────────────────┤
 * │ R2 (right_trig) │ Flywheel ON — sube gradual a POWER_TARGET│
 * │   (soltado)     │ Flywheel baja gradual hasta 0            │
 * │ left_bumper*    │ Flywheel potencia lejana fija (POWER_FAR)│
 * │ square          │ Flywheel potencia máxima fija (POWER_MAX)│
 * │ options         │ Flywheel reversa (limpiar atascos)       │
 * ├─────────────────┼──────────────────────────,e────────────────┤
 * │ L2 toggle       │ Servo acomodador: gira / para            │
 * ├─────────────────┼──────────────────────────────────────────┤
 * │ dpad arriba     │ CoreHex un paso adelante (empuja pelota) │
 * │ dpad abajo      │ CoreHex un paso atrás                    │
 * └─────────────────┴──────────────────────────────────────────┘
 *
 * * left_bumper es L1 para velocidad Y para flywheel lejano.
 *   En conducción normal (sin R2 activo) no hay conflicto.
 *   Si se vuelve problema, reasignar flywheel lejano a triangle.
 *
 * Flywheel rampa gradual:
 *   Ajusta FLYWHEEL_RAMP_RATE para controlar velocidad de subida/bajada:
 *     0.005 → muy gradual (~3.4 s)
 *     0.01  → moderado   (~1.7 s)   ← valor actual
 *     0.02  → rápido     (~0.85 s)
 *
 * Servo acomodador (Servo normal 0.0–1.0):
 *   SERVO_STOP = 0.5 (detenido)
 *   SERVO_RUN  = 0.7 (gira hacia flywheel)
 *   Si gira al revés, cambiar SERVO_RUN a 0.3
 */
@Autonomous(name = "Auto")
@Disabled
public class autonomo extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor   flywheel;
    private DcMotorEx coreHex;
    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private Servo     gateServo;

    // -------------------------------------------------------------------------
    // Conducción — velocidades
    // -------------------------------------------------------------------------
    private static final double SPEED_SLOW   = 0.4;   // L1 presionado
    private static final double SPEED_NORMAL = 0.7;   // sin bumper
    private static final double SPEED_TURBO  = 1.0;   // R1 presionado

    private static final double DRIVE_KP  = 0.002;    // Corrección de encoder
    private static final double SLEW_RATE = 0.05;     // Suavizado de aceleración

    private int    lastLeftEncoder  = 0;
    private int    lastRightEncoder = 0;
    private double driveCorrection  = 0.0;
    private double lastLeftPower    = 0.0;
    private double lastRightPower   = 0.0;

    // -------------------------------------------------------------------------
    // Flywheel — potencias y rampa
    // -------------------------------------------------------------------------
    /** Potencia objetivo al mantener R2 */
    private static final double POWER_TARGET = 0.85;

    /** Potencia fija para tiro lejano */
    private static final double POWER_FAR    = 0.87;

    /** Potencia máxima */
    private static final double POWER_MAX    = 1.00;

    /**
     * Incremento de potencia por ciclo (~20 ms).
     * Rango recomendado: 0.005 – 0.02
     */
    private static final double FLYWHEEL_RAMP_RATE = 0.01;

    private double flywheelPower = 0.0;

    // -------------------------------------------------------------------------
    // Servo acomodador
    // -------------------------------------------------------------------------
    private static final double SERVO_STOP = 0.5;
    private static final double SERVO_RUN  = 0.7;   // Cambiar a 0.3 si gira al revés

    private boolean servoToggleOn = false;
    private boolean prevL2        = false;

    // -------------------------------------------------------------------------
    // CoreHex — empuje de pelotas por pasos
    // -------------------------------------------------------------------------
    private static final int    RAMP_STEP_TICKS = 50;
    private static final int    RAMP_MAX_TICKS  =  500;
    private static final int    RAMP_MIN_TICKS  = -500;
    private static final double RAMP_HOLD_POWER = 0.4;

    private int     rampTargetTicks = 0;
    private boolean prevDpadUp      = false;
    private boolean prevDpadDown    = false;

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
        gateServo  = hardwareMap.get(Servo.class,     "servo");

        // Flywheel
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Drivetrain
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CoreHex — posición 0 y RUN_TO_POSITION
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHex.setTargetPosition(0);
        coreHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coreHex.setPower(RAMP_HOLD_POWER);

        // Servo — detenido al inicio
        gateServo.setPosition(SERVO_STOP);

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
            controlServoToggle();
            controlCoreHex();
            updateTelemetry();
        }
    }

    // =========================================================================
    // CONDUCCIÓN — Arcade Drive
    // Velocidad controlada por bumpers (L1/R1), independiente del disparador
    // =========================================================================
    private void arcadeDrive() {
        // L1 = lento | R1 = turbo | ninguno = normal
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

        // Normalización para que la suma nunca supere ±1.0
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
    // FLYWHEEL — Rampa gradual con R2 (right_trigger)
    // Completamente independiente de la velocidad de conducción
    // =========================================================================
    private void controlFlywheel() {
        if (gamepad1.options) {
            // Reversa para limpiar atascos — potencia fija, sin rampa
            flywheelPower = -0.5;
        } else if (gamepad1.square) {
            // Potencia máxima fija
            flywheelPower = POWER_MAX;
        } else if (gamepad1.right_trigger > 0.5) {
            // R2: subir gradualmente hasta POWER_TARGET
            flywheelPower = Math.min(flywheelPower + FLYWHEEL_RAMP_RATE, POWER_TARGET);
        } else {
            // Sin input de disparo: bajar gradualmente hasta 0
            flywheelPower = Math.max(flywheelPower - FLYWHEEL_RAMP_RATE, 0.0);
        }

        flywheel.setPower(flywheelPower);
    }

    // =========================================================================
    // SERVO ACOMODADOR — Toggle con L2 (left_trigger)
    // =========================================================================
    private void controlServoToggle() {
        boolean currentL2 = gamepad1.left_trigger > 0.5;

        // Flanco ascendente: actúa solo al presionar, no al mantener
        if (currentL2 && !prevL2) {
            servoToggleOn = !servoToggleOn;
            gateServo.setPosition(servoToggleOn ? SERVO_RUN : SERVO_STOP);
        }

        prevL2 = currentL2;
    }

    // =========================================================================
    // COREHEX — Empuje de pelotas por pasos con dpad
    // =========================================================================
    private void controlCoreHex() {
        boolean currentUp   = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;

        if (currentUp && !prevDpadUp) {
            rampTargetTicks = Math.min(rampTargetTicks + RAMP_STEP_TICKS, RAMP_MAX_TICKS);
            coreHex.setTargetPosition(rampTargetTicks);
        }

        if (currentDown && !prevDpadDown) {
            rampTargetTicks = Math.max(rampTargetTicks - RAMP_STEP_TICKS, RAMP_MIN_TICKS);
            coreHex.setTargetPosition(rampTargetTicks);
        }

        prevDpadUp   = currentUp;
        prevDpadDown = currentDown;
    }

    // =========================================================================
    // TELEMETRÍA
    // =========================================================================
    private void updateTelemetry() {
        telemetry.addLine("=== revV4 ===");

        // Velocidad activa
        String speed = gamepad1.left_bumper  ? "LENTA (L1)"  :
                gamepad1.right_bumper ? "TURBO (R1)"  : "NORMAL";
        telemetry.addData("Velocidad",          speed);
        telemetry.addData("Drive L / R",        "%.2f / %.2f", lastLeftPower, lastRightPower);
        telemetry.addData("Encoder L / R",      "%d / %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());

        telemetry.addLine("---");
        telemetry.addData("Flywheel potencia",  "%.3f", flywheelPower);
        telemetry.addData("Flywheel objetivo",  POWER_TARGET);
        telemetry.addData("Servo acomodador",   servoToggleOn ? "GIRANDO" : "DETENIDO");

        telemetry.addLine("---");
        telemetry.addData("CoreHex actual",     coreHex.getCurrentPosition());
        telemetry.addData("CoreHex objetivo",   rampTargetTicks);

        telemetry.update();
    }
}