package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * StarterBot TeleOp + Auto — V2
 *
 * CAMBIOS RESPECTO A V1:
 *  [Fix 1] Drivetrain: curveDrive() — al girar, el lado interno reduce velocidad
 *          pero NO se invierte, permitiendo avanzar y curvar simultáneamente.
 *          Adaptado para omni adelante + tracción por cadena atrás.
 *
 *  [Fix 2] CoreHex como apuntador de rampa: usa RUN_TO_POSITION con encoder.
 *          Los botones inclinan la rampa incrementalmente; al soltar, el motor
 *          mantiene la posición actual (no se cae por gravedad).
 *
 *  [Fix 3] Servo de bloqueo (Servo estándar, NO CRServo):
 *          - Reposo (sin botón): posición BLOQUEADO — detiene las pelotas.
 *          - Activo (botón dpad_up): posición ABIERTO — libera una pelota.
 *          - Al soltar, regresa automáticamente a BLOQUEADO.
 */
@TeleOp(name = "StarterBot TeleOp V2", group = "TeleOp")
@Disabled
public class REVStarterBotTeleOpV2 extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx flywheel;
    private DcMotorEx coreHex;       // DcMotorEx para usar RUN_TO_POSITION con hold
    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private Servo     gateServo;     // [Fix 3] Servo estándar, reemplaza CRServo

    // -------------------------------------------------------------------------
    // Constantes flywheel (ticks/segundo)
    // -------------------------------------------------------------------------
    private static final int VELOCITY_BANK = 1300;
    private static final int VELOCITY_FAR  = 1900;
    private static final int VELOCITY_MAX  = 2200;
    private static final int VELOCITY_TOLERANCE = 100;

    // -------------------------------------------------------------------------
    // Encoder drivetrain
    // -------------------------------------------------------------------------
    private static final double TICKS_PER_INCH = (28.0 * 15.0) / (3.0 * Math.PI);

    // -------------------------------------------------------------------------
    // [Fix 1] Parámetros de curveDrive
    //
    //  TURN_REDUCTION: qué tanto reduce la velocidad el lado interno al girar.
    //    0.0 → giro sobre el eje (lado interno se detiene)
    //    0.5 → giro suave (lado interno va al 50 % del avance)  ← RECOMENDADO
    //    1.0 → sin giro (ambos lados iguales)
    //
    //  TURN_DEADBAND: zona muerta del stick de giro para ignorar ruido.
    // -------------------------------------------------------------------------
    private static final double TURN_REDUCTION = 0.5;
    private static final double TURN_DEADBAND  = 0.05;

    // -------------------------------------------------------------------------
    // [Fix 2] CoreHex — apuntador de rampa
    //
    //  RAMP_STEP_TICKS : cuántos ticks mueve cada pulsación de botón
    //  RAMP_MAX_TICKS  : límite superior (rampa arriba al máximo)
    //  RAMP_MIN_TICKS  : límite inferior (rampa abajo al mínimo)
    //  RAMP_HOLD_POWER : potencia que usa RUN_TO_POSITION para sostener posición
    // -------------------------------------------------------------------------
    private static final int    RAMP_STEP_TICKS = 50;
    private static final int    RAMP_MAX_TICKS  =  500;
    private static final int    RAMP_MIN_TICKS  = -500;
    private static final double RAMP_HOLD_POWER = 0.4;

    /** Posición objetivo actual de la rampa (en ticks del encoder) */
    private int rampTargetTicks = 0;

    /** Edge detection para los botones de rampa */
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;

    // -------------------------------------------------------------------------
    // [Fix 3] Servo de compuerta (posiciones 0.0 – 1.0)
    //
    //  Ajusta GATE_BLOCKED y GATE_OPEN según el ángulo físico de tu servo.
    //  Prueba con el Driver Station en modo "Servo" para encontrar los valores.
    // -------------------------------------------------------------------------
    private static final double GATE_BLOCKED = 0.0;   // Posición de reposo (bloquea)
    private static final double GATE_OPEN    = 0.6;   // Posición activa (libera pelota)

    // -------------------------------------------------------------------------
    // Slew Rate Limiter
    // -------------------------------------------------------------------------
    private double lastLeftPower  = 0.0;
    private double lastRightPower = 0.0;
    private static final double SLEW_RATE = 0.05;

    // -------------------------------------------------------------------------
    // Disparo automático (no bloqueante)
    // -------------------------------------------------------------------------
    private enum ShootState { IDLE, PUSHING, FEEDING, PAUSE }
    private ShootState shootState = ShootState.IDLE;
    private int        ringsLeft  = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private static final int SHOOT_PUSH_MS  = 400;
    private static final int SHOOT_FEED_MS  = 500;
    private static final int SHOOT_PAUSE_MS = 400;

    // -------------------------------------------------------------------------
    // Selección de modo
    // -------------------------------------------------------------------------
    private enum OperationMode { TELEOP, AUTO_BLUE, AUTO_RED }
    private OperationMode selectedMode = OperationMode.TELEOP;
    private boolean prevButtonY = false;

    // -------------------------------------------------------------------------
    // Timers auto
    // -------------------------------------------------------------------------
    private final ElapsedTime autoLaunchTimer = new ElapsedTime();
    private final ElapsedTime autoDriveTimer  = new ElapsedTime();

    // =========================================================================
    // PUNTO DE ENTRADA
    // =========================================================================
    @Override
    public void runOpMode() {
        initHardware();
        selectModeLoop();
        waitForStart();

        switch (selectedMode) {
            case AUTO_BLUE: doAutoBlue(); break;
            case AUTO_RED:  doAutoRed();  break;
            default:        doTeleOp();   break;
        }
    }

    // =========================================================================
    // INICIALIZACIÓN
    // =========================================================================
    private void initHardware() {

        flywheel  = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex   = hardwareMap.get(DcMotorEx.class, "coreHex");   // DcMotorEx
        leftDrive  = hardwareMap.get(DcMotor.class,  "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,  "rightDrive");
        gateServo  = hardwareMap.get(Servo.class,    "servo");      // Servo estándar

        // Flywheel — control PID de velocidad
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // Drivetrain
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // [Fix 2] CoreHex — inicializar en posición 0 y modo RUN_TO_POSITION
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHex.setTargetPosition(0);
        coreHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coreHex.setPower(RAMP_HOLD_POWER);  // Mantiene posición desde el inicio

        // [Fix 3] Servo — posición de bloqueo por defecto
        gateServo.setPosition(GATE_BLOCKED);

        telemetry.addLine("Hardware inicializado — listo");
        telemetry.update();
    }

    // =========================================================================
    // SELECCIÓN DE MODO
    // =========================================================================
    private void selectModeLoop() {
        while (opModeInInit()) {
            boolean currentY = gamepad1.y;
            if (currentY && !prevButtonY) {
                selectedMode = cycleMode(selectedMode);
            }
            prevButtonY = currentY;

            telemetry.addLine("Presiona Y para cambiar modo");
            telemetry.addData("Modo seleccionado", selectedMode.name());
            telemetry.addLine("Presiona START para comenzar");
            telemetry.update();
        }
    }

    private OperationMode cycleMode(OperationMode current) {
        switch (current) {
            case TELEOP:    return OperationMode.AUTO_BLUE;
            case AUTO_BLUE: return OperationMode.AUTO_RED;
            default:        return OperationMode.TELEOP;
        }
    }

    // =========================================================================
    // TELEOP
    // =========================================================================
    private void doTeleOp() {
        while (opModeIsActive()) {

            curveDrive();                     // [Fix 1]
            controlRamp();                    // [Fix 2]
            controlGateServo();               // [Fix 3]
            controlFlywheel();
            updateShooterStateMachine();

            if (gamepad1.a && shootState == ShootState.IDLE) {
                startShootSequence(3);
            }

            updateTelemetry();
        }
    }

    // =========================================================================
    // [Fix 1] CURVE DRIVE — avanza y curva sin invertir ningún lado
    //
    //  Lógica:
    //   - "forward" mueve ambos lados por igual.
    //   - "turn" REDUCE (no invierte) la potencia del lado interno.
    //   - El lado externo mantiene la potencia completa de avance.
    //
    //  Ejemplo (forward=0.8, turn derecha=0.5, TURN_REDUCTION=0.5):
    //   leftPower  = 0.8                   → lado externo, velocidad plena
    //   rightPower = 0.8 × (1 - 0.5×0.5)  = 0.8 × 0.75 = 0.60 → lado interno reducido
    //   Resultado: el robot avanza y curva suavemente a la derecha.
    // =========================================================================
    private void curveDrive() {

        double speedMultiplier;
        if      (gamepad1.left_trigger  > 0.5) speedMultiplier = 0.4;
        else if (gamepad1.right_trigger > 0.5) speedMultiplier = 1.0;
        else                                   speedMultiplier = 0.7;

        double forward = -gamepad1.left_stick_y;
        double turn    =  gamepad1.right_stick_x;

        // Aplicar zona muerta al giro
        if (Math.abs(turn) < TURN_DEADBAND) turn = 0.0;

        double leftPower;
        double rightPower;

        if (turn > 0) {
            // Girando a la DERECHA → reducir lado derecho (interno)
            leftPower  = forward;
            rightPower = forward * (1.0 - TURN_REDUCTION * turn);
        } else if (turn < 0) {
            // Girando a la IZQUIERDA → reducir lado izquierdo (interno)
            leftPower  = forward * (1.0 + TURN_REDUCTION * turn); // turn es negativo
            rightPower = forward;
        } else {
            // Sin giro — recto
            leftPower  = forward;
            rightPower = forward;
        }

        // Escalar si algún valor supera ±1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower  /= maxPower;
            rightPower /= maxPower;
        }

        leftPower  *= speedMultiplier;
        rightPower *= speedMultiplier;

        // Slew rate
        leftPower  = applySlewRate(leftPower,  lastLeftPower);
        rightPower = applySlewRate(rightPower, lastRightPower);

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
    // [Fix 2] CONTROL DE RAMPA (CoreHex con RUN_TO_POSITION)
    //
    //  - dpad_up   → sube la rampa (incrementa posición objetivo)
    //  - dpad_down → baja la rampa (decrementa posición objetivo)
    //  - Al soltar: el motor mantiene la última posición (hold).
    //  - Límites software: RAMP_MIN_TICKS y RAMP_MAX_TICKS.
    //
    //  Edge detection: solo mueve un step por pulsación para control preciso.
    //  Si prefieres movimiento continuo mientras mantienes el botón,
    //  elimina las variables prevDpadUp/Down y el bloque edge detection.
    // =========================================================================
    private void controlRamp() {

        boolean currentUp   = gamepad1.dpad_up;
        boolean currentDown = gamepad1.dpad_down;

        // Flanco ascendente dpad_up → subir un step
        if (currentUp && !prevDpadUp) {
            rampTargetTicks = Math.min(rampTargetTicks + RAMP_STEP_TICKS, RAMP_MAX_TICKS);
            coreHex.setTargetPosition(rampTargetTicks);
        }

        // Flanco ascendente dpad_down → bajar un step
        if (currentDown && !prevDpadDown) {
            rampTargetTicks = Math.max(rampTargetTicks - RAMP_STEP_TICKS, RAMP_MIN_TICKS);
            coreHex.setTargetPosition(rampTargetTicks);
        }

        prevDpadUp   = currentUp;
        prevDpadDown = currentDown;

        // El motor permanece en RUN_TO_POSITION y sostiene la posición siempre
        // No se necesita cambiar el modo aquí; setPower() ya se fijó en initHardware()
    }

    // =========================================================================
    // [Fix 3] SERVO DE COMPUERTA
    //
    //  Lógica tipo "momentáneo":
    //   - Mientras se mantiene presionado circle → OPEN (libera pelota)
    //   - Al soltar → BLOCKED (vuelve a bloquear automáticamente)
    //
    //  Si prefieres modo "toggle" (un toque abre, otro toque cierra),
    //  reemplaza la lógica por un boolean gateOpen + edge detection.
    // =========================================================================
    private void controlGateServo() {
        if (gamepad1.circle) {
            gateServo.setPosition(GATE_OPEN);
        } else {
            gateServo.setPosition(GATE_BLOCKED);
        }
    }

    // =========================================================================
    // CONTROL FLYWHEEL (sin cambios respecto a V1)
    // =========================================================================
    private void controlFlywheel() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            setFlywheelAndFeed(VELOCITY_FAR);
        } else if (gamepad1.right_bumper) {
            setFlywheelAndFeed(VELOCITY_BANK);
        } else if (gamepad1.square) {
            flywheel.setVelocity(VELOCITY_MAX);
        } else {
            flywheel.setVelocity(0);
        }
    }

    private void setFlywheelAndFeed(int targetVelocity) {
        flywheel.setVelocity(targetVelocity);
        boolean upToSpeed = flywheel.getVelocity() >= targetVelocity - VELOCITY_TOLERANCE;
        // Nota: el servo de compuerta ahora lo controla controlGateServo().
        // Aquí solo manejamos el motor de alimentación (si lo tienes separado).
    }

    // =========================================================================
    // DISPARO AUTOMÁTICO — Máquina de estados (sin cambios respecto a V1)
    // =========================================================================
    private void startShootSequence(int rings) {
        ringsLeft  = rings;
        shootState = ShootState.PUSHING;
        gateServo.setPosition(GATE_OPEN);
        shootTimer.reset();
    }

    private void updateShooterStateMachine() {
        switch (shootState) {
            case PUSHING:
                if (shootTimer.milliseconds() >= SHOOT_PUSH_MS) {
                    shootState = ShootState.FEEDING;
                    shootTimer.reset();
                }
                break;

            case FEEDING:
                if (shootTimer.milliseconds() >= SHOOT_FEED_MS) {
                    shootState = ShootState.PAUSE;
                    shootTimer.reset();
                }
                break;

            case PAUSE:
                if (shootTimer.milliseconds() >= SHOOT_PAUSE_MS) {
                    ringsLeft--;
                    if (ringsLeft > 0) {
                        shootState = ShootState.PUSHING;
                        shootTimer.reset();
                    } else {
                        gateServo.setPosition(GATE_BLOCKED);  // Cierra compuerta al terminar
                        shootState = ShootState.IDLE;
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    // =========================================================================
    // AUTO DRIVE
    // =========================================================================
    private void autoDrive(double speed, double leftDistInches, double rightDistInches, int timeoutMs) {

        int leftTarget  = leftDrive.getCurrentPosition()  + (int)(leftDistInches  * TICKS_PER_INCH);
        int rightTarget = rightDrive.getCurrentPosition() + (int)(rightDistInches * TICKS_PER_INCH);

        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        autoDriveTimer.reset();

        while (opModeIsActive()
                && (leftDrive.isBusy() || rightDrive.isBusy())
                && autoDriveTimer.milliseconds() < timeoutMs) {
            telemetry.addData("AutoDrive L", leftDrive.getCurrentPosition());
            telemetry.addData("AutoDrive R", rightDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // =========================================================================
    // RUTINAS AUTÓNOMAS
    // =========================================================================
    private void doAutoBlue() {
        runAutoLaunchPhase();
        autoDrive(0.5, -12, -12, 5000);
        autoDrive(0.5,  -8,   8, 5000);
        autoDrive(1.0, -50, -50, 5000);
    }

    private void doAutoRed() {
        runAutoLaunchPhase();
        autoDrive(0.5, -12, -12, 5000);
        autoDrive(0.5,   8,  -8, 5000);
        autoDrive(1.0, -50, -50, 5000);
    }

    private void runAutoLaunchPhase() {
        autoLaunchTimer.reset();
        while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10_000) {
            setFlywheelAndFeed(VELOCITY_BANK);
            telemetry.addData("Lanzando — tiempo", "%.1f s", autoLaunchTimer.seconds());
            telemetry.update();
        }
        flywheel.setVelocity(0);
        gateServo.setPosition(GATE_BLOCKED);
    }

    // =========================================================================
    // TELEMETRÍA
    // =========================================================================
    private void updateTelemetry() {
        telemetry.addData("Modo",              selectedMode.name());
        telemetry.addData("Flywheel vel",      "%.0f t/s", flywheel.getVelocity());
        telemetry.addData("Drive L / R",       "%.2f / %.2f", lastLeftPower, lastRightPower);
        telemetry.addData("Encoder L / R",     "%d / %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Rampa pos actual",  coreHex.getCurrentPosition());
        telemetry.addData("Rampa pos objetivo",rampTargetTicks);
        telemetry.addData("Servo compuerta",   gateServo.getPosition() == GATE_OPEN ? "ABIERTO" : "BLOQUEADO");
        telemetry.addData("Disparo estado",    shootState.name());
        telemetry.addData("Anillos restantes", ringsLeft);
        telemetry.update();
    }
}