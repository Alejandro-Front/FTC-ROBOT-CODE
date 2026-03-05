package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * StarterBot TeleOp + Auto — Versión refactorizada
 *
 * Mejoras incluidas:
 *  - Arcade Drive con normalización, slew rate y triggers de velocidad
 *  - Detección de flanco (edge detection) para el botón Y en selección de modo
 *  - shootThreeRings() no bloqueante usando estados y un timer
 *  - Sin sleep() dentro de TeleOp
 *  - Métodos con responsabilidades claras y sin duplicación
 *  - Telemetría útil
 */
@TeleOp(name = "StarterBot TeleOp + Auto (Refactorings)", group = "TeleOp")
public class REVStarterBotTeleOpRefactored extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx flywheel;
    private DcMotor   coreHex;
    private DcMotor   leftDrive;
    private DcMotor   rightDrive;
    private CRServo   hopperServo;

    // -------------------------------------------------------------------------
    // Constantes de velocidad del flywheel (ticks/segundo)
    // -------------------------------------------------------------------------
    private static final int VELOCITY_BANK = 1300;
    private static final int VELOCITY_FAR  = 1900;
    private static final int VELOCITY_MAX  = 2200;

    /** Tolerancia para considerar que el flywheel alcanzó la velocidad objetivo */
    private static final int VELOCITY_TOLERANCE = 100;

    // -------------------------------------------------------------------------
    // Conversión de distancia: pulgadas → ticks del encoder
    //   Fórmula: (CPR × reducción) / (diámetro_rueda × π)
    //   CPR=28, reducción=15, diámetro=3 in
    // -------------------------------------------------------------------------
    private static final double TICKS_PER_INCH = (28.0 * 15.0) / (3.0 * Math.PI);

    // -------------------------------------------------------------------------
    // Selección de modo (antes de iniciar)
    // -------------------------------------------------------------------------
    private enum OperationMode { TELEOP, AUTO_BLUE, AUTO_RED }
    private OperationMode selectedMode = OperationMode.TELEOP;

    /** Estado previo del botón Y para detección de flanco ascendente */
    private boolean prevButtonY = false;

    // -------------------------------------------------------------------------
    // Slew Rate Limiter — evita cambios bruscos de potencia
    // -------------------------------------------------------------------------
    private double lastLeftPower  = 0.0;
    private double lastRightPower = 0.0;

    /** Máximo cambio de potencia por ciclo de loop (~20 ms) */
    private static final double SLEW_RATE = 0.05;

    // -------------------------------------------------------------------------
    // Sistema de disparo no bloqueante (estado + timer)
    // -------------------------------------------------------------------------
    private enum ShootState { IDLE, PUSHING, FEEDING, PAUSE }
    private ShootState shootState  = ShootState.IDLE;
    private int        ringsLeft   = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // Duraciones de cada fase del ciclo de disparo (ms)
    private static final int SHOOT_PUSH_MS  = 400;
    private static final int SHOOT_FEED_MS  = 500;
    private static final int SHOOT_PAUSE_MS = 400;

    // -------------------------------------------------------------------------
    // Timers para auto
    // -------------------------------------------------------------------------
    private final ElapsedTime autoLaunchTimer = new ElapsedTime();
    private final ElapsedTime autoDriveTimer  = new ElapsedTime();

    // =========================================================================
    // PUNTO DE ENTRADA
    // =========================================================================
    @Override
    public void runOpMode() {

        initHardware();
        selectModeLoop();   // Permite elegir modo antes de START

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

    /**
     * Mapea el hardware y aplica la configuración inicial de motores.
     * Centralizar aquí evita repetición y facilita ajustes futuros.
     */
    private void initHardware() {

        flywheel    = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex     = hardwareMap.get(DcMotor.class,   "coreHex");
        leftDrive   = hardwareMap.get(DcMotor.class,   "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class,   "rightDrive");
        hopperServo = hardwareMap.get(CRServo.class,   "servo");

        // El flywheel usa control PID interno del SDK para velocidad constante
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Invertir motores según orientación física del robot
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        // rightDrive queda en FORWARD (predeterminado)

        // Freno activo: el robot no se desliza al soltar el joystick
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hopperServo.setPower(0);

        telemetry.addLine("Hardware inicializado — listo");
        telemetry.update();
    }

    // =========================================================================
    // SELECCIÓN DE MODO (fase INIT)
    // =========================================================================

    /**
     * Bucle que corre durante la fase de inicialización.
     * Usa detección de flanco (edge detection) en el botón Y para que
     * cada pulsación cambie el modo exactamente una vez.
     */
    private void selectModeLoop() {

        while (opModeInInit()) {

            boolean currentY = gamepad1.y;

            // Solo cambia al detectar el frente ascendente (presión nueva)
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

    /** Cicla entre los modos de operación disponibles */
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

            arcadeDrive();
            controlFlywheel();
            controlFeederAndServo();
            updateShooterStateMachine();  // Disparo no bloqueante

            // Iniciar ciclo de 3 disparos al presionar A
            if (gamepad1.a && shootState == ShootState.IDLE) {
                startShootSequence(3);
            }

            updateTelemetry();
        }
    }

    // =========================================================================
    // CONDUCCIÓN — Arcade Drive
    // =========================================================================

    /**
     * Arcade Drive con:
     *  - Triggers para cambio de velocidad (lento / normal / turbo)
     *  - Normalización: evita que la suma supere ±1.0
     *  - Slew Rate Limiter: suaviza la aceleración
     */
    private void arcadeDrive() {

        // Selección de multiplicador de velocidad
        double speedMultiplier;
        if      (gamepad1.left_trigger  > 0.5) speedMultiplier = 0.4;  // Lento
        else if (gamepad1.right_trigger > 0.5) speedMultiplier = 1.0;  // Turbo
        else                                   speedMultiplier = 0.7;  // Normal

        double forward = -gamepad1.left_stick_y;   // Adelante / atrás
        double turn    =  gamepad1.right_stick_x;  // Giro

        double leftPower  = forward - turn;
        double rightPower = forward + turn;

        // Normalización: si algún valor supera 1.0, escala ambos proporcionalmente
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower  /= maxPower;
            rightPower /= maxPower;
        }

        // Aplicar multiplicador de velocidad
        leftPower  *= speedMultiplier;
        rightPower *= speedMultiplier;

        // Slew Rate Limiter: limita el delta de potencia por ciclo
        leftPower  = applySlewRate(leftPower,  lastLeftPower);
        rightPower = applySlewRate(rightPower, lastRightPower);

        lastLeftPower  = leftPower;
        lastRightPower = rightPower;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    /**
     * Limita el cambio de potencia al máximo permitido por SLEW_RATE.
     * @param target  Potencia objetivo
     * @param current Potencia actual
     * @return        Potencia ajustada
     */
    private double applySlewRate(double target, double current) {
        double delta = target - current;
        delta = Math.max(-SLEW_RATE, Math.min(SLEW_RATE, delta));
        return current + delta;
    }

    // =========================================================================
    // CONTROL FLYWHEEL
    // =========================================================================

    /**
     * El flywheel usa setVelocity() (PID del SDK) para mantener RPM constantes.
     * options = reversa manual | bumpers = modos auto | circle/square = velocidades fijas
     */
    private void controlFlywheel() {

        if (gamepad1.options) {
            flywheel.setPower(-0.5);             // Reversa manual (limpieza de atascos)
        } else if (gamepad1.left_bumper) {
            setFlywheelAndFeed(VELOCITY_FAR);    // Disparo lejano
        } else if (gamepad1.right_bumper) {
            setFlywheelAndFeed(VELOCITY_BANK);   // Disparo corto (bank shot)
        } else if (gamepad1.circle) {
            flywheel.setVelocity(VELOCITY_BANK);
        } else if (gamepad1.square) {
            flywheel.setVelocity(VELOCITY_MAX);
        } else {
            flywheel.setVelocity(0);
        }
    }

    /**
     * Activa el flywheel a la velocidad dada y habilita el alimentador
     * solo cuando la velocidad real es suficiente (evita disparos flojos).
     *
     * @param targetVelocity Velocidad objetivo en ticks/segundo
     */
    private void setFlywheelAndFeed(int targetVelocity) {

        flywheel.setVelocity(targetVelocity);
        hopperServo.setPower(-1);  // Hopper siempre empuja hacia la rueda

        boolean upToSpeed = flywheel.getVelocity() >= targetVelocity - VELOCITY_TOLERANCE;
        coreHex.setPower(upToSpeed ? 1.0 : 0.0);
    }

    // =========================================================================
    // CONTROL COREHEX Y SERVO (manual)
    // =========================================================================

    /**
     * Control manual del alimentador (coreHex) y del servo del hopper.
     * Solo activo cuando el estado de disparo automático es IDLE,
     * para no interferir con la secuencia automática.
     */
    private void controlFeederAndServo() {

        if (shootState != ShootState.IDLE) return; // La máquina de estados tiene prioridad

        // CoreHex: cross = subir | triangle = bajar
        if      (gamepad1.cross)    coreHex.setPower( 0.5);
        else if (gamepad1.triangle) coreHex.setPower(-0.5);
        else                        coreHex.setPower( 0.0);

        // Servo del hopper: dpad_left / dpad_right
        if      (gamepad1.dpad_left)  hopperServo.setPower( 1.0);
        else if (gamepad1.dpad_right) hopperServo.setPower(-1.0);
        else                          hopperServo.setPower( 0.0);
    }

    // =========================================================================
    // DISPARO AUTOMÁTICO — Máquina de estados (no bloqueante)
    // =========================================================================

    /**
     * Inicia la secuencia de disparo de N anillos.
     * No bloquea el loop principal; la lógica avanza en updateShooterStateMachine().
     */
    private void startShootSequence(int rings) {
        ringsLeft  = rings;
        shootState = ShootState.PUSHING;
        hopperServo.setPower(-1);
        shootTimer.reset();
    }

    /**
     * Avanza la máquina de estados del disparador.
     * Debe llamarse cada iteración del loop principal.
     *
     * Estados:
     *  PUSHING  → mueve el servo para empujar el anillo
     *  FEEDING  → activa coreHex para alimentar el siguiente
     *  PAUSE    → breve pausa antes del siguiente anillo
     *  IDLE     → secuencia terminada
     */
    private void updateShooterStateMachine() {

        switch (shootState) {

            case PUSHING:
                if (shootTimer.milliseconds() >= SHOOT_PUSH_MS) {
                    coreHex.setPower(1.0);
                    shootState = ShootState.FEEDING;
                    shootTimer.reset();
                }
                break;

            case FEEDING:
                if (shootTimer.milliseconds() >= SHOOT_FEED_MS) {
                    coreHex.setPower(0.0);
                    shootState = ShootState.PAUSE;
                    shootTimer.reset();
                }
                break;

            case PAUSE:
                if (shootTimer.milliseconds() >= SHOOT_PAUSE_MS) {
                    ringsLeft--;
                    if (ringsLeft > 0) {
                        // Siguiente anillo
                        shootState = ShootState.PUSHING;
                        shootTimer.reset();
                    } else {
                        // Secuencia completa
                        hopperServo.setPower(0.0);
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
    // AUTO DRIVE — Basado en encoders con timeout de seguridad
    // =========================================================================

    /**
     * Mueve el robot usando encoders con timeout de seguridad.
     *
     * @param speed            Potencia de avance (0.0 – 1.0)
     * @param leftDistInches   Distancia motor izquierdo en pulgadas (negativo = atrás)
     * @param rightDistInches  Distancia motor derecho en pulgadas
     * @param timeoutMs        Tiempo máximo permitido en milisegundos
     */
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

        // Detener y restaurar modo normal
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // =========================================================================
    // RUTINAS AUTÓNOMAS
    // =========================================================================

    /**
     * Lanza anillos durante 10 s, luego navega al objetivo azul.
     * La secuencia de conducción es especular a AUTO_RED.
     */
    private void doAutoBlue() {
        runAutoLaunchPhase();

        // Navegación: atrás, girar izquierda, avanzar
        autoDrive(0.5, -12, -12, 5000);
        autoDrive(0.5,  -8,   8, 5000);
        autoDrive(1.0, -50, -50, 5000);
    }

    /**
     * Igual que AUTO_BLUE pero gira en sentido contrario (derecha).
     */
    private void doAutoRed() {
        runAutoLaunchPhase();

        // Navegación: atrás, girar derecha, avanzar
        autoDrive(0.5, -12, -12, 5000);
        autoDrive(0.5,   8,  -8, 5000);
        autoDrive(1.0, -50, -50, 5000);
    }

    /**
     * Fase de lanzamiento compartida por ambos modos auto (10 segundos).
     * Reutilizar este método elimina la duplicación entre AUTO_BLUE y AUTO_RED.
     */
    private void runAutoLaunchPhase() {

        autoLaunchTimer.reset();

        while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10_000) {
            setFlywheelAndFeed(VELOCITY_BANK);

            telemetry.addData("Lanzando — tiempo", "%.1f s", autoLaunchTimer.seconds());
            telemetry.update();
        }

        // Apagar mecanismos al terminar la fase
        flywheel.setVelocity(0);
        coreHex.setPower(0);
        hopperServo.setPower(0);
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
        telemetry.addData("Disparo estado",    shootState.name());
        telemetry.addData("Anillos restantes", ringsLeft);
        telemetry.update();
    }
}