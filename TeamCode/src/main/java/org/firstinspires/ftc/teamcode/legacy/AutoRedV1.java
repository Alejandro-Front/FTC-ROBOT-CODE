package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AutoRojo — Alianza Roja
 *
 * Secuencia:
 *   1. Avanzar distancia fija (encoders)
 *   2. Giro inicial estimado hacia zona de disparo (encoders)
 *   3. Alineación fina con AprilTag (Yaw del robotPose)
 *   4. Rampar flywheel
 *   5. Alimentar pelotas con CoreHex
 *   6. Bajar flywheel
 *
 * ── Variables configurables ──────────────────────────────────────────────────
 *   DRIVE_INCHES          → distancia a avanzar en pulgadas
 *   TURN_DEGREES          → giro inicial estimado (+ = izquierda, - = derecha)
 *   APRILTAG_TARGET_ID    → ID del AprilTag de la zona de disparo (-1 = cualquiera)
 *   FLYWHEEL_TARGET       → potencia objetivo del flywheel (0.0–1.0)
 *   FEED_TIME_MS          → milisegundos de alimentación con CoreHex
 *   ALIGN_TOLERANCE_DEG   → error de yaw aceptable para considerar alineado
 * ─────────────────────────────────────────────────────────────────────────────
 */
@Autonomous(name = "AutoRedV1", group = "Autonomous")
@Disabled
public class AutoRedV1 extends LinearOpMode {

    // =========================================================================
    // VARIABLES CONFIGURABLES — ajusta estas sin tocar el resto del código
    // =========================================================================

    /** Distancia a avanzar desde la posición inicial (pulgadas) */
    private static final double DRIVE_INCHES        = 24.0;

    /**
     * Giro estimado antes de buscar el AprilTag.
     * Positivo = gira a la IZQUIERDA, Negativo = gira a la DERECHA.
     * Alianza Roja: ajusta según tu posición en el campo.
     */
    private static final double TURN_DEGREES        = -45.0;

    /**
     * ID del AprilTag objetivo en la zona de disparo.
     * Usa -1 para aceptar cualquier tag visible.
     */
    private static final int    APRILTAG_TARGET_ID  = -1;

    /** Potencia objetivo del flywheel al disparar */
    private static final double FLYWHEEL_TARGET     = 0.85;

    /** Tiempo de alimentación con CoreHex (ms) */
    private static final long   FEED_TIME_MS        = 3000;

    /** Error de yaw (grados) para considerar el robot alineado */
    private static final double ALIGN_TOLERANCE_DEG = 2.0;

    /** Potencia de giro del CoreHex durante la alimentación */
    private static final double COREHEX_POWER       = 0.8;

    // =========================================================================
    // Drivetrain — constantes físicas
    // =========================================================================
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_REV        = 537.7; // goBILDA 5203 312 RPM
    private static final double COUNTS_PER_INCH       =
            COUNTS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    /** Potencia base de conducción autónoma */
    private static final double DRIVE_POWER  = 0.5;
    /** Potencia de giro autónomo */
    private static final double TURN_POWER   = 0.4;
    /** Tolerancia de encoder para detener movimiento (counts) */
    private static final int    ENC_TOLERANCE = 20;

    // =========================================================================
    // Flywheel
    // =========================================================================
    private static final double FLYWHEEL_RAMP_RATE = 0.01;

    // =========================================================================
    // Hardware
    // =========================================================================
    private DcMotor        leftDrive;
    private DcMotor        rightDrive;
    private DcMotor        flywheel;
    private DcMotorEx      coreHex;
    private CRServo        gateServo;

    private AprilTagProcessor aprilTag;
    private VisionPortal      visionPortal;

    // =========================================================================
    // PUNTO DE ENTRADA
    // =========================================================================
    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addLine("AutoRojo listo. Presiona START.");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        // ── Paso 1: Avanzar ──────────────────────────────────────────────────
        telemetry.addLine("Paso 1: Avanzando...");
        telemetry.update();
        driveInches(DRIVE_INCHES, DRIVE_POWER);

        sleep(300);

        // ── Paso 2: Giro estimado ─────────────────────────────────────────────
        telemetry.addLine("Paso 2: Girando hacia zona de disparo...");
        telemetry.update();
        turnDegrees(TURN_DEGREES, TURN_POWER);

        sleep(300);

        // ── Paso 3: Alineación fina con AprilTag ─────────────────────────────
        telemetry.addLine("Paso 3: Alineando con AprilTag...");
        telemetry.update();
        alignWithAprilTag();

        // ── Paso 4: Rampar flywheel ───────────────────────────────────────────
        telemetry.addLine("Paso 4: Rampando flywheel...");
        telemetry.update();
        rampFlywheel(FLYWHEEL_TARGET);

        // ── Paso 5: Alimentar pelotas ─────────────────────────────────────────
        telemetry.addLine("Paso 5: Alimentando pelotas...");
        telemetry.update();
        coreHex.setPower(COREHEX_POWER);
        sleep(FEED_TIME_MS);
        coreHex.setPower(0);

        // ── Paso 6: Bajar flywheel ────────────────────────────────────────────
        telemetry.addLine("Paso 6: Deteniendo flywheel...");
        telemetry.update();
        rampFlywheel(0.0);

        visionPortal.close();
        telemetry.addLine("¡Autónomo completo!");
        telemetry.update();
        sleep(2000);
    }

    // =========================================================================
    // HARDWARE
    // =========================================================================
    private void initHardware() {
        leftDrive  = hardwareMap.get(DcMotor.class,   "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,   "rightDrive");
        flywheel   = hardwareMap.get(DcMotor.class,   "flywheel");
        coreHex    = hardwareMap.get(DcMotorEx.class, "coreHex");
        gateServo  = hardwareMap.get(CRServo.class,   "servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        coreHex.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHex.setPower(0);

        gateServo.setPower(0);
        flywheel.setPower(0);
    }

    // =========================================================================
    // APRIL TAG
    // =========================================================================
    private void initAprilTag() {
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation =
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Alineación fina: usa el Yaw del robotPose del AprilTag para girar
     * hasta que el robot quede mirando al objetivo dentro de ALIGN_TOLERANCE_DEG.
     * Si no detecta el tag en 3 segundos, continúa con la posición actual.
     */
    private void alignWithAprilTag() {
        long startTime = System.currentTimeMillis();
        boolean aligned = false;

        while (opModeIsActive() && !aligned) {

            // Timeout de seguridad: si no ve el tag en 3 s, avanza igual
            if (System.currentTimeMillis() - startTime > 3000) {
                telemetry.addLine("AprilTag no detectado — continuando sin alinear");
                telemetry.update();
                break;
            }

            AprilTagDetection target = getTargetDetection();

            if (target != null && target.robotPose != null) {
                double yawError = target.robotPose.getOrientation()
                        .getYaw(AngleUnit.DEGREES);

                telemetry.addData("Yaw error (deg)", "%.2f", yawError);
                telemetry.update();

                if (Math.abs(yawError) <= ALIGN_TOLERANCE_DEG) {
                    aligned = true;
                    stopDrive();
                } else {
                    // Gira proporcionalmente al error; limita entre 0.15 y TURN_POWER
                    double correction = Math.max(0.15,
                            Math.min(TURN_POWER, Math.abs(yawError) * 0.01));

                    if (yawError > 0) {
                        // Robot debe girar a la derecha
                        leftDrive.setPower( correction);
                        rightDrive.setPower(-correction);
                    } else {
                        // Robot debe girar a la izquierda
                        leftDrive.setPower(-correction);
                        rightDrive.setPower( correction);
                    }
                }
            } else {
                // Tag aún no visible, espera
                stopDrive();
                sleep(50);
            }
        }
        stopDrive();
    }

    /** Devuelve la detección del tag objetivo, o null si no hay ninguna visible. */
    private AprilTagDetection getTargetDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                if (APRILTAG_TARGET_ID == -1 || d.id == APRILTAG_TARGET_ID) {
                    return d;
                }
            }
        }
        return null;
    }

    // =========================================================================
    // MOVIMIENTO — Encoders
    // =========================================================================

    /** Avanza (positivo) o retrocede (negativo) la cantidad de pulgadas indicada. */
    private void driveInches(double inches, double power) {
        int counts = (int)(inches * COUNTS_PER_INCH);

        resetEncoders();

        int targetLeft  =  counts;
        int targetRight =  counts;

        leftDrive.setTargetPosition(targetLeft);
        rightDrive.setTargetPosition(targetRight);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while (opModeIsActive() &&
                (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Enc L/R", "%d / %d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Gira en el lugar los grados indicados.
     * Positivo = izquierda, Negativo = derecha.
     * Fórmula: arco = (degrees/360) * 2π * (trackWidth/2)
     * Ajusta TRACK_WIDTH_INCHES a la separación real entre centros de rueda.
     */
    private static final double TRACK_WIDTH_INCHES = 13.5;

    private void turnDegrees(double degrees, double power) {
        double arcInches = (Math.abs(degrees) / 360.0)
                * 2.0 * Math.PI * (TRACK_WIDTH_INCHES / 2.0);
        int counts = (int)(arcInches * COUNTS_PER_INCH);

        resetEncoders();

        // Giro izquierda: rueda derecha avanza, izquierda retrocede
        int targetLeft  = (degrees > 0) ? -counts :  counts;
        int targetRight = (degrees > 0) ?  counts : -counts;

        leftDrive.setTargetPosition(targetLeft);
        rightDrive.setTargetPosition(targetRight);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while (opModeIsActive() &&
                (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Girando...", "%.1f grados", degrees);
            telemetry.update();
        }

        stopDrive();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // =========================================================================
    // FLYWHEEL — Rampa gradual
    // =========================================================================

    /** Sube o baja el flywheel gradualmente hasta el objetivo. */
    private void rampFlywheel(double target) {
        double current = flywheel.getPower();
        while (opModeIsActive() && Math.abs(current - target) > FLYWHEEL_RAMP_RATE) {
            if (current < target) current += FLYWHEEL_RAMP_RATE;
            else                  current -= FLYWHEEL_RAMP_RATE;
            flywheel.setPower(current);
            telemetry.addData("Flywheel", "%.3f → %.3f", current, target);
            telemetry.update();
            sleep(20);
        }
        flywheel.setPower(target);
    }
}