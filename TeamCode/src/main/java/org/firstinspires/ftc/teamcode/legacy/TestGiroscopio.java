package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * TestGiroscopio — Prueba de IMU del Control Hub
 *
 * Muestra en telemetría:
 *   Yaw   → rotación horizontal (izquierda/derecha) — más útil para auto
 *   Pitch → inclinación adelante/atrás
 *   Roll  → inclinación lateral
 *   Velocidad angular en los 3 ejes
 *
 * Controles:
 *   A/Cross  → resetea el Yaw a 0
 */
@TeleOp(name = "TestGiroscopio", group = "Test")
@Disabled
public class TestGiroscopio extends LinearOpMode {

    private IMU imu;

    @Override
    public void runOpMode() {

        // --- Inicializar IMU ---
        imu = hardwareMap.get(IMU.class, "imu");

        // Orientación del Control Hub en el robot.
        // Ajusta LOGO_UP y USB_FORWARD según cómo esté montado tu Control Hub:
        //   LogoFacingDirection: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
        //   UsbFacingDirection:  UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
        IMU.Parameters parametros = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parametros);

        telemetry.addLine("TestGiroscopio listo");
        telemetry.addLine("A/Cross = resetear Yaw a 0");
        telemetry.update();

        waitForStart();

        // Resetea al arrancar para que empiece en 0
        imu.resetYaw();

        while (opModeIsActive()) {

            // Reset con A/Cross
            if (gamepad1.cross) {
                imu.resetYaw();
            }

            // Leer ángulos
            YawPitchRollAngles angulos = imu.getRobotYawPitchRollAngles();

            double yaw   = angulos.getYaw(AngleUnit.DEGREES);
            double pitch = angulos.getPitch(AngleUnit.DEGREES);
            double roll  = angulos.getRoll(AngleUnit.DEGREES);

            // Leer velocidad angular
            AngularVelocity velocidad = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // --- Telemetría ---
            telemetry.addLine("=== IMU — Control Hub ===");
            telemetry.addLine("");

            telemetry.addLine("-- Ángulos --");
            telemetry.addData("Yaw   (horizontal)", "%.2f°", yaw);
            telemetry.addData("Pitch (adelante)   ", "%.2f°", pitch);
            telemetry.addData("Roll  (lateral)    ", "%.2f°", roll);

            telemetry.addLine("");
            telemetry.addLine("-- Velocidad angular (°/s) --");
            telemetry.addData("Vel. Yaw  ", "%.2f°/s", velocidad.zRotationRate);
            telemetry.addData("Vel. Pitch", "%.2f°/s", velocidad.xRotationRate);
            telemetry.addData("Vel. Roll ", "%.2f°/s", velocidad.yRotationRate);

            telemetry.addLine("");
            telemetry.addLine("A/Cross → resetear Yaw");

            telemetry.update();
        }
    }
}