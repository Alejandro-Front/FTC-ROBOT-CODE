package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CoreHex;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Autonomo basico — avanza hacia adelante con CoreHex encendido.
 *
 * Parametros ajustables:
 *   DRIVE_POWER  — potencia de avance (0.0 a 1.0)
 *   DRIVE_TIME   — tiempo de avance en segundos
 */
@Autonomous(name = "Autonomo V1", group = "Auto")
public class AutoBasico extends LinearOpMode {

    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_TIME  = 3.0;

    private Drivetrain drivetrain;
    private CoreHex coreHex;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry);
        coreHex    = new CoreHex(hardwareMap);

        drivetrain.init();
        coreHex.init();

        telemetry.addLine("Auto Basico — listo");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIME) {
            drivetrain.setPower(DRIVE_POWER, DRIVE_POWER);
            coreHex.setPower(0.8);

            telemetry.addData("Tiempo", "%.1f / %.1f s", timer.seconds(), DRIVE_TIME);
            telemetry.addData("Drive", "%.2f", DRIVE_POWER);
            telemetry.addData("CoreHex", "ENCENDIDO");
            telemetry.update();
        }

        drivetrain.setPower(0, 0);
        coreHex.setPower(0);

        telemetry.addLine("Autonomo terminado");
        telemetry.update();
    }
}
