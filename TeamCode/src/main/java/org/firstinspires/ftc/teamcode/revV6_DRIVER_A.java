package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ControlProfile;
import org.firstinspires.ftc.teamcode.subsystems.CoreHex;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * StarterBot TeleOp — revV6
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
 * │ Flecha arriba    │ Sube potencia del disparador (+0.05)             │
 * │ Flecha abajo     │ Baja potencia del disparador (-0.05)             │
 * └──────────────────┴─────────────────────────────────────────────────┘
 */
@TeleOp(name = "revV6 DRIVER_A", group = "TeleOp")
public class revV6_DRIVER_A extends LinearOpMode {

    private Drivetrain drivetrain;
    private CoreHex coreHex;
    private Flywheel flywheel;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry);
        coreHex = new CoreHex(hardwareMap, gamepad1, ControlProfile.DRIVER_A);
        flywheel = new Flywheel(hardwareMap, gamepad1, ControlProfile.DRIVER_A);

        drivetrain.init();
        coreHex.init();
        flywheel.init();

        telemetry.addLine("Driver A — R2=feeder, L2=rev, X=flywheel toggle");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
            coreHex.update();
            flywheel.update();
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== revV6 ===");

        String speed = gamepad1.left_bumper  ? "LENTA  (L1)" :
                gamepad1.right_bumper ? "TURBO  (R1)" : "NORMAL";
        telemetry.addData("Velocidad",   speed);
        telemetry.addData("Drive L / R", "%.2f / %.2f", drivetrain.getLastLeftPower(), drivetrain.getLastRightPower());

        telemetry.addLine("---");
        String coreHexStatus;
        if      (coreHex.isRunningForward()) coreHexStatus = "NORMAL (R2)";
        else if (coreHex.isRunningReverse()) coreHexStatus = "REVERSA — desatorando (L2)";
        else                                 coreHexStatus = "DETENIDO";
        telemetry.addData("CoreHex", coreHexStatus);

        telemetry.addLine("---");
        telemetry.addData("Disparador", flywheel.isOn() ? "ENCENDIDO (A)" : "APAGADO");
        telemetry.addData("Potencia disparador", "%.0f%%  [↑/↓ para ajustar]", flywheel.getPower() * 100.0);

        int filled = (int) Math.round(flywheel.getPower() * 10);
        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < 10; i++) bar.append(i < filled ? "█" : "░");
        bar.append("]");
        telemetry.addData("Nivel", bar.toString());

        telemetry.update();
    }
}
