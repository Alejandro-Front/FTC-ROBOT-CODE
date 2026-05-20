package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "REV Starter Bot TeleOp", group = "TeleOp")
@Disabled
public class REVStarterBotTeleOpJava extends LinearOpMode {

    // Declaración de los motores y servo
    private DcMotor flywheel;
    private DcMotor coreHex;
    private DcMotor leftDrive;
    private CRServo servo;
    private DcMotor rightDrive;

    // Velocidades del flywheel (ticks por segundo)
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    @Override
    public void runOpMode() {

        // Conectar los dispositivos usando los nombres del archivo de configuración XML
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        servo = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Configuración inicial de motores
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Activar freno cuando no hay potencia
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo detenido al inicio
        servo.setPower(0);

        telemetry.addLine("Robot listo");
        telemetry.update();

        // Esperar a que presionen START
        waitForStart();

        while (opModeIsActive()) {

            // Ejecutar los métodos de control
            controlMovimiento();
            controlFlywheel();
            controlCoreHexYServo();

            // Mostrar información en la telemetría
            telemetry.addData("Velocidad Flywheel", ((DcMotorEx) flywheel).getVelocity());
            telemetry.addData("Potencia Flywheel", flywheel.getPower());
            telemetry.addData("Motor izquierdo", leftDrive.getPower());
            telemetry.addData("Motor derecho", rightDrive.getPower());
            telemetry.update();
        }
    }

    /**
     * Control del movimiento del robot
     * Joystick izquierdo = adelante / atrás
     * Joystick derecho = giro
     */
    private void controlMovimiento() {

        double x = gamepad1.right_stick_x; // giro
        double y = -gamepad1.left_stick_y; // avance

        double leftPower = y - x;
        double rightPower = y + x;

        // Limitar potencia entre -1 y 1
        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    /**
     * Control manual del CoreHex y del servo agitador
     */
    private void controlCoreHexYServo() {

        // Control del motor CoreHex (alimentador)
        if (gamepad1.cross) {
            coreHex.setPower(0.5);
        }
        else if (gamepad1.triangle) {
            coreHex.setPower(-0.5);
        }
        else {
            coreHex.setPower(0);
        }

        // Control del servo del hopper
        if (gamepad1.dpad_left) {
            servo.setPower(1);
        }
        else if (gamepad1.dpad_right) {
            servo.setPower(-1);
        }
        else {
            servo.setPower(0);
        }
    }

    /**
     * Control del flywheel
     */
    private void controlFlywheel() {

        if (gamepad1.options) {
            // Control manual del flywheel
            flywheel.setPower(-0.5);
        }
        else if (gamepad1.left_bumper) {
            disparoLejano();
        }
        else if (gamepad1.right_bumper) {
            disparoCercano();
        }
        else if (gamepad1.circle) {
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        }
        else if (gamepad1.square) {
            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        }
        else {
            ((DcMotorEx) flywheel).setVelocity(0);
        }
    }

    /**
     * Disparo cercano al objetivo
     */
    private void disparoCercano() {

        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        servo.setPower(-1);

        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    /**
     * Disparo desde más lejos
     */
    private void disparoLejano() {

        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        servo.setPower(-1);

        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }
}