package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor; 
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="StarterBot TeleOp + Auto Mejorado")
public class REVStarterBotTeleOpAutoJava extends LinearOpMode {

    // Motores y servo
    private DcMotor flywheel;
    private DcMotor coreHex;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private CRServo servo;

    // Velocidades del lanzador
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    // Modos
    private static final String TELEOP = "TELEOP";
    private static final String AUTO_BLUE = "AUTO BLUE";
    private static final String AUTO_RED = "AUTO RED";

    private String operationSelected = TELEOP;

    // Conversión pulgadas a ticks
    private static final double WHEELS_INCHES_TO_TICKS = (28.0 * 5.0 * 3.0) / (3.0 * Math.PI);

    private ElapsedTime autoLaunchTimer = new ElapsedTime();
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    // Control de velocidad
    double driveSpeed = 0.7;

    // Aceleración suave
    double lastLeftPower = 0;
    double lastRightPower = 0;
    double accelLimit = 0.05;

    @Override
    public void runOpMode() {

        // Conectar hardware
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        servo = hardwareMap.get(CRServo.class, "servo");

        // Configuración motores
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo.setPower(0);

        // Selección de modo antes de iniciar
        while (opModeInInit()) {

            operationSelected = selectOperation(operationSelected, gamepad1.y);

            telemetry.addLine("Presiona Y para cambiar modo");
            telemetry.addData("Modo seleccionado", operationSelected);
            telemetry.addLine("Presiona START para comenzar");
            telemetry.update();
        }

        waitForStart();

        if(operationSelected.equals(AUTO_BLUE)){
            doAutoBlue();
        }
        else if(operationSelected.equals(AUTO_RED)){
            doAutoRed();
        }
        else{
            doTeleOp();
        }
    }

    // TELEOP
    private void doTeleOp(){

        while(opModeIsActive()){

            splitStickArcadeDrive();
            setFlywheelVelocity();
            manualCoreHexAndServoControl();

            if(gamepad1.a){
                shootThreeRings();
            }

            telemetry.addData("Flywheel Velocity", ((DcMotorEx)flywheel).getVelocity());
            telemetry.addData("Left Encoder", leftDrive.getCurrentPosition());
            telemetry.addData("Right Encoder", rightDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    // CONTROL DE MOVIMIENTO
    private void splitStickArcadeDrive(){

        if(gamepad1.left_trigger > 0.5){
            driveSpeed = 0.4;
        }
        else if(gamepad1.right_trigger > 0.5){
            driveSpeed = 1.0;
        }
        else{
            driveSpeed = 0.7;
        }

        double x = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;

        double leftPower = (y - x) * driveSpeed;
        double rightPower = (y + x) * driveSpeed;

        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        leftPower = lastLeftPower + Math.max(-accelLimit, Math.min(accelLimit, leftPower - lastLeftPower));
        rightPower = lastRightPower + Math.max(-accelLimit, Math.min(accelLimit, rightPower - lastRightPower));

        lastLeftPower = leftPower;
        lastRightPower = rightPower;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    // CONTROL COREHEX Y SERVO
    private void manualCoreHexAndServoControl(){

        if(gamepad1.cross){
            coreHex.setPower(0.5);
        }
        else if(gamepad1.triangle){
            coreHex.setPower(-0.5);
        }
        else{
            coreHex.setPower(0);
        }

        if(gamepad1.dpad_left){
            servo.setPower(1);
        }
        else if(gamepad1.dpad_right){
            servo.setPower(-1);
        }
        else{
            servo.setPower(0);
        }
    }

    // CONTROL FLYWHEEL
    private void setFlywheelVelocity(){

        if(gamepad1.options){
            flywheel.setPower(-0.5);
        }
        else if(gamepad1.left_bumper){
            FAR_POWER_AUTO();
        }
        else if(gamepad1.right_bumper){
            BANK_SHOT_AUTO();
        }
        else if(gamepad1.circle){
            ((DcMotorEx)flywheel).setVelocity(bankVelocity);
        }
        else if(gamepad1.square){
            ((DcMotorEx)flywheel).setVelocity(maxVelocity);
        }
        else{
            ((DcMotorEx)flywheel).setVelocity(0);
        }
    }

    // DISPARO AUTOMÁTICO
    private void shootThreeRings(){

        for(int i=0;i<3;i++){

            servo.setPower(-1);
            sleep(400);

            coreHex.setPower(1);
            sleep(500);

            coreHex.setPower(0);
            sleep(400);
        }

        servo.setPower(0);
    }

    // MODOS AUTOMÁTICOS DEL FLYWHEEL
    private void BANK_SHOT_AUTO(){

        ((DcMotorEx)flywheel).setVelocity(bankVelocity);

        servo.setPower(-1);

        if(((DcMotorEx)flywheel).getVelocity() >= bankVelocity-100){
            coreHex.setPower(1);
        }
        else{
            coreHex.setPower(0);
        }
    }

    private void FAR_POWER_AUTO(){

        ((DcMotorEx)flywheel).setVelocity(farVelocity);

        servo.setPower(-1);

        if(((DcMotorEx)flywheel).getVelocity() >= farVelocity-100){
            coreHex.setPower(1);
        }
        else{
            coreHex.setPower(0);
        }
    }

    // AUTO DRIVE
    private void autoDrive(double speed,int leftDistanceInch,int rightDistanceInch,int timeout_ms){

        autoDriveTimer.reset();

        leftDrive.setTargetPosition((int)(leftDrive.getCurrentPosition()+leftDistanceInch*WHEELS_INCHES_TO_TICKS));
        rightDrive.setTargetPosition((int)(rightDrive.getCurrentPosition()+rightDistanceInch*WHEELS_INCHES_TO_TICKS));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms){
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // AUTO BLUE
    private void doAutoBlue(){

        autoLaunchTimer.reset();

        while(opModeIsActive() && autoLaunchTimer.milliseconds()<10000){

            BANK_SHOT_AUTO();

            telemetry.addData("Launcher",autoLaunchTimer.seconds());
            telemetry.update();
        }

        ((DcMotorEx)flywheel).setVelocity(0);
        coreHex.setPower(0);
        servo.setPower(0);

        autoDrive(0.5,-12,-12,5000);
        autoDrive(0.5,-8,8,5000);
        autoDrive(1,-50,-50,5000);
    }

    // AUTO RED
    private void doAutoRed(){

        autoLaunchTimer.reset();

        while(opModeIsActive() && autoLaunchTimer.milliseconds()<10000){

            BANK_SHOT_AUTO();

            telemetry.addData("Launcher",autoLaunchTimer.seconds());
            telemetry.update();
        }

        ((DcMotorEx)flywheel).setVelocity(0);
        coreHex.setPower(0);
        servo.setPower(0);

        autoDrive(0.5,-12,-12,5000);
        autoDrive(0.5,8,-8,5000);
        autoDrive(1,-50,-50,5000);
    }

    // SELECCIÓN DE MODO
    private String selectOperation(String state, boolean cycleNext){

        if(cycleNext){

            if(state.equals(TELEOP)){
                state = AUTO_BLUE;
            }
            else if(state.equals(AUTO_BLUE)){
                state = AUTO_RED;
            }
            else{
                state = TELEOP;
            }
        }

        return state;
    }
}