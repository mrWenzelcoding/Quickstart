package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Orange Teleop Test 2")
public class orangeTeleopTest2 extends LinearOpMode {

    // --- Hardware declarations ---
    DcMotor front_left, front_right, back_left, back_right, bigWheel, intake;
    DcMotorEx shootWheel;
    Servo test;

    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    boolean isShooting = false;
    boolean isMoving = false;
    boolean wasXPressed = false;
    boolean lastButtonState = false;
    int servoState = 0;
    boolean lastRTriggerState = false;
    boolean lastRBumperState = false;
    boolean lastYButtonState = false;
    int shooterState = 0;

    // The "Latch" variable
    boolean artifactSecured = false;

    // NEW: Variables to track the previous state of the override buttons
    boolean lastLeftBumper = false;
    boolean lastAButton = false;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");
        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        shootWheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        test = hardwareMap.get(Servo.class, "test");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // --- Motor directions ---
        front_left.setDirection(DcMotor.Direction.REVERSE);
        bigWheel.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        shootWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // --- Run modes & zero power behavior ---
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(50, 0, 0, 13.0);
        shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        waitForStart();

        // --- Main control loop ---
        while (opModeIsActive()) {

            // --- Capture Current Button States ---
            boolean currentLeftBumper = gamepad2.left_bumper;
            boolean currentAButton = gamepad2.a;

            // --- Sensor Math (Color + Distance) ---
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];

            boolean isGreen = (hue > 130 && hue < 165 && saturation > 0.3);
            boolean isPurple = (hue > 220 && hue < 250 && saturation > 0.3);
            boolean artifactDetected = isGreen || isPurple;

            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            boolean isClose = distanceCm < 2.0;

            // --- The "Latch" Logic ---
            // 1. Lock the system if a game piece is detected
            if (artifactDetected || isClose) {
                artifactSecured = true;
            }

            // 2. MODIFIED: Un-latch the system ONLY when the override buttons are released
            // True if the button is currently false (released) but was true (pressed) in the last loop
            boolean leftBumperReleased = !currentLeftBumper && lastLeftBumper;
            boolean aButtonReleased = !currentAButton && lastAButton;

            if (leftBumperReleased || aButtonReleased) {
                artifactSecured = false;
            }

            // --- Drive Math ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))
            ));

            front_left.setPower(frontLeftPower / max);
            back_left.setPower(backLeftPower / max);
            front_right.setPower(frontRightPower / max);
            back_right.setPower(backRightPower / max);

            // --- Shooter Logic ---
            boolean currentRTrigger = gamepad2.right_trigger > 0.2;
            if (currentRTrigger && !lastRTriggerState) {
                if (shooterState == 1) shooterState = 0;
                else shooterState = 1;
            }
            lastRTriggerState = currentRTrigger;

            boolean currentRBumper = gamepad2.right_bumper;
            if (currentRBumper && !lastRBumperState) {
                if (shooterState == 2) shooterState = 0;
                else shooterState = 2;
            }
            lastRBumperState = currentRBumper;

            boolean currentYButton = gamepad2.y;
            if(currentYButton && !lastYButtonState){
                if(shooterState == 2) shooterState = 0;
                else shooterState = 3;
            }
            lastYButtonState = currentYButton;

            if (shooterState == 1) {
                tunedPIDF = new PIDFCoefficients(42, 0, 0, 12.0);
                shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
                test.setPosition(0.40);
                sleep(50);
                shootWheel.setVelocity(1325);
            } else if (shooterState == 2) {
                tunedPIDF = new PIDFCoefficients(50, 0, 0, 13.0);
                shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
                test.setPosition(0.30);
                sleep(50);
                shootWheel.setVelocity(1600);
            } else if (shooterState == 3) {
                tunedPIDF = new PIDFCoefficients(70, 0, 0, 13.0);
                shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
                test.setPosition(0.2);
                sleep(50);
                shootWheel.setVelocity(1950);
            } else {
                shootWheel.setVelocity(0);
            }

            // --- Intake & Big Wheel Unified Logic ---
            if (gamepad2.x && !wasXPressed) {
                isMoving = !isMoving;
            }
            wasXPressed = gamepad2.x;

            // 1) Intake Control
            if (gamepad2.dpad_down) {
                intake.setPower(-1);
            } else if (isMoving) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // 2 & 3) Big Wheel Control
            if (currentLeftBumper) {
                bigWheel.setPower(-1);
            } else if (currentAButton) {
                bigWheel.setPower(-0.3);
            } else if (gamepad2.b) {
                bigWheel.setPower(1);
            } else if (isMoving) {
                if (artifactSecured) {
                    bigWheel.setPower(0);
                } else {
                    bigWheel.setPower(-0.33);
                }
            } else {
                bigWheel.setPower(0);
            }

            // --- Update Last Button States for the Next Loop ---
            // NEW: Store the current state to act as the "previous" state in the next millisecond
            lastLeftBumper = currentLeftBumper;
            lastAButton = currentAButton;

            // --- Telemetry ---
            telemetry.addData("Shooter Speed", shootWheel.getVelocity());
            telemetry.addData("Sensor Hue", hue);
            telemetry.addData("Distance (cm)", distanceCm);
            telemetry.addData("Artifact Secured (Latched)", artifactSecured);
            telemetry.update();

            idle();
        }
    }
}