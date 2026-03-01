package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Orange Teleop")


public class orangeTeleop extends LinearOpMode {

    // --- Hardware declarations ---
    DcMotor front_left, front_right, back_left, back_right, bigWheel, intake;
    DcMotorEx shootWheel;
    Servo test;
    boolean isShooting = false;
    boolean isMoving = false;
    boolean wasAPressed = false;
    boolean lastButtonState = false;
    int servoState = 0;
    boolean lastRBumperState = false;
    boolean lastRTriggerState = false;
    boolean lastYButtonState = false;
    int shooterState = 0; // 0 = Off, 1 = Low Speed (1450), 2 = High Speed (1800)

    @Override
    public void runOpMode() {

        // --- 1) Hardware mapping ---
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");
        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        shootWheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        test = hardwareMap.get(Servo.class, "test");


        // --- Motor directions ---
        front_left.setDirection(DcMotor.Direction.REVERSE);
        bigWheel.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        shootWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE); // change later


        // --- Run modes & zero power behavior ---
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(50, 0, 0, 13.0);
        shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        waitForStart();

        // --- Main control loop ---
        while (opModeIsActive()) {

            //Declaring f/b-y strafe-x and rotation-rx
            double y  = 0; // forward/backward
            double x  = 0; // left/right
            double rx = 0; // rotation

            //Gamepad Code
            y  = -gamepad1.left_stick_y;  // forward/backward
            x  =  gamepad1.left_stick_x;  // strafe left/right
            rx =  gamepad1.right_stick_x; // turn left/right

            if (gamepad2.b) {
                bigWheel.setPower(1);
            } else if (gamepad2.a) {
                bigWheel.setPower(-0.3);
            } else if (gamepad2.left_bumper) {
                bigWheel.setPower(-1);
            } else {
                bigWheel.setPower(0);
            }




            // Logic for Right Trigger (1450)
            boolean currentRTrigger = gamepad2.right_trigger > 0.2;
            if (currentRTrigger && !lastRTriggerState) {
                if (shooterState == 1) shooterState = 0; // Toggle off if already on
                else shooterState = 1;                   // Switch to Low Speed
            }
            lastRTriggerState = currentRTrigger;

            // Logic for Right Bumper (1800)
            boolean currentRBumper = gamepad2.right_bumper;
            if (currentRBumper && !lastRBumperState) {
                if (shooterState == 2) shooterState = 0; // Toggle off if already on
                else shooterState = 2;                   // Switch to High Speed
            }
            lastRBumperState = currentRBumper;

            boolean currentYButton =gamepad2.y;
            if(currentYButton && !lastYButtonState){
                if(shooterState ==2) shooterState =0;
                else shooterState=3;
            }
            lastYButtonState = currentYButton;

            // Apply the states to the motor
            if (shooterState == 1) {
                tunedPIDF = new PIDFCoefficients(42, 0, 0, 12.0);
                shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);
                test.setPosition(0.40);
                sleep(50);
                shootWheel.setVelocity(1300);
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
            }

            else {
                shootWheel.setVelocity(0);
            }


            //   Mecanum drive math ---
            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            // --- Normalize powers -
            double max = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))
            ));
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;

            // --- Send power to motors ---
            front_left.setPower(frontLeftPower);
            back_left.setPower(backLeftPower);
            front_right.setPower(frontRightPower);
            back_right.setPower(backRightPower);



            // Logic for the Leading-Edge Toggle
            // This tracks the button's previous state and only toggles once on the press.
            if (gamepad2.x && !wasAPressed) {
                // Toggle the state
                isMoving = !isMoving;
            }

            // Update tracking variable for the next loop iteration
            wasAPressed = gamepad2.x;

            // Motor Power Control
            if(isMoving){
                intake.setPower(1);
            } else {
                intake.setPower(0);}

            if (gamepad2.dpad_down){
                intake.setPower(-1);
            }


            telemetry.addData("speed", shootWheel.getVelocity());
            telemetry.update();

            idle(); // yields CPU briefly so Sim stays responsive
        }
    }

}
