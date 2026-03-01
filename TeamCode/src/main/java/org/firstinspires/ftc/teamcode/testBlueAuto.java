package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

// NEW: Sensor Imports
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Blue Auto")
public class testBlueAuto extends LinearOpMode {

    // --- Hardware declarations ---
    DcMotor bigWheel, intake;
    DcMotorEx shootWheel;
    Servo test;

    // NEW: Sensor declarations
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    // PedroPath Variables
    private Follower follower;
    private double timeLeft = 30 - getRuntime();
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    @Override
    public void runOpMode() {
        // --- INIT PHASE ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.500, 123.400,Math.toRadians(144)));

        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        shootWheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        test = hardwareMap.get(Servo.class, "test");

        // NEW: Hardware map the sensors
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // Motor Directions and Encoders
        shootWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shootWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(50, 0, 0, 12.7);
        shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);

        buildPaths();

        telemetry.addLine("Ready to Start!");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        waitForStart();

        // --- START PHASE ---

        shootWheel.setVelocity(1600);
        sleep(10);
        test.setPosition(0.25);
        sleep(10);
        drive(Path1);
        sleep(10);
        bigWheel.setPower(-1);
        sleep(10);
        intake.setPower(1);
        sleep(2000);

        bigWheel.setPower(0);
        sleep(10);
        intake.setPower(0);
        sleep(500);
        drive(Path2);
        sleep(500);
        follower.setMaxPower(0.6);
        sleep(10);
        intake.setPower(1);
        sleep(10);

        // MODIFIED: Intake power is now -0.33
        bigWheel.setPower(-0.33);
        sleep(10);

        drive(Path3);
        // MODIFIED: smartSleep keeps checking the sensor while waiting
        smartSleep(1800);

        follower.setMaxPower(1);
        sleep(10);
        drive(Path4);
        sleep(500);
        intake.setPower(1);
        sleep(10);
        bigWheel.setPower(-1); // Shooting power
        smartSleep(2000);

        drive(Path5);
        sleep(500);
        follower.setMaxPower(0.6);
        sleep(10);
        intake.setPower(1);
        sleep(10);

        // MODIFIED: Intake power is now -0.33
        bigWheel.setPower(-0.33);
        sleep(10);

        drive(Path6);
        smartSleep(1800);

        follower.setMaxPower(1);
        sleep(10);
        drive(Path9);
        sleep(10);
        drive(Path8);
        sleep(10);
        drive(Path7);
        sleep(10);

        bigWheel.setPower(-1); // Shooting power
        smartSleep(2000);

        bigWheel.setPower(0);
        sleep(10);
        shootWheel.setVelocity(0);
        sleep(10);
        drive(Path10);
        intake.setPower(0);
    }

    // --- HELPER FUNCTIONS ---

    public void drive(PathChain path) {
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // NEW: Actively check the sensor while driving!
            checkSensorAndStopIntake();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    // NEW: Replaces long sleep() calls so PedroPathing and Sensors keep running
    public void smartSleep(long milliseconds) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < milliseconds) {
            follower.update(); // Keeps robot localized accurately
            checkSensorAndStopIntake();
        }
    }

    // NEW: The core sensor logic isolated into a clean method
    public void checkSensorAndStopIntake() {
        // Only trigger the stop logic if we are actively trying to intake (-0.33).
        // This prevents the sensor from accidentally stopping the wheel when we are trying to shoot (-1).
        if (Math.abs(bigWheel.getPower() - (-0.33)) < 0.01) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            boolean isGreen = (hsvValues[0] > 90 && hsvValues[0] < 150 && hsvValues[1] > 0.3);
            boolean isPurple = (hsvValues[0] > 250 && hsvValues[0] < 310 && hsvValues[1] > 0.3);
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            // If a ball is detected, cut the wheel power to 0 immediately
            if (isGreen || isPurple || distanceCm < 2.0) {
                bigWheel.setPower(0);
            }
        }
    }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(19.500, 123.400), new Pose(52.800, 90.500)))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(137))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(52.800, 90.500), new Pose(49.33, 83.59)))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49.330, 83.590), new Pose(17.0, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17.0, 84.000), new Pose(52.800, 90.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(52.800, 90.500), new Pose(52.800, 57.000)))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(52.800, 57.000), new Pose(13.0, 57.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13.0, 57.000), new Pose(24.000, 57.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.000, 57.000), new Pose(52.800, 90.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(52.800, 90.500), new Pose(42.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(250))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.000, 75.000), new Pose(36.8, 68.200)))
                .setConstantHeadingInterpolation(Math.toRadians(137))
                .build();
    }
}