package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // <--- Changing to LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto")
public class redAuto extends LinearOpMode {

    // --- Hardware declarations ---
    DcMotor bigWheel, intake;
    DcMotorEx shootWheel;
    Servo test;


    //PedroPath Variables
    private Follower follower;
    private double timeLeft = 30 - getRuntime();
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    @Override
    public void runOpMode() {
        //  INIT PHASE
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-19.500, 123.400,Math.toRadians(180-144)));
        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        shootWheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        test = hardwareMap.get(Servo.class, "test");

        //Motor Directions and Encoders
        shootWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shootWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(70, 0, 0, 12.7);
        shootWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tunedPIDF);



        //flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        buildPaths(); // Build the paths just like before

        telemetry.addLine("Ready to Start!");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        waitForStart(); // Waits here until you press PLAY

        // --- 2. START PHASE (The Linear Recipe) ---

        // Step 1: Drive to the shooting spot

        shootWheel.setVelocity(1550);
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
        bigWheel.setPower(-0.05);
        sleep(10);
        drive(Path3);
        sleep(1800);
        follower.setMaxPower(1);
        sleep(10);
        drive(Path4);
        sleep(500);
        intake.setPower(1);
        sleep(10);
        bigWheel.setPower(-1);
        sleep(2000);
        drive(Path5);
        sleep(500);
        follower.setMaxPower(0.6);
        sleep(10);
        intake.setPower(1);
        sleep(10);
        bigWheel.setPower(-0.05);
        sleep(10);
        drive(Path6);
        sleep(1800);
        follower.setMaxPower(1);
        sleep(10);
        drive(Path9);
        sleep(10);
        drive(Path8);
        sleep(10);
        drive(Path7);
        sleep(10);
        bigWheel.setPower(-1);
        sleep(2000);
        bigWheel.setPower(0);
        sleep(10);
        shootWheel.setVelocity(0);
        sleep(10);
        drive(Path10);
        intake.setPower(0);

    }

    // --- HELPER FUNCTIONS (The Magic) ---

    // This function handles the "Busy Work" so you don't have to see it in the main code
    public void drive(PathChain path) {
        follower.followPath(path, true);

        // This loop freezes the code here until the robot is done driving
        while (opModeIsActive() && follower.isBusy()) {
            follower.update(); // CRITICAL: This does the math!

            // You can add telemetry here if you want to see position
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }


    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-19.500, 123.400),
                                new Pose(144-52.800, 90.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-144), Math.toRadians(180-137))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-52.800, 90.500),
                                new Pose(144-49.33, 83.59)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-137), Math.toRadians(180-180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-49.330, 83.590),
                                new Pose(144-17.0, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-17.0, 84.000),
                                new Pose(144-44.800, 90.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-137))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-52.800, 90.500),
                                new Pose(144-52.800, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-137), Math.toRadians(180-180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-52.800, 60.000),
                                new Pose(144-13.0, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-13.0, 60.000),
                                new Pose(144-24.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-24.000, 60.000),
                                new Pose(144-52.800, 90.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-137))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-52.800, 90.500),
                                new Pose(144-42.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-137), Math.toRadians(180-250))
                .build();


        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-42.000, 75.000),
                                new Pose(144-36.8, 68.200)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-137))
                .build();




    }

}
