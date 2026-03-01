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

@Autonomous(name = "Blue Auto Back")
public class blueAutoBack extends LinearOpMode {

    // --- Hardware declarations ---
    DcMotor bigWheel, intake;
    DcMotorEx shootWheel;
    Servo test;


    //PedroPath Variables
    private Follower follower;
    private double timeLeft = 30 - getRuntime();
    private PathChain Path1, Path2, Path3, Path4;

    @Override
    public void runOpMode() {
        //  INIT PHASE
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55.00, 9.790,Math.toRadians(110)));
        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        shootWheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        test = hardwareMap.get(Servo.class, "test");

        //Motor Directions and Encoders
        shootWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shootWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients tunedPIDF = new PIDFCoefficients(200, 0, 0, 13.0);
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

        test.setPosition(0.15);
        sleep(10);
        shootWheel.setVelocity(1900);
        sleep(2000);
        bigWheel.setPower(-0.5);
        sleep(10);
        intake.setPower(0.8);
        sleep(5000);
        bigWheel.setPower(0);
        sleep(10);
        intake.setPower(0);
        sleep(50);
        drive(Path1);
        sleep(10);
        intake.setPower(1);
        sleep(10);
        bigWheel.setPower(-0.065);
        sleep(10);
        follower.setMaxPower(0.6);
        sleep(10);
        drive(Path2);
        sleep(2200);
        bigWheel.setPower(0);
        sleep(10);
        intake.setPower(0);
        sleep(10);
        drive(Path3);
        sleep(10);
        intake.setPower(0.8);
        sleep(10);
        bigWheel.setPower(-0.5);
        sleep(4500);
        bigWheel.setPower(0);
        sleep(10);
        intake.setPower(0);
        sleep(10);
        shootWheel.setVelocity(0);
        sleep(10);
        drive(Path4);
        sleep(10);



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

    // A simple function to handle your shooter logic
//    public void shootBall() {
//        flywheelMotor.setVelocity(2000);
//
//        // Sleep is okay in LinearOpMode!
//        // We wait 1.5 seconds for the shot to happen.
//        sleep(1500);
//
//        flywheelMotor.setVelocity(0);
//    }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.000, 9.790),
                                new Pose(50.500, 35.900)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.500, 35.900),
                                new Pose(10.22, 35.9)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.220, 35.90),
                                new Pose(56.0, 9.600)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.0, 9.600),
                                new Pose(47.400, 19.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(195))
                .build();


    }

}
