package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class pidfTuning extends OpMode {

    DcMotorEx flywheel;
    DcMotor bigWheel, intake;
    Servo test;



    // Theoretical Maximum RPM for a Gobilda 6000RPM motor is ~6000
    // In ticks per second: 6000 RPM / 60 sec * 28 ticks/rev = ~2800 ticks/sec
    double targetVelocity = 2000;


    // Start with F = 32767 / MaxVelocity (approx 11-12 for this motor)
    // Start with P = 10% of F
    public static double F = 13.0;
    public static double P = 52.0;
    public static double I = 0;
    public static double D = 0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shootWheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
        test = hardwareMap.get(Servo.class, "test");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        // CRITICAL: Reset the mode to clear old settings
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Allow driver to adjust F and P on the fly
        if(gamepad1.dpad_up) F += 0.1;
        if(gamepad1.dpad_down) F -= 0.1;
        if(gamepad1.dpad_right) P += 0.1;
        if(gamepad1.dpad_left) P -= 0.1;

        // Apply the new PIDF coefficients
        PIDFCoefficients pidfOrig = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // We only update if the values changed to save bandwidth
        if(pidfOrig.f != F || pidfOrig.p != P) {
            PIDFCoefficients newPIDF = new PIDFCoefficients(P, I, D, F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        }

        // Run the motor
        flywheel.setVelocity(targetVelocity);

        // Update tracking variable for the next loop iteration

        test.setPosition(0.15);


        if (gamepad1.b) {
            bigWheel.setPower(1);
        } else if (gamepad1.a) {
            bigWheel.setPower(-0.3);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1);
            bigWheel.setPower(-1);
        } else {
            bigWheel.setPower(0);
        }
        // TELEMETRY IS KEY HERE
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
        telemetry.addData("F (Feedforward)", F);
        telemetry.addData("P (Proportional)", P);
        telemetry.update();
    }
}

