package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MotorValues")

public class MotorValues extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();
    private ElapsedTime headingFixTimer = new ElapsedTime();

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        drivetrain.initEncoders();

        clawButtonTimer.reset();
        dPadTimer.reset();
        boolean clawButton = false;

        double speedScale = 0.8;

        ILC.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ILC.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
            int FL = drivetrain.frontLeftDrive.getCurrentPosition();
            int BL = drivetrain.backLeftDrive.getCurrentPosition();
            int FR = drivetrain.frontRightDrive.getCurrentPosition();
            int BR = drivetrain.backRightDrive.getCurrentPosition();
            
            int RS = ILC.rightSlide.getCurrentPosition();
            int LS = ILC.leftSlide.getCurrentPosition();
            
            telemetry.addData("front left", FL);
            telemetry.addData("back left", BL);
            telemetry.addData("front right", FR);
            telemetry.addData("back right", BR);

            telemetry.addData("lifty lifty right", RS);
            telemetry.addData("lifty lifty left", LS);
        }
    }
}

