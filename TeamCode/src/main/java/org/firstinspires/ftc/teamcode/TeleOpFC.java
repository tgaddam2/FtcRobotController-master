package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOpFC")

public class TeleOpFC extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();
    private ElapsedTime headingFixTimer = new ElapsedTime();

    boolean clawChanging = false;

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        IMU imu;
        IMU.Parameters parameters;
        YawPitchRollAngles angles;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        drivetrain.initEncoders();

        clawButtonTimer.reset();
        dPadTimer.reset();

        boolean clawButton = false;
        boolean clawIsClosed = true;
        double finalArmPosition = 0;

        boolean bottomClawOpen = false;

        double speedScale = 0.8;

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
            // Control Robot Movement
            angles = imu.getRobotYawPitchRollAngles();
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 - angles.getYaw(AngleUnit.RADIANS);
            double rightX = gamepad1.right_stick_x;
            final double FLPower = speedScale * (r * Math.sin(driveAngle) + rightX);
            final double BLPower = speedScale * (r * Math.cos(driveAngle) + rightX);
            final double FRPower = speedScale * (r * Math.cos(driveAngle) - rightX);
            final double BRPower = speedScale * (r * Math.sin(driveAngle) - rightX);

            drivetrain.frontLeftDrive.setPower(FLPower);
            drivetrain.backLeftDrive.setPower(BLPower);
            drivetrain.frontRightDrive.setPower(FRPower);
            drivetrain.backRightDrive.setPower(BRPower);

            if(gamepad1.left_trigger > 0.75) {
                speedScale = 0.7;

                drivetrain.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else {
                speedScale = 0.4;

                drivetrain.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // fix robot zero if needed
            if(gamepad1.y && headingFixTimer.milliseconds() >= 500) {
                headingFixTimer.reset();
                drivetrain.imu.resetYaw();
            }
            
//             if(!clawChanging) {


             if(clawIsClosed && gamepad2.dpad_up) {
                 ILC.dPadMove("up");
             }
             else if (clawIsClosed && gamepad2.dpad_down) {
                 ILC.dPadMove("down");
             }

             if(gamepad2.left_stick_button) {
                 ILC.undeployPush();
             }
             else if (gamepad2.right_stick_button) {
                 ILC.deployPush();
             }

             if(gamepad2.b) {
                 ILC.setArmGrab();
             }

             if(gamepad2.right_bumper) {
//                 ILC.openClaw();
//                 clawIsClosed = false;

                 if(bottomClawOpen) {
                     ILC.openTopClaw();
                 }

                 ILC.openBottomClaw();
                 bottomClawOpen = true;
             }
             else if(gamepad2.left_bumper) {
                 ILC.closeClaw();
                 clawIsClosed = true;
             }

             if(clawIsClosed && gamepad2.y) {
                 clawChanging = true;
                 finalArmPosition = 0.65;
                 ILC.changeClawPos("out");
             }
             else if(gamepad2.a) {
                 clawChanging = true;
                 finalArmPosition = 0.9;
                 ILC.changeClawPos("in");
                 clawIsClosed = false;
                 bottomClawOpen = false;
             }
//             }

            if(ILC.armServo.getPosition() == finalArmPosition) {
                clawChanging = false;
            }

            // if(gamepad1.y && headingFixTimer.milliseconds() >= 500) {
            //     clawButtonTimer.reset();
            //     drivetrain.imu.resetYaw();
            // }
        }
    }
}

