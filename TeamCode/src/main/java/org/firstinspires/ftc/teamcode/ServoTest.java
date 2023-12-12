package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTest")

public class ServoTest extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    ElapsedTime sleepTimer;

    int newLeftArmPos;
    int newRightArmPos;

    @Override public void runOpMode() throws InterruptedException {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        drivetrain.initEncoders();

        //double armPos = 0;
        double topClawPos = 0.5;
        double bottomClawPos = 0.5;
        double pushPos = 0;

        //ILC.armServo.setPosition(armPos);
        ILC.topClawServo.setPosition(topClawPos);
        ILC.bottomClawServo.setPosition(bottomClawPos);
        ILC.pushServo.setPosition(pushPos);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // if(gamepad1.dpad_up) {
            //     armPos += 0.1;
            // }
            // else if(gamepad1.dpad_down) {
            //     armPos -= 0.1;
            // }

            // if(gamepad1.dpad_right) {
            //     clawPos += 0.1;
            // }
            // else if(gamepad1.dpad_left) {
            //     clawPos -= 0.1;
            // }

            // if(gamepad1.right_bumper) {
            //     intakePos += 0.1;
            // }
            // else if(gamepad1.left_bumper) {
            //     intakePos -= 0.1;
            // }

            if (gamepad1.dpad_up) {
                double current = ILC.armServo.getPosition();
                current += 0.01;
                ILC.armServo.setPosition(current);
                // wait(100);
                sleep(100);
            }
            else if (gamepad1.dpad_down) {
                double current = ILC.armServo.getPosition();
                current -= 0.01;
                ILC.armServo.setPosition(current);
                // wait(100);
                sleep(100);
            }

            if (gamepad1.dpad_right) {
                double current = ILC.topClawServo.getPosition();
                current += 0.05;
                ILC.topClawServo.setPosition(current);
                // wait(100);
                sleep(100);
            } 
            else if (gamepad1.dpad_left) {
                double current = ILC.topClawServo.getPosition();
                current -= 0.05;
                ILC.topClawServo.setPosition(current);
                // wait(100);
                sleep(100);
            }

            if (gamepad1.b) {
                double current = ILC.bottomClawServo.getPosition();
                current += 0.05;
                ILC.bottomClawServo.setPosition(current);
                // wait(100);
                sleep(100);
            }
            else if (gamepad1.x) {
                double current = ILC.bottomClawServo.getPosition();
                current -= 0.05;
                ILC.bottomClawServo.setPosition(current);
                // wait(100);
                sleep(100);
            }

            if (gamepad1.right_bumper) {
                double current = ILC.pushServo.getPosition();
                current += 0.05;
                ILC.pushServo.setPosition(current);
                // wait(100);
                sleep(100);
            } 
            else if (gamepad1.left_bumper) {
                double current = ILC.pushServo.getPosition();
                current -= 0.05;
                ILC.pushServo.setPosition(current);
                // wait(100);
                sleep(100);
            }

            // ILC.armServo.setPosition(armPos);
            // ILC.clawServo.setPosition(clawPos);
            // ILC.intakeServo.setPosition(intakePos);

            telemetry.addData("arm position", ILC.armServo.getPosition());
            telemetry.addData("top claw position", ILC.topClawServo.getPosition());
            telemetry.addData("bottom claw position", ILC.bottomClawServo.getPosition());
            telemetry.addData("push position", ILC.pushServo.getPosition());

            telemetry.update();
        }
    }
}

