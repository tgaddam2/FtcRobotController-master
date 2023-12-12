    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

    public class IntakeLiftCamera {
    public DcMotor rightSlide;
    public DcMotor leftSlide;

    public Servo topClawServo;
    public double topClawClose = 0.7;
    public double topClawClaw = 0.6; // manual
    public double topClawOpen = 0.50; // manual

    public Servo bottomClawServo;
    public double bottomClawClose = 0.6;
    public double bottomClawClaw = 0.58; // manual
    public double bottomClawOpen = 0.5; // manual
    public double bottomClawOpenOpen = 0.5; // manual

    public Servo pushServo;
    public double pushPush = 1;
    public double pushUnpush = 0.25;

    public Servo armServo;

    public double armArm = 0.60;
    public double armClearance = 0.5;
    public double armScore = 0.9;

    public WebcamName webcam;

    CameraBlueOrange camera;

    double slideSpeed = 0.6;
    
    // [right, left]
    int[] slideGrabPosition = {5, 5};
    int[] slideClearPositionUp = {325, 325};
    int[] slideClearPositionDown = {600, 600};
    int[] slideScorePosition = {2000, 2000};

    int[] maxPositions = {4200, 4200}; // potentially 4300

    int[] minPositions = {0, 0};

    LinearOpMode opMode;

    ElapsedTime sleepTimer;

    CameraBlueOrange.SEDPipeline pipeline = new CameraBlueOrange.SEDPipeline();

    public IntakeLiftCamera(LinearOpMode op) {
        opMode = op;
    }

    public String getSignalPos() {
        String position = camera.getStringPosition();
        return position;
    }

    public void closeClaw() {
        topClawServo.setPosition(topClawClaw);
        bottomClawServo.setPosition(bottomClawClaw);
        //fixThisCode();
    }

    public void openClaw() {
        bottomClawServo.setPosition(bottomClawOpen);
        topClawServo.setPosition(topClawOpen);
        //fixThisCode();
    }

    public void openTopClaw() {
        topClawServo.setPosition(topClawOpen);
    }

    public void openBottomClaw() {
        bottomClawServo.setPosition(bottomClawOpen);
    }

    public void setArmGrab() {
        armServo.setPosition(armArm);
    }


    public void deployPush() {
        pushServo.setPosition(pushPush);
    }

    public void undeployPush() {
        pushServo.setPosition(pushUnpush);
    }

    public void dPadMove(String direction) {
        int rightPos = rightSlide.getCurrentPosition();
        int leftPos = leftSlide.getCurrentPosition();

        if(direction.equals("up") && rightPos <= maxPositions[0] && leftPos <= maxPositions[1]) {
            rightPos += 100;
            leftPos += 100;
        }
        else if(direction.equals("down") && rightPos >= minPositions[0] && leftPos >= minPositions[1]) {
            rightPos -= 100;
            leftPos -= 100;
        }

        rightSlide.setTargetPosition(rightPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(slideSpeed);

        leftSlide.setTargetPosition(leftPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(slideSpeed);
    }

    public void initIntakeLiftCamera(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        rightSlide = hwMap.get(DcMotor.class, "RightSlide");
        leftSlide = hwMap.get(DcMotor.class, "LeftSlide");

        // Define and Initialize Servos
        topClawServo = hwMap.get(Servo.class, "topClawServo");
        bottomClawServo = hwMap.get(Servo.class, "bottomClawServo");
        armServo = hwMap.get(Servo.class, "armServo");
        pushServo = hwMap.get(Servo.class, "pushServo");

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setPower(0);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setPower(0);

        armServo.setPosition(armArm);
        topClawServo.setPosition(topClawClaw);
        bottomClawServo.setPosition(bottomClawClaw);
    }

    public void encoderMove(int[] encoderPos, double runtimeMilli) {
        ElapsedTime runtimeTimer = new ElapsedTime();
        runtimeTimer.startTime();

        rightSlide.setTargetPosition(encoderPos[0]);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(slideSpeed);

        leftSlide.setTargetPosition(encoderPos[1]);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(slideSpeed);

        while(leftSlide.isBusy() && rightSlide.isBusy() & runtimeTimer.milliseconds() <= runtimeMilli) {

        }

        rightSlide.setPower(0);
        leftSlide.setPower(0);
    }

    public void changeClawPos(String position) {
        //fixAllThisCode();
        if(position.equals("out")) {
            topClawServo.setPosition(topClawClaw);
            bottomClawServo.setPosition(bottomClawClaw);

            pushServo.setPosition(pushPush);

            encoderMove(slideClearPositionUp, 5000);

            armServo.setPosition(armClearance);

            encoderMove(slideScorePosition, 5000);

            armServo.setPosition(armScore);
            pushServo.setPosition(pushUnpush);
        }
        else if(position.equals("in")) {
            topClawServo.setPosition(topClawClose);
            bottomClawServo.setPosition(bottomClawClose);

            encoderMove(slideScorePosition, 5000);

//            topClawServo.setPosition(topClawClose);
//            bottomClawServo.setPosition(bottomClawClose);

//            pushServo.setPosition(pushPush);

            armServo.setPosition(armClearance);

            encoderMove(slideClearPositionUp, 5000);

            armServo.setPosition(armClearance);

            encoderMove(slideGrabPosition, 3000);

            armServo.setPosition(armArm);

            wait(500);

            bottomClawServo.setPosition(bottomClawOpen);
            topClawServo.setPosition(topClawOpen);
        }

        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initCameraBlueOrange(HardwareMap hwMap) {
        webcam = hwMap.get(WebcamName.class, "Webcam 1");

        camera = new CameraBlueOrange(opMode.hardwareMap);
        camera.initCamera();
    }

    public void wait(int milli) {
        sleepTimer.reset();
        sleepTimer.startTime();
        while(sleepTimer.milliseconds() < milli) {
            if(opMode.isStopRequested()) {
                break;
            }
        }
    }
}
