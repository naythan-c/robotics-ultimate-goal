package org.firstinspires.ftc.teamcode.archived;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="testAuto", group="Autonomous")
@Disabled
public class TestAuto extends LinearOpMode {

    //Declare motors/servos variables
    private ElapsedTime runtime = new ElapsedTime();
    //Initialize Motors/Servos
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DcMotor clawArm = null;
    private Servo clawServo = null;
    double clawClose = 0.92;
    double clawOpen = 0.6;

    private DcMotor shooter = null;
//    private Servo kicker = null;
    double kickerInit = 0.2;
    double kickerTo = 0.56;
    private Servo shootFlap;
    double flapAngle = 0.06; //Higher = Steeper

//    private Servo leftLift = null;
//    private Servo rightLift = null;

    double leftLiftUp = 1 - 0.92; //0 Top
    double rightLiftUp = 0.89; //1 Top



    //NEED TO FIND THESE NUMBERS. LEFT AT DEFAULT FOR NOW
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //Vision
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {

        //Hardware Maps
        //Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        //Servos
        shooter = hardwareMap.get(DcMotor.class, "shooter");
//        kicker = hardwareMap.get(Servo.class, "kicker");
//        kicker.setPosition(kickerInit);
        shootFlap = hardwareMap.get(Servo.class, "shootFlap");
        shootFlap.setPosition(flapAngle);

//        leftLift = hardwareMap.get(Servo.class, "leftLift");
//        rightLift = hardwareMap.get(Servo.class, "rightLift");
//
//
//        leftLift.setPosition(leftLiftUp);
//        rightLift.setPosition(rightLiftUp);

        //Set motor run modes
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        //Claw Arm
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(clawOpen);


//        int cameraMonitorViewId = this
//                .hardwareMap
//                .appContext
//                .getResources().getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                );
////        if (USING_WEBCAM) {
//        camera = OpenCvCameraFactory
//                .getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
////        } else {
////            camera = OpenCvCameraFactory
////                    .getInstance()
////                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
////        }
//
//        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
//
//        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
//
//        UGContourRingPipeline.Config.setHORIZON(HORIZON);
//
//        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //Initialized
        telemetry.addData("Status", "Initialized");

//        String height = String.valueOf(pipeline.getHeight());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        switch (height) {
//            case "ZERO":
//                telemetry.addData("Path", "Running Path 0");
//                break;
//            case "ONE":
//                telemetry.addData("Path", "Running Path 1");
//                break;
//            case "FOUR":
//                telemetry.addData("Path", "Running Path 4");
//                break;
//            default:
//        }

//        //Movement
//        encoderDrive(0.3,  2,  2, 2, 2, 2);
//        sleep(500);
//        //Strafe
//        encoderDrive(0.3,  -18,  18, 18, -18, 5);
//        sleep(500);
//        //Forward
//        encoderDrive(0.3,  39,  39, 39, 39, 5);
//        //Turn on Fly Wheel
//        shooter.setPower(1);
//        sleep(5000);
//        kick(1);
//        //2nd Goal
//        encoderDrive(0.3,  6.5,  -6.5, -6.5, 6.5, 5);
//        kick(1);
//        //3rd Goal
//        encoderDrive(0.3,  7,  -7, -7, 7, 5);
//        kick(1);
//        //Strafe right and rotate toward high goal
//        shootFlap.setPosition(flapAngle + 0);
//        encoderDrive(0.3,  8,  -8, -8, 8, 5);
//        encoderDrive(0.3,  2,  -2, 2, -2, 5);
//        kick (4);
//        shooter.setPower(0);
//        //Park
//        encoderDrive(0.3,  8,  8, 8, 8, 5);
        clawServo.setPosition(clawClose);
        sleep(1000);
        armAngle(135, 0.2);
        sleep(5000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void armAngle(double degrees, double power) {
        int newTarget = clawArm.getCurrentPosition() + (int)(degrees * 1.4933);
        clawArm.setTargetPosition(newTarget);
        // Turn On RUN_TO_POSITION
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        clawArm.setPower(power);

        double currentTime = runtime.milliseconds();
        int clawPosition = clawArm.getCurrentPosition();
        boolean clawStuck = false;

        while (clawArm.isBusy() && !clawStuck) {
            //Check if motor stuck
            if (runtime.milliseconds() > currentTime + 100) {
                currentTime = runtime.milliseconds();

                if (clawPosition + 4 > clawArm.getCurrentPosition()) {
                    clawArm.setPower(0);
                    clawStuck = true;
                } else {
                    clawPosition = clawArm.getCurrentPosition();
                }

            }
        }

        //Stop
        clawArm.setPower(0);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void kick (int kickCount) {
        for (int i = 0; i < kickCount; i++) {
//            kicker.setPosition(kickerTo);
            sleep(200);
//            kicker.setPosition(kickerInit);
            sleep(200);
        }
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                                            robot.leftDrive.getCurrentPosition(),
//                                            robot.rightDrive.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}

