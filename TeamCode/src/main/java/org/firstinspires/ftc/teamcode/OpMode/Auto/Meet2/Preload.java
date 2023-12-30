package org.firstinspires.ftc.teamcode.OpMode.Auto.Meet2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Lift;

@Disabled
@Autonomous(name = "Preload", group = "Autonomous")

public class Preload extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    public Servo chopStickLeft;
    public Servo chopStickRight;
    public Servo chopsticksRotateRight;
    public CRServo intakeSpinner;
    public Servo airplaneLauncher;

    private Lift lift;

    public void init_servos() {

        chopStickRight = hardwareMap.get(Servo.class, "cRR");
        chopStickLeft = hardwareMap.get(Servo.class, "cLL");
        intakeSpinner = hardwareMap.get(CRServo.class, "spinner");
        airplaneLauncher = hardwareMap.get(Servo.class, "drone");
        chopsticksRotateRight = hardwareMap.get(Servo.class, "rot");
//        linearActuatorClaw = hardwareMap.get(Servo.class, "linact");

        chopStickLeft.setDirection(Servo.Direction.REVERSE);
        intakeSpinner.setDirection(CRServo.Direction.REVERSE);

    }

    public void openChopsticks() {

        chopStickLeft.setPosition(0.23);
        chopStickRight.setPosition(0.53);

    }
    public void closeChopsticks() {

        chopStickLeft.setPosition(0.31);
        chopStickRight.setPosition(0.74);


        // right servo is 0.9 at locked position
        // left servo is 0.6 at locked position
    }

    public void rotateChopsticks(boolean intake) {

        if (intake) {

            //closeChopsticks();
            chopsticksRotateRight.setPosition(.65);


        } else {

            chopsticksRotateRight.setPosition(0.33);
            //openChopsticks();

        }

    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        lift = new Lift(hardwareMap);

        lift.setZeroPosition();

        init_servos();

        //Vision vision = new Vision();
        //penCvCamera camera;
        //String webcamName = "Webcam 1";

        // setup for the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
//        camera.setPipeline(vision);

        //camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });

        // no need for waitForStart() with while loop
//        while (!isStarted()) {
//            telemetry.addData("red position: ", vision.getRedPosition());
//            telemetry.addData("blue position: ", vision.getBluePosition());
//
//            telemetry.addData("red mid: ", vision.getRedPercentMid());
//            telemetry.addData("red right: ", vision.getRedPercentRight());
//            telemetry.addData("red left: ", vision.getRedPercentLeft());
//
//            telemetry.addData("blue mid: ", vision.getBluePercentMid());
//            telemetry.addData("blue right: ",vision.getBluePercentRight());
//            telemetry.addData("blue left: ",vision.getBluePercentLeft());
//            telemetry.update();
//
//        }
        runtime.reset();

        //Vision.Position parkingPosition;

        //int pposition = -1;

//        while (pposition == -1) {
//            parkingPosition = vision.getBluePosition();
//            switch (parkingPosition) {
//                case LEFT:
//                    pposition = 0;
//                    break;
//                case CENTER:
//                    pposition = 1;
//                    break;
//                case RIGHT:
//                    pposition = 2;
//                    break;
//            }
//        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        TrajectorySequence go = drive.trajectorySequenceBuilder(new Pose2d(-65.02, -3.41, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(-22.42, -2.97))
//                .lineTo(new Vector2d(-39.93, 58.49))
//                .build();

//        TrajectorySequence go = drive.trajectorySequenceBuilder(new Pose2d(66.25, -1.40, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(33.24, 13.49))
//                .lineTo(new Vector2d(34.71, 56.82))
//                .addDisplacementMarker(() -> {
//                    lift.setLayerTwo();
//
//                    rotateChopsticks(true);
//
//                    openChopsticks();
//                    closeChopsticks();
//                })
//                .build();


        TrajectorySequence go = drive.trajectorySequenceBuilder(new Pose2d(66.40, -3.32, Math.toRadians(90.00)))
                .lineTo(new Vector2d(28.08, -3.32))
                .lineTo(new Vector2d(27.34, 55.93))
                .addDisplacementMarker(() -> {
                    lift.setLayerTwo();

//                    waitSeconds(2);

                    rotateChopsticks(true);

                    openChopsticks();
                    closeChopsticks();
                })
                .build();




        drive.setPoseEstimate(go.start());

        lift.update();
        drive.update();

        waitForStart();

        if (isStopRequested()) return;

        PoseStorage.currentPose = drive.getPoseEstimate();

        Pose2d poseEstimate = drive.getPoseEstimate();

        drive.followTrajectorySequence(go);

        lift.update();
        drive.update();

        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());


        //telemetry.addLine(String.valueOf(pposition));
        telemetry.update();
    }
}
