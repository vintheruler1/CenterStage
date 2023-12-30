package org.firstinspires.ftc.teamcode.WIP.OpMode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Conveyor;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Bucket;
import org.firstinspires.ftc.teamcode.WIP.common.vision.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedAutoRight", group = "Autonomous")
public class RedAutoRight extends LinearOpMode {

    enum State {

        SPIKE_MARK,
        YELLOW_PRELOAD,
        TRAVEL_TO_STACK,
        PICK_UP_PIXELS,
        SCORE_ON_BACKDROP,
        PARK,
        IDLE

    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(63.39, 12.32, Math.toRadians(180));

    private CSVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    TrajectorySequence SPIKE_MARK;
    TrajectorySequence YELLOW_PRELOAD;

    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        Conveyor conveyor = new Conveyor(hardwareMap, robot);
        Bucket bucket = new Bucket(hardwareMap, robot, lift);

        int counter = 2;

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        visionProcessor = new CSVisionProcessor();

        CSVisionProcessor.StartingPosition startingPos = CSVisionProcessor.StartingPosition.NONE;

        startingPos = visionProcessor.getStartingPosition();
        telemetry.addData("Identified", visionProcessor.getStartingPosition());
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        switch( startingPos ){
            case LEFT:
                SPIKE_MARK = drive.trajectorySequenceBuilder(new Pose2d(63.39, 12.32, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(32.07, 9.20), Math.toRadians(180.00))
                        .addDisplacementMarker(() -> {
                            lift.setZeroPosition();
                            bucket.rotateBucket(false);
                            bucket.openPixelOne();
                        })
                    .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(30.58, 50.62, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .build();
                return;

            case CENTER:
                SPIKE_MARK = drive.trajectorySequenceBuilder(new Pose2d(63.39, 12.32, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(32.36, 15.44), Math.toRadians(180.00))
                        .addDisplacementMarker(() -> {
                            lift.setZeroPosition();
                            bucket.rotateBucket(false);
                            bucket.openPixelOne();
                        })
                    .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(35.52, 49.97, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .build();
                return;

            case RIGHT:
                SPIKE_MARK = drive.trajectorySequenceBuilder(new Pose2d(63.39, 12.32, Math.toRadians(180.00)))
                        .splineTo(new Vector2d(31.32, 15.74), Math.toRadians(90.00))
                        .addDisplacementMarker(() -> {
                            lift.setZeroPosition();
                            bucket.rotateBucket(false);
                            bucket.openPixelOne();
                        })
                        .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(42.01, 51.22, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .build();
                return;

            case NONE:
                startingPos = visionProcessor.getStartingPosition();
        }

        TrajectorySequence TRAVEL_TO_STACK = drive.trajectorySequenceBuilder(YELLOW_PRELOAD.end())
//                .lineToSplineHeading(new Pose2d(35.69, -9.08, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(35.69, -62.66, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    lift.setZeroPosition();
                    bucket.rotateBucket(true);
                    bucket.loadingPosition();
                })
//                .lineTo(new Vector2d(36.57, -62.66))
                .build();

        TrajectorySequence PICK_UP_PIXELS = drive.trajectorySequenceBuilder(TRAVEL_TO_STACK.end())
                .addDisplacementMarker(() -> {
                    conveyor.autoIntakeHeight();
                    conveyor.runConveyor();
                })
                .build();

        TrajectorySequence SCORE_ON_BACKDROP = drive.trajectorySequenceBuilder(TRAVEL_TO_STACK.end())
//                .lineToSplineHeading(new Pose2d(36.04, -14.72, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(35.69, 13, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    conveyor.stopConveyor();
                    lift.setLayerTwo();
                    bucket.rotateBucket(false);
                })
                .lineToSplineHeading(new Pose2d(35.69, 50.62, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    conveyor.smallerPixelStackHeight();
                    conveyor.smallerPixelStackHeight();
                })
                .waitSeconds(.2)
                .addDisplacementMarker(() -> {
                    bucket.releasePixels();
                })
                .waitSeconds(.1)
                .build();

        TrajectorySequence PARK = drive.trajectorySequenceBuilder(new Pose2d(34.81, 48.91, Math.toRadians(90.45)))
//                .lineToSplineHeading(new Pose2d(13.13, 49.26, Math.toRadians(90)))
                .lineTo(new Vector2d(12, 48.62))
                .addDisplacementMarker(() -> {
                    lift.setZeroPosition();
                    bucket.loadingBucket();
                    conveyor.stopConveyor();
                })
                .lineTo(new Vector2d(12, 61.60))
//                .lineTo(new Vector2d(14.54, 61.60))
                .build();

        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Around 3 of these?

        waitForStart();

        visionPortal.stopStreaming();

        if (isStopRequested()) return;

        currentState = State.SPIKE_MARK;

        drive.followTrajectorySequenceAsync(SPIKE_MARK);

        telemetry.addData("CURRENT TRAJ: ", "SPIKE MARK");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {

                case SPIKE_MARK:
                    if (!drive.isBusy()) {
                        currentState = State.YELLOW_PRELOAD;
                        drive.followTrajectorySequenceAsync(YELLOW_PRELOAD);
                    }
                    break;

                case YELLOW_PRELOAD:
                    if (!drive.isBusy()) {
                        currentState = State.TRAVEL_TO_STACK;
                        drive.followTrajectorySequenceAsync(TRAVEL_TO_STACK);
                    }

                case TRAVEL_TO_STACK:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_PIXELS;
                        drive.followTrajectorySequenceAsync(PICK_UP_PIXELS);
                    }

                case PICK_UP_PIXELS:
                    if (!drive.isBusy()) { // waitTimer1.seconds() > waitTime1
                        currentState = State.SCORE_ON_BACKDROP;
                        drive.followTrajectorySequenceAsync(SCORE_ON_BACKDROP); // ASYNC????
                    }

                case SCORE_ON_BACKDROP:
                    if (!drive.isBusy() && counter > 0) {
                        currentState = State.PICK_UP_PIXELS;
                        drive.followTrajectorySequenceAsync(PICK_UP_PIXELS); // ASYNC????
                        counter--;
                    } else if (counter <= 0) {
                        currentState = State.PARK;
                    }

                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(PARK);
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }

            drive.update();
            lift.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("CURRENT TRAJ", String.valueOf(currentState));
            telemetry.addData("COUNTER", String.valueOf(counter));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }

    }

}
