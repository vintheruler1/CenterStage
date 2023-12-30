package org.firstinspires.ftc.teamcode.WIP.OpMode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.WIP.common.vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Bucket;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Conveyor;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Legacy.Hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "REDAUTORIGHTTESTING", group = "Autonomous")
public class Testing extends LinearOpMode {

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
                        .waitSeconds(.2)
                    .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(30.58, 50.62, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .waitSeconds(.2)
                        .addDisplacementMarker(conveyor::autoIntakeHeight)
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
                        .waitSeconds(.2)
                    .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(35.52, 49.97, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .waitSeconds(.2)
                        .addDisplacementMarker(conveyor::autoIntakeHeight)
                        .build();
                return;

            case RIGHT:
                SPIKE_MARK = drive.trajectorySequenceBuilder(new Pose2d(63.39, 12.32, Math.toRadians(180.00)))
                        .lineToSplineHeading(new Pose2d(31.32, 15.74, Math.toRadians(90.00))) // @ spike mark
                        .addDisplacementMarker(() -> {
                            lift.setZeroPosition();
                            bucket.rotateBucket(false);
                            bucket.openPixelOne();
                        })
                        .waitSeconds(.2)
                        .build();

                YELLOW_PRELOAD = drive.trajectorySequenceBuilder(SPIKE_MARK.end())
                        .addDisplacementMarker(lift::setLayerOne)
                        .lineToSplineHeading(new Pose2d(42.01, 51.22, Math.toRadians(90.00)))
                        .addDisplacementMarker(bucket::releasePixels)
                        .addDisplacementMarker(conveyor::autoIntakeHeight)
                        .waitSeconds(.2)

                        .build();
                return;

            case NONE:
                startingPos = visionProcessor.getStartingPosition();
        }


        waitForStart();

        visionPortal.stopStreaming();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            TrajectorySequence main = drive.trajectorySequenceBuilder(YELLOW_PRELOAD.end())

                    .lineToSplineHeading(new Pose2d(35.69, 6, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        lift.setZeroPosition();
                        bucket.rotateBucket(true);
                        bucket.loadingPosition();
                    })
                    .lineToSplineHeading(new Pose2d(35.69, -62.66, Math.toRadians(90))) // @ stacks
                    .addDisplacementMarker(() -> {
                        conveyor.runConveyor();
                    })
                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(35.69, 12, Math.toRadians(90))) // @ half way to backdrop
                    .addDisplacementMarker(() -> {
                        lift.setLayerTwo();
                        bucket.rotateBucket(false);
                        bucket.closeonPixels();
                        conveyor.stopConveyor();
                    })

                    .lineToSplineHeading(new Pose2d(35.69, 50.62, Math.toRadians(90.00))) // @ backdrop
                    .addDisplacementMarker(() -> {
                        bucket.releasePixels();
                        conveyor.smallerPixelStackHeight();
                        conveyor.smallerPixelStackHeight();
                    })

                    .waitSeconds(.5)

                    .lineTo(new Vector2d(45, 45)) // @ parking
                    .splineToConstantHeading(new Vector2d(60, 48), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        lift.setZeroPosition();
                        bucket.rotateBucket(true);
                        conveyor.stopConveyor();
                        bucket.releasePixels();
                    })
                    .build();

            drive.update();
            lift.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("COUNTER", String.valueOf(counter));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }

    }

}
