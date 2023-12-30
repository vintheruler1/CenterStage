package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "PositionLock", group = "Autonomous")
public class PositionLock extends LinearOpMode {

    SampleMecanumDrive drive;

    double xyP = 1;
    double headingP = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            lockTo(new Pose2d(0, 0, 0));
            drive.update();

        }

    }

    public void lockTo(Pose2d targetPos) {

        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        xy.times(xyP),
                        heading * headingP
                )

        );

    }

}
