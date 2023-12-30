package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(63.39, 12.32, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(31.32, 15.74, Math.toRadians(90.00))) // @ spike mark
                                .waitSeconds(.3)
                                .lineToSplineHeading(new Pose2d(42.01, 51.22, Math.toRadians(90.00))) // @ preload
                                .waitSeconds(.5)

                                .lineToSplineHeading(new Pose2d(35.69, 6, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(35.69, -62.66, Math.toRadians(90))) // @ stacks
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(35.69, 12, Math.toRadians(90))) // @ half way to backdrop
                                .lineToSplineHeading(new Pose2d(35.69, 50.62, Math.toRadians(90.00))) // @ backdrop
                                .waitSeconds(.5)

                                .lineToSplineHeading(new Pose2d(35.69, 6, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(35.69, -62.66, Math.toRadians(90))) // @ stacks
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(35.69, 12, Math.toRadians(90))) // @ half way to backdrop
                                .lineToSplineHeading(new Pose2d(35.69, 50.62, Math.toRadians(90.00))) // @ backdrop
                                .waitSeconds(.5)

                                .lineToSplineHeading(new Pose2d(35.69, 6, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(35.69, -62.66, Math.toRadians(90))) // @ stacks
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(35.69, 12, Math.toRadians(90))) // @ half way to backdrop
                                .lineToSplineHeading(new Pose2d(35.69, 50.62, Math.toRadians(90.00))) // @ backdrop
                                .waitSeconds(.5)

                                .lineTo(new Vector2d(45, 45)) // @ parking
                                .splineToConstantHeading(new Vector2d(60, 48), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(90))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("./meepmeep+roadrunner.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img) // MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK -> not rotated like roadrunner
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}