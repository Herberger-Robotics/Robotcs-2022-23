package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-35, 70, Math.toRadians(90)))
                            .splineTo(new Vector2d(-35, 20), Math.toRadians(90))
                            .turn(Math.toRadians(90))
                            .splineTo(new Vector2d(-30, 20), Math.toRadians(180))
                            .splineTo(new Vector2d(-65, 20), Math.toRadians(180))
                            .turn(-Math.toRadians(90))
                            .splineTo(new Vector2d(-65, 65), Math.toRadians(90))
                            .splineTo(new Vector2d(-65, 45), Math.toRadians(90))
                            .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}