package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static Vector2d left_blue = new Vector2d(49,38);
    private static Vector2d middle_blue = new Vector2d(49,32);
    private static Vector2d right_blue = new Vector2d(49, 26);


    private static Vector2d right_red = new Vector2d(49,-38);
    private static Vector2d middle_red = new Vector2d(49,-32);
    private static Vector2d left_red = new Vector2d(49, -26);


    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //14.25
                .setConstraints(44.13626700549987, 30, 2.4687686920166017, Math.toRadians(60), 17.22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, -63, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(16.50, -30.5, Math.toRadians(270)), Math.toRadians(90))
                                .forward(3)
                                .splineToConstantHeading(new Vector2d(25.50, -48.5), Math.toRadians(0))

                                .splineToConstantHeading(new Vector2d(32.50, -44.5), Math.toRadians(90))

                                .splineTo(middle_red, Math.toRadians(0))
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}