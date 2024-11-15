package org.firstinspires.ftc.teamcode.ExampleCodes.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose1 = new Pose2d(-10, -62, Math.toRadians(90)); //initial position of bot

        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)

                //.splineToConstantHeading(new Vector2d(-10, -42), Math.toRadians(90)) //go to speciman hang position for drive motors
                .lineTo(new Vector2d(-10, -42))
                //  .lineToConstantHeading(new Vector2d(-10, -42), Math.toRadians(90));
        //.l
                .build();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose2 = new Pose2d(-10, -42, Math.toRadians(90)); //initial position of bot

        drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)


                //.splineToConstantHeading(new Vector2d(-48, -42), Math.toRadians(90)) //strafe left
                .lineTo(new Vector2d(-47, -42))
                .build();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose3 = new Pose2d(-47, -42, Math.toRadians(90)); //initial position of bot

        drive.setPoseEstimate(startPose3);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)


                //.splineToConstantHeading(new Vector2d(-32, -42), Math.toRadians(90)) // take motor near sample
                .lineTo(new Vector2d(-47, -32))
                .build();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose4 = new Pose2d(-47, -32, Math.toRadians(90)); //initial position of bot

        drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)

                //.splineToConstantHeading(new Vector2d(-56, -54), Math.toRadians(90)) //go near basket
                .lineTo(new Vector2d(-56, -54))
                .build();

        waitForStart();

        if (isStopRequested()) return;
/*
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
*/

        drive.setPoseEstimate(startPose1);
        // Follow the trajectory sequence
        drive.followTrajectorySequence(sequence1);
        drive.followTrajectorySequence(sequence2);
        drive.followTrajectorySequence(sequence3);
        drive.followTrajectorySequence(sequence4);


    }

}
