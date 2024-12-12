package org.firstinspires.ftc.teamcode.ExampleCodes.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name="_VW_ObsDeck_Specimen1",group = "drive")
public class _VWAutoIntoDeepObsDeck2 extends LinearOpMode {
    public static double ANGLE = 90; // deg
    DcMotor Armmot ;
    DcMotor viper;
    Servo gripper;
    static final int    VIPER_RETRACT_ENCODER_FRM_GRND = -100; //encoder reading which keep viper/gripeer from up from mother earth , this avoid gripper drag
    public static double RUNTIME1 = 4.0;
    private ElapsedTime timer;
    private int armEncoderValue = 0;
    private int viperEncoderValue = 0;
    private int odoEncoderXValue = 0;
    private int odoEncoderYValue = 0;
    static final int    HCHAMBER_POS_VIPER_ENCODE_VALUE    =   1200;     //
    static final int    HCHAMBER_POS_ARM_ENCODE_VALUE    =   980;     //
    static final int    DISTANCE_TOGO_FOR_CHAMBER  = 20; //Distance robot has to travel so it can position for HCHAMBER specimen hang operation

    static final int    LCHAMBER__POS_VIPER_ENCODE_VALUE    =   0;     //
    static final int    LCHAMBER__POS_ARM_ENCODE_VALUE    =   0;     //

    public static int chamber_viper_retract_sleep = 900;
    //public static int chamber_arm_retract_sleep = 300;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Armmot = hardwareMap.get(DcMotor.class, "armmot");
        Armmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        Armmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper = hardwareMap.get(Servo.class, "gripper");

        Armmot.setDirection(DcMotor.Direction.REVERSE);
        viper.setDirection(DcMotor.Direction.REVERSE);
        Armmot.setPower(0);

        //Prepare trajectory sequence1:  Head towards chamber with 1st specimen
        Pose2d startPose1 = new Pose2d(10, -62, Math.toRadians(90)); //initial position of bot
        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)
                .lineTo(new Vector2d(10, -43))

                .build();

        //Prepare trajectory sequence2:  drag both specimen home
        Pose2d startPose2 = //drive.getPoseEstimate();//
                new Pose2d(10, -43, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                //.turn(Math.toRadians(190))
                //.lineTo(new Vector2d(41, -47)) //align with specimen y value
                //.lineTo(new Vector2d(41, -56)) //go for specimen grab
                .lineTo(new Vector2d(36, -43)) //strafe right
                .lineTo(new Vector2d(36, -8)) //go above
                .lineTo(new Vector2d(46, -8)) //strafe again
                .lineTo(new Vector2d(46, -52)) //bring specimen to obs

                .lineTo(new Vector2d(45, -8))
                .lineTo(new Vector2d(56, -8))  //go for another sample
                .lineTo(new Vector2d(56, -52))  //bring another sample to obs\
                .lineTo(new Vector2d(55, -8))
                .lineTo(new Vector2d(63, -8))

                .lineTo(new Vector2d(63, -60))
                .build();



        //Prepare trajectory sequence3:  go to grab a specimen
        Pose2d startPose3 = //drive.getPoseEstimate();//
                 new Pose2d(59, -46, Math.toRadians(270)); //initial position of bot
        //drive.setPoseEstimate(startPose3);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)
                //.lineTo(new Vector2d(5, -43)) //Go near chamber
                .lineTo(new Vector2d(53, -56))
                .build();

        //Prepare trajectory sequence4:   go chamber with 2nd specimen
        Pose2d startPose4 = //drive.getPoseEstimate();//
                 new Pose2d(53, -56, Math.toRadians(270)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                .lineTo(new Vector2d(0, -43)) //Go near chamber
                .turn(Math.toRadians(190))
                .build();
        //Prepare trajectory sequence5:  go park in obs area
        Pose2d startPose5 = //drive.getPoseEstimate();
                new Pose2d(0, -43, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose5);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                .lineTo(new Vector2d(36, -52))
                .build();

        waitForStart();
        GripperClose();
        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        timer = new ElapsedTime();



        // Build a trajectory sequence


        // Follow the trajectory sequence
        drive.followTrajectorySequence(sequence1); //go to chamber with 1st specimen
        sleep(500);
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        sleep(900);
        MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE+100);/* Extend viper to get exactly near high chamber */
        sleep(2000);
        GripperOpen();
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE-100); /* move arm little down to prevent specimen touching  gripper after hang */
        sleep(900);
        GripperClose();
        MoveViper(-30);; // Retract viper to original position
        sleep(chamber_viper_retract_sleep);
        MoveArm(0);// Retract arm to original position
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        GripperOpen();
        //Pickup 2nd
        drive.followTrajectorySequence(sequence2);

        if (isStopRequested()) return;

    }

    public void MoveViper(int vEncoderValue)
    {
        viper.setTargetPosition(vEncoderValue);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(0.6);
    }

    public void MoveArm(int aEncoderValue)
    {
        Armmot.setTargetPosition(aEncoderValue);
        Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Armmot.setPower(0.4);
    }

    public void GripperOpen()
    {
        gripper.setPosition(0.2);
    }

    public void GripperClose()
    {
        gripper.setPosition(0.5);
    }

}