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
@Autonomous(name="_VW_Basket_Net_AscentZone",group = "drive")
public class _VWAutoBasket_Net_ascentZone extends LinearOpMode {

    public static double ANGLE = 65; // deg
    DcMotor Armmot ;
    DcMotor viper;
    Servo gripper;
    public static int    VIPER_RETRACT_ENCODER_FRM_GRND = -100; //encoder reading which keep viper/gripeer from up from mother earth , this avoid gripper drag
    public static double RUNTIME1 = 4.0;
    private ElapsedTime timer;
    private int armEncoderValue = 0;
    private int viperEncoderValue = 0;
    private int odoEncoderXValue = 0;
    private int odoEncoderYValue = 0;
    public static int    HCHAMBER_POS_VIPER_ENCODE_VALUE    =   1200;     //
    public static int    HCHAMBER_POS_ARM_ENCODE_VALUE    =   980;     //
    public static int    HBASKET_POS_VIPER_ENCODE_VALUE    =   2100;     //
    public static int    HBASKET_POS_ARM_ENCODE_VALUE    =   1930;     //

    static final int    DISTANCE_TOGO_FOR_CHAMBER  = 20; //Distance robot has to travel so it can position for HCHAMBER specimen hang operation

    static final int    LCHAMBER__POS_VIPER_ENCODE_VALUE    =   0;     //
    static final int    LCHAMBER__POS_ARM_ENCODE_VALUE    =   0;     //

    public static  int    Sample1Align_x    =   -49;     //
    public static int    Sample1Align_y    =   -8;     //

    public static  int    Sample2Align_x    =   -58;     //
    public static int    Sample2Align_y    =   -8;     //

    public static  int    Sample3Align_x    =   -66;     //
    public static int    Sample3Align_y    =   -8;     //


    public static  int    Sample1_y_parking    =   -62;     //
    public static int    Sample2_y_parking    =   -58;     //
    public static int    Sample3_y_parking    =   -54;     //


    public static  int    BasketAlign_x    =   -58;     //
    public static int    BasketAlign_y    =   -52;


    public static int   Ascent_x = -35;
    public static  int Ascent_y = -9;
    public static  int Ascent_angle = -80;

    public static int chamber_viper_retract_sleep = 900;
    public static int chamber_arm_retract_sleep = 300;





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

        //Prepare trajectory sequence1:  Head towards chamber
        Pose2d startPose1 = new Pose2d(-10, -62, Math.toRadians(90)); //initial position of bot
        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)
                .lineTo(new Vector2d(-10, -43))
                .build();

        //Prepare trajectory sequence2:  strafe towards yellow sample
        Pose2d startPose2 = //drive.getPoseEstimate();
        new Pose2d(-10, -43, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                .lineTo(new Vector2d(-34, -43)) //strafe left
                .lineTo(new Vector2d(-34, -8)) //go above
                .lineTo(new Vector2d(Sample1Align_x, Sample1Align_y)) //strafe again
                .lineTo(new Vector2d(Sample1Align_x, Sample1_y_parking)) //bring specimen to net zone

                .lineTo(new Vector2d(Sample1Align_x, Sample1Align_y))
                .lineTo(new Vector2d(Sample2Align_x, Sample2Align_y))  //go for another sample
                .lineTo(new Vector2d(Sample2Align_x, Sample2_y_parking))  //bring another sample to net zone\
                .lineTo(new Vector2d(Sample2Align_x, Sample2Align_y))
                .lineTo(new Vector2d(Sample3Align_x, Sample3Align_y))

                .lineTo(new Vector2d(Sample3Align_x, Sample3_y_parking))
                .build();


/*
        //Prepare trajectory sequence3:  go forward towards yellow sample
        Pose2d startPose3 = //drive.getPoseEstimate();
        new Pose2d(Sample1Align_x, Sample1Align_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose3);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)
                .lineTo(new Vector2d(Sample1_x,Sample1_y ))
                .build();

        //Prepare trajectory sequence4:  backup towards basket
        Pose2d startPose4 = //drive.getPoseEstimate();
        new Pose2d(Sample1_x             , Sample1_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                .lineTo(new Vector2d(BasketAlign_x, BasketAlign_y))
                .turn(Math.toRadians(-1 * ANGLE))
                .build();

        //Prepare trajectory sequence5:  go towards second yellow sample
        Pose2d startPose5 = //drive.getPoseEstimate();
        new Pose2d(BasketAlign_x, BasketAlign_y, Math.toRadians(90-ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose5);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                .turn(Math.toRadians(ANGLE))
                .lineTo(new Vector2d(Sample2_x, Sample2_y))
                .build();

        //Prepare trajectory sequence6:  backup towards basket for second yellow sample drop
        Pose2d startPose6 = //drive.getPoseEstimate();
                 new Pose2d(Sample2_x, Sample2_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose6);
        TrajectorySequence sequence6 = drive.trajectorySequenceBuilder(startPose6)
                .lineTo(new Vector2d(BasketAlign_x, BasketAlign_y))
                .turn(Math.toRadians(-1*ANGLE))
                .build();
*/
        //Prepare trajectory sequence7:  go to ascent zone
        Pose2d startPose7 = //drive.getPoseEstimate();
                 new Pose2d(BasketAlign_x, BasketAlign_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose7);
        TrajectorySequence sequence7 = drive.trajectorySequenceBuilder(startPose7)
                .lineTo(new Vector2d(Ascent_x, Ascent_y))
                .turn(Math.toRadians(Ascent_angle))
                .build();
/*
        TrajectorySequence sequenceFWD = drive.trajectorySequenceBuilder(new Pose2d())

                .back(Basket_distance) // Move back 50 inches
                .build();

        TrajectorySequence sequenceBACK = drive.trajectorySequenceBuilder(new Pose2d())

                .forward(Basket_distance) // Move back 50 inches
                .build();
*/


        waitForStart();

        //Auto Start here
        GripperClose();

        //retract viper from ground so that it does not touch ground
        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot

        timer = new ElapsedTime();
        if (isStopRequested()) return;

        // execute  trajectory sequence1
        drive.followTrajectorySequence(sequence1);
        sleep(500);

        //hang specimen to high chamber , start extending arm first and then viper to avoid viper hitting ground
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        sleep(900);
        MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE+100);/* Extend viper to get exactly near high chamber */
        sleep(2000);
        GripperOpen();
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE-70); /* move arm little down to prevent specimen touching  gripper after hang */
        sleep(900);
        GripperClose();
        MoveViper(-30);; // Retract viper to original position
        sleep(chamber_viper_retract_sleep);
        MoveArm(0);// Retract arm to original position
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        sleep(chamber_arm_retract_sleep);

        // execute  trajectory sequence2 , strafe towards yellow sample
        drive.followTrajectorySequence(sequence2);




        // execute  trajectory sequence7 : go to ascent zone
        drive.followTrajectorySequence(sequence7);

        //execute arm and viper operation to rest them on low rung
        MoveArm(1200);
        sleep(2000);
        MoveViper(1900);
        sleep(800);
        MoveArm(900);


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
        gripper.setPosition(0.5);
    }

    public void GripperClose()
    {
        gripper.setPosition(0.2);
    }


}