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
 * /*
 * This Program score a specimen on high chamber
  then grab another specimen from human player, human player feeding a specimen from outside , put it in high chamber and then park on observation zone
 */
@Config
//@Autonomous(name="_VW_ObsDeck_Specimen2HumanOut",group = "drive")
public class _VWAuto_ObsDeckHumanS2 extends LinearOpMode {
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
    public static int    HCHAMBER_POS_VIPER_ENCODE_VALUE    =   0;     //
    public static int    HCHAMBER_POS_ARM_ENCODE_VALUE    =   1100;     //
    static final int    DISTANCE_TOGO_FOR_CHAMBER  = 20; //Distance robot has to travel so it can position for HCHAMBER specimen hang operation

    public static  int    StartPosition_x    =   10;     //
    public static int    StartPosition_y    =   -62;     //

    public static  int    ChamberPosition_x    =   10;     //
    public static int    ChamberPosition_y    =   -35;
    public static int comebackFrmChamber = -10;

    public static  int    parking_x    =   52;     //
    public static int    parking_y    =   -65;     //

    public static  int    Specimen2Hang_x    =   5;     //
    public static int    Specimen2Hang_y    =   -38;     //

    public static  int    Specimen2align_x    =   56;     //
    public static int    Specimen2align_y    =   -52;     //

    //public static  int    Specimen2location_x    =   53;     //
    //public static int    Specimen2location_y    =   -49;     //

    public static int    Specimen2HumanDelay    =   2000;     //

    public static int chamber_viper_retract_sleep = 00;
    //public static int chamber_arm_retract_sleep = 300;
    public static  int    HPLAYER_POS_VIPER_ENCODE_VALUE    =   -94;     //
    public static  int    HPLAYER_POS_ARM_ENCODE_VALUE    =   658;     //
    public static double ANGLE = 182; // deg
    public static double ANGLE_S2 = -178;
    //public static int    S2_HCHAMBER_POS_VIPER_ENCODE_VALUE    =   -94;     //
    public static int    S2_HCHAMBER_POS_ARM_ENCODE_VALUE    =   658;
    public static int S2_JHATKA = 300;
    public static int TRAVEL_TIME_TO_CHAMBER = 300;

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
        Pose2d startPose1 = new Pose2d(StartPosition_x, StartPosition_y, Math.toRadians(90)); //initial position of bot
        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)
                .lineTo(new Vector2d(ChamberPosition_x, ChamberPosition_y))
                .build();

        //Prepare trajectory sequence2: go back a little
        Pose2d startPose2 = //drive.getPoseEstimate();//
                new Pose2d(ChamberPosition_x, ChamberPosition_y, Math.toRadians(90));
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                .lineTo(new Vector2d(ChamberPosition_x, ChamberPosition_y+comebackFrmChamber))
                .build();

        //Prepare trajectory sequence3:  go to grab a specimen
        Pose2d startPose3 = //drive.getPoseEstimate();//
                 new Pose2d(ChamberPosition_x, ChamberPosition_y+comebackFrmChamber, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose3);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)
                //.lineTo(new Vector2d(5, -43)) //Go near chamber
                .lineTo(new Vector2d(36, -43)) //strafe right
                .lineTo(new Vector2d(36, -8)) //go above
                .lineTo(new Vector2d(46, -8)) //strafe again
                .lineTo(new Vector2d(46, -52)) //bring specimen to obs

                .lineTo(new Vector2d(45, -8))
                .lineTo(new Vector2d(56, -8))  //go for another sample
                .lineTo(new Vector2d(Specimen2align_x, Specimen2align_y))  //bring another sample to obs\
                .turn(Math.toRadians(ANGLE))


//                .lineTo(new Vector2d(Specimen2location_x, Specimen2location_y))
                .build();

        //Prepare trajectory sequence4:   go chamber with 2nd specimen
        Pose2d startPose4 = //drive.getPoseEstimate();//
                 new Pose2d(Specimen2align_x, Specimen2align_y, Math.toRadians(90+ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                .lineTo(new Vector2d(Specimen2Hang_x, Specimen2Hang_y)) //Go near chamber
                .turn(Math.toRadians(ANGLE_S2))
                .build();
        //Prepare trajectory sequence5:  go park in obs area
        Pose2d startPose5 = //drive.getPoseEstimate();
                new Pose2d(Specimen2Hang_x, Specimen2Hang_y, Math.toRadians(90+ANGLE_S2)); //initial position of bot
        //drive.setPoseEstimate(startPose5);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                .lineTo(new Vector2d(parking_x, parking_y))
                .turn(Math.toRadians(-90))
                .build();

        waitForStart();
        GripperClose();
        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        timer = new ElapsedTime();



        // Build a trajectory sequence


        // Follow the trajectory sequence
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near high chamber */
        sleep(500);
        drive.followTrajectorySequence(sequence1); //go to chamber with 1st specimen
        sleep(TRAVEL_TIME_TO_CHAMBER);
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE-S2_JHATKA);/* lower arm to hang specimen */
        sleep(300);
        GripperOpen();
        //sleep(300);

        //Go for Pickup 2nd Specimen from human player
        drive.followTrajectorySequence(sequence2);


        Armmot.setTargetPosition(HPLAYER_POS_ARM_ENCODE_VALUE); //Align arm near wall height
        Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Armmot.setPower(0.4);
        viper.setTargetPosition(HPLAYER_POS_VIPER_ENCODE_VALUE); //set viper touching  to wall
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(0.4);
        //sleep(Specimen2HumanDelay);
        drive.followTrajectorySequence(sequence3); //strafe two samples and turn to grab specimen */
        sleep(Specimen2HumanDelay);
        GripperClose();

        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near high chamber */





        drive.followTrajectorySequence(sequence4); //go to chamber with 2nd specimen
        MoveArm(S2_HCHAMBER_POS_ARM_ENCODE_VALUE-  S2_JHATKA );/* Extend  arm */
        GripperOpen();
        //MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE); /* move arm little down to prevent specimen touching  gripper after hang */
        //sleep(300);
        //MoveViper(-30);; // Retract viper to original position
        sleep(300);
        GripperClose();
        drive.followTrajectorySequence(sequence5); //go to parking in obs area
        sleep(chamber_viper_retract_sleep);
        MoveArm(0);// Retract arm to original position
        //viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot

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