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
 *  * /*
 * This Program score a specimen on high chamber
  then grab another specimen from human player, human player feeding a specimen by placing it on floor , robot put it in high chamber and then park on observation zone
 */
@Config
@Autonomous(name="_VWAuto_BasketS2",group = "drive")
public class _VWAuto_BasketS2 extends LinearOpMode {
    DcMotor Armmot ;
    DcMotor viper;
    Servo gripper;
    static final int    VIPER_RETRACT_ENCODER_FRM_GRND = -100; //encoder reading which keep viper/gripeer from up from mother earth , this avoid gripper drag
    public static double RUNTIME1 = 4.0;
    private ElapsedTime timer;

    public static  int    Sample1Align_x    =   -51;     //
    public static int    Sample1Align_y    =   -37;     //

    public static int    HCHAMBER_POS_VIPER_ENCODE_VALUE    =   0;     //
    public static int    HCHAMBER_POS_ARM_ENCODE_VALUE    =   1100;     //

    public static int    StartPosition_x    =   -10;     //
    public static int    StartPosition_y    =   -62;     //

    public static int    ChamberPosition_x    =   -10;     //
    public static int    ChamberPosition_y    =   -35;
    public static int comebackFrmChamber = -10;

    //public static int    parking_x    =   52;     //
    //public static int    parking_y    =   -65;     //

    //public static int    Specimen2Hang_x    =   5;     //
    //public static int    Specimen2Hang_y    =   -38;     //

    //public static int    Specimen2align_x    =   56;     //
    //public static int    Specimen2align_y    =   -52;     //

    public static int    DelayStartForRaisingARMandViper    = 800  ;     //
    public static int DelayAfterJhatka = 300;
    //public static int    Specimen2HumanDelay    =   500;     //

    //public static int chamber_viper_retract_sleep = 00;
    //public static int chamber_arm_retract_sleep = 300;
    //public static  int    HPLAYER_POS_VIPER_ENCODE_VALUE    =   -94;     //
    //public static  int    HPLAYER_POS_ARM_ENCODE_VALUE    =   658;     //
    public static double ANGLE = 120; // deg
    public static double ANGLE_S2 = -93;
    public static double ANGLE_S3 = 100;

    //public static int    S2_HCHAMBER_POS_VIPER_ENCODE_VALUE    =   -94;     //
    //public static int    S2_HCHAMBER_POS_ARM_ENCODE_VALUE    =   658;
    public static int S2_JHATKA = 300;
    public static int TRAVEL_TIME_TO_CHAMBER = 300;
    public static int MAX_WAIT_GOING_CHAMBER_MS = 5000;
    public static int MAX_WAIT_GOING_S1_MS = 5000;
    public static int MAX_WAIT_GOING_BASKET_MS = 2000;
    public static int DELAY_FOR_S1_GRAB = 2000;
    public static int DELAY_FOR_S2_GRAB = 2000;
    public static int DELAY_FOR_S2_TO_BASKET = 1000;
    public static int S1_POS_ARM_ENCODE_VALUE = 240;
    public static int S1_POS_VIPER_ENCODE_VALUE = 850;

    public static int S2_POS_ARM_ENCODE_VALUE = 250;
    public static int S2_POS_VIPER_ENCODE_VALUE = 2100;

    public final int   HBASKET_POS_VIPER_ENCODE_VALUE    =   2100;     //
    public static int  HBASKET_POS_ARM_ENCODE_VALUE    =   1450;     //

    public final int   HBASKET_S2POS_VIPER_ENCODE_VALUE    =   2200;
    public final int   HBASKET_S2POS_ARM_ENCODE_VALUE = 1400;
    public static  int    BasketAlign_x    =   -58;     //
    public static int    BasketAlign_y    =   -52;
    public static int    Basket_distance    =   12;
    public static int    Basket_distance_S2 = 10;
    public static double PowerArm = 0.4;
    public static double PowerViper = 0.6;
    public static int ARMStandPOsition = 1650;

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
/*
        //Prepare trajectory sequence1:  Head towards chamber with 1st specimen
        Pose2d startPose1 = new Pose2d(10, -62, Math.toRadians(90)); //initial position of bot
        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)
                .lineTo(new Vector2d(10, -43))

                .build();

        //Prepare trajectory sequence2:  go to get specimen 2 from human
        Pose2d startPose2 = //drive.getPoseEstimate();//
                new Pose2d(10, -43, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                .lineTo(new Vector2d(Specimen2align_x, Specimen2align_y)) //bring specimen to obs
                .turn(Math.toRadians(ANGLE))
                .build();



        //Prepare trajectory sequence3:  go to grab a specimen
        Pose2d startPose3 = //drive.getPoseEstimate();//
                 new Pose2d(Specimen2align_x, Specimen2align_y, Math.toRadians(90+ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose3);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)
                //.lineTo(new Vector2d(5, -43)) //Go near chamber
                .lineTo(new Vector2d(Specimen2location_x, Specimen2location_y))
                .build();

        //Prepare trajectory sequence4:   go chamber with 2nd specimen
        Pose2d startPose4 = //drive.getPoseEstimate();//
                 new Pose2d(Specimen2location_x, Specimen2location_y, Math.toRadians(90+ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                .lineTo(new Vector2d(Specimen2Hang_x, Specimen2Hang_y)) //Go near chamber
                .turn(Math.toRadians(ANGLE))
                .build();
        //Prepare trajectory sequence5:  go park in obs area
        Pose2d startPose5 = //drive.getPoseEstimate();
                new Pose2d(Specimen2Hang_x, Specimen2Hang_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose5);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                .lineTo(new Vector2d(parking_x, parking_y))
                .build();
*/
        timer = new ElapsedTime();
        waitForStart();

        //Start Code from here

        GripperClose();
        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot




        // Build a trajectory sequence


        // Follow the trajectory sequence
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        //MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near high chamber */
        sleep(DelayStartForRaisingARMandViper);

        //Prepare trajectory sequence1:  Head towards chamber with 1st specimen
        Pose2d startPose1 = new Pose2d(StartPosition_x, StartPosition_y, Math.toRadians(90)); //initial position of bot
        drive.setPoseEstimate(startPose1);
        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(startPose1)
                .lineTo(new Vector2d(ChamberPosition_x, ChamberPosition_y))
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence1); //go to chamber with 1st specimen
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_CHAMBER_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }

        //sleep(TRAVEL_TIME_TO_CHAMBER);
        MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE-S2_JHATKA);/* lower arm to hang specimen */
        sleep(DelayAfterJhatka);
        GripperOpen();
       
        //sleep(300);
        //Prepare trajectory sequence2: go back a little and strafe both samples
        Pose2d startPose2 = //drive.getPoseEstimate();//
                new Pose2d(ChamberPosition_x, ChamberPosition_y, Math.toRadians(90));
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                .lineTo(new Vector2d(ChamberPosition_x, ChamberPosition_y+comebackFrmChamber))
				.lineTo(new Vector2d(Sample1Align_x, Sample1Align_y))
                .build();
        timer.reset();
		drive.followTrajectorySequence(sequence2);

        //sleep(900);
        MoveViper(S1_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near sample 1 */
        while (opModeIsActive() && viper.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }

        MoveArm(S1_POS_ARM_ENCODE_VALUE);/* lower  arm */
        while (opModeIsActive() && Armmot.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }
        sleep(DELAY_FOR_S1_GRAB);
        GripperClose();

        MoveArm(HBASKET_POS_ARM_ENCODE_VALUE);
        MoveViper(HBASKET_POS_VIPER_ENCODE_VALUE);
        //Prepare trajectory sequence3:  backup towards basket
        Pose2d startPose3 = //drive.getPoseEstimate();
                new Pose2d(Sample1Align_x             , Sample1Align_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(startPose3)
                .lineTo(new Vector2d(BasketAlign_x, BasketAlign_y))
                .turn(Math.toRadians(ANGLE))
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence3);
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_BASKET_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }

        //Go to basket and drop specimen
        TrajectorySequence sequenceFWD = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(Basket_distance) // Move fwd
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequenceFWD);
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_BASKET_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }
        GripperOpen();

        TrajectorySequence sequenceBACK = drive.trajectorySequenceBuilder(new Pose2d())
                .back(Basket_distance) // Move fwd
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequenceBACK);
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_BASKET_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }

        Pose2d startPose4 = //drive.getPoseEstimate();
                new Pose2d(Sample1Align_x             , Sample1Align_y, Math.toRadians(90-ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                //.lineTo(new Vector2d(BasketAlign_x, BasketAlign_y))
                .turn(Math.toRadians(ANGLE_S2))
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence4);
        //sleep(900);
        MoveViper(S2_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near sample 1 */
        while (opModeIsActive() && viper.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }

        MoveArm(S2_POS_ARM_ENCODE_VALUE);/* lower  arm */
        while (opModeIsActive() && Armmot.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }

        sleep(DELAY_FOR_S2_GRAB);
        GripperClose();
        MoveArm(HBASKET_S2POS_ARM_ENCODE_VALUE);
        while (opModeIsActive() && Armmot.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }
        MoveViper(HBASKET_S2POS_VIPER_ENCODE_VALUE);


        sleep(DELAY_FOR_S2_TO_BASKET);
        /*while (opModeIsActive() && viper.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_S1_MS)) {
            //wait for arm to lower down completely
        }*/
/*
        Pose2d startPose5 = //drive.getPoseEstimate();
                new Pose2d(Sample1Align_x             , Sample1Align_y, Math.toRadians(90-ANGLE_S2)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                //.lineTo(new Vector2d(BasketAlign_x, BasketAlign_y))

                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence5);
*/

        //Go to basket and drop specimen
        TrajectorySequence sequenceFWD1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(ANGLE_S3))
                .forward(Basket_distance_S2) // Move fwd
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequenceFWD1);
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_BASKET_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }
        GripperOpen();

        TrajectorySequence sequenceBACK1 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(Basket_distance_S2) // Move fwd
                //.turn(Math.toRadians(-1* ANGLE_S3))
                .build();
        timer.reset();
        drive.followTrajectorySequence(sequenceBACK1);
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GOING_BASKET_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }
        GripperClose();
        //MoveArm(ARMStandPOsition);; // Retract viper to original position
        //drive.followTrajectorySequence(sequence5); //go to parking in obs area
        //MoveArm(0);// Retract arm to original position
        //viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot


        //BasketAlign_x
    //    sleep(Specimen2HumanDelay);
    //    drive.followTrajectorySequence(sequence3); //grab specimen
    //    GripperClose();
    //    drive.followTrajectorySequence(sequence4); //go to chamber with 2nd specimen
//
//
    //    sleep(500);
    //    MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
    //    sleep(900);
    //    MoveViper(HCHAMBER_POS_VIPER_ENCODE_VALUE+300);/* Extend viper to get exactly near high chamber */
    //    sleep(2000);
    //    GripperOpen();
    //    MoveArm(HCHAMBER_POS_ARM_ENCODE_VALUE-100); /* move arm little down to prevent specimen touching  gripper after hang */
    //    sleep(900);
    //    GripperClose();
    //    MoveViper(-30);; // Retract viper to original position
    //    drive.followTrajectorySequence(sequence5); //go to parking in obs area
    //    sleep(chamber_viper_retract_sleep);
    //    MoveArm(0);// Retract arm to original position
    //    viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot

        if (isStopRequested()) return;

    }

    public void MoveViper(int vEncoderValue)
    {
        viper.setTargetPosition(vEncoderValue);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(PowerViper);
    }

    public void MoveArm(int aEncoderValue)
    {
        Armmot.setTargetPosition(aEncoderValue);
        Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Armmot.setPower(PowerArm);
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