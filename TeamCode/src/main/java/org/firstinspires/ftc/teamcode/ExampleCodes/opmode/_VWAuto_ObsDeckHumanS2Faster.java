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
@Autonomous(name="_VWA_ObsDeckHumanS2",group = "drive")
public class _VWAuto_ObsDeckHumanS2Faster extends LinearOpMode {
    DcMotor Armmot ;
    DcMotor viper;
    Servo gripper;
    static final int    VIPER_RETRACT_ENCODER_FRM_GRND = -100; //encoder reading which keep viper/gripeer from up from mother earth , this avoid gripper drag
    private ElapsedTime timer;
    //public static int    HCHAMBER_POS_VIPER_ENCODE_VALUE    =   0;     //
    public static int    HCHAMBER_POS_ARM_ENCODE_VALUE    =   1100;     //

    public static int    StartPosition_x    =   10;     //
    public static int    StartPosition_y    =   -62;     //

    public static int    ChamberPosition_x    =   10;     //
    public static int    ChamberPosition_y    =   -35;
    public static int comebackFrmChamber = -10;

    public static int    parking_x    =   46;     //
    public static int    parking_y    =   -56;     //

    public static int    Specimen2Hang_x    =   00;     //
    public static int    Specimen2Hang_y    =   -32;     //

    public static int    Specimen1align_x    =   46;     //
    public static int    Specimen1align_y    =   -48;     //

    public static int SpecimenPick_x = 46;
    public static int SpecimenPick_y = -54;
    public static int StepOutDistance = -10;

    public static int    DelayStartForRaisingARMandViper    = 800  ;     //
    public static int DelayAfterJhatka = 300;
    public static int    Specimen2HumanDelay    =   500;     //

    //public static int chamber_viper_retract_sleep = 00;
    //public static int chamber_arm_retract_sleep = 300;
    public static  int    HPLAYER_POS_VIPER_ENCODE_VALUE    =   -94;     //
    public static  int    HPLAYER_POS_ARM_ENCODE_VALUE    =   650;     //
    public static double ANGLE = 182; // deg
    public static double ANGLE_S2 = -178;
    public static int    S2_HCHAMBER_POS_VIPER_ENCODE_VALUE    =   0;     //
    public static int    S2_HCHAMBER_POS_ARM_ENCODE_VALUE    =   1200;
    public static int S2_JHATKA = 300;
    public static int ChamberFWDDistatnce = 12;
    //public static int TRAVEL_TIME_TO_CHAMBER = 300;
    public static int MAX_WAIT_GOING_CHAMBER_MS = 5000;
    public static int MAX_WAIT_STRAFE_2SPECIMEN_MS = 15000;
    public static int MAX_WAIT_GO_CHAMBER_S2_MS = 5000;


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









        timer = new ElapsedTime();
        waitForStart();

        //Start Code from here

        GripperClose();
        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        //timer = new ElapsedTime();

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


        //Prepare trajectory sequence2: go back a little
        Pose2d startPose2 = //drive.getPoseEstimate();//
                new Pose2d(ChamberPosition_x, ChamberPosition_y, Math.toRadians(90));
        //drive.setPoseEstimate(startPose2);
        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(startPose2)
                .lineTo(new Vector2d(ChamberPosition_x, ChamberPosition_y+comebackFrmChamber))
                .lineTo(new Vector2d(36, -43)) //strafe right
                .lineTo(new Vector2d(36, -8)) //go above
                .lineTo(new Vector2d(46, -8)) //strafe again
                .lineTo(new Vector2d(Specimen1align_x, Specimen1align_y)) //bring specimen to obs
                .lineTo(new Vector2d(Specimen1align_x, Specimen1align_y-StepOutDistance))
                .turn(Math.toRadians(ANGLE))
                //.lineTo(new Vector2d(56, -8))  //go for another sample
                .lineTo(new Vector2d(SpecimenPick_x, SpecimenPick_y))  //bring another sample to obs\

                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence2);

        MoveArm(HPLAYER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        MoveViper(HPLAYER_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near high chamber */
        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_STRAFE_2SPECIMEN_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }
        sleep(Specimen2HumanDelay);
        GripperClose();


        MoveArm(S2_HCHAMBER_POS_ARM_ENCODE_VALUE);/* Extend  arm */
        MoveViper(S2_HCHAMBER_POS_VIPER_ENCODE_VALUE);/* Extend viper to get exactly near high chamber */
        //Prepare trajectory sequence4:   go chamber with 2nd specimen
        Pose2d startPose4 = //drive.getPoseEstimate();
                new Pose2d(SpecimenPick_x, SpecimenPick_y, Math.toRadians(90+ANGLE)); //initial position of bot
        //drive.setPoseEstimate(startPose4);
        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(startPose4)
                .lineTo(new Vector2d(Specimen2Hang_x, Specimen2Hang_y)) //Go near chamber
                .turn(Math.toRadians(ANGLE_S2))
                .lineTo(new Vector2d(Specimen2Hang_x, Specimen2Hang_y+ChamberFWDDistatnce)) //Go near chamber

                .build();
        timer.reset();
        drive.followTrajectorySequence(sequence4); //go to chamber with 2nd specimen

        // Wait until trajectory is completed
        while (opModeIsActive() && drive.isBusy() && (timer.milliseconds() < MAX_WAIT_GO_CHAMBER_S2_MS)) {
            // Get and display the current pose during the trajectory execution
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);
            telemetry.update();
        }
        MoveArm(S2_HCHAMBER_POS_ARM_ENCODE_VALUE-  S2_JHATKA );/* Extend  arm */
        sleep(DelayAfterJhatka);
        GripperOpen();


        //Prepare trajectory sequence5:  go park in obs area
        Pose2d startPose5 = //drive.getPoseEstimate();
                new Pose2d(Specimen2Hang_x, Specimen2Hang_y, Math.toRadians(90)); //initial position of bot
        //drive.setPoseEstimate(startPose5);
        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(startPose5)
                .lineTo(new Vector2d(parking_x, parking_y))
                .build();
        drive.followTrajectorySequence(sequence5); //go to parking in obs area
        GripperClose();
        MoveArm(0);// Retract arm to original position
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