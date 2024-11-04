package org.firstinspires.ftc.teamcode.ExampleCodes.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous(group = "drive")
public class _VWAutoIntoDeep  extends LinearOpMode {
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
    static final int    HBASKET_POS_VIPER_ENCODE_VALUE    =   3100;     //
    static final int    HBASKET_POS_ARM_ENCODE_VALUE    =   1930;     //

    static final int    LBASKET_POS_VIPER_ENCODE_VALUE    =   0;     //
    static final int    LBASKET_POS_ARM_ENCODE_VALUE    =   0;     //

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



        waitForStart();

        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        timer = new ElapsedTime();

        // Build a trajectory sequence

        MoveArm(300);
        sleep(2000);
        MoveViper(300);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(10)
                .strafeLeft(70)

                .turn(Math.toRadians(-70))
                //.forward(10)
                .build();

        // Follow the trajectory sequence
        drive.followTrajectorySequence(sequence);
        sleep(2000);
        MoveArm(HBASKET_POS_ARM_ENCODE_VALUE);
        MoveViper(HBASKET_POS_VIPER_ENCODE_VALUE);
        sleep(4000);
        GripperOpen();
        sequence = drive.trajectorySequenceBuilder(new Pose2d())

                .forward(10) // Move back 50 inches
                .build();
        // Follow the trajectory sequence
        drive.followTrajectorySequence(sequence);

        GripperClose();
        MoveViper(0);; // Retract arm to original position
        sleep(2000);
        MoveArm(0);

        sequence = drive.trajectorySequenceBuilder(new Pose2d())

                .forward(27) // Move back 50 inches
                .strafeLeft(30)
                .forward(14)
                .build();
        // Follow the trajectory sequence
        drive.followTrajectorySequence(sequence);

        MoveArm(1200);
        sleep(2000);
        MoveViper(1900);
        sleep(1000);
        MoveArm(900);


        if (isStopRequested()) return;

        // Continuous telemetry during autonomous operation
        while (opModeIsActive()) {
            // Telemetry updates
            //Read encoder values
            viperEncoderValue = viper.getCurrentPosition();
            armEncoderValue = Armmot.getCurrentPosition();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("viper encoder", viperEncoderValue);
            telemetry.addData("arm encoder", armEncoderValue);
            telemetry.update();
            // Optional: Add a small sleep to avoid overwhelming the telemetry
            sleep(100); // Sleep for 100 milliseconds
        }
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