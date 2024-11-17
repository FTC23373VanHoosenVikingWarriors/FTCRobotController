package org.firstinspires.ftc.teamcode.ExampleCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
This is teleop program where viper motor is give 0 power in idle condition, to avoid overheating of motor
 */
@TeleOp(name="ArmMotNoLimitViper0Pwr", group="Final")

public class ArmMotNoLimitViper0Pwr extends LinearOpMode{
    /*
    Battery side is the front side!
        RLM : Rear Left Motor(motor0, 0)
        RRM : Rear Right Motor(motor1, 1)
        FLM : Front Left Motor(motor2, 2)
        RM : Front Right motor(motor3, 3)
    */
    DcMotor RLM;
    DcMotor RRM;
    DcMotor FLM;
    DcMotor FRM;
    DcMotor Armmot ;
    DcMotor viper;
    Servo gripper;

    private final int COUNTS_PER_ROTATION = 1440; // Adjust based on your motor's specs
    private double maxArmDistance; // Maximum distance in inches
    private double maArmCounts; // Maximum counts based on the distance
    private double wheelDiameter = 10;
    private int armEncoderValue = 0;
    private int viperEncoderValue = 0;
    private int odoEncoderXValue = 0;
    private int odoEncoderYValue = 0;

//    static final int    MAX_POS_VIPER_ENCODE_VALUE    =   5000;     // Control Max stretch of viper motor to avoid mechanical stress
 //   static final int    MAX_NEG_VIPER_ENCODE_VALUE    =   -100;     // Control Max stretch of viper motor to avoid mechanical stress

//    static final int    MAX_POS_ARM_ENCODE_VALUE    =   5000;     // Control Max stretch of arm motors to avoid mechanical stress
//    static final int    MAX_NEG_ARM_ENCODE_VALUE    =   0;     // Control Max stretch of arm motors to avoid mechanical stress

    static final int    VIPER_RETRACT_ENCODER_FRM_GRND = -100; //encoder reading which keep viper/gripeer from up from mother earth , this avoid gripper drag
    static final int    ARM_ENCODER_READING_AFTER_START = 500; //Initial reading where ARM could be for first operation driver wants to perform
    static final int    VIPER_ENCODER_READING_AFTER_START = 500; //Initial reading where viper could be for first operation driver wants to perform

    static final int    HBASKET_POS_VIPER_ENCODE_VALUE    =   2100;     //
    static final int    HBASKET_POS_ARM_ENCODE_VALUE    =   1800;     //

    static final int    HPLAYER_POS_VIPER_ENCODE_VALUE    =   -15;     //
    static final int    HPLAYER_POS_ARM_ENCODE_VALUE    =   2781;     //

    static final int    HANG_POS_VIPER_ENCODE_VALUE    =   1460;     //
    static final int    HANG_POS_ARM_ENCODE_VALUE    =   1290;     //


    @Override
    public void runOpMode() {

        //Variables for chassis movement
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double varArmmot;
        int armHoldReading = ARM_ENCODER_READING_AFTER_START; //Initial reading where ARM could be for first operation driver wants to perform
        int viperHoldReading = VIPER_ENCODER_READING_AFTER_START;//Initial reading where viper could be for first operation driver wants to perform
        double drivePower = 0.65; //global drive power level


//Hardware maps
        FLM = hardwareMap.get(DcMotor.class, "FLM");
    //    FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //    FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//odo x
    //    FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRM = hardwareMap.get(DcMotor.class, "FRM");
    //    FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //    FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//odo Y
    //    FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RRM = hardwareMap.get(DcMotor.class, "RRM");
    //    RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //    RRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    RRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RLM = hardwareMap.get(DcMotor.class, "RLM");
    //    RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //    RLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    RLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Armmot = hardwareMap.get(DcMotor.class, "armmot");
        Armmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        Armmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper = hardwareMap.get(Servo.class, "gripper");

        // Set a calibratable distance (e.g., 12 inches)
        maxArmDistance = 12; // Change this value as needed
        maArmCounts = maxArmDistance * (COUNTS_PER_ROTATION / (Math.PI * wheelDiameter)); // Calculate counts based on wheel diameter



        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        RRM.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.FORWARD);
        Armmot.setDirection(DcMotor.Direction.REVERSE);
        viper.setDirection(DcMotor.Direction.REVERSE);
        Armmot.setPower(0);

        viper.setTargetPosition(VIPER_RETRACT_ENCODER_FRM_GRND);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(0); //Lift viper above ground so it does not get dragged with robot

        // Wait for the game to start (driver presses PLAY)


        telemetry.addData("VW Robot", "Initialized");
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveY = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);

            double frontLeftPower = (driveY + strafe + turn) / denominator;
            double backLeftPower = (driveY - strafe + turn) / denominator;
            double frontRightPower = (driveY - strafe - turn) / denominator;
            double backRightPower = (driveY + strafe - turn) / denominator;

            FLM.setPower(drivePower * frontLeftPower);
            RLM.setPower(drivePower * backLeftPower);
            FRM.setPower(drivePower * frontRightPower);
            RRM.setPower(drivePower * backRightPower);

            //Read encoder values
            odoEncoderXValue = FRM.getCurrentPosition();
            odoEncoderYValue = FLM.getCurrentPosition();
            viperEncoderValue = viper.getCurrentPosition();
            armEncoderValue = Armmot.getCurrentPosition();

            varArmmot = gamepad2.right_stick_y;



            //control arm
            if((varArmmot != 0)  /*  &&
                    (armEncoderValue <= MAX_POS_ARM_ENCODE_VALUE) &&
                    (armEncoderValue >= MAX_NEG_ARM_ENCODE_VALUE) */
            ) {
                Armmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Armmot.setPower(-(varArmmot / 2));
                armHoldReading = Armmot.getCurrentPosition();
            }
            else
            {

                if((gamepad2.x ) || (gamepad2.y ) || (gamepad2.back))
                {
                    //Do nothing here
                }
                else{
                    Armmot.setTargetPosition(armHoldReading);
                    Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Armmot.setPower(0.4);
                }


            }


            //Viper Control
            if ((gamepad2.dpad_up) /*  && (viperEncoderValue <= MAX_POS_VIPER_ENCODE_VALUE)*/) {

                viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper.setPower(0.6);
                viperHoldReading = viper.getCurrentPosition();
            } else if (gamepad2.dpad_down /* && (viperEncoderValue >= MAX_NEG_VIPER_ENCODE_VALUE) */) {

                viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper.setPower(-0.6);
                viperHoldReading = viper.getCurrentPosition();
            } else {
                if((gamepad2.x ) || (gamepad2.y ) || (gamepad2.back))
                {
                    //Do nothing here
                }
                else{
                viper.setTargetPosition(viperHoldReading);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(0);
                }
            }

            //grab using gripper
            if(gamepad2.right_bumper){
                gripper.setPosition(0.5);

            }
            else{
                gripper.setPosition(0.2);
            }

            //target lower basket
            if(gamepad2.x )
            {
                Armmot.setTargetPosition(HPLAYER_POS_ARM_ENCODE_VALUE);
                Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Armmot.setPower(0.4);
                armHoldReading = HPLAYER_POS_ARM_ENCODE_VALUE;

                viper.setTargetPosition(HPLAYER_POS_VIPER_ENCODE_VALUE);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(0.6);
                viperHoldReading = HPLAYER_POS_VIPER_ENCODE_VALUE;
            }
/*
            //target higher basket
            if(gamepad2.y )
            {
                Armmot.setTargetPosition(HBASKET_POS_ARM_ENCODE_VALUE);
                Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Armmot.setPower(0.4);
                armHoldReading = HBASKET_POS_ARM_ENCODE_VALUE;

                viper.setTargetPosition(HBASKET_POS_VIPER_ENCODE_VALUE);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(0.6);
                viperHoldReading = HBASKET_POS_VIPER_ENCODE_VALUE;
            }

            //ready to hang higher rung
            if(gamepad2.back)
            {
                Armmot.setTargetPosition(HANG_POS_ARM_ENCODE_VALUE);
                Armmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Armmot.setPower(0.4);
                armHoldReading = HANG_POS_ARM_ENCODE_VALUE;

                viper.setTargetPosition(HANG_POS_VIPER_ENCODE_VALUE);
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(0.6);
                viperHoldReading = HANG_POS_VIPER_ENCODE_VALUE;
            }

*/
            // Adding telemetry readouts
            telemetry.addData(">", "Robot Running");
            telemetry.addData("Y", driveY);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.addData("ODO X encoder", odoEncoderXValue);
            telemetry.addData("ODO Y encoder", odoEncoderYValue);
            telemetry.addData("viper encoder", viperEncoderValue);
            telemetry.addData("arm encoder", armEncoderValue);
            telemetry.addData("gamepad2 right_stick_y", gamepad2.right_stick_y);


            telemetry.update();
        } //whileOpMode end

    } // runOpMode end
} //end class