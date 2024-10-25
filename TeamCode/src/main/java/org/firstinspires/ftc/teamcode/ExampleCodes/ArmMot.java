package org.firstinspires.ftc.teamcode.ExampleCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*************************************************************************************
 * Robot Control keys
 * Servo - gamepad2.right_bumper
 * Viper/Arm extend - gamepad2.dpad_up
 * Viper/Arm retract - gamepad2.dpad_down
 * Arm swing - gamepad2.left_stick_y
 * driveY = -gamepad1.left_stick_y;
 * strafe = gamepad1.left_stick_x;
 * turn = gamepad1.right_stick_x;
 *
 *************************************************************************************/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="ArmMotOpMode", group="Final")

public class ArmMot extends LinearOpMode{
    /*
    Battery side is the front side!
        RLM : Rear Left Motor(motor0, 0)
        RRM : Rear Right Motor(motor1, 1)
        FLM : Front Left Motor(motor2, 2)
        RM : Front Right motor(motor3, 3)
        Color : Color sensor
        Distance : Distance sensor
        */
    DcMotor RLM;
    DcMotor RRM;
    DcMotor FLM;
    DcMotor FRM;
    DcMotor Armmot1 ;
    DcMotor Armmot2 ;
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

    static final int    MAX_POS_VIPER_ENCODE_VALUE    =   1150;     // Control Max stretch of viper motor to avoid mechanical stress
    static final int    MAX_NEG_VIPER_ENCODE_VALUE    =   -250;     // Control Max stretch of viper motor to avoid mechanical stress

    static final int    MAX_POS_ARM_ENCODE_VALUE    =   1150;     // Control Max stretch of arm motors to avoid mechanical stress
    static final int    MAX_NEG_ARM_ENCODE_VALUE    =   -250;     // Control Max stretch of arm motors to avoid mechanical stress


    @Override
    public void runOpMode() {

        //Variables for chassis movement
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double varArmmot1;
        double varArmmot2;
        double varViper;

        double drivePower = 0.55; //global drive power level
        

//Hardware maps
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//odo x

        FRM = hardwareMap.get(DcMotor.class, "FRM");
        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//odo Y

        RRM = hardwareMap.get(DcMotor.class, "RRM");
        RLM = hardwareMap.get(DcMotor.class, "RLM");

        Armmot1 = hardwareMap.get(DcMotor.class, "armmot1");
        Armmot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armmot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        Armmot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        Armmot2 = hardwareMap.get(DcMotor.class, "armmot2");
        Armmot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armmot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm
        Armmot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        Armmot1.setDirection(DcMotor.Direction.FORWARD);
        Armmot2.setDirection(DcMotor.Direction.FORWARD);
        viper.setDirection(DcMotor.Direction.FORWARD);
        Armmot1.setPower(0);
        Armmot2.setPower(0);
       
        
        viper.setPower(-0.3); //Lift viper above ground so it does not get dragged with robot
        sleep(300);
        viper.setPower(0);
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
            armEncoderValue = Armmot1.getCurrentPosition();
            
            varArmmot1 = gamepad2.left_stick_y;
            varArmmot2 = gamepad2.left_stick_y;
            //varViper = gamepad2.right_stick_x;


            //control arm
            if((gamepad2.left_stick_y != 0)  && 
               (armEncoderValue < MAX_POS_ARM_ENCODE_VALUE) &&
               (armEncoderValue > MAX_NEG_ARM_ENCODE_VALUE)
              ) {
                //limit power to arm motor after a particular number of rotations todo
                Armmot1.setPower((varArmmot1) / 2);
                Armmot2.setPower((varArmmot2) / 2);
            }
            else
            {
                 //limit power to arm motor after a particular number of rotations todo
                Armmot1.setPower(0);
                Armmot2.setPower(0);
            }
            
          
            //Viper Control
            if ((gamepad2.dpad_up)  && (viperEncoderValue < MAX_POS_VIPER_ENCODE_VALUE)) {
                viper.setPower(0.3);
            } else if (gamepad2.dpad_down && (viperEncoderValue > MAX_NEG_VIPER_ENCODE_VALUE)) {
                viper.setPower(-0.3);
            } else {
                viper.setPower(0);
            }

            //grab using gripper
            if(gamepad2.right_bumper){
                gripper.setPosition(1);
            }
            else{
                gripper.setPosition(0);
            }

            // Adding telemetry readouts
            telemetry.addData(">", "Robot Running");
            telemetry.addData("Y", driveY);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.addData("ODO X encoder", odoEncoderXValue);
            telemetry.addData("ODO Y encoder", odoEncoderYValue);
            telemetry.addData("viper encoder", viperEncoderValue);
            telemetry.addData("arm encoder", armEncoderValue);
            telemetry.update();
        } //whileOpMode end

    } // runOpMode end
} //end class
