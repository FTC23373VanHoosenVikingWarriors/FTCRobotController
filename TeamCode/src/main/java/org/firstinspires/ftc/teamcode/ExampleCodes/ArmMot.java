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
        boolean holdArmPosition = true; //variable to hold arm in position , to prevent arm falling down due to gravity


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
        Armmot2 = hardwareMap.get(DcMotor.class, "armmot2");
        Armmot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Armmot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm

        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//make use of encoder to limit spin of motor so we dont damage arm

        gripper = hardwareMap.get(Servo.class, "gripper");
        //gripper.setPosition();

//   motor = hardwareMap.get(DcMotor.class, "motor"); // Replace with your motor's name
        //   motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set a calibratable distance (e.g., 12 inches)
        maxArmDistance = 12; // Change this value as needed
        maArmCounts = maxArmDistance * (COUNTS_PER_ROTATION / (Math.PI * wheelDiameter)); // Calculate counts based on wheel diameter

        Armmot1.setPower(0);
        Armmot2.setPower(0);
        viper.setPower(0);



       // FLM.setDirection(DcMotor.Direction.REVERSE);
       // FRM.setDirection(DcMotor.Direction.REVERSE);
       // RRM.setDirection(DcMotor.Direction.REVERSE);
       // RLM.setDirection(DcMotor.Direction.REVERSE);
        Armmot1.setDirection(DcMotor.Direction.FORWARD);
        Armmot2.setDirection(DcMotor.Direction.FORWARD);
        viper.setDirection(DcMotor.Direction.FORWARD);

        /* viper.setPower(0.5); //Lift viper above ground so it does not get dragged with robot
        move it up and then set power to zero tood, encoder needed to test */


        telemetry.addData("Status", "Initialized");
        telemetry.update();


// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //stop wheels from spinning forever
            FLM.setPower(0);
            RLM.setPower(0);
            FRM.setPower(0);
            RRM.setPower(0);
            //driveY = -gamepad1.left_stick_y;
            //strafe = gamepad1.left_stick_x;
            //turn = gamepad1.right_stick_x;
            varArmmot1 = gamepad2.left_stick_y;
            varArmmot2 = gamepad2.left_stick_y;
            //varViper = gamepad2.right_stick_x;


            //control arm
            if(gamepad2.left_stick_y != 0) {
                //limit power to arm motor after a particular number of rotations todo
                Armmot1.setPower((varArmmot1) / 2);
                Armmot2.setPower((varArmmot2) / 2);
            }
            else
            {
                if(holdArmPosition) {
                    //hold arm in  position
                    Armmot1.setPower(0.1);
                    Armmot2.setPower(0.1);
                    holdArmPosition = false;
                }
                else
                {
                    Armmot1.setPower(-0.1);
                    Armmot2.setPower(-0.1);
                    holdArmPosition = true;
                }
            }
            Armmot2.getCurrentPosition();

            //viper.setPower(varViper/ 0.9); //arm extension , this code needs to be refined.
            //Viper Control
            if (gamepad2.dpad_up) {
                viper.setDirection(DcMotor.Direction.FORWARD);
                viper.setPower(0.7);
            } else if (gamepad2.dpad_down) {
                viper.setDirection(DcMotor.Direction.REVERSE);
                viper.setPower(0.2);
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

            //Go forward
            if(gamepad1.dpad_up){
                RLM.setDirection(DcMotor.Direction.REVERSE);
                RRM.setDirection(DcMotor.Direction.FORWARD);
                FLM.setDirection(DcMotor.Direction.REVERSE);
                FRM.setDirection(DcMotor.Direction.FORWARD);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }
            //Backward
            if(gamepad1.dpad_down){
                RLM.setDirection(DcMotor.Direction.FORWARD);
                RRM.setDirection(DcMotor.Direction.REVERSE);
                FLM.setDirection(DcMotor.Direction.FORWARD);
                FRM.setDirection(DcMotor.Direction.REVERSE);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }
            //Crabwalk left
            if(gamepad1.dpad_left){
                RLM.setDirection(DcMotor.Direction.REVERSE);
                RRM.setDirection(DcMotor.Direction.REVERSE);
                FLM.setDirection(DcMotor.Direction.FORWARD);
                FRM.setDirection(DcMotor.Direction.FORWARD);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }
            //Crabwalk right
            if(gamepad1.dpad_right){
                RLM.setDirection(DcMotor.Direction.FORWARD);
                RRM.setDirection(DcMotor.Direction.FORWARD);
                FLM.setDirection(DcMotor.Direction.REVERSE);
                FRM.setDirection(DcMotor.Direction.REVERSE);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }
            //Turn left
            if(gamepad1.x){
                RLM.setDirection(DcMotor.Direction.FORWARD);
                RRM.setDirection(DcMotor.Direction.FORWARD);
                FLM.setDirection(DcMotor.Direction.FORWARD);
                FRM.setDirection(DcMotor.Direction.FORWARD);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }
            //Turn right
            if(gamepad1.b){
                RLM.setDirection(DcMotor.Direction.REVERSE);
                RRM.setDirection(DcMotor.Direction.REVERSE);
                FLM.setDirection(DcMotor.Direction.REVERSE);
                FRM.setDirection(DcMotor.Direction.REVERSE);

                FLM.setPower(0.5);
                RLM.setPower(0.5);
                FRM.setPower(0.5);
                RRM.setPower(0.5);
            }



            odoEncoderXValue = FRM.getCurrentPosition();
            odoEncoderYValue = FLM.getCurrentPosition();
            viperEncoderValue = viper.getCurrentPosition();
            armEncoderValue = Armmot1.getCurrentPosition();


/* //logic to limit arm motor
            double power = -gamepad1.left_stick_y; // Invert for forward movement
            int currentPosition = Armmot1.getCurrentPosition();

            // Check if moving forward or backward exceeds the max counts
            if (power > 0 && currentPosition < maxCounts) {
                Armmot1.setPower(power);
            } else if (power < 0 && currentPosition > 0) {
                Armmot1.setPower(power);
            } else {
                Armmot1.setPower(0); // Stop if limit is reached
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

            telemetry.update();
        } //whileOpMode end

    } // runOpMode end
} //end class
