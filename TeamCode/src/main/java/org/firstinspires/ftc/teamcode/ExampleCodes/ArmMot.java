package org.firstinspires.ftc.teamcode.ExampleCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



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

    private final int COUNTS_PER_ROTATION = 1440; // Adjust based on your motor's specs
    private double maxArmDistance; // Maximum distance in inches
    private double maArmCounts; // Maximum counts based on the distance
    private double wheelDiameter = 10;

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
        FRM = hardwareMap.get(DcMotor.class, "FRM");
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

//   motor = hardwareMap.get(DcMotor.class, "motor"); // Replace with your motor's name
        //   motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set a calibratable distance (e.g., 12 inches)
        maxArmDistance = 12; // Change this value as needed
        maArmCounts = maxArmDistance * (COUNTS_PER_ROTATION / (Math.PI * wheelDiameter)); // Calculate counts based on wheel diameter

        Armmot1.setPower(0);
        Armmot2.setPower(0);
        viper.setPower(0);



        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        RRM.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.FORWARD);
        Armmot1.setDirection(DcMotor.Direction.FORWARD);
        Armmot2.setDirection(DcMotor.Direction.FORWARD);
        viper.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveY = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            varArmmot1 = gamepad2.left_stick_x;
            varArmmot2 = gamepad2.left_stick_x;
            varViper = gamepad2.right_stick_y;



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

            //limit power to arm motor after a particular number of rotations todo
            Armmot1.setPower(drivePower * varArmmot1);
            Armmot2.setPower(drivePower * varArmmot2);
            Armmot2.getCurrentPosition();

            viper.setPower(varViper); //arm extension , this code needs to be refined.

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

            telemetry.update();
        } //whileOpMode end

    } // runOpMode end
} //end class