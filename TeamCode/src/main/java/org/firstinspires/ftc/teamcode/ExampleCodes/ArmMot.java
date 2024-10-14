package org.firstinspires.ftc.teamcode.ExampleCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


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

    @Override
    public void runOpMode() {
        //Variables for chassis movement
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        boolean varArmmot1;
        boolean varArmmot2;

        double drivePower = 0.55; //global drive power level


//Hardware maps
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        RRM = hardwareMap.get(DcMotor.class, "RRM");
        RLM = hardwareMap.get(DcMotor.class, "RLM");
        Armmot1 = hardwareMap.get(DcMotor.class, "Armmot1");
        Armmot2 = hardwareMap.get(DcMotor.class, "Armmot2");

        Armmot1.setPower(0.5);
        Armmot2.setPower(0.5);


        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        RRM.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.FORWARD);
        Armmot1.setDirection(DcMotor.Direction.FORWARD);
        Armmot2.setDirection(DcMotor.Direction.FORWARD);



        telemetry.addData("Status", "Initialized");
        telemetry.update();


// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveY = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            varArmmot1 = gamepad1.dpad_up;
            varArmmot2 = gamepad1.dpad_up;

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
            Armmot1.setPower(0.5);
            Armmot2.setPower(0.5);


// Adding telemetry readouts
            telemetry.addData(">", "Robot Running");
            telemetry.addData("Y", driveY);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);

            telemetry.update();
        } //whileOpMode end

    } // runOpMode end
} //end class