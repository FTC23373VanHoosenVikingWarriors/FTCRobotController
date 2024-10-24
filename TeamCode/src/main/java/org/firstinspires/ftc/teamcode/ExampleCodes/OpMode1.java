package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoMotor;

@TeleOp(name="TeleOP", group="Final")

public class OpMode1 extends LinearOpMode{
    DcMotor RLM;
    DcMotor RRM;
    DcMotor FLM;
    DcMotor FRM;
    DcMotor armmot1;
    DcMotor armmot2;
    //Servo gripper;
    DcMotor viper;

    @Override
    public void runOpMode() {
        //Variables for chassis movement
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double armPower = 0;
        double extPower = 0;
        double drivePower = 0.55; //global drive power level


//Hardware maps
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        RRM = hardwareMap.get(DcMotor.class, "RRM");
        RLM = hardwareMap.get(DcMotor.class, "RLM");
        armmot1 = hardwareMap.get(DcMotor.class, "armmot1");
        armmot2 = hardwareMap.get(DcMotor.class, "armmot2");
        //gripper = hardwareMap.get(Servo.class, "gripper");
        viper = hardwareMap.get(DcMotor.class, "viper");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        RRM.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.FORWARD);
        armmot1.setDirection(DcMotor.Direction.FORWARD);
        armmot2.setDirection(DcMotor.Direction.FORWARD);
        viper.setDirection(DcMotor.Direction.FORWARD);
        //gripper.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    
// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveY = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            armPower = -gamepad2.left_stick_y;
            extPower = gamepad1.left_stick_x;


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
            armmot1.setPower(armPower);
            armmot2.setPower(armPower);
            viper.setPower(extPower);

            //if (gamepad2.a){
            //    gripper.setPosition(1);
            //}
            //else{
            //    gripper.setPosition(0);
            //}
// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Y", driveY);
                telemetry.addData("strafe", strafe);
                telemetry.addData("turn", turn);
                telemetry.addData("arm", armPower);
                telemetry.addData("extension", extPower);

                telemetry.update();
        } //whileOpMode end                                        

    } // runOpMode end
} //end class

