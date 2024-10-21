package org.firstinspires.ftc.teamcode.ExampleCodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends LinearOpMode {

    public Servo gripper_servo;

    // called when init button is  pressed.
    @Override
    public void runOpMode() {
        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = this.gamepad1.left_stick_y;
            // check to see if we need to move the servo.
            if (gamepad1.y) {
                // move to 0 degrees.
                gripper_servo.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                gripper_servo.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                gripper_servo.setPosition(1);
            }
            telemetry.addData("Servo Position", gripper_servo.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
// speed servo is main servo used for gripper.