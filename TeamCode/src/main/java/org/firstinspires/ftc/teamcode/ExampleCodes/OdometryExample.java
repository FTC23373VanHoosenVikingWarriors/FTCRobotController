//
/* Purpose of this file : VW programming team should understand why and how to use odometry */

/* Why to use odometry
Using two odometry wheels in the autonomous mode of FTC can significantly enhance the accuracy of your robot’s navigation. Here’s how it typically works:

    Placement: The two odometry wheels are usually placed perpendicular to each other. One wheel tracks the forward and backward movement (Y-axis), while the other tracks the side-to-side movement (X-axis).
    This setup allows the robot to accurately measure its position on the field in two dimensions.

    Data Collection: As the robot moves, the odometry wheels rotate and generate data about the distance
    traveled. This data is collected by encoders attached to the wheels, which convert the rotational
    movement into digital signals that can be processed by the robot’s control system.

    Position Calculation: The control system uses the data from the odometry wheels to calculate the
    robot’s position on the field. By integrating the distance traveled in both the X and Y directions,
     the system can determine the robot’s current coordinates relative to its starting position.

    Path Correction: During autonomous mode, the robot’s control system continuously monitors the data
     from the odometry wheels to ensure it stays on the intended path. If the robot deviates from its
     planned trajectory, the control system can make real-time adjustments to correct its course.

    Combining with Gyroscope: Often, the odometry system is combined with a gyroscope to track the
    robot’s orientation (heading). This combination allows for more precise control, especially during
    complex maneuvers like turns and strafing.

Using two odometry wheels provides a robust method for tracking the robot’s position and ensuring accurate navigation during autonomous tasks. This setup can be particularly beneficial for tasks that require precise movements, such as placing game elements in specific locations.
 */

/*
* Here’s a basic example of FTC Java code that uses odometry wheels to travel a distance of 10 meters and then make a left turn.
*  This example assumes you have a working odometry setup and a basic understanding of FTC programming.
* *
*
* This code initializes the odometry wheels and uses them to travel a specified distance (10 meters) before making a left turn.
* You’ll need to adjust the constants and the turning logic based on your specific robot’s configuration and behavior.
* */
package org.firstinspires.ftc.teamcode.ExampleCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Odometry Example", group = "Linear Opmode")
public class OdometryExample extends LinearOpMode {

    // Declare hardware
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx leftOdometryWheel;
    private DcMotorEx rightOdometryWheel;

    // Constants
    private static final double TICKS_PER_REV = 8192; // Example value, adjust based on your encoder
    private static final double WHEEL_DIAMETER = 4.0; // in inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private static final double DISTANCE_TO_TRAVEL = 10 * 39.37; // 10 meters in inches

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        leftOdometryWheel = hardwareMap.get(DcMotorEx.class, "left_odometry");
        rightOdometryWheel = hardwareMap.get(DcMotorEx.class, "right_odometry");

        // Reset encoders
        leftOdometryWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometryWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_USING_ENCODER mode
        leftOdometryWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometryWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            // Travel forward 10 meters
            travelDistance(DISTANCE_TO_TRAVEL);

            // Make a left turn
            turnLeft(90); // Assuming 90 degrees turn
        }
    }

    private void travelDistance(double distanceInInches) {
        int targetPosition = (int) (distanceInInches * TICKS_PER_INCH);

        leftOdometryWheel.setTargetPosition(targetPosition);
        rightOdometryWheel.setTargetPosition(targetPosition);

        leftOdometryWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometryWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

        while (opModeIsActive() && leftOdometryWheel.isBusy() && rightOdometryWheel.isBusy()) {
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Left", leftOdometryWheel.getCurrentPosition());
            telemetry.addData("Current Right", rightOdometryWheel.getCurrentPosition());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void turnLeft(int degrees) {
        // Implement your turning logic here
        // This is a placeholder for turning left 90 degrees
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);

        sleep(1000); // Adjust the sleep time based on your robot's turning speed

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
