/* modified the OdometryExample code to avoid an obstacle and go around it,
you can add logic to detect the obstacle and then adjust the robot’s path accordingly

In this modified code, a DistanceSensor is used to detect obstacles.
When an obstacle is detected within a certain threshold distance,
the robot stops and executes a maneuver to go around the obstacle before resuming its original path.
 */
/*
package org.firstinspires.ftc.teamcode.ExampleCodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Odometry Obstacle Avoidance", group = "Linear Opmode")
public class OdometryObstacleAvoidance extends LinearOpMode {

    // Declare hardware
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx leftOdometryWheel;
    private DcMotorEx rightOdometryWheel;
    private DistanceSensor frontSensor;

    // Constants
    private static final double TICKS_PER_REV = 8192; // Example value, adjust based on your encoder
    private static final double WHEEL_DIAMETER = 4.0; // in inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private static final double DISTANCE_TO_TRAVEL = 10 * 39.37; // 10 meters in inches
    private static final double OBSTACLE_DISTANCE_THRESHOLD = 10.0; // in inches

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        leftOdometryWheel = hardwareMap.get(DcMotorEx.class, "left_odometry");
        rightOdometryWheel = hardwareMap.get(DcMotorEx.class, "right_odometry");
        frontSensor = hardwareMap.get(DistanceSensor.class, "front_sensor");

        // Reset encoders
        leftOdometryWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometryWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_USING_ENCODER mode
        leftOdometryWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometryWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            // Travel forward 10 meters with obstacle avoidance
            travelDistanceWithObstacleAvoidance(DISTANCE_TO_TRAVEL);

            // Make a left turn
            turnLeft(90); // Assuming 90 degrees turn
        }
    }

    private void travelDistanceWithObstacleAvoidance(double distanceInInches) {
        int targetPosition = (int) (distanceInInches * TICKS_PER_INCH);

        leftOdometryWheel.setTargetPosition(targetPosition);
        rightOdometryWheel.setTargetPosition(targetPosition);

        leftOdometryWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOdometryWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

        while (opModeIsActive() && leftOdometryWheel.isBusy() && rightOdometryWheel.isBusy()) {
            if (frontSensor.getDistance(DistanceUnit.INCH) < OBSTACLE_DISTANCE_THRESHOLD) {
                // Obstacle detected, stop and go around
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                // Go around the obstacle
                goAroundObstacle();

                // Resume travel
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
            }

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Left", leftOdometryWheel.getCurrentPosition());
            telemetry.addData("Current Right", rightOdometryWheel.getCurrentPosition());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void goAroundObstacle() {
        // Example logic to go around an obstacle
        // Adjust these values based on your robot's capabilities and obstacle size

        // Move right to avoid the obstacle
        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        sleep(500); // Adjust the sleep time based on your robot's turning speed

        // Move forward past the obstacle
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(1000); // Adjust the sleep time based on the obstacle size

        // Move left to get back on the original path
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(500); // Adjust the sleep time based on your robot's turning speed

        // Move forward to continue on the original path
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(1000); // Adjust the sleep time based on the obstacle size
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
*/