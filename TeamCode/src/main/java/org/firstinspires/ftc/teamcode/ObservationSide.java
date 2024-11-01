package org.firstinspires.ftc.teamcode;

// RR-specific imports

// Non-RR imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// The following imports will work only after proper tuning.

/*import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
*/


/*
Note that this is just the autonomous structure.
Any of the movement values have not been tested on the field.
For example, in '.forward(30)', the value of 30 may not be correct.
Change these values only after the tuning process has been completed.
There are errors in this that I have not solved.
Many errors can be solved after tuning but some require some more attention
 */

@Autonomous(name="ObservationSide", group ="Autonomous")

public class ObservationSide extends LinearOpMode{
    //The following hardware init code should work and can be used as final.
    public static class Arm{
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "armmot1");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // checks lift's current position
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    arm.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses or goes below
                // 3000 encoder ticks or 0 encoder ticks, then powers it off
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown(){
            return new LiftDown();
        }
    }

    public static class Gripper {
        Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.2);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.5);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Viper{
        private DcMotorEx viper;

        public Viper(HardwareMap hardwareMap) {
            viper = hardwareMap.get(DcMotorEx.class, "viper");
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            viper.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = viper.getCurrentPosition();
                packet.put("viperPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    viper.setPower(0);
                    return false;
                }
            }
        }

        public Action extend() {
            return new Extend();
        }

        public class Unextend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = viper.getCurrentPosition();
                packet.put("viperPos", pos);
                if (pos > -100.0) {
                    return true;
                } else {
                    viper.setPower(0);
                    return false;
                }
            }
        }

        public Action unextend(){
            return new Unextend();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        /*The following line will work once the Mecanum Drive tuning is done.
        Uncomment it then*/
        //MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        //The following 'drive.actionBuilder' error should go away after proper tuning is done.
        //Keep those tuning files in the 'org.firstinspires.ftc.teamcode' folder.
        //Remember that these values in any way do not represent the robots movements.
        //These values are just examples.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        Actions.runBlocking(claw.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        //Fix these errors. I have not attempted.
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        Arm.liftUp(),
                        Gripper.openClaw(),
                        Arm.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    } // end of runOp
}//main end

