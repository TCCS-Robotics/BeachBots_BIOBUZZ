package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Background.MecanumDrive_Smooth;
import org.firstinspires.ftc.teamcode.Background.RobotHardware;

@TeleOp(name = "Beach Bots Teleop", group = "Robot")
//@Disabled
public class BeachBotsTeleop extends OpMode {

    //Import Hardware Class & Mecanum Drive Class
    RobotHardware robot = new RobotHardware();
    MecanumDrive_Smooth drive;

    //Declare variables
    private boolean previousResetButton = false;


    @Override
    //////////////////////// Initialization //////////////////////////////
    public void init() {
        //Initialize Hardware Class & Mecanum Drive Class
        robot.init(hardwareMap);
        drive = new MecanumDrive_Smooth(robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    /////////////// TeleOp Loop ////////////////////////////////////
    public void loop() {

        // Update Pinpoint before calculating field-relative drive.
        robot.odo.update();

        // GAMEPAD 1: Back resets position and IMU once per button press.
        if (gamepad1.back && !previousResetButton) {
            robot.odo.resetPosAndIMU();
        }
        previousResetButton = gamepad1.back;


        //////////////   INTAKE   //////////////

        // GAMEPAD 2: intake forward/reverse, proportional to triggers.
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;


                //////////////   LAUNCHER   //////////////

        // GAMEPAD 2: hold right bumper to spin up and launch.
        if (gamepad2.right_bumper) {
            robot.launcher.setVelocity(robot.LAUNCHER_TARGET_VELOCITY);
        }
        else {
            robot.launcher.setPower(0);
        }

        if (gamepad2.a) {
            robot.windmillServo.setDirection(CRServo.Direction.FORWARD);
            robot.windmillServo.setPower(1);
        }
        else {
            robot.windmillServo.setPower(0);
        }

        // Intake Power Settings
        intakePower = Math.max(-1.0, Math.min(1.0, intakePower));
        robot.intake.setPower(intakePower);
        robot.leftIntakeServo.setPower(intakePower);
        robot.rightIntakeServo.setPower(intakePower);




    ////////////////////////    Drive Code        //////////////////////////
                            /*
                            Left Bumper = Robot Centric
                            Right Trigger = Slow Mode
                            Back BUtton = Reset Yaw
                             */

        //Reset the Yaw with the Back Button
        // Read gamepad 1
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // GAMEPAD 1: Back resets position and IMU once per button press.
        if (gamepad1.back && !previousResetButton) {
            robot.odo.resetPosAndIMU();
        }
        previousResetButton = gamepad1.a;
        // Slow mode while holding right trigger
        if (gamepad1.right_trigger > 0) {
            drive.setMaxSpeed(0.25);
        } else {
            drive.setMaxSpeed(1.0);
        }

        // Left bumper switches to robot-relative
        if (gamepad1.left_bumper) {
            drive.driveRobotRelative(forward, right, rotate);
        } else {
            drive.driveFieldRelative(forward, right, rotate);
        }


        //////////////////         TELEMETRY              ////////////////////

        telemetry.addData("Drive mode", gamepad1.left_bumper ? "Robot-centric" : "Field-centric");
        telemetry.addData("Heading (radians)", robot.odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Launcher actual (ticks/s)", robot.launcher.getVelocity());
        telemetry.addLine("Press Back to reset Yaw");
        telemetry.addLine("Hold Left Bumper for Robot Centric");
        telemetry.addLine("Hold Right Trigger for Slow Mode");
        telemetry.addData("Heading", drive.getHeading());
        telemetry.update();

    }//Closes TeleOp Loop

////////////////////////////////// ENTER FUNCTIONS HERE//////////////////////////////////////


///////////////////     Put STOP OPMODE BEHAVIOR HERE  /////////////////////////////
    @Override
    public void stop() {
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.intake.setPower(0);
        robot.launcher.setVelocity(0);
        robot.leftIntakeServo.setPower(0);
        robot.rightIntakeServo.setPower(0);
        robot.windmillServo.setPower(0);
    }/// Closes Stop Op Mode Behavior

}//Closes Op Mode
