package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field and Robot Relative_Pinpoint", group = "Robot")
//@Disabled
public class Field_Relative_Pinpoint extends OpMode {
    //Declare Hardware

    // Motors - format DcMotor What_You_Name_It
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;




    //Servos - Format Servo What_You_Name_It or CRSERVO What_You_Name_It (for Continuous Rotation Servos)


    //Sensors
    GoBildaPinpointDriver odo;

    @Override
    //////////////////////// Initialization //////////////////////////////
    public void init() {

    /////////////////////////////////Hardware Map///////////////////////////////
        //Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorBR");


        //Servos

        //Sensors
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        /*Set Motor Directions so all wheels rotate forward with positive power values
            This will be DIFFERENT on EACH ROBOT depending on wheel and motor placement*/
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset the Pinpoint position
        odo.resetPosAndIMU();
    }

    @Override
    /////////////// TeleOp Loop ////////////////////////////////////
    public void loop() {
        telemetry.addLine("Press Back to reset Yaw");
        telemetry.addLine("Hold Left Bumper for Robot Centric");
        telemetry.addLine("Hold Right Trigger for Slow Mode");
        telemetry.addLine("Left joystick sets robot direction");
        telemetry.addLine("Right joystick turns the robot");

        //Update Pinpoint Telemetry
        odo.update();
        telemetry.addData("Heading",odo.getHeading(AngleUnit.RADIANS));


    //////////////   LAUNCHER   //////////////




    ////////////////////////    Drive Code        //////////////////////////
                                    /*
                                    Left Bumper = Robot Centric
                                    Right Trigger = Slow Mode
                                    Back BUtton = Reset Yaw
                                     */

        //Reset the Yaw with the Back Button
        if (gamepad1.a) {
            odo.resetPosAndIMU();
        }

        //Drive Functions Calles
        if (gamepad1.left_bumper && gamepad1.right_trigger > 0) {
            drive(-gamepad1.left_stick_y *.10, gamepad1.left_stick_x *.10, gamepad1.right_stick_x *.10);
        }
        else if (gamepad1.left_bumper && gamepad1.right_trigger == 0){
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        else if (!gamepad1.left_bumper && gamepad1.right_trigger > 0) {
            driveFieldRelative(-gamepad1.left_stick_y *.10, gamepad1.left_stick_x *.10, gamepad1.right_stick_x *.10);
        }
        else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }// Ends Drive If else statements

    }//Closes TeleOp Loop

////////////////////////////////// ENTER FUNCTIONS HERE//////////////////////////////////////

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Get heading from Pinpoint
        double heading = odo.getHeading (AngleUnit.RADIANS);

        // Rotate joystick vector by robot heading
        theta = normalizeRadians(theta - heading);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    //This is the Robot Relative Drive
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

}//Closes Op Mode
