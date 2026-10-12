package org.firstinspires.ftc.teamcode.Background;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrive {

// Reference to the shared robot hardware
private final RobotHardware robot;

    // Maximum allowed drive speed
    private double maxSpeed = 1.0;


    // Constructor
    public MecanumDrive(RobotHardware robot) {
        this.robot = robot;
    }

    // Set maximum speed (0.0 to 1.0)
    public void setMaxSpeed(double speed) {
        maxSpeed = Math.max(0.0, Math.min(1.0, speed));
    }

    // FIELD-RELATIVE DRIVE
    public void driveFieldRelative(
            double forward, double right, double rotate) {

        // Convert joystick input to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Read current robot heading from Pinpoint
        double heading = robot.odo.getHeading(AngleUnit.RADIANS);

        // Rotate the joystick vector by robot heading
        theta = normalizeRadians(theta - heading);

        // Convert back to robot-relative coordinates
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Send calculated movement to the motors
        driveRobotRelative(newForward, newRight, rotate);
    }

    // ROBOT-RELATIVE DRIVE
    public void driveRobotRelative(
            double forward, double right, double rotate) {

        // Calculate power for each mecanum wheel
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Normalize motor powers to avoid exceeding 1.0
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // Apply power to each motor
        robot.frontLeftDrive.setPower(
                maxSpeed * frontLeftPower / maxPower);

        robot.frontRightDrive.setPower(
                maxSpeed * frontRightPower / maxPower);

        robot.backLeftDrive.setPower(
                maxSpeed * backLeftPower / maxPower);

        robot.backRightDrive.setPower(
                maxSpeed * backRightPower / maxPower);
    }

    // STOP ALL DRIVE MOTORS
    public void stop() {
        driveRobotRelative(0, 0, 0);
    }

    // RESET PINPOINT POSITION AND IMU
    public void resetPosition() {
        robot.odo.resetPosAndIMU();
    }

    // GET CURRENT HEADING IN RADIANS
    public double getHeading() {
        return robot.odo.getHeading(AngleUnit.RADIANS);
    }

}//Closes Public Class
