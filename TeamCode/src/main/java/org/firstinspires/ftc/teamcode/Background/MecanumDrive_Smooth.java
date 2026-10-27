package org.firstinspires.ftc.teamcode.Background;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
Features:
 *  - Field-relative drive: pushing the stick "up" moves toward the same field direction,
 *    even after the robot turns (using the Pinpoint heading).
 *  - Robot-relative drive: pushing the stick "up" moves toward the robot's front.
 *  - Smooth acceleration and faster deceleration of driver commands.
 *  - Immediate stop method for stopping without waiting for the ramp.

 This class uses the motors and Pinpoint already set up by RobotHardware.
 * Call ONE drive method repeatedly in the TeleOp loop, not just once.

     // 'final' means this reference will continue to point to the same hardware object.
    // 'private' means only code inside this class can use this variable directly.

 */

public class MecanumDrive_Smooth {
    private final RobotHardware robot; // Calls the Hardware Class

    /////////////////////   Variables   /////////////////////
    private double maxSpeed = 1.0;

    // Power units per second. Adjust these to tune driving feel.
        private double accelerationRate = 2.0;
        private double decelerationRate = 8.0;

    /*
    Ramp rate	    Time to full power	        Driving feel
        1.0	            1.00 seconds	     Very smooth, slower response
        1.5	            0.67 seconds	     Smooth
        2.0	            0.50 seconds	     Good starting point
        3.0	            0.33 seconds	     Responsive
        5.0	            0.20 seconds	     Very responsive
     */


        private boolean rampingEnabled = true;  // Set to false if you want to drive without acceleration/deceleration ramping.
        private static final double DEADZONE = 0.05; // prevent unintended movement

    // Remember the smoothed commands from the PREVIOUS TeleOp loop.
        private double smoothForward = 0.0;
        private double smoothRight = 0.0;
        private double smoothRotate = 0.0;

    // Stopwatch: measures elapsed time so ramping does not depend on loop speed.
        private final ElapsedTime rampTimer = new ElapsedTime();

    // Track which driving mode was used last time. We reset the ramp if it changes
    // because field-relative and robot-relative directions mean different things.
        private boolean lastCallWasFieldRelative = false;
        private boolean driveModeInitialized = false;

   //Constructor - passes your existing hardware object into the drive class, giving it access to your motors and Pinpoint.
    public MecanumDrive_Smooth(RobotHardware robot) {
        this.robot = robot;rampTimer.reset();
    }// Begin measuring time for the first ramp update.

    /////////////////////   Set Speed and Acceleration Rates   /////////////////////
        public void setMaxSpeed(double speed) {maxSpeed = Math.max(0.0, Math.min(1.0, speed));} //Set the overall speed limit. Values are kept between 0 and 1.


    /** Set how quickly commands INCREASE/DECREASE toward full speed. */
        public void setAccelerationRate(double rate) {accelerationRate = Math.max(0.1, rate);}// Prevent zero or negative rates, which would make ramping unusable.

        public void setDecelerationRate(double rate) {decelerationRate = Math.max(0.1, rate);}   //Set how quickly commands DECREASE toward zero

    /** Enable or disable ramping. Switching clears the old smoothed commands. */
        public void setRampingEnabled(boolean enabled) {
            if (rampingEnabled != enabled) {
                resetRamp();
            }
            rampingEnabled = enabled;
        }

    /////////////////////  Reset ramp state without commanding any motor movement./////////////////////
    private void resetRamp() {
        smoothForward = 0.0;
        smoothRight = 0.0;
        smoothRotate = 0.0;
        rampTimer.reset();
    }

    /**
     * Move ONE command (forward, right, or rotate) toward the driver's target.
     *
     * target  = what the joystick is asking for right now (-1 to +1).
     * current = last loop's smoothed command.
     * dt      = seconds since the previous update.
     *
     * Example: accelerationRate=2, dt=0.02 -> change by at most 0.04.
     */

    private double rampAxis(double target, double current, double dt) {
    // If ramping is off, follow the stick immediately.
        if (!rampingEnabled) return target;

    // When changing sign, brake to zero before accelerating backwards.
        if (target * current < 0.0) {
            double step = decelerationRate * dt;
            if (Math.abs(current) <= step) return 0.0;
            return current - Math.copySign(step, current);
        }

    // If the requested magnitude is smaller, slow down quickly. - Otherwise, accelerate at the gentler rate.
        double rate = Math.abs(target) < Math.abs(current)
                ? decelerationRate : accelerationRate;
    // Maximum allowed change for this ONE loop.
        double maxChange = rate * dt;
    // Clamp the difference so it cannot jump too far in one update.
        double change = Math.max(-maxChange,
                Math.min(maxChange, target - current));
        return current + change;
    }

    //////////////////////  DEADZONE   -    Prevent unwanted Movement    ///////////////////
    /** Turn tiny joystick values into exactly zero. */
    private double deadzone(double input) {
        return Math.abs(input) < DEADZONE ? 0.0 : input;
    }

    /////////////////////  Smooth joystick commands in the chosen reference frame./////////////////////
    //Smooth all three joystick commands before calculating wheel powers.
    //FieldRelative tells us which coordinate system the commands use.
    private void updateRamp(double forward, double right, double rotate, boolean fieldRelative) {
    // Avoid interpreting previous field-relative inputs as robot-relative.
        if (driveModeInitialized && lastCallWasFieldRelative != fieldRelative) {
            resetRamp();
        }
        driveModeInitialized = true;
        lastCallWasFieldRelative = fieldRelative;

    // dt is time in seconds since the last update.
    // Limit it to 0.1 seconds to avoid a huge jump after a long pause.
        double dt = Math.min(rampTimer.seconds(), 0.1);
        rampTimer.reset();

    // Apply the deadzone, then ramp each axis independently.
        smoothForward = rampAxis(deadzone(forward), smoothForward, dt);
        smoothRight = rampAxis(deadzone(right), smoothRight, dt);
        smoothRotate = rampAxis(deadzone(rotate), smoothRotate, dt);
    }

    /////////////////////  FIELD-RELATIVE DRIVE: ramp field-frame inputs, then rotate into robot frame./////////////////////
    public void driveFieldRelative(double forward, double right, double rotate) {
    // Smooth the driver's field-relative commands first. by updating the ramping values
        updateRamp(forward, right, rotate, true);

    // Convert forward/right into a direction angle and magnitude.
        double theta = Math.atan2(smoothForward, smoothRight);
        double r = Math.hypot(smoothRight, smoothForward);
    // Read the robot's current heading from the Pinpoint in radians.
        double heading = robot.odo.getHeading(AngleUnit.RADIANS);
    // Subtract robot heading to convert field direction to robot direction.
        theta = normalizeRadians(theta - heading);

    // Convert the adjusted angle back to forward/right commands.
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

    // Turn robot-relative commands into individual wheel powers.
        applyMotorPowers(newForward, newRight, smoothRotate);
    }

    /////////////////////  ROBOT-RELATIVE DRIVE: ramp robot-frame inputs. /////////////////////
    public void driveRobotRelative(double forward, double right, double rotate) {
        updateRamp(forward, right, rotate, false);
        applyMotorPowers(smoothForward, smoothRight, smoothRotate);
    }

    /////////////////////  Original Mecanum calculations and normalization, unchanged.  ////////////
    // MECANUM MIXING: calculate the power needed at each wheel.
    //The + and - signs let mecanum wheels drive, strafe, and rotate.
    private void applyMotorPowers(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

    // Normalization: if any wheel exceeds 1.0, scale ALL four equally.
    // Keeping their proportions preserves the requested movement direction.
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

    // Apply the overall speed limit and send power to the four motors.
        robot.frontLeftDrive.setPower(maxSpeed * frontLeftPower / maxPower);
        robot.frontRightDrive.setPower(maxSpeed * frontRightPower / maxPower);
        robot.backLeftDrive.setPower(maxSpeed * backLeftPower / maxPower);
        robot.backRightDrive.setPower(maxSpeed * backRightPower / maxPower);
    }

/////////////////////  Immediate stop: bypass the ramp and clear previous commands. ////////////////
    public void stop() {
        resetRamp();
        applyMotorPowers(0.0, 0.0, 0.0);
    }

    //Reset Pinpoint's position estimate and IMU orientation.
        public void resetPosition() {
        robot.odo.resetPosAndIMU();
    }

    //Return the current Pinpoint heading in RADIANS
        public double getHeading() {
        return robot.odo.getHeading(AngleUnit.RADIANS);
    }
}//Closes Mecanum Drive_Smooth Class
