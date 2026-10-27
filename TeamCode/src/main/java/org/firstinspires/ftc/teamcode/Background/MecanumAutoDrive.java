package org.firstinspires.ftc.teamcode.Background;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * NONBLOCKING encoder-only mecanum autonomous drive.
 *
 * Usage in an iterative OpMode:
 *   1. Call move(...) ONCE to start a movement.
 *   2. Call update() on every loop until SUCCESS or TIMEOUT.
 *   3. Start the next movement only after the previous one finishes.
 *
 * No Pinpoint measurements are used. Motor encoder targets are relative to
 * the CURRENT encoder position, so multiple movements can be chained.
 */
public class MecanumAutoDrive {

/////////////////////     CALIBRATION VARIABLES     /////////////////////
    // 312 RPM goBILDA Yellow Jacket (19.2:1) nominal output-shaft counts.
    public static double TICKS_PER_REV = 537.7;             // goBILDA 19.2:1 Encoder Resolution 537.7 PPR at the Output Shaft
    public static double WHEEL_DIAMETER_INCHES = 96.0 / 25.4; // goBILDA Mecanum Grip Wheel Diameter 96 mm wheel.
    public static double STRAFE_FACTOR = 1.15;              // Tune based on measured sideways travel.
    public static double TURN_TICKS_PER_DEGREE = 10.0;      // Placeholder: tune with a measured 90-degree turn.
    public static double DEFAULT_TIMEOUT_SECONDS = 5.0;    // Maximum allowed movement time.
    public static int POSITION_TOLERANCE_TICKS = 20;        // Allowed error for EACH wheel.

/////////////////////     HARDWARE     /////////////////////
    // Array order is ALWAYS FL, FR, BL, BR.
    private final DcMotor[] motors;

/////////////////////     MOVEMENT STATE VARIABLES     /////////////////////
    public enum Direction { FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT }
    public enum Result { IDLE, RUNNING, SUCCESS, TIMEOUT, CANCELLED, INVALID }

    private final ElapsedTime driveTimer = new ElapsedTime();
    private final int[] targets = new int[4];
    private boolean movementActive = false;
    private double timeoutSeconds = DEFAULT_TIMEOUT_SECONDS;
    private Direction currentDirection = null;
    private Result lastResult = Result.IDLE;

//////////////////////// INITIALIZATION //////////////////////////////
    public MecanumAutoDrive(RobotHardware robot) {
        // Call robot.init(hardwareMap) BEFORE constructing this helper.
        motors = new DcMotor[] {
                robot.frontLeftDrive, robot.frontRightDrive,
                robot.backLeftDrive, robot.backRightDrive
        };
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targets[i] = motors[i].getCurrentPosition();
        }
    }

//////////////////////// START MOVEMENT - CALL ONCE //////////////////////////////
    /**
     * Start a new movement without waiting for it to finish.
     * @param direction Robot-relative movement direction.
     * @param amount Positive inches for translation, positive degrees for rotation.
     * @param power Motor power magnitude from (0, 1].
     * @param timeout Safety timeout in seconds.
     * @return true if the movement was accepted and started.
     */


    public boolean move(Direction direction, double amount, double power, double timeout) {
        /** Amount is Distance is inches for translation, degrees for rotation. Returns true on success. */
        // Never overwrite targets while an earlier movement is still active.
        if (movementActive) return false;

        if (direction == null || !Double.isFinite(amount) || amount < 0 ||
                !Double.isFinite(power) || power <= 0 || power > 1 ||
                !Double.isFinite(timeout) || timeout <= 0 ||
                TICKS_PER_REV <= 0 || WHEEL_DIAMETER_INCHES <= 0 ||
                STRAFE_FACTOR <= 0 || TURN_TICKS_PER_DEGREE <= 0) {
            lastResult = Result.INVALID;
            return false;
        }

        // Zero-distance command is immediately complete; do not energize motors.
        if (amount == 0) {
            lastResult = Result.SUCCESS;
            return true;
        }

        // Convert requested inches/degrees into wheel encoder ticks.
        boolean turning = direction == Direction.ROTATE_LEFT || direction == Direction.ROTATE_RIGHT;
        double ticks = turning ? amount * TURN_TICKS_PER_DEGREE
                : amount * TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES)
                * ((direction == Direction.LEFT || direction == Direction.RIGHT) ? STRAFE_FACTOR : 1.0);
        if (!Double.isFinite(ticks) || ticks > Integer.MAX_VALUE / 2.0) {
            lastResult = Result.INVALID;
            return false;
        }
        int step = (int) Math.round(ticks);
        if (step == 0) {
            lastResult = Result.SUCCESS;
            return true;
        }

        // Mecanum wheel patterns: FL, FR, BL, BR. - To change motor direction to switch robot driving directions
        // These signs assume RobotHardware's motor directions make +ticks drive forward.
        int[] signs;
        switch (direction) {
            case BACKWARD:     signs = new int[] {-1, -1, -1, -1}; break;
            case RIGHT:        signs = new int[] { 1, -1, -1,  1}; break;
            case LEFT:         signs = new int[] {-1,  1,  1, -1}; break;
            case ROTATE_RIGHT: signs = new int[] { 1, -1,  1, -1}; break;
            case ROTATE_LEFT:  signs = new int[] {-1,  1, -1,  1}; break;
            case FORWARD:
            default:           signs = new int[] { 1,  1,  1,  1}; break;
        }

        // Calculate ALL four targets from CURRENT encoder readings.
        // This avoids resetting encoders between autonomous steps.
        for (int i = 0; i < motors.length; i++) {
            long target = (long) motors[i].getCurrentPosition() + (long) signs[i] * step;
            if (target > Integer.MAX_VALUE || target < Integer.MIN_VALUE) {
                lastResult = Result.INVALID;
                return false;
            }
            targets[i] = (int) target;
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // In RUN_TO_POSITION, target position determines direction.
            motors[i].setPower(power);
        }
        currentDirection = direction;
        timeoutSeconds = timeout;
        driveTimer.reset();
        movementActive = true;
        lastResult = Result.RUNNING;
        return true;
    }

    // Convenience overload uses the default timeout.
    public boolean move(Direction direction, double amount, double power) {
        return move(direction, amount, power, DEFAULT_TIMEOUT_SECONDS);
    }

//////////////////////// SIX EASY-TO-READ COMMANDS //////////////////////////////
    // These are all NONBLOCKING: they START the movement but do not wait for it so you can do other operations while these are happening.
    // Like ramping up the flywheel while you are driving to a location.
    public boolean forward(double inches, double power, double timeout) {
        return move(Direction.FORWARD, inches, power, timeout);
    }
    public boolean backward(double inches, double power, double timeout) {
        return move(Direction.BACKWARD, inches, power, timeout);
    }
    public boolean strafeLeft(double inches, double power, double timeout) {
        return move(Direction.LEFT, inches, power, timeout);
    }
    public boolean strafeRight(double inches, double power, double timeout) {
        return move(Direction.RIGHT, inches, power, timeout);
    }
    public boolean rotateLeft(double degrees, double power, double timeout) {
        return move(Direction.ROTATE_LEFT, degrees, power, timeout);
    }
    public boolean rotateRight(double degrees, double power, double timeout) {
        return move(Direction.ROTATE_RIGHT, degrees, power, timeout);
    }

//////////////////////// CHECK MOVEMENT - CALL EACH LOOP //////////////////////////////
    public Result update() {
        // Return the most recent result instead of incorrectly reporting success when idle.
        if (!movementActive) return lastResult;

        // A movement is complete ONLY when all four wheels are within tolerance.
        boolean allAtTarget = true;
        for (int i = 0; i < motors.length; i++) {
            if (Math.abs((long) targets[i] - motors[i].getCurrentPosition()) > POSITION_TOLERANCE_TICKS) {
                allAtTarget = false;
            }
        }
        if (allAtTarget) {
            finish(Result.SUCCESS);
        } else if (driveTimer.seconds() >= timeoutSeconds) {
            // Prevent a stalled robot from driving indefinitely.
            finish(Result.TIMEOUT);
        }
        return lastResult;
    }

//////////////////////// TELEMETRY HELPERS //////////////////////////////
    public boolean isActive() { return movementActive; }
    public Result getResult() { return lastResult; }
    public Direction getDirection() { return currentDirection; }
    public double getElapsedSeconds() { return driveTimer.seconds(); }
    public int getTarget(int wheelIndex) { return targets[wheelIndex]; } // 0 FL, 1 FR, 2 BL, 3 BR.
    public int getCurrentPosition(int wheelIndex) { return motors[wheelIndex].getCurrentPosition(); }

//////////////////////// STOP / CANCEL //////////////////////////////
    // Stop from OpMode.stop() or when an emergency/cancellation occurs.
    public void stop() { finish(Result.CANCELLED); }

    private void finish(Result result) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        movementActive = false;
        lastResult = result;
    }
} // Closes MecanumAutoDrive_Hybrid class
