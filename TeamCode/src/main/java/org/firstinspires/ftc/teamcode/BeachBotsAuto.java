package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Background.MecanumAutoDrive;
import org.firstinspires.ftc.teamcode.Background.RobotHardware;

@Autonomous(name = "BeachBots Auto Hybrid", group = "BeachBots")
public class BeachBotsAuto extends OpMode {

/////////////////////     HARDWARE     /////////////////////
    // RobotHardware owns all hardwareMap names, motor directions, and servo setup.
    private final RobotHardware robot = new RobotHardware();
    // Separate class handles all four mecanum drive motor encoder targets.
    private MecanumAutoDrive drive;

/////////////////////     VARIABLES     /////////////////////
    // These values survive between calls to loop().
    private static final double LAUNCH_TIME_SECONDS = 10.0; // Time spent launching.
    private static final double INTAKE_POWER = 0.5; // Feed balls once launcher is up to speed.
    private static final double DRIVE_SPEED = 0.5; // Range 0.0 to 1.0.
    private static final double DRIVE_TIMEOUT_SECONDS = 3.0; // Safety limit for drive.
    private static final double BACKWARD_DISTANCE_IN = 120.0 / 25.4; // Original StarterBot -120 mm.

    private final ElapsedTime autoTimer = new ElapsedTime();

/////////////////////     AUTONOMOUS STATE MACHINE     /////////////////////
    // A state machine runs one step at a time without blocking loop().
    // START_DRIVE runs the drive command only ONCE; DRIVING checks its progress.
    private enum AutoState {
        LAUNCH,
        START_DRIVE,
        DRIVING,
        COMPLETE,
        ERROR }

    private AutoState autoState = AutoState.LAUNCH;
    private MecanumAutoDrive.Result driveResult = MecanumAutoDrive.Result.RUNNING;

    @Override
//////////////////////// INITIALIZATION //////////////////////////////
    public void init() {
        robot.init(hardwareMap); // Map and configure all BeachBots hardware.
        drive = new MecanumAutoDrive(robot); // Reuse the four mapped drive motors.
        autoState = AutoState.LAUNCH;
        stopMechanisms(); // Do not spin launcher or feeders before START.

        telemetry.addLine("BeachBots Encoder Auto Ready");
        telemetry.addLine("Sequence: Launch, drive backward, stop");
        telemetry.update();
    }

    @Override
//////////////////////// RUNS ONCE WHEN PLAY IS PRESSED //////////////////////////////
    public void start() {
        autoTimer.reset(); // Begin timing the launcher sequence.
    }

    @Override
//////////////////////// AUTONOMOUS LOOP //////////////////////////////
    public void loop() {
        // Each pass through loop() executes the current state, then updates telemetry.
        switch (autoState) {
            case LAUNCH:
    ///////////////////// LAUNCHER CONTROLS /////////////////////
                robot.launcher.setVelocity(RobotHardware.LAUNCHER_TARGET_VELOCITY);

                // Only feed when the measured launcher velocity is high enough.
                boolean launcherReady = robot.launcher.getVelocity()
                        >= RobotHardware.LAUNCHER_MIN_VELOCITY;
                robot.windmillServo.setPower(launcherReady ? 1.0 : 0.0);
                robot.intake.setPower(launcherReady ? INTAKE_POWER : 0.0);

                if (autoTimer.seconds() >= LAUNCH_TIME_SECONDS) {
                    stopMechanisms();
                    autoState = AutoState.START_DRIVE;
                }
                break;

            case START_DRIVE:
    ///////////////////// BEGIN BACKWARD MOVEMENT /////////////////////
                // This method starts the movement and returns immediately.
                // Do not call it repeatedly in DRIVING or the targets would restart.
                if (drive.backward(BACKWARD_DISTANCE_IN, DRIVE_SPEED, DRIVE_TIMEOUT_SECONDS)) {
                    autoState = AutoState.DRIVING;
                } else {
                    autoState = AutoState.ERROR; // Reject invalid or overlapping commands.
                }
                break;

            case DRIVING:
    ///////////////////// CHECK MOTOR ENCODERS /////////////////////
                // update() checks all four wheels and the safety timeout.
                driveResult = drive.update();
                if (driveResult == MecanumAutoDrive.Result.SUCCESS) {
                    autoState = AutoState.COMPLETE;
                } else if (driveResult != MecanumAutoDrive.Result.RUNNING) {
                    autoState = AutoState.ERROR;
                }
                break;

            case COMPLETE:
                // Finished successfully. Motors were stopped by drive.update().
                break;

            case ERROR:
                // A drive timeout occurred. Leave motors stopped for safety.
                break;
        }

    //////////////////////// TELEMETRY /////////////////////////////////
        // Monitor progress and encoder positions on the Driver Station.
        telemetry.addData("Auto State", autoState);
        telemetry.addData("Elapsed Auto Time (s)", "%.1f", autoTimer.seconds());
        telemetry.addData("Launcher Velocity (ticks/s)", "%.0f", robot.launcher.getVelocity());
        telemetry.addData("Drive Result", driveResult);
        telemetry.addData("FL / FR Encoders", "%d / %d",
                robot.frontLeftDrive.getCurrentPosition(), robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("BL / BR Encoders", "%d / %d",
                robot.backLeftDrive.getCurrentPosition(), robot.backRightDrive.getCurrentPosition());
        telemetry.addData("FL / FR Targets", "%d / %d", drive.getTarget(0), drive.getTarget(1));
        telemetry.addData("BL / BR Targets", "%d / %d", drive.getTarget(2), drive.getTarget(3));
        if (autoState == AutoState.ERROR) telemetry.addLine("WARNING: Drive failed or timed out!");
        if (autoState == AutoState.COMPLETE) telemetry.addLine("Auto Complete!");
        telemetry.update();
    } // Closes Autonomous Loop

//////////////////////// STOP WHEN OPMODE ENDS //////////////////////////////
    @Override
    public void stop() {
        if (drive != null) drive.stop();
        stopMechanisms();
    }

//////////////////////// HELPER FUNCTIONS //////////////////////////////
    private void stopMechanisms() {
        // Stop all launcher and intake motors/servos at the end of launching.
        robot.launcher.setVelocity(0);
        robot.intake.setPower(0);
        robot.windmillServo.setPower(0);
        robot.leftIntakeServo.setPower(0);
        robot.rightIntakeServo.setPower(0);
    }
} // Closes BeachBotsAuto class
