package org.firstinspires.ftc.teamcode.Background;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotHardware {
    //DcMotorEx has extended capabilities like velocity control

    //Drive Motors     Format  - public DcMotor name; or public DcMotorEx name;
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;

    //Mechanism Motors
    public DcMotor intake;
    public DcMotorEx launcher;

    //Servos -  Format  - public Servo name; - or public CRServo name; -
    public CRServo leftIntakeServo;
    public CRServo rightIntakeServo;
    public CRServo windmillServo;

    //Sensors   FORMAT public Sensor Type name;
    public GoBildaPinpointDriver odo;

    // LAUNCHER SETTINGS - Target and Min Velocity
    public static final int LAUNCHER_TARGET_VELOCITY = 1250;
    public static final int LAUNCHER_MIN_VELOCITY = 1200;

    //////////////////////////    INITIALIZE ALL HARDWARE    /////////////////////////////
    public void init(HardwareMap hardwareMap) {

        // Drive Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorBR");

        // Mechanism Motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        // Servos
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        // Sensors
        odo = hardwareMap.get(
                GoBildaPinpointDriver.class, "odo");

        // Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake Mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Directions
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
        windmillServo.setDirection(CRServo.Direction.FORWARD);

        // Launcher Configuration
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(1, 0, 0, 11.5));

        // Initialize Servos
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Reset Odometry
        odo.resetPosAndIMU();

    }//Closes Initialization


}// Closes Public Class
