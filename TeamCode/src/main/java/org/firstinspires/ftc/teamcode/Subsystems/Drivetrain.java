package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.Utility.Pose2D;
import org.firstinspires.ftc.teamcode.Utility.SparkfunLocalizer;

public class Drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //TODO: Declare OpMode member for the Odometry System
    // If using GoBilda Pinpoint computer then use PinPointLocalizer class
    // If using Sparkfun OTOS then use SparfunLocalizer class
    //public PinPointLocalizer localizer;
    public SparkfunLocalizer localizer;

    ElapsedTime time = new ElapsedTime();


    //drivetrain motors
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    //PID controllers for moving to target pose
    PIDController xPID;
    PIDController yPID;
    PIDController headingPID;

    public boolean targetReached = false;
    Pose2D targetPose;

    //Static Variables
    //TODO Adjust drive constants based on auto performance
    public static double HEADING_KP = 0.9;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static final double DRIVE_KP = 0.01;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0;//0.0003;
    public static final double DRIVE_MAX_ACC = 2000;
    public static final double DRIVE_MAX_VEL = 3500;
    public static final double DRIVE_MAX_OUT = 0.95;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //Initialize PID controllers
        xPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        yPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        xPID.maxOut = DRIVE_MAX_OUT;
        yPID.maxOut = DRIVE_MAX_OUT;
        headingPID.maxOut = DRIVE_MAX_OUT;

        //TODO Change constructor based on localization system
        //localizer = new PinPointLocalizer(myOpMode);
        localizer = new SparkfunLocalizer(myOpMode);

        localizer.init();

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        useEncoders();

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }


    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOp() {
        //update localizer (should display position to dashboard)
        localizer.update();

        //drive train
        double max;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        leftFrontPower = (drive + turn - strafe);
        rightFrontPower = (drive - turn + strafe);
        leftBackPower = (drive + turn + strafe);
        rightBackPower = (drive - turn - strafe);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        //Slow and Turbo Buttons
        //TODO Adjust factors affecting slow and turbo buttons
        //turbo button (full power)
        if (myOpMode.gamepad1.right_bumper) {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
        //slow button (fraction of full power)
        else if (myOpMode.gamepad1.left_bumper) {
            leftFrontDrive.setPower(leftFrontPower / 7);
            rightFrontDrive.setPower(rightFrontPower / 7);
            leftBackDrive.setPower(leftBackPower / 7);
            rightBackDrive.setPower(rightBackPower / 7);
        }
        //default power
        else {
            leftFrontDrive.setPower(leftFrontPower / 2);
            rightFrontDrive.setPower(rightFrontPower / 2);
            leftBackDrive.setPower(leftBackPower / 2);
            rightBackDrive.setPower(rightBackPower / 2);
        }
    }

    public void update(){

        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xPID.calculate(targetPose.getX(DistanceUnit.INCH), localizer.pos.getX(DistanceUnit.INCH));
        double yPower = yPID.calculate(targetPose.getY(DistanceUnit.INCH), localizer.pos.getY(DistanceUnit.INCH));

        //double wrappedAngle = angleWrap(thetaTarget - localizer.heading);
        double tPower = headingPID.calculate(targetPose.getHeading(AngleUnit.DEGREES),localizer.pos.getHeading(AngleUnit.DEGREES));

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-localizer.pos.getHeading(AngleUnit.DEGREES)) - yPower * Math.sin(-localizer.pos.getHeading(AngleUnit.DEGREES));
        double yPower_rotated = xPower * Math.sin(-localizer.pos.getHeading(AngleUnit.DEGREES)) + yPower * Math.cos(-localizer.pos.getHeading(AngleUnit.DEGREES));

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

        //check if drivetrain is still working towards target
        targetReached = xPID.targetReached && yPID.targetReached && headingPID.targetReached;

        localizer.update();
    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;
    }

    public void driveToPose(double xTarget, double yTarget, double degreeTarget) {
        //check if drivetrain is still working towards target
        targetReached = xPID.targetReached && yPID.targetReached && headingPID.targetReached;
        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xPID.calculate(xTarget, localizer.pos.getX(DistanceUnit.INCH));
        double yPower = yPID.calculate(yTarget, localizer.pos.getY(DistanceUnit.INCH));

        //double wrappedAngle = angleWrap(thetaTarget - localizer.heading);
        double tPower = headingPID.calculate(degreeTarget,localizer.pos.getHeading(AngleUnit.DEGREES));

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-localizer.pos.getHeading(AngleUnit.DEGREES)) - yPower * Math.sin(-localizer.pos.getHeading(AngleUnit.DEGREES));
        double yPower_rotated = xPower * Math.sin(-localizer.pos.getHeading(AngleUnit.DEGREES)) + yPower * Math.cos(-localizer.pos.getHeading(AngleUnit.DEGREES));

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);
    }

    public void stop(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
