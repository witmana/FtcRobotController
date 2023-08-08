package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MotionProfile.motionProfile;
import static org.firstinspires.ftc.teamcode.MotionProfile.motionProfileTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Turret {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    private ElapsedTime runtime = new ElapsedTime();

    //imu
    public BNO055IMU imu;

    public CRServo turret = null;

    //turret constants
    public static final double TURRET_KP = 0.015; //0.015
    public static final double TURRET_KI = 0.00;  //0
    public static final double TURRET_KD = 0.01;  //0.01
    public static final double TURRET_MAX_ACC = 600;
    public static final double TURRET_MAX_VEL = 200; //700 before change
    public static final double TURRET_MAX_OUT = 0.95;

    public enum TurretMode{
        MANUAL,
        TOCONE,
        TOJUNCTION
    }
    public static TurretMode turretMode = TurretMode.TOCONE;

    PIDController turretPID;

    public Orientation lastAngles = new Orientation ();
    public double globalAngle;

    double startingAngle;
    double targetAngle;
    double angleDifference;

    public Turret (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.maxOut = TURRET_MAX_OUT;

        turret = myOpMode.hardwareMap.get(CRServo.class, "turret");

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        //turret.setZeroPowerBehavior(CRServo.ZeroPowerBehavior.BRAKE);
        myOpMode.telemetry.addData(">", "Turret Initialized");
    }

    public void teleOp() {
        if (myOpMode.gamepad2.right_stick_x > 0.2 || myOpMode.gamepad2.right_stick_x < -0.2) {
            turretMode = TurretMode.MANUAL;
        }

        if(turretMode == TurretMode.MANUAL) {
            turret.setPower(-myOpMode.gamepad2.right_stick_x/2);
        }
        else if(turretMode == TurretMode.TOCONE) {
            turretProfiledPIDNoLoop(targetAngle, startingAngle, runtime.seconds());
        }
        else if(turretMode == TurretMode.TOJUNCTION) {
            turretProfiledPIDNoLoop(targetAngle, startingAngle, runtime.seconds());
        }
        else {
            turret.setPower(0);
        }
    }

    void turretProfiledPIDNoLoop(double angleTarget, double startingAngle, double time){
        double angleDifference = angleTarget-startingAngle;
        double direction = 1;
        if(angleDifference < 0){
            direction =-1;
        }

        double turretPower = turretPID.calculate(direction*motionProfile(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time)+startingAngle, getAngle());

        turret.setPower(turretPower);

        myOpMode.telemetry.addData("turretPower", turretPower);
        myOpMode.telemetry.addData("instantTarget", motionProfile(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time));
        myOpMode.telemetry.addData("profileTime", motionProfileTime(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time));
        myOpMode.telemetry.addData("turretPosition" , getAngle());

        myOpMode.telemetry.update();
    }

    void turretProfiledPID(float angleTarget){
        double startingAngle = getAngle();
        double angleDifference = angleTarget-startingAngle;
        double direction = 1;
        if(angleDifference < 0){
            direction =-1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();


        while(myOpMode.opModeIsActive() &&
                time.seconds() < 0.5 + motionProfileTime(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time.seconds()))
        {

            double turretPower = turretPID.calculate((direction*motionProfile(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time.seconds()))+startingAngle, getAngle());

            turret.setPower(turretPower);

            myOpMode.telemetry.addData("turretPower", turretPower);
            myOpMode.telemetry.addData("instantTarget", motionProfile(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time.seconds()));
            myOpMode.telemetry.addData("profileTime", motionProfileTime(TURRET_MAX_ACC, TURRET_MAX_VEL, angleDifference, time.seconds()));
            myOpMode.telemetry.addData("turretPosition" , getAngle());

            myOpMode.telemetry.update();
        }
        turret.setPower(0);

    }

    public void turretToPositionPIDClass(double targetPosition){

        double out = turretPID.calculate(targetPosition, getAngle());

        turret.setPower(out);
        myOpMode.telemetry.addData("ANGLE: ", getAngle());

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle +=deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    void newTarget(double target){
        startingAngle = getAngle();;
        targetAngle = target;
        angleDifference = targetAngle-startingAngle;
    }
}
