package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MotionProfile.motionProfile;
import static org.firstinspires.ftc.teamcode.MotionProfile.motionProfileTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime time = new ElapsedTime();

    //drivetrain
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    //drive
    PIDController flPID;
    PIDController frPID;
    PIDController blPID;
    PIDController brPID;

    double driveKP = 0.01;
    double driveKI = 0.0;
    double driveKD = 0;//0.0003;
    double driveMaxAcc = 2000;
    double driveMaxVelocity = 3500;
    double driveMaxOut = 0.95;



    public Drivetrain (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        flPID = new PIDController(driveKP,driveKI,driveKD);
        frPID = new PIDController(driveKP,driveKI,driveKD);
        blPID = new PIDController(driveKP,driveKI,driveKD);
        brPID = new PIDController(driveKP,driveKI,driveKD);

        flPID.maxOut = driveMaxOut;
        frPID.maxOut = driveMaxOut;
        blPID.maxOut = driveMaxOut;
        brPID.maxOut = driveMaxOut;

        frontLeft  = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft  = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");


    }

    public void resetEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void useEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOp(){
        //drive train
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn  = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

        //frontLeftPower    = Range.clip(drive + turn - strafe, -1.0, 1.0) ;
        //frontRightPower   = Range.clip(drive - turn + strafe, -1.0, 1.0) ;
        //backLeftPower    = Range.clip(drive + turn + strafe, -1.0, 1.0) ;
        //backRightPower   = Range.clip(drive - turn - strafe, -1.0, 1.0) ;

        frontLeftPower    = (drive + turn - strafe)/denominator ;
        frontRightPower   = (drive - turn + strafe)/denominator ;
        backLeftPower    = (drive + turn + strafe)/denominator ;
        backRightPower   = (drive - turn - strafe)/denominator ;


        if (myOpMode.gamepad1.right_bumper)
        {
            frontLeft.setPower(frontLeftPower/7);
            frontRight.setPower(frontRightPower/7);
            backLeft.setPower(backLeftPower/7);
            backRight.setPower(backRightPower/7);
        }
        else if (myOpMode.gamepad1.left_bumper)
        {
            frontLeft.setPower(2*frontLeftPower);
            frontRight.setPower(2*frontRightPower);
            backLeft.setPower(2*backLeftPower);
            backRight.setPower(2*backRightPower);
        }
        else
        {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

    }

    void driveStraightProfiledPID(float distance){
        float direction = 1;
        if(distance < 0){
            direction =-1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();


        while(myOpMode.opModeIsActive() &&
                time.seconds() < 0.5 + motionProfileTime(driveMaxAcc, driveMaxVelocity, distance, time.seconds()))
        //(!flPID.targetReached(distance, frontLeft.getCurrentPosition()) ||
        //!frPID.targetReached(distance, frontRight.getCurrentPosition()) ||
        //!blPID.targetReached(distance, backLeft.getCurrentPosition()) ||
        //!brPID.targetReached(distance, backRight.getCurrentPosition())))
        {

            double flPower = flPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), frontLeft.getCurrentPosition());
            double frPower = frPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), frontRight.getCurrentPosition());
            double blPower = blPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), backLeft.getCurrentPosition());
            double brPower = brPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), backRight.getCurrentPosition());

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);


            myOpMode.telemetry.addData("flPower", flPower);
            myOpMode.telemetry.addData("instantTarget", motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()));
            myOpMode.telemetry.addData("profileTime", motionProfileTime(driveMaxAcc, driveMaxVelocity, distance, time.seconds()));
            myOpMode.telemetry.addData("frontLeft" , frontLeft.getCurrentPosition());
            myOpMode.telemetry.addData("frontRight" , frontRight.getCurrentPosition());
            myOpMode.telemetry.addData("backLeft" , backLeft.getCurrentPosition());
            myOpMode.telemetry.addData("backRight" , backRight.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //to go to the right, have wheels on the right move toward each other and have
    //wheels on the left move away from each other
    //just changing distance to -1 but have to somehow make the loop above end to strafe
    void driveSideProfiledPID(float distance){

        float direction = 1;
        if(distance < 0){
            direction =-1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();


        while(myOpMode.opModeIsActive() &&
                time.seconds() < motionProfileTime(driveMaxAcc, driveMaxVelocity, distance, time.seconds())
            //(!flPID.targetReached(distance, frontLeft.getCurrentPosition()) ||
            //!frPID.targetReached(distance, frontRight.getCurrentPosition()) ||
            //!blPID.targetReached(distance, backLeft.getCurrentPosition()) ||
            //!brPID.targetReached(distance, backRight.getCurrentPosition()))
        ){

            double flPower = flPID.calculate(-1*direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), frontLeft.getCurrentPosition());
            double frPower = frPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), frontRight.getCurrentPosition());
            double blPower = blPID.calculate(direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), backLeft.getCurrentPosition());
            double brPower = brPID.calculate(-1*direction*motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()), backRight.getCurrentPosition());

            //double flPower = flPID.calculate(targetPosition, front_left.getCurrentPosition());
            //double frPower = frPID.calculate(targetPosition, front_right.getCurrentPosition());
            //double blPower = blPID.calculate(targetPosition, back_left.getCurrentPosition());
            //double brPower = brPID.calculate(targetPosition, back_right.getCurrentPosition());

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            myOpMode.telemetry.addData("flPower", flPower);
            myOpMode.telemetry.addData("instantTarget", motionProfile(driveMaxAcc, driveMaxVelocity, distance, time.seconds()));
            myOpMode.telemetry.addData("profileTime", motionProfileTime(driveMaxAcc, driveMaxVelocity, distance, time.seconds()));
            myOpMode.telemetry.addData("frontLeft" , frontLeft.getCurrentPosition());
            myOpMode.telemetry.addData("frontRight" , frontRight.getCurrentPosition());
            myOpMode.telemetry.addData("backLeft" , backLeft.getCurrentPosition());
            myOpMode.telemetry.addData("backRight" , backRight.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
