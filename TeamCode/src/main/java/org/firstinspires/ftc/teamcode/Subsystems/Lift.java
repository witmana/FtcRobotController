package org.firstinspires.ftc.teamcode.Subsystems;

    //package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utility.PIDController;

public class Lift {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    //lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;
    //public TouchSensor touch = null;

    //TODO adjust explicit states for subsystem
    public enum LiftMode {
        MANUAL,
        LOW_BASKET,
        HIGH_BASKET,
        HIGH_CHAMBER,
        LOW_CHAMBER,
        RETRACTED
    }

    //TODO adjust lift constants (encoder values of desired positions)
    public static final int EXT_RETRACTED = 0;
    public static final int EXT_MAX_LIMIT = 2548;
    public static final int EXT_HIGH_BASKET= 2548;
    public static final int EXT_HIGH_CHAMBER = 1180;
    public static final int EXT_LOW_BASKET = 1180;
    public static final double LIFT_SPEED = 0.5;
    public static final double LIFT_HOLD_POWER = 0;

    PIDController leftLiftPID;
    PIDController rightLiftPID;

    //TODO Adjust Lift Constants
    public static final double LIFT_KP = 0.005;
    public static final double LIFT_KI = 0;
    public static final double LIFT_KD = 0.0;
    public static final double LIFT_MAX_POWER = 0.9;

    public LiftMode liftMode = LiftMode.MANUAL;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        leftLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD,LIFT_MAX_POWER);
        rightLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD,LIFT_MAX_POWER);

        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");

        //TODO Set lift directions, positive should be up/out
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // brake and encoders
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Lift Initialized");
    }

    //Behavior common to both Auto and TeleOp
    public void update() {
        myOpMode.telemetry.addData("leftLiftPosition", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("rightLiftPosition", rightLift.getCurrentPosition());
        myOpMode.telemetry.addData("liftMode", liftMode);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftMode == LiftMode.MANUAL) {
            //TODO Adjust to desired control scheme and check conditions
            if (myOpMode.gamepad2.left_stick_y < 0.1 && leftLift.getCurrentPosition() < EXT_MAX_LIMIT) {
                leftLift.setPower(myOpMode.gamepad2.left_stick_y);
                rightLift.setPower(myOpMode.gamepad2.left_stick_y);
            } else if (myOpMode.gamepad2.left_stick_y > 0.1 && leftLift.getCurrentPosition() > EXT_RETRACTED) {
                leftLift.setPower(myOpMode.gamepad2.left_stick_y);
                rightLift.setPower(myOpMode.gamepad2.left_stick_y);
            } else {
                leftLift.setPower(LIFT_HOLD_POWER);
                rightLift.setPower(LIFT_HOLD_POWER);
            }
        } else if (liftMode == LiftMode.HIGH_CHAMBER) {
            liftToPositionPIDClass(EXT_HIGH_CHAMBER);
        } else if (liftMode == LiftMode.RETRACTED) {
            liftToPositionPIDClass(EXT_RETRACTED);
        } else if (liftMode == LiftMode.HIGH_BASKET) {
            liftToPositionPIDClass(EXT_HIGH_BASKET);
        } else if (liftMode == LiftMode.LOW_BASKET) {
            liftToPositionPIDClass(EXT_LOW_BASKET);
        }
    }

    //Switching modes based on button presses
    public void teleOp(){
        update();
        //setting lift state
        //TODO Update based on desired control scheme
        if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1){
            liftMode = LiftMode.MANUAL;
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(myOpMode.gamepad2.y){
            liftMode = LiftMode.HIGH_BASKET;
        }else if(myOpMode.gamepad2.a){
            liftMode = LiftMode.HIGH_CHAMBER;
        }else if(myOpMode.gamepad2.b){
            liftMode = LiftMode.LOW_BASKET;
        }else if(myOpMode.gamepad2.x){
            liftMode = LiftMode.RETRACTED;
        }
    }

    //sends lift to the target encoder position
    public void liftToPositionPIDClass(double targetPosition) {
        double leftOut = leftLiftPID.calculate(targetPosition, leftLift.getCurrentPosition());
        double rightOut = rightLiftPID.calculate(targetPosition, rightLift.getCurrentPosition());

        leftLift.setPower(leftOut);
        rightLift.setPower(rightOut);

        myOpMode.telemetry.addData("LiftLeftPower: ", leftOut);
        myOpMode.telemetry.addData("LiftRightPower: ", rightOut);
        myOpMode.telemetry.addData("Running to", targetPosition);
        myOpMode.telemetry.addData("Currently at",  " at %7d :%7d",
                leftLift.getCurrentPosition(), rightLift.getCurrentPosition());
    }

    //Optional function for using built in PID with RUN_TO_POSITION rather than custom PID
    public void liftToTargetPosition(double speed,
                             int targetPosition) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rightLift.setTargetPosition(targetPosition);
            leftLift.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start
            leftLift.setPower(Math.abs(speed));
            rightLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", targetPosition);
                myOpMode.telemetry.addData("Currently at",  " at %7d :%7d",
                        leftLift.getCurrentPosition(), rightLift.getCurrentPosition());
        }
              // optional pause after each move.
    }

    public void stop(){
        leftLift.setPower(0);
        rightLift.setPower(0);
    }

}

