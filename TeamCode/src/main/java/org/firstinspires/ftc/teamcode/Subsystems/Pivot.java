package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Pivot {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotor pivot = null;

    //TODO Adjust based on desired states
    public enum PivotMode {
        PIVOT_SUBMERSIBLE,
        PIVOT_HIGH_BASKET,
        PIVOT_HIGH_CHAMBER,
        MANUAL
    }

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //TODO Update values based on desired position
    public static final int PIVOT_HIGH_BASKET = 1774;
    public static final int PIVOT_HIGH_CHAMBER = -900;
    public static final int PIVOT_SUBMERSIBLE = -1114;
    public static final int PIVOT_LOW_LIMIT = -1200;
    public static final int PIVOT_HIGH_LIMIT = 1800;
    public static final double PIVOT_SPEED = 0.5;

    public PivotMode pivotMode = PivotMode.MANUAL;

    //Constructor
    public Pivot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        pivot = myOpMode.hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Pivot Initialized");
    }

    public void update() {
        myOpMode.telemetry.addData("pivotPosition", pivot.getCurrentPosition());
        myOpMode.telemetry.addData("pivotMode", pivotMode);

        if (pivotMode == PivotMode.MANUAL) {
            if (pivot.getCurrentPosition() > PIVOT_LOW_LIMIT && myOpMode.gamepad2.right_stick_y > 0.1) {
                pivot.setPower(myOpMode.gamepad2.right_stick_y / 2);
            } else if (pivot.getCurrentPosition() < PIVOT_HIGH_LIMIT && myOpMode.gamepad2.right_stick_y < -.1) {
                pivot.setPower(myOpMode.gamepad2.right_stick_y / 2);
            } else {
                pivot.setPower(0);
            }
        } else if (pivotMode == PivotMode.PIVOT_SUBMERSIBLE) {
            pivotToTargetPosition(PIVOT_SPEED, PIVOT_SUBMERSIBLE);
        } else if (pivotMode == PivotMode.PIVOT_HIGH_BASKET) {
            pivotToTargetPosition(PIVOT_SPEED, PIVOT_HIGH_BASKET);
        }
    }

    public void teleOp(){
        update();
        //Set states based on gamepad presses
        //TODO Update based on desired control scheme
        if(Math.abs(myOpMode.gamepad2.right_stick_y) > 0.1){
            pivotMode = PivotMode.MANUAL;
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(myOpMode.gamepad2.y){
            pivotMode = PivotMode.PIVOT_HIGH_BASKET;
        }else if(myOpMode.gamepad2.x){
            pivotMode = PivotMode.PIVOT_SUBMERSIBLE;
        }

    }

    //This function uses built in PID to hit target position
    //Optionally see Lift class for custom implementation
    public void pivotToTargetPosition(double speed,
                                      int targetPosition){

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            pivot.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion
            pivot.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", targetPosition);
            //myOpMode.telemetry.addData("Currently at",  " at %7d :%7d", targetPosition.getCurrentPositio
        }

    }

    public void stop(){
        pivot.setPower(0);
    }
}
