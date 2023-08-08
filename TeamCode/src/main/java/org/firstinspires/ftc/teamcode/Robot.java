package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    public Drivetrain drivetrain;
    public Scoring scoring;
    public Turret turret;
    public Lift lift;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        scoring = new Scoring(myOpMode);
        turret = new Turret(myOpMode);
        lift = new Lift(myOpMode);

        drivetrain.init();
        scoring.init();
        turret.init();
        lift.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
        turret.teleOp();
        scoring.teleOp();
    }

    public void deliver() {
        scoring.pivot.setPosition(scoring.CLAW_HOVER);
        scoring.extension.setPosition(scoring.EXTENSION_OUT - 0.2);
        myOpMode.sleep(500);
        scoring.claw.setPosition(scoring.CLAW_OPEN);
        myOpMode.sleep(250);
        scoring.extension.setPosition(scoring.EXTENSION_IN);
        myOpMode.sleep(500);
    }

    public void retrieve() {
        scoring.pivot.setPosition(scoring.CLAW_DOWN);
        myOpMode.sleep(250);
        scoring.extension.setPosition(scoring.EXTENSION_OUT - 0.24);
        myOpMode.sleep(500);
        scoring.claw.setPosition(scoring.CLAW_CLOSED);
        myOpMode.sleep(250);
        lift.liftToPositionPIDClass(750);
        myOpMode.sleep(400);
        scoring.extension.setPosition(scoring.EXTENSION_IN);
        myOpMode.sleep(400); //new one
    }

    public void toJunctionLeft() {
        runtime.reset();
        scoring.pivot.setPosition(scoring.CLAW_HOVER);
        turret.newTarget(180);
        while (runtime.seconds() < 2.25) {
            scoring.extension.setPosition(scoring.EXTENSION_IN);
            turret.turretProfiledPIDNoLoop(turret.targetAngle, turret.startingAngle, runtime.seconds());
            lift.liftToPositionPIDClass(3600);//3300
        }

        turret.turret.setPower(0);
        turret.turretPID.reset();
    }

    public void toStackLeft(int liftHeight) {
        runtime.reset();
        turret.newTarget(-30.0);
        while (runtime.seconds() < 1.75) {
            scoring.extension.setPosition(scoring.EXTENSION_IN);
            turret.turretProfiledPIDNoLoop(turret.targetAngle, turret.startingAngle, runtime.seconds());
            scoring.pivot.setPosition(scoring.CLAW_UP);
            lift.liftToPositionPIDClass(liftHeight);
        }
        turret.turretPID.reset();
        turret.turret.setPower(0);
    }

    public void toJunctionRight() {
        runtime.reset();
        scoring.pivot.setPosition(scoring.CLAW_HOVER);
        turret.newTarget(-62.0);
        while (runtime.seconds() < 2.25) {
            turret.turretProfiledPIDNoLoop(turret.targetAngle, turret.startingAngle, runtime.seconds());
            lift.liftToPositionPIDClass(3600);
        }
        turret.turret.setPower(0);
        turret.turretPID.reset();
    }

    public void toStackRight(int liftHeight) {
        runtime.reset();
        turret.newTarget(150.0);
        while (runtime.seconds() < 1.75) {
            scoring.extension.setPosition(scoring.EXTENSION_IN);
            turret.turretProfiledPIDNoLoop(turret.targetAngle, turret.startingAngle, runtime.seconds());
            scoring.pivot.setPosition(scoring.CLAW_UP);
            lift.liftToPositionPIDClass(liftHeight);
        }
        turret.turretPID.reset();
        turret.turret.setPower(0);
    }
}
