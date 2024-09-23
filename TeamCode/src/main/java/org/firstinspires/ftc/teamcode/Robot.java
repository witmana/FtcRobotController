package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    public Drivetrain drivetrain;
    public Lift lift;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);

        drivetrain.init();
        lift.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
    }
}
