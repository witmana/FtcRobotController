package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//General template for a subsystem class

public class Subsystem {
    /* Declare OpMode members. */

    //Always Declare an instance of LinearOpMode. This provides access to opMode members like gamepads and telemetry
    LinearOpMode myOpMode;

    //Declare hardware devices like motors, servos, sensors etc.
    //ex. Servo claw;

    //Declare constants such as PID values or servo starting positions - Use all caps, underscore format
    //ex: public static final double CLAW_UP = 0.9;

    //Declare variables that will change throughout program - use camelCase format
    //ex: public double clawPosition;

    //Constructor with reference to the calling opMode
    public Subsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Init function called when driver hits 'init' (before 'wait for start')
    public void init() {
        //Hardware map initializations should go here
        //Set starting conditions for motors or servos (ex. Direction, Brake Mode etc)
        //Set initial values for variables
        myOpMode.telemetry.addData(">", "Subsystem Initialized");
    }

    public void teleOp() {

        //Consider this like an 'update' function for the Subsystem
        //Include gamepad behavior specific to this subsystem
        //Include behavior for each "State" in which the subsystem could exist (optionally put in sepearate function)
    }

    public void states() {
        //Optionally separate control of subsystem states here for organizational purposes
    }

    //Include other subsystem specific functions below
    //ex. public void resetLift(){}
}
