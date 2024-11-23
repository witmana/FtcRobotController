package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardware{
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        ElapsedTime runtime = new ElapsedTime();

        //TODO Add instance of Vision subsystem...either Webcam, HuskyLens, or Limelight
        public Drivetrain drivetrain;
        public Scoring scoring;
        public Pivot pivot;
        public Lift lift;

        // Define a constructor that allows the OpMode to pass a reference to itself.
        public RobotHardware(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            drivetrain = new Drivetrain(myOpMode);
            scoring = new Scoring(myOpMode);
            pivot = new Pivot(myOpMode);
            lift = new Lift(myOpMode);

            drivetrain.init();
            scoring.init();
            pivot.init();
            lift.init();

            myOpMode.telemetry.addData(">", "Hardware Initialized");
        }

        public void teleOp() {
            drivetrain.teleOp();
            scoring.teleOp();
            pivot.teleOp();
            lift.teleOp();

            //TODO Add automation of subsystems here!
            //IMPORTANT NOTE: You should choose if subsystem modes are altered here or in subsystem classes
            //Probably best to do it here to avoid conflicts
            //Ex. Setting up subsystems for intaking and delivery
            if(myOpMode.gamepad2.x){
                pivot.pivotMode = Pivot.PivotMode.PIVOT_SUBMERSIBLE;
                lift.liftMode = Lift.LiftMode.RETRACTED;
                scoring.wrist.setPosition(Scoring.WRIST_MID);
                scoring.claw.setPosition(Scoring.CLAW_OPEN);
            }else if (myOpMode.gamepad2.y){
                pivot.pivotMode = Pivot.PivotMode.PIVOT_HIGH_BASKET;
                lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                scoring.wrist.setPosition(Scoring.WRIST_OUT);
            }
        }

        public void update(){
            drivetrain.update();
            pivot.update();
            lift.update();
        }

        public void stop(){
            drivetrain.stop();
            pivot.stop();
            lift.stop();
        }

}


