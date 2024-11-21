package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TestBotHardware {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        ElapsedTime runtime = new ElapsedTime();

        //TODO Add instance of Vision subsystem...either Webcam, HuskyLens, or Limelight
        public TestBotDrivetrain drivetrain;


        // Define a constructor that allows the OpMode to pass a reference to itself.
        public TestBotHardware(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            drivetrain = new TestBotDrivetrain(myOpMode);

            drivetrain.init();

            myOpMode.telemetry.addData(">", "Hardware Initialized");
        }

        public void teleOp() {
            drivetrain.teleOp();


            //TODO Add automation of subsystems here!
            //IMPORTANT NOTE: You should choose if subsystem modes are altered here or in subsystem classes
            //Probably best to do it here to avoid conflicts
            //Ex. Setting up subsystems for intaking and delivery

        }

        public void update(){
            drivetrain.update();
        }

        public void stop(){
            drivetrain.stop();
        }

}


