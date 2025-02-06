package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.PIDController;

public class TestBotHardware {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        ElapsedTime runtime = new ElapsedTime();

        //TODO Add instance of Vision subsystem...either Webcam, HuskyLens, or Limelight
        public TestBotDrivetrain drivetrain;
        public Servo extension;

        public HuskyLens huskyLens;
        double servoPosition = 0.25;

        // Define a constructor that allows the OpMode to pass a reference to itself.
        public TestBotHardware(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            huskyLens = myOpMode.hardwareMap.get(HuskyLens.class, "huskylens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

            drivetrain = new TestBotDrivetrain(myOpMode);

            drivetrain.init();

            extension = myOpMode.hardwareMap.get(Servo.class, "extension");

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
            extension.setPosition(servoPosition);
        }

    public void alignWithSample(){
        double xTarget = 200;
        double rightMostX = 0;
        int rightMostIndex = 0;
        PIDController strafeController;
        strafeController = new PIDController(TestBotDrivetrain.DRIVE_KP, TestBotDrivetrain.DRIVE_KI, TestBotDrivetrain.DRIVE_KD, TestBotDrivetrain.DRIVE_MAX_OUT);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        myOpMode.telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            myOpMode.telemetry.addData("Block", blocks[i].toString());
            if(blocks[i].x > rightMostX){
                rightMostX = blocks[i].x;
                rightMostIndex = i;
                servoPosition = (424.53-blocks[i].y)/380.74;
            }

        }
        myOpMode.telemetry.addData("RightMostX", rightMostX);
        if(rightMostX < xTarget){
            double strafePower = strafeController.calculate(xTarget, rightMostX);
            drivetrain.leftFrontDrive.setPower(-strafePower);
            drivetrain.leftBackDrive.setPower(strafePower);
            drivetrain.rightFrontDrive.setPower(strafePower);
            drivetrain.rightBackDrive.setPower(-strafePower);
            myOpMode.telemetry.addData("StrafePower", strafePower);
        }else{
            drivetrain.leftFrontDrive.setPower(0);
            drivetrain.leftBackDrive.setPower(0);
            drivetrain.rightFrontDrive.setPower(0);
            drivetrain.rightBackDrive.setPower(0);
        }
    }

        public void stop(){
            drivetrain.stop();
        }
}


