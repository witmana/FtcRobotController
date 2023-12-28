package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Simple Vision OpMode", group = "Concept")

public class SimpleVisionOpMode extends LinearOpMode {

    private SimpleVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        visionProcessor = new SimpleVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                visionProcessor);

        telemetry.addData("Position: ", visionProcessor.getSelection());
        telemetry.update();
        waitForStart();

        visionPortal.stopStreaming();

        if (opModeIsActive()) {

            //now you can move your robot based on the value of the 'selection' stored in the vision processor
            if(visionProcessor.getSelection() == SimpleVisionProcessor.Selected.LEFT){
                //strafe left
                //drive straight
            }else if(visionProcessor.getSelection() == SimpleVisionProcessor.Selected.RIGHT){
                //strafe right
                //drive straight
            }else{
                //drive straight ex;
                //robot.encoderDrive(0.5,24,5);
            }

            //You could also do other stuff afterwards...
            //turn right 90
            //robot.turnCW(0.5,90);
        }
    }
}
