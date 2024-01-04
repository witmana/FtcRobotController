package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/*This is adapted from sample code in "Learn Java for FTC" For more detailed background,
check out the vision section in that resource.

The vision processor has three "regions of interest" and will detect the one with the greatest
saturation of color (the least grey region). This makes it very effective at detecting a
Team Element/Prop, regardless of the color or shape. However, you need to adjust the location of
the regions of interest in the Vision Processor based on the camera's location on your robot. Note
that this opMode stores the location of the prop during initialization, BEFORE start is pressed.

To use this opMode
    1) Add the "SimpleVisionProcessor" sample code to your control hub
    2) Add a copy of this "SimpleVisionOpMode" to your control hub
    3) Make sure the webcam is added to your robot configuration and named "Webcam 1"
    4) Run the opMode and use the "camera stream" on the driver station to see where the regions are
    located
    5) Adjust the position of the regions of interest (in the processor) so that it is detecting
    the three possible positions of the Team Element/Prop based on your camera position.
    6) Add your robot actions to the if/else statements based on the Team Element/Prop position
 */

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

        while(!isStarted()) {
            telemetry.addData("Position: ", visionProcessor.getSelection());
            telemetry.update();
        }

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
