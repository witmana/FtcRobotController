package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;


@Autonomous(name = "LeftRegionals", group = "Linear Opmode")

public class AutoLeft extends LinearOpMode {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Marlbots.tflite";

    private static final String[] LABELS = {
            "Marlbots",
            "M"
    };

    private static final String VUFORIA_KEY =
            "AfDxTOz/////AAABmZP0ZciU3EdTii04SAkq0dI8nEBh4mM/bXMf3H6bRJJbH/XCSdLIe5SDSavwPb0wJvUdnsmXcal43ZW2YJRG6j65bfewYJPCb+jGn7IW7kd5rKWs11G7CtFSMGEOhA5NU8gi39eHW0pmXC8NEXBn3CmK67TIENGm/YBN6f+xmkmDvBQjaJc2hJ93HPvhAnIiAbJT9/fWijwg9IovTok/xAcAcuIKz3XK/lnJXu6XdJ1MyRtoXO7yf1W4ReDHngWCtKI9B7bAnD6zPNhZoVLVzl34E8XKed/dGShIoCmIUTe0HoUniP0ye3AnwhFgxLhgPcysF8uVqKN0VKBpDH1zU7J7keZdjWHM6jvn29oLMK7W";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    private String position = "RIGHT";

    public void runOpMode() {
        robot.init();

        initVuforia();
        initTfod();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 4.0 / 3.0);
        }

        runtime.reset();
        while (!isStarted() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());

                        if (updatedRecognitions.size() >= 2 && recognition.getLabel() == "M")
                            position = "LEFT";
                        else if (updatedRecognitions.size() == 1 && recognition.getLabel() == "M")
                            position = "MID";
                        else if (updatedRecognitions.size() < 1)
                            position = "RIGHT";
                        telemetry.addData("position", position);

                        i++;
                    }
                    telemetry.update();
                }
            }
        }

        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        waitForStart();

        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        robot.drivetrain.driveSideProfiledPID(3250);
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        robot.drivetrain.driveStraightProfiledPID(-700);
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        robot.toStackLeft(575);
        robot.lift.resetLiftPID();

        robot.retrieve();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        robot.toStackLeft(425);
        robot.lift.resetLiftPID();

        robot.retrieve();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        runtime.reset();
        robot.turret.newTarget(-30.0);
        while (runtime.seconds() < 2 && opModeIsActive()) {
            robot.turret.turretProfiledPIDNoLoop(robot.turret.targetAngle, robot.turret.startingAngle, runtime.seconds());
            robot.lift.resetLift(-1);
            robot.scoring.pivot.setPosition(robot.scoring.CLAW_UP);
            robot.scoring.extension.setPosition(robot.scoring.EXTENSION_IN);
            telemetry.update();
        }
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        robot.drivetrain.encoderTurn(18f, 1);

        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        if (position == "RIGHT") {
            robot.drivetrain.driveSideProfiledPID(-2100);
        } else if (position == "MID") {
            robot.drivetrain.driveSideProfiledPID(-600);
        } else {
            robot.drivetrain.driveSideProfiledPID(800);
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
