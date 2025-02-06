package org.firstinspires.ftc.teamcode.TestBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Utility.Pose2D;

@Config
@Autonomous(name="TestBotCyclingAuto", group="Linear OpMode")
public class TestBotCyclingAuto extends LinearOpMode {

    TestBotHardware robot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_BASKET,
        DELIVER_SAMPLE,
        DRIVE_TO_FLOOR_SAMPLE,
        RETRIEVE_FLOOR_SAMPLE,
        DRIVE_TO_SUBMERSIBLE,
        RETRIEVE_SUBMERSIBLE_SAMPLE,
        PARK,
        IDLE
    }

    //Constants so these can be tuned in the dashboard
    public static double basketX = 12;
    public static double basketY = 12;
    public static double basketHeading = -45;

    public static double floorSampleX = 24;
    public static double floorSampleY = 9;
    public static double floorSampleHeading = 0;

    public static double submersibleX = 48;
    public static double submersibleY = -12;
    public static double submersibleHeading = -90;

    public static double parkingX = 48;
    public static double parkingY = -20;
    public static double parkingHeading = -90;

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    int samplesScored;


    @Override
    public void runOpMode() {
        //calling constructor
        robot = new TestBotHardware(this);

        //calling init function
        robot.init();

        samplesScored = 0;

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        //robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        robot.drivetrain.localizer.myOtos.setPosition(startPose);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        timer.reset();
        totalTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_BASKET:
                    robot.drivetrain.driveToPose(basketX, basketY,basketHeading);
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 2){
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case DELIVER_SAMPLE:
                    robot.drivetrain.driveToPose(basketX, basketY,basketHeading);
                    if(timer.seconds() > 2.0 && samplesScored < 2){
                        switchState(State.RETRIEVE_FLOOR_SAMPLE);
                        samplesScored++;
                    }else if(timer.seconds() >2.0){
                        switchState(State.DRIVE_TO_SUBMERSIBLE);
                        samplesScored++;
                    }
                    break;
                case DRIVE_TO_FLOOR_SAMPLE:
                    robot.drivetrain.driveToPose(floorSampleX, floorSampleY + samplesScored*6,floorSampleHeading+samplesScored*15);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        switchState(State.RETRIEVE_FLOOR_SAMPLE);
                    }
                    break;
                case RETRIEVE_FLOOR_SAMPLE:
                    if(timer.seconds() < 1) {
                        robot.extension.setPosition(0.5);
                    }else{
                        robot.extension.setPosition(0.25);
                    }
                    robot.drivetrain.driveToPose(floorSampleX, floorSampleY,floorSampleHeading);
                    if(timer.seconds() > 2.0){
                        switchState(State.DRIVE_TO_BASKET);
                    }
                    break;
                case DRIVE_TO_SUBMERSIBLE:
                    if(timer.seconds() <1.6 ){
                        robot.drivetrain.driveToPose(submersibleX, submersibleY+12,submersibleHeading);
                    }else{
                        robot.drivetrain.driveToPose(submersibleX, submersibleY,submersibleHeading);
                    }
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.5){
                        switchState(State.RETRIEVE_SUBMERSIBLE_SAMPLE);
                    }
                    break;
                case RETRIEVE_SUBMERSIBLE_SAMPLE:
                    robot.drivetrain.driveToPose(submersibleX, submersibleY,submersibleHeading);
                    if(timer.seconds() > 2.0){
                        switchState(State.DRIVE_TO_BASKET);
                    }
                    break;
                case PARK:
                    robot.drivetrain.driveToPose(parkingX, parkingY,parkingHeading);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        switchState(State.IDLE);
                    }
                    break;
                case IDLE:
                    robot.stop();
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();

            telemetry.addData("state", currentState);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();

        }
    }

    void switchState(State newState){
        currentState = newState;
        timer.reset();
    }

}
