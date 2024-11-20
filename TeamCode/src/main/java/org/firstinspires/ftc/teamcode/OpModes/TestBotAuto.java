package org.firstinspires.ftc.teamcode.OpModes;

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
import org.firstinspires.ftc.teamcode.Subsystems.TestBotHardware;
import org.firstinspires.ftc.teamcode.Utility.Pose2D;
@Autonomous(name="TestBotAuto", group="Linear OpMode")
@Config
public class TestBotAuto extends LinearOpMode {

    TestBotHardware robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_SUBMERSIBLE,
        DELIVER_SPECIMEN,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_SUBMERSIBLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    public static Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 24,0, AngleUnit.DEGREES,0);

    @Override
    public void runOpMode() {
        //calling constructor
        robot = new TestBotHardware(this);


        //calling init function
        robot.init();

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        //robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        robot.drivetrain.localizer.myOtos.setPosition(startPose);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_SUBMERSIBLE:
                    robot.drivetrain.setTargetPose(targetPose);

                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached){
                        currentState = State.DELIVER_SPECIMEN;
                        timer.reset();
                    }
                    break;
                case DELIVER_SPECIMEN:
                    if(timer.seconds() > 2.0){
                        currentState = State.PARK;
                    }
                    break;
                case PARK:
                    robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));

                    if(robot.drivetrain.targetReached){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    robot.stop();
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();

            telemetry.addData("state", currentState);
            telemetry.update();

        }
    }

}
