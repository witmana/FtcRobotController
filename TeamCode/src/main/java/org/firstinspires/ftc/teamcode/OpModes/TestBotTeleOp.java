package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.TestBotHardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestBotTeleOp", group="Linear OpMode")
public class TestBotTeleOp extends LinearOpMode {

    //declaring instance of robot hardware
    TestBotHardware robot;


    @Override
    public void runOpMode() {
        //calling constructor
        robot = new TestBotHardware(this);
        //calling init function
        robot.init();
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.teleOp();
        }
    }
}
