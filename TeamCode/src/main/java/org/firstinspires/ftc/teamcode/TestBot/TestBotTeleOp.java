package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
            telemetry.addData("leftStickY", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
