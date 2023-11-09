package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name="Sample TeleOp", group="Linear Opmode")
public class SampleTeleOp extends LinearOpMode {

    //Declare an instance of the RobotHardware
    RobotHardware robot;

    @Override
    public void runOpMode() {

        //Initialize the RobotHardware object
        robot = new RobotHardware(this);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Call the robot's teleOp function!
            robot.teleOp();

        }
    }
}
