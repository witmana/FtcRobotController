package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime time = new ElapsedTime();

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void teleOp(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-myOpMode.gamepad1.left_stick_y, 0.0),
                -myOpMode.gamepad1.right_stick_x)
        );

        drive.updatePoseEstimate();

        myOpMode.telemetry.addData("x", drive.pose.position.x);
        myOpMode.telemetry.addData("y", drive.pose.position.y);
        myOpMode.telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        myOpMode.telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
