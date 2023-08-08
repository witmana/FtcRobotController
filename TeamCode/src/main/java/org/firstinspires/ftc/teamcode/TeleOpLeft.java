package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpLeft extends LinearOpMode {

    Robot   robot       = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init();

        robot.turret.newTarget(10.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            if(gamepad2.a)
            {
                robot.turret.turretMode = Turret.TurretMode.TOCONE;
                robot.turret.newTarget(10.0);
                runtime.reset();
                robot.lift.liftMode = Lift.LiftMode.GROUND;
                robot.lift.resetLiftPID();
                robot.scoring.pivotPosition = robot.scoring.CLAW_DOWN;
                robot.scoring.extensionPosition = robot.scoring.EXTENSION_MID;
            }
            if(gamepad2.b)
            {
                robot.turret.turretMode = Turret.TurretMode.TOJUNCTION;
                robot.turret.newTarget(148.0);
                runtime.reset();
                robot.lift.liftMode = Lift.LiftMode.HIGH;
                robot.lift.resetLiftPID();
                robot.scoring.pivotPosition = robot.scoring.CLAW_HOVER;
                robot.scoring.extensionPosition = robot.scoring.EXTENSION_MID;
            }
            if(gamepad2.x)
            {
                robot.scoring.pivotPosition = robot.scoring.CLAW_UP;
                robot.scoring.extensionPosition = robot.scoring.EXTENSION_IN;
                robot.lift.liftMode = Lift.LiftMode.GROUND;
                robot.lift.resetLiftPID();
            }
            if(gamepad2.y)
            {
                robot.turret.turretMode = Turret.TurretMode.TOJUNCTION;
                robot.turret.newTarget(200.0);
                runtime.reset();
                robot.lift.liftMode = Lift.LiftMode.MEDIUM;
                robot.lift.resetLiftPID();
                robot.scoring.pivotPosition = robot.scoring.CLAW_HOVER;
                robot.scoring.extensionPosition = robot.scoring.EXTENSION_MID;
            }

            robot.teleOp();

            telemetry.addData("Angle: ", robot.turret.getAngle());
            telemetry.addData("liftLeftHeight; ", robot.lift.liftLeft.getCurrentPosition());
            telemetry.addData("liftRightHeight; ", robot.lift.liftRight.getCurrentPosition());
            telemetry.addData("turretMode:", robot.turret.turretMode);
            telemetry.update();

        }
    }
}
