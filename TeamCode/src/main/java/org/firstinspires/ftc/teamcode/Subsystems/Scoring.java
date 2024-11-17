package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

    public class Scoring {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        //servos
        public Servo claw = null;
        // public Servo extension = null;
        public Servo wrist = null;

        // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
        //TODO Update based on desired values
        public static final double CLAW_OPEN = 0.5;
        public static final double CLAW_CLOSED = 0;

        public static final double WRIST_OUT = 0.6;
        public static final double WRIST_IN = 0.3;
        public static final double WRIST_MID = 0.4;

        public Scoring(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            claw = myOpMode.hardwareMap.get(Servo.class, "claw");
            // extension = myOpMode.hardwareMap.get(Servo.class, "extension");
            wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");

            //set initial positions
            claw.setPosition(CLAW_CLOSED);
            wrist.setPosition(WRIST_IN);

            myOpMode.telemetry.addData(">", "Scoring Initialized");
        }

        public void teleOp() {
            //set servo positions
            //TODO Update based on desired control scheme
            if (myOpMode.gamepad2.left_bumper) {
                claw.setPosition(CLAW_OPEN);
            } else if (myOpMode.gamepad2.right_bumper) {
                claw.setPosition(CLAW_CLOSED);
            }

            if (myOpMode.gamepad2.dpad_up) {
                wrist.setPosition(WRIST_OUT);
            } else if (myOpMode.gamepad2.dpad_down) {
                wrist.setPosition(WRIST_IN);
            } else if (myOpMode.gamepad2.dpad_left) {
                wrist.setPosition(WRIST_MID);
            }
        }

    }

