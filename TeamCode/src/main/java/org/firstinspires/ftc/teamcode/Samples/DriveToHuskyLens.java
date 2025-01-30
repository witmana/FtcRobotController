/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Samples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.TestBot.TestBotHardware;
import org.firstinspires.ftc.teamcode.Utility.PIDController;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@Autonomous (name = "Sensor: HuskyLens", group = "Sensor")

public class DriveToHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;

    TestBotHardware testBotHardware = new TestBotHardware(this);

    private HuskyLens huskyLens;
    public static double Kp = 0.05;
    double Ki = 0;
    double Kd = 0;
    public static double maxOut = 0.3;
    double servoPosition = 0;

    @Override
    public void runOpMode()
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        testBotHardware.init();

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
       // Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        //rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https:-          -p//wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while(opModeIsActive()) {
            /*
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            */
             /*
            SETUP FOR RELATIONSHIP BETWEEN HUSKYLENS AND ROBOT
            When front of testbot is pushed against submersible, the front bar of the submersible
            should JUST be visible at the bottom edge of the image
            (in other words, submersible bar even with bottom of screen)

            Servo values as they correspond to specimen y values
            Servo       Specimen Y
            0.55        222
            0.68        167
            0.75        140
            0.79        111
            0.84        93
            0.95        78

              */

            if(gamepad1.dpad_up){
                servoPosition+=0.01;
            }else if(gamepad1.dpad_down){
                servoPosition-=0.01;
            }
            testBotHardware.extension.setPosition(servoPosition);
            telemetry.addData("ServoPosition", servoPosition);


            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            /*
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 *
                 * These values have Java type int (integer).
                 */
            driveToHuskyLens();
            telemetry.update();
        }
    }
    public void driveToHuskyLens(){
        double xTarget = 200;
        double rightMostX = 0;
        int rightMostIndex = 0;
        PIDController strafeController;
        strafeController = new PIDController(Kp, Ki, Kd, maxOut);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            if(blocks[i].x > rightMostX){
                rightMostX = blocks[i].x;
                rightMostIndex = i;
                servoPosition = (424.53-blocks[i].y)/380.74;
            }

        }
        telemetry.addData("RightMostX", rightMostX);
        if(rightMostX < xTarget){
            double strafePower = strafeController.calculate(xTarget, rightMostX);
            testBotHardware.drivetrain.leftFrontDrive.setPower(-strafePower);
            testBotHardware.drivetrain.leftBackDrive.setPower(strafePower);
            testBotHardware.drivetrain.rightFrontDrive.setPower(strafePower);
            testBotHardware.drivetrain.rightBackDrive.setPower(-strafePower);
            telemetry.addData("StrafePower", strafePower);
        }else{
            testBotHardware.drivetrain.leftFrontDrive.setPower(0);
            testBotHardware.drivetrain.leftBackDrive.setPower(0);
            testBotHardware.drivetrain.rightFrontDrive.setPower(0);
            testBotHardware.drivetrain.rightBackDrive.setPower(0);
        }
    }
}

