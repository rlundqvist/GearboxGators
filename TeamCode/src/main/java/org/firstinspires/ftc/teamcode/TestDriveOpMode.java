/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**
 * Control capabilities expected in TeleOp mode:
 * 1. Drive forward and fack
 * 2. Turn left and right
 * 3. Strafe left and right
 * 4. Smaller/slower movements either with separate controls or ability to change speed
 * 5. Ability to extend (lower robot) and retract (raise robot) rack and pinion
 * 6. Quick/simple ability to extend collection arm for lunar height
 * 7. Quick/simple ability to retract collection arm completely
 * 8. More fine-grained control of collection arm
 * 9. Identify minerals in hopper?
 * 10. Wave a flag?
 * 11. Extend one or two arms for moving minerals on floor
 * 13. Switch on/off LED lights?
 */


@TeleOp(name="Concept: Drive TeleOp", group="Linear Opmode")
//@Disabled
public class TestDriveOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rackDrive = null;

    private enum DriveModes {POV, TANK, STRAFE}
    private DriveModes driveMode = DriveModes.STRAFE;
    private String driveModeStr = "STRAFE";
    private boolean strafeFound = false;
    private boolean povFound = false;
    private boolean tankFound = false;
    private boolean latchingFound = false;
    private boolean teleopFound = false;
    private boolean stopFound = false;
    private boolean wasLB = false;
    private boolean wasRB = false;
    private static double calLeftBack = 1.0;
    private static double calRightBack = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class,"left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive");
        rackDrive = hardwareMap.get(DcMotor.class, "rack_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Determine Resource IDs for sounds built into the RC application.
        int strafeSoundID = hardwareMap.appContext.getResources().getIdentifier("strafe","raw", hardwareMap.appContext.getPackageName());
        int povSoundID = hardwareMap.appContext.getResources().getIdentifier("pov","raw", hardwareMap.appContext.getPackageName());
        int tankSoundID = hardwareMap.appContext.getResources().getIdentifier("tank","raw", hardwareMap.appContext.getPackageName());
        int latchingSoundID = hardwareMap.appContext.getResources().getIdentifier("latching","raw", hardwareMap.appContext.getPackageName());
        int teleopSoundID = hardwareMap.appContext.getResources().getIdentifier("teleop","raw", hardwareMap.appContext.getPackageName());
        int stopSoundID = hardwareMap.appContext.getResources().getIdentifier("stop","raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (strafeSoundID != 0)  strafeFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, strafeSoundID);
        if (povSoundID != 0) povFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, povSoundID);
        if (tankSoundID != 0) tankFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, tankSoundID);
        if (latchingSoundID != 0) latchingFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, latchingSoundID);
        if (teleopSoundID != 0) teleopFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, teleopSoundID);
        if (stopSoundID != 0) stopFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, stopSoundID);

        // Display sound status
        telemetry.addData("strafe resource", strafeFound ? "Found" : "NOT found\n Add strafe.wav to /src/main/res/raw" );
        telemetry.addData("pov resource", povFound ? "Found" : "Not found\n Add pov.wav to /src/main/res/raw" );
        telemetry.addData("tank resource", tankFound ? "Found" : "Not found\n Add tank.wav to /src/main/res/raw" );
        telemetry.addData("latching resource", tankFound ? "Found" : "Not found\n Add latching.wav to /src/main/res/raw" );
        telemetry.addData("teleop resource", tankFound ? "Found" : "Not found\n Add teleop.wav to /src/main/res/raw" );
        telemetry.addData("stop resource", tankFound ? "Found" : "Not found\n Add stop.wav to /src/main/res/raw" );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (teleopFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, teleopSoundID);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower = 0.0;
            double leftBackPower = 0.0;
            double rightFrontPower = 0.0;
            double rightBackPower = 0.0;
            double rackPower = 0.0;

            // Temporary variables to make code and calculations clearer
            double left;
            double right;
            double drive;
            double turn;
            double strafe;

            // Temporary variables tracking gampads
            boolean isLB;
            boolean isRB;

            // TODO: Define controller keys

            // GAME-PAD 1
            // X
            // Y
            // A
            // B
            // Dpad Left
            // Dpad Right
            // Dpad Up
            // Dpad Down
            // Left Bumper
            // Left Trigger
            // Right Bumper
            // Right Trigger
            // Left Stick X         Forward/Backward movement (Tank Mode: Left Track only)
            // Left Stick Y         Strafe: Left/Right movement
            // Left Stick Button
            // Right Stick X        Tank Mode: Right Track Only
            // Right Stick Y        Turning Left/Right
            // Right Stick Button
            // Back
            // Guide
            // Id
            // Start

            // GAME-PAD 2
            // X
            // Y
            // A
            // B
            // Dpad Left
            // Dpad Right
            // Dpad Up
            // Dpad Down
            // Left Bumper
            // Left Trigger
            // Right Bumper
            // Right Trigger
            // Left Stick X
            // Left Stick Y
            // Left Stick Button
            // Right Stick X
            // Right Stick Y
            // Right Stick Button
            // Back
            // Guide
            // Id
            // Start

            // Calculate power level to each motor based on drive-mode.
            switch (driveMode) {

                case TANK:
                    // Tank Mode uses one stick to control each wheel.
                    // This mode is hard to drive forward slowly and keep straight.
                    if (gamepad1.dpad_down || gamepad1.dpad_up) {
                        left = (gamepad1.dpad_down ? -0.3 : gamepad1.dpad_up ? 0.3 : 0);
                        right = left;
                    } else {
                        left = -gamepad1.left_stick_y;
                        right = -gamepad1.right_stick_y;
                    }
                    leftFrontPower = left;
                    leftBackPower = left;
                    rightFrontPower = right;
                    rightBackPower = right;
                    break;

                case POV:
                    // POV Mode uses left stick to go forward, and right stick to turn.
                    // This mode is easier to drive straight.
                    if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                        drive = (gamepad1.dpad_down ? -0.3 : gamepad1.dpad_up ? 0.3 : 0);
                        turn = (gamepad1.dpad_left? -0.4 : gamepad1.dpad_right ? 0.4 : 0);
                    } else {
                        drive = -gamepad1.left_stick_y;
                        turn = gamepad1.right_stick_x;
                    }
                    leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
                    leftBackPower = Range.clip(drive + turn, -1.0, 1.0);
                    rightFrontPower = Range.clip(drive - turn, -1.0, 1.0);
                    rightBackPower = Range.clip(drive - turn, -1.0, 1.0);
                    break;

                case STRAFE:
                    // Strafe Mode is similar to POV mode in that the movement is controlled by left stick and right stick turns.
                    // Strafe Mode also supports moving sideways based on X movements of the left stick.
                    if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                        drive = (gamepad1.dpad_down ? -0.3 : gamepad1.dpad_up ? 0.3 : 0);
                        strafe = (gamepad1.dpad_left? 1.0 : gamepad1.dpad_right ? -1.0 : 0);
                   } else {
                        drive = -gamepad1.left_stick_y;
                        strafe = -gamepad1.left_stick_x;
                    }
                    turn = gamepad1.right_stick_x;
                    leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
                    leftBackPower = Range.clip(drive + turn - (calLeftBack*strafe), -1.0, 1.0);
                    rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
                    rightBackPower = Range.clip(drive - turn + (calRightBack*strafe), -1.0, 1.0);
                    break;

            }

            // Switching Drive Mode when user presses left bumper and only if was not already pressed.
            if ((isLB = gamepad1.left_bumper) && !wasLB) {

                    // Check what the current drive mode is so we can switch to the next.
                    switch (driveMode) {

                        case POV:
                            // Switching mode POV --> TANK
                            if (tankFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, tankSoundID);
                            driveMode = DriveModes.TANK;
                            driveModeStr = "TANK";
                            break;

                        case TANK:
                            // Switching mode TANK --> STRAFE
                            if (strafeFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, strafeSoundID);
                            driveMode = DriveModes.STRAFE;
                            driveModeStr = "STRAFE";
                            break;

                        case STRAFE:
                            // Switching mode STRAFE --> POV
                            if (povFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, povSoundID);
                            driveMode = DriveModes.POV;
                            driveModeStr = "POV";
                            break;
                    }
            }

            if ((isRB = gamepad1.right_bumper) && !wasRB) {
                if (latchingFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, latchingSoundID);
            }

            //Basic rack drive using the second controller dpad or first controller X/Y buttons
            rackPower = -1.0*(gamepad1.right_trigger) + gamepad1.left_trigger;

            // Send calculated power to wheels.
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            rackDrive.setPower(rackPower);

            // Show the elapsed game time and Drive Mode.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", driveModeStr);
            telemetry.addData("Motor Power", "LF(%.2f) LB(%.2f) RF(%.2f) RB(%.2f)", leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Rack Power", rackPower);
            telemetry.update();

            // Save last button states
            wasLB = isLB;
            wasRB = isRB;
        }
        if (stopFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, stopSoundID);
    }
}
