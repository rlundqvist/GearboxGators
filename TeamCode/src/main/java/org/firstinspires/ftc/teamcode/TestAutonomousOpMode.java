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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

@Autonomous(name="Test Autonomous OpMode", group="Linear Opmode")
//@Disabled
public class TestAutonomousOpMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AVQIvK3/////AAABmSpkdI0smUlNgdcjJD9G/vN7/679ySpR3GdwNzNg717pJ0chtCNh8z+aJVbw67Z6YMVMxURf0aqiGDxRnZEzzXsYpKXM7+iOkjQMEFkhHKzDqQTisuhPEMNaLWqnhLmu/Ejm7THmC4nKiRBHBNH4vxkaKg7nnUxpAVsuK4zsB+uo2Qk8DdixYVacY46ec/OkvYGsgJCpo3eVmaDDtKmQNnUti9KNUi8C0IhKKAVH3LLOaffxwKvSneEX8ys2rBx8DOyX+4yQGkqqKmFBVq2ACXLubogbIZsacofWSteEUM+kZT4I1VsNoZy/RUih5k0ioINE3Ze8bWSqQ5BBG0u7XkULQ/SDBocUPk4qBVNnLWQq";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members.
    HardwareRobot robot           = new HardwareRobot();

    private boolean lowerFound = false;
    private boolean delatchFound = false;
    private boolean d2samplesFound = false;
    private boolean samplingFound = false;
    private boolean movegoldFound = false;
    private boolean d2allianceFound = false;
    private boolean dropmarkerFound = false;
    private boolean d2craterFound = false;
    private boolean touchcraterFound = false;
    private boolean waitingFound = false;
    private boolean silverFound = false;
    private boolean goldFound = false;
    private boolean mineralFound = false;

    private static final double FORWARD = 1.0;
    private static final double BACKWARD = -1.0;
    private static final double LEFT = 1.0;
    private static final double RIGHT = -1.0;

    @Override
    public void runOpMode() {

        UpdateTelemetryStatus("Initializing");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        // Initialize the hardware variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Determine Resource IDs for sounds built into the RC application.
        int lowerSoundID = hardwareMap.appContext.getResources().getIdentifier("lower","raw", hardwareMap.appContext.getPackageName());
        int delatchSoundID = hardwareMap.appContext.getResources().getIdentifier("delatch","raw", hardwareMap.appContext.getPackageName());
        int d2samplesSoundID = hardwareMap.appContext.getResources().getIdentifier("d2samples","raw", hardwareMap.appContext.getPackageName());
        int samplingSoundID = hardwareMap.appContext.getResources().getIdentifier("sampling","raw", hardwareMap.appContext.getPackageName());
        int movegoldSoundID = hardwareMap.appContext.getResources().getIdentifier("movegold","raw", hardwareMap.appContext.getPackageName());
        int d2allianceSoundID = hardwareMap.appContext.getResources().getIdentifier("d2alliance","raw", hardwareMap.appContext.getPackageName());
        int dropmarkerSoundID = hardwareMap.appContext.getResources().getIdentifier("dropmarker","raw", hardwareMap.appContext.getPackageName());
        int d2craterSoundID = hardwareMap.appContext.getResources().getIdentifier("d2crater","raw", hardwareMap.appContext.getPackageName());
        int touchcraterSoundID = hardwareMap.appContext.getResources().getIdentifier("touchcrater","raw", hardwareMap.appContext.getPackageName());
        int waitingSoundID = hardwareMap.appContext.getResources().getIdentifier("waiting","raw", hardwareMap.appContext.getPackageName());
        int silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver","raw", hardwareMap.appContext.getPackageName());
        int goldSoundID = hardwareMap.appContext.getResources().getIdentifier("gold","raw", hardwareMap.appContext.getPackageName());
        int mineralSoundID = hardwareMap.appContext.getResources().getIdentifier("object","raw", hardwareMap.appContext.getPackageName());


        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (lowerSoundID != 0)  lowerFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, lowerSoundID);
        if (delatchSoundID != 0) delatchFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, delatchSoundID);
        if (d2samplesSoundID != 0) d2samplesFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, d2samplesSoundID);
        if (samplingSoundID != 0) samplingFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, samplingSoundID);
        if (movegoldSoundID != 0) movegoldFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, movegoldSoundID);
        if (d2allianceSoundID != 0) d2allianceFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, d2allianceSoundID);
        if (dropmarkerSoundID != 0) dropmarkerFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, dropmarkerSoundID);
        if (d2craterSoundID != 0) d2craterFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, d2craterSoundID);
        if (touchcraterSoundID != 0) touchcraterFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, touchcraterSoundID);
        if (waitingSoundID != 0) waitingFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, waitingSoundID);
        if (silverSoundID != 0) silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);
        if (goldSoundID != 0) goldFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);
        if (mineralSoundID != 0) mineralFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, mineralSoundID);

        // Display sound status
        telemetry.addData("lower resource", lowerFound ? "Found" : "NOT found\n Add lower.wav to /src/main/res/raw" );
        telemetry.addData("delatch resource", delatchFound ? "Found" : "Not found\n Add delatch.wav to /src/main/res/raw" );
        telemetry.addData("drive to sample resource", d2samplesFound ? "Found" : "Not found\n Add d2samples.wav to /src/main/res/raw" );
        telemetry.addData("sampling resource", samplingFound ? "Found" : "Not found\n Add sampling.wav to /src/main/res/raw" );
        telemetry.addData("move silver resource", movegoldFound ? "Found" : "Not found\n Add movesilver.wav to /src/main/res/raw" );
        telemetry.addData("drive to alliance resource", d2allianceFound ? "Found" : "Not found\n Add  d2alliance.wav to /src/main/res/raw" );
        telemetry.addData("drop marker resource", dropmarkerFound ? "Found" : "Not found\n Add dropmarker.wav to /src/main/res/raw" );
        telemetry.addData("drive to crater resource", d2craterFound ? "Found" : "Not found\n Add d2crater.wav to /src/main/res/raw" );
        telemetry.addData("touch crater resource", touchcraterFound ? "Found" : "Not found\n Add touchcrater.wav to /src/main/res/raw" );
        telemetry.addData("waiting resource", waitingFound ? "Found" : "Not found\n Add waiting.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", d2craterFound ? "Found" : "Not found\n Add silver.wav to /src/main/res/raw" );
        telemetry.addData("gold resource", touchcraterFound ? "Found" : "Not found\n Add gold.wav to /src/main/res/raw" );
        telemetry.addData("mineral resource", waitingFound ? "Found" : "Not found\n Add mineral.wav to /src/main/res/raw" );
        telemetry.update();


        UpdateTelemetryStatus("Waiting for User Action START");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Lower
        if (lowerFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, lowerSoundID);
        UpdateTelemetryStatus("Lowering");
        sleep(5000);

        // De-latch
        if (delatchFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, delatchSoundID);
        UpdateTelemetryStatus("De-latching");
        sleep(3000);

        // Drive to samples
        if (d2samplesFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2samplesSoundID);
        DriveForward(3000);

        // Sampling
        if (samplingFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, samplingSoundID);
        UpdateTelemetryStatus("Identifying gold through sampling");

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
        double startTime = runtime.seconds();
        while (opModeIsActive() && (runtime.seconds()<(startTime+20))) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (mineralFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mineralSoundID);
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }


        // Move Silver Sample
        if (movegoldFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, movegoldSoundID);
        UpdateTelemetryStatus("Removing silver");
        sleep(3000);

        // Drive to Alliance Area
        if (d2allianceFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2allianceSoundID);
        TurnLeft(1000);
        DriveForward(5000);

        // Drop Team Marker
        if (dropmarkerFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, dropmarkerSoundID);
        UpdateTelemetryStatus("Dropping team market");
        sleep(3000);

        // Drive to Crater
        if (d2craterFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2craterSoundID);
        DriveBackward(1000);
        TurnRight(4000);
        DriveForward(6000);

        // Touch Crater
        if (touchcraterFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, touchcraterSoundID);
        UpdateTelemetryStatus("Touching crater wall");
        sleep(3000);

        // Autonomous Waiting
        if (waitingFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, waitingSoundID);
        UpdateTelemetryStatus("Autonomous mode over");

    }

    void DriveForward (double milliseconds) {
        ElapsedTime timer = new ElapsedTime();

        robot.DrivePOV(FORWARD,0);

        while (opModeIsActive() && (timer.milliseconds() < milliseconds)) {
            UpdateTelemetryStatusTime("Drive Forward",timer.seconds());
        }

        robot.DrivePOV(0,0);
    }

    void DriveBackward (double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        robot.DrivePOV(BACKWARD, 0);

        while (opModeIsActive() && (timer.milliseconds() < milliseconds)) {
            UpdateTelemetryStatusTime("Drive Backward",timer.seconds());
        }

        robot.DrivePOV(0,0);
    }

    void TurnLeft (double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        robot.DrivePOV(0,LEFT);
        while (opModeIsActive() && (timer.milliseconds() < milliseconds)) {
            UpdateTelemetryStatusTime("Turn Left",timer.seconds());
        }

        robot.DrivePOV(0,0);
    }

    void TurnRight (double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        robot.DrivePOV(0,RIGHT);
        while (opModeIsActive() && (timer.milliseconds() < milliseconds)) {
            UpdateTelemetryStatusTime("Turn Right",timer.seconds());
        }

        robot.DrivePOV(0,0);
    }

    void UpdateTelemetryStatusTime(String status, double seconds) {
        // Show the elapsed game time and Drive Mode.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Mode", status);
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", seconds);
        telemetry.update();
    }
    void UpdateTelemetryStatus(String status) {
        // Show the elapsed game time and Drive Mode.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Mode", status);

        telemetry.update();
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
