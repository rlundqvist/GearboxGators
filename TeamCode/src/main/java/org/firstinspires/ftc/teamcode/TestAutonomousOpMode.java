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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


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
 *
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 *
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 *
 *  * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *  *
 *  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *  *
 *  * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

@Autonomous(name="Concept: Autonomous OpMode", group="Linear Opmode")
//@Disabled
public class TestAutonomousOpMode extends LinearOpMode {

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;   // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;      // the height of the center of the target image above the floor

    // Select which camera on the mobile that you want use.
    // The FRONT camera is the one on the same side as the screen.
    // Usually the back camera has higher resolution.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Define "vuforia" - the variable we will use to store our instance of the Vuforia
    private VuforiaLocalizer vuforia;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private String targetSeen = "";

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AVQIvK3/////AAABmSpkdI0smUlNgdcjJD9G/vN7/679ySpR3GdwNzNg717pJ0chtCNh8z+aJVbw67Z6YMVMxURf0aqiGDxRnZEzzXsYpKXM7+iOkjQMEFkhHKzDqQTisuhPEMNaLWqnhLmu/Ejm7THmC4nKiRBHBNH4vxkaKg7nnUxpAVsuK4zsB+uo2Qk8DdixYVacY46ec/OkvYGsgJCpo3eVmaDDtKmQNnUti9KNUi8C0IhKKAVH3LLOaffxwKvSneEX8ys2rBx8DOyX+4yQGkqqKmFBVq2ACXLubogbIZsacofWSteEUM+kZT4I1VsNoZy/RUih5k0ioINE3Ze8bWSqQ5BBG0u7XkULQ/SDBocUPk4qBVNnLWQq";

   // Store our instance of the Tensor Flow Object Detection engine.
    private TFObjectDetector tfod;

    private static int objectsDetected = 0;
    private static int goldIndex = -1;

    // Timer used to track progress and handle time-outs
    private ElapsedTime runtime = new ElapsedTime();
    private double timeOut;


    private HardwareRobot robot = new HardwareRobot();

    // Identifying if a particular Sound file is found and can be preloaded
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

    private static double xCoord;
    private static double yCoord;
    private static double zCoord;
    private static double roll;
    private static double pitch;
    private static double heading;

    private static final double FORWARD = 1.0;
    private static final double BACKWARD = -1.0;
    private static final double LEFT = 1.0;
    private static final double RIGHT = -1.0;

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private String systemStatusStr;
    private String calStatusStr;
    private static double headingIMU;
    private static double pitchIMU;
    private static double rollIMU;
    private static double mag;

    private enum autoStates { LOWERING, DELATCHING, MOVETO_SAMPLES, IDENTIFY_GOLD, REMOVE_GOLD, MOVETO_ALLIANCE_ZONE, DROP_MARKER, MOVETO_CRATER, LOWER_ARM, FINISHED}
    private autoStates autoState = autoStates.LOWERING;

    private static boolean newState = true;
    private static String stateLabel = "Initializing";

    @Override
    public void runOpMode() {

        telemetry.addData("State", "Initializing... (0%)");
        telemetry.addData("IMU", "Init")
        telemetry.update();
        // TO-DOS
        // 1. Remove logging from IMU code
        // 2. Add telemetry status and updates to monitor progress
        // 3. What to do if we dont know where gold is? Guess? Ignore?
        // 4. Define new motors in the Robot class
        // 5. Implement driving by encoder
        // 6. Decide on strategy after Gold sampling
        //      - there is a risk to bump into other robots and get stuck
        //      - should we ask other teams how they drive and different modes?
        //      - Use sensors?
        //      - Do whatever is easiest/closest zone/crater
        // 7. Add handling of additional sensors
        //      - Magnetic
        //      - Touch
        //      - Color
        //      - Distance
        //      - Webcams

        /************************************************
         * Initialize the Inertial Measurement Unit - IMU
         ************************************************/
        // Initialize the Inertial Measurement Unit (IMU) - "Gyro"
        // Set up the parameters with which we will use our IMU.
        // Note that integration algorithm here just reports accelerations to the logcat log;
        // it doesn't actually provide positional information.
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Created by the Calibration OpMode
        IMUparameters.loggingEnabled      = true;
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        telemetry.addData("State", "Initializing... (60%)");
        telemetry.addData("IMU","DONE");
        telemetry.addData("Vuforia", "Navigation Init");
        telemetry.update();


        /*************************************************************************************************
         *  Initialize Vuforia which is used for sample identification with Tensor Flow and for Navigation
         *************************************************************************************************/
        // Initialize Vuforia used for BOTH navigation and for object detection/identification through Tensor Flow
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer
        //
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        // We can pass Vuforia the handle to a camera preview resource (on the RC phone);
        // If no camera monitor is desired, use the parameterless constructor instead (commented out below).
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Trackables are loaded to enable navigation/location only

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        telemetry.addData("State", "Initializing... (70%)");
        telemetry.addData("IMU","DONE");
        telemetry.addData("Vuforia", "DONE");
        telemetry.addData("Tensor Flow", "Object Detection Init");
        telemetry.update();

        // Validate that a Tensor Flow object can be created
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /***************************************************************
         * Initialize all sounds and validate that they can be preloaded
         ***************************************************************/
        telemetry.addData("State", "Initializing... (80%)");
        telemetry.addData("IMU","DONE");
        telemetry.addData("Vuforia", "DONE");
        telemetry.addData("Tensor Flow", "DONE");
        telemetry.addData("Sound Resources", "Pre-loading");
        telemetry.update();

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

        telemetry.addData("State", "Initializing... (90%)");
        telemetry.addData("IMU","DONE");
        telemetry.addData("Vuforia", "DONE");
        telemetry.addData("Tensor Flow", "DONE");
        telemetry.addData("Sound Resources", "DONE");
        telemetry.addData("Robot", "Hardware Init");
        telemetry.update();

        // Display sound status
//        telemetry.addData("lower resource", lowerFound ? "Found" : "NOT found\nAdd lower.wav to /src/main/res/raw" );
//        telemetry.addData("delatch resource", delatchFound ? "Found" : "Not found\nAdd delatch.wav to /src/main/res/raw" );
//        telemetry.addData("drive to sample resource", d2samplesFound ? "Found" : "Not found\nAdd d2samples.wav to /src/main/res/raw" );
//        telemetry.addData("sampling resource", samplingFound ? "Found" : "Not found\n dd sampling.wav to /src/main/res/raw" );
//        telemetry.addData("move silver resource", movegoldFound ? "Found" : "Not found\nAdd movesilver.wav to /src/main/res/raw" );
//        telemetry.addData("drive to alliance resource", d2allianceFound ? "Found" : "Not found\nAdd  d2alliance.wav to /src/main/res/raw" );
//        telemetry.addData("drop marker resource", dropmarkerFound ? "Found" : "Not found\nAdd dropmarker.wav to /src/main/res/raw" );
//        telemetry.addData("drive to crater resource", d2craterFound ? "Found" : "Not found\nAdd d2crater.wav to /src/main/res/raw" );
//        telemetry.addData("touch crater resource", touchcraterFound ? "Found" : "Not found\nAdd touchcrater.wav to /src/main/res/raw" );
//        telemetry.addData("waiting resource", waitingFound ? "Found" : "Not found\nAdd waiting.wav to /src/main/res/raw" );
//        telemetry.addData("silver resource", d2craterFound ? "Found" : "Not found\nAdd silver.wav to /src/main/res/raw" );
//        telemetry.addData("gold resource", touchcraterFound ? "Found" : "Not found\nAdd gold.wav to /src/main/res/raw" );
//        telemetry.addData("mineral resource", waitingFound ? "Found" : "Not found\nAdd mineral.wav to /src/main/res/raw" );
//        telemetry.update();


        // Initialize the hardware variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        telemetry.addData("State", "READY to go - Press PLAY");
        telemetry.addData("IMU","DONE");
        telemetry.addData("Vuforia", "DONE");
        telemetry.addData("Tensor Flow", "DONE");
        telemetry.addData("Sound Resources", "DONE");
        telemetry.addData("Robot", "DONE");
        telemetry.update();

        autoState = autoStates.LOWERING;;

        // Wait for the game to start (driver presses PLAY button on Driver Station)
        waitForStart();

        /**************************
         * Starting Autonomous 30s
         *************************/
        runtime.reset();

        // Start the logging of measured acceleration
        // Not sure this is needed
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Start tracking the data sets we care about.
        targetsRoverRuckus.activate();

        // Activate Tensor Flow Object Detection
        if (tfod != null) {
            tfod.activate();
        }

        int tmpCounter = 0;

        /*******************************************************************************************
         *  RUNNING in Autonomous Mode
         *******************************************************************************************/
        while (opModeIsActive()) {

            /*************************************************************
             * Update IMU-based angels and gravity in ALL autonomous modes
             *************************************************************/
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();

            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            systemStatusStr = imu.getSystemStatus().toShortString();
            calStatusStr = imu.getCalibrationStatus().toString();
            headingIMU = angles.firstAngle;
            rollIMU = angles.secondAngle;
            pitchIMU = angles.thirdAngle;
            mag = Math.sqrt(gravity.xAccel*gravity.xAccel  + gravity.yAccel*gravity.yAccel + gravity.zAccel*gravity.zAccel);


            /********************************************************************
             *  Aim to perform Vuforia-based positioning in ALL autonomous stages
             ********************************************************************/
            // Check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            targetSeen = " ";
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    targetSeen = trackable.getName();

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform; // Update with new position
                        VectorF translation = lastLocation.getTranslation();
                        xCoord = translation.get(0) / mmPerInch;
                        yCoord = translation.get(1) / mmPerInch;
                        zCoord = translation.get(2) / mmPerInch;
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        roll = rotation.firstAngle;
                        pitch = rotation.secondAngle;
                        heading = rotation.thirdAngle;
                    }
                    break;
                }
            }

            /**************************************************************************************
             * Aim to identify which out of three minerals is GOLD using Vuforia and Tensor Flow
             * in LOWERING, DELATCHING, MOVETO_SAMPLES and IDENTIFY_GOLD states.
             * Ideally we will have identified where the Gold is long before we have moved out to
             * the sample area.
             * Stop doing it once the position of the Gold mineral has been successfully identified
             **************************************************************************************/
            if ((tfod != null) &&    // Tensor Flow is running
                    (goldIndex == -1) &&  // Gold has not yet been identified
                    (autoState == autoStates.LOWERING || autoState == autoStates.DELATCHING ||
                         autoState == autoStates.MOVETO_SAMPLES || autoState == autoStates.IDENTIFY_GOLD)) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    objectsDetected = updatedRecognitions.size();
                    telemetry.addData("Tensor Flow", "Objects Detected %d", objectsDetected);
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
                            // All minerals have been identified
                            goldIndex = goldMineralX;
                            if (mineralFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mineralSoundID);
                        }
                    }
                }
            }

            switch (autoState) {

                /****************************************************************************
                 * First state after Autonomous mode has been initiated.
                 * Lower robot to ground by extending rack & pinion to predetermined position
                 * While robot is being lowered, Tensor Flow aims to identify how the minerals
                 * in the sampling field are ordered.
                 ****************************************************************************/
                case LOWERING:
                    if (newState) {
                        stateLabel = "Lowering robot to ground";
                        if (lowerFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, lowerSoundID);
                        newState = false;
                        // Start driving rack to full position using encoder
                        timeOut = runtime.milliseconds() + 10000;
//                    } else if (!robot.rackDrive.isBusy() || (runtime.milliseconds()>timeOut)){
                    } else if (runtime.milliseconds()>timeOut){
                        // Stop rack-motor
                        autoState = autoStates.DELATCHING;
                        newState = true;
                    }
                    break;

                /*********************************************************************************
                 * Robot has reached the ground. Move sideways to disconnect from the lunar lander
                 * While moving the robot sideways, Tensor Flow aims to identify how the minerals
                 * in the sampling field are ordered.
                 *********************************************************************************/
                case DELATCHING:
                    if (newState) {
                        stateLabel = "Disconnecting from lunar latch";
                        if (delatchFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, delatchSoundID);
                        newState = false;
                        // Move sideways for a few inches
                        timeOut = runtime.milliseconds() + 10000;
//                    } else if (!robot.left_front_drive.isBusy() || (runtime.milliseconds()>timeOut)){
                    } else if (runtime.milliseconds()>timeOut){
                        // Stop rack-motor
                        autoState = autoStates.MOVETO_SAMPLES;
                        newState = true;
                    }
                    break;

                /*********************************************************************************
                 * Robot has disconnected from the lunar lander and is driving towards the mineral
                 * sampling location. While driving, Tensor Flow aims to identify how the minerals
                 * are ordered.
                 *********************************************************************************/
                case MOVETO_SAMPLES:
                    if (newState) {
                        stateLabel = "Moving to sampling-area";
                        if (d2samplesFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2samplesSoundID);
                        newState = false;
                        // Move out to the sampling area
                        // Using coordinates or relative measurements?
                        timeOut = runtime.milliseconds() + 10000;
//                    } else if (!robot.left_front_drive.isBusy() || (runtime.milliseconds()>timeOut)){
                    } else if (runtime.milliseconds()>timeOut){
                        // Stop motors
                        autoState = autoStates.IDENTIFY_GOLD;
                        newState = true;
                    }
                    break;

                /*******************************************************************************
                 * Robot is standing in front of the three mineral samples. If we havent already,
                 * we are now attempting to identify where the Gold is located:
                 *  -1: We do not know where it is
                 *   0: The Gold is to the left   (GOLD - SILVER - SILVER)
                 *   1: The Gold is in the middle (SILVER - GOLD - SILVER)
                 *   2: The Gold is to the right  (SILVER - SILVER - GOLD)
                 * We attempt to identify the location in already BEFORE we stop in front of
                 * the samples.
                 ******************************************************************************/
                case IDENTIFY_GOLD:
                    if (newState) {
                        stateLabel = "Identifying where the Gold is";
                        if (samplingFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, samplingSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                    }
                    if (goldIndex != -1 || (runtime.milliseconds()>timeOut))  {
                        // Stop Tensor Flow
                         if (tfod != null) {
                            tfod.shutdown();
                        }
                        autoState = autoStates.REMOVE_GOLD;
                        newState = true;
                        if (goldIndex == -1) {
                            // We have not figured out where the gold is...
                            // Shall we guess with 1/3 chance....or avoid ALL samples and just drive around?
                        } else if (goldFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                    }
                    break;

                /********************************************************************************
                 * Moving the robot so that we push the Gold mineral away from the sampling area
                 * A goldIndex of -1 will simply be ignored and robot will do NOTHING
                 ********************************************************************************/
                case REMOVE_GOLD:
                    if (newState && goldIndex != -1) {
                        stateLabel = "Moving the Gold mineral";
                        if (movegoldFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, movegoldSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                        // Use goldIndex to figure out where to go
                    } else if (runtime.milliseconds()>timeOut || goldIndex == -1)  {
                       // Stop any motors
                       autoState = autoStates.MOVETO_ALLIANCE_ZONE;
                       newState = true;
                    }
                    break;

                /**********************************************************************************
                 * Drive to OUR alliance zone. Use position to identify where we are starting.
                 * Must implement sensors and/or different modes to avoid the other robots.
                 * Avoid touching mineral samples, lunar lander and other robots.
                 * Consider using color sensor under the robot to identify when we are inside the
                 * alliance zone.
                 **********************************************************************************/
                case MOVETO_ALLIANCE_ZONE:
                    if (newState) {
                        stateLabel = "Moving to Alliance Zone";
                        if (d2allianceFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2allianceSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                        // Move to alliance area...
                    }
                    else if (runtime.milliseconds() > timeOut) {
                        // Stop any movement
                        autoState = autoStates.DROP_MARKER;
                        newState = true;
                    }
                    break;

                /**********************************************************************************
                 * Robot has reached the alliance zone. Drop the team marker in a way so that it
                 * stays within the alliance zone. If using the large mineral collection mechanism,
                 * consider what happens if team marker is dropped from high level.
                 **********************************************************************************/
                case DROP_MARKER:
                    if (newState) {
                        stateLabel = "Dropping team marker";
                        if (dropmarkerFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, dropmarkerSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                        // Move motors to drop marker
                    } else if (runtime.milliseconds() > timeOut) {
                        // Stop all movement
                        autoState = autoStates.MOVETO_CRATER;
                        newState = true;
                    }
                    break;

                /**********************************************************************************
                 * Robot move to (closest?) crater regardless of its current location
                 **********************************************************************************/
                case MOVETO_CRATER:
                    if (newState) {
                        stateLabel = "Moving to crater";
                        if (d2craterFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, d2craterSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                        // Move motors to drop marker
                    } else if (runtime.milliseconds() > timeOut) {
                        // Stop all movement
                        autoState = autoStates.LOWER_ARM;
                        newState = true;
                    }
                    break;

                /**********************************************************************************
                 * Robot has reached the edge of the crater. Lower the collection arm to make
                 * we touch the crater and get points.
                 **********************************************************************************/
                case LOWER_ARM:
                    if (newState) {
                        stateLabel = "Lovering arm to crater";
                        if (touchcraterFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, touchcraterSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                        // Move motors to drop marker
                    } else if (runtime.milliseconds() > timeOut) {
                        // Stop all movement
                        autoState = autoStates.FINISHED;
                        newState = true;
                    }
                    break;

                /**********************************************************************************
                 * All commands are executed, waiting until Autonomous is over, user terminates or
                 * function times out
                 **********************************************************************************/
                case FINISHED:
                    if (newState) {
                        stateLabel = "Waiting until autonomous is done";
                        if (waitingFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, waitingSoundID);
                        newState = false;
                        timeOut = runtime.milliseconds() + 10000;
                    } else if (runtime.milliseconds() > timeOut) {
                        break;
                    }
                    break;
            }
            /**********************************************************************************
             * For every WHILE loop during the autonomous mode we update the telemetry data
             **********************************************************************************/
            telemetry.addData("Run-time", runtime.toString());
            telemetry.addData("Status", stateLabel);
            telemetry.addData("IMU", "Accel = %.1f   Cal = %s", mag, calStatusStr);
            telemetry.addData("  Rot (deg)", "{Roll, Pitch, Heading} = %.1f, %.1f, %.1f", rollIMU, pitchIMU, headingIMU);

            if (targetVisible) { // Do we have a valid position from Vuforia?
                telemetry.addData("Vuforia Target ", targetSeen);
                telemetry.addData("  Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", xCoord, yCoord, zCoord);
                telemetry.addData("  Rot (deg)", "{Roll, Pitch, Heading} = %.1f, %.1f, %.1f", roll, pitch, heading);
            }
            else {
                telemetry.addData("Vuforia Target ", targetSeen);
                telemetry.addData("  Pos (in)", "{X, Y, Z} = ");
                telemetry.addData("  Rot (deg)", "{Roll, Pitch, Heading} = ");
            }
            switch(goldIndex){
                case -1:
                    telemetry.addData("Tensor Flow", "Objects %d  -  Gold Not Identified", objectsDetected);
                    break;
                case 0:
                    telemetry.addData("Tensor Flow", "Objects %d  -  Gold LEFT", objectsDetected);
                    break;
                case 1:
                    telemetry.addData("Tensor Flow", "Objects %d  -  Gold MIDDLE", objectsDetected);
                    break;
                case 2:
                    telemetry.addData("Tensor Flow", "Objects %d  -  Gold RIGHT", objectsDetected);
                    break;
            }
             telemetry.update();
        }
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
}
