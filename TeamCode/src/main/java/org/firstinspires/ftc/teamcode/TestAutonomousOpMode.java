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
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * This file contains an concept implementation of a linear autonomous "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the teleop period of an FTC match. The names of
 * OpModes appear on the menu of the FTC Driver Station. When an selection is made from the menu,
 * the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode/
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
    private static int numGold = 0;
    private static int numSilver = 0;

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
    private static boolean noPreviousCoord = true;

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

    private static double xFactor = 1.0;
    private static double yFactor = 1.0;

    private static final double     DRIVE_SPEED             = 0.1;
    private static final double     TURN_SPEED              = 0.1;
    private static final double     LOWER_SPEED             = 0.1;

    static private int leg = 1;

    @Override
    public void runOpMode() {

        telemetry.addData("State", "Initializing... (0%)");
        telemetry.addData("IMU", "Init");
        telemetry.update();

        robot.init(hardwareMap);

        // TODO: Define all new motors, servors and sensors
        //
        // TODO: Strategic decision - How to move to alliance depot without bumping into alliance partner?
        //      - there is a risk to bump into other robots and get stuck
        //      - should we ask other teams how they drive and use multiple different modes/routes?
        //      - Use sensors?
        //      - Do whatever is easiest/closest zone/crater
        //
        // TODO: Installe and leverage additional sensors
        //      - Magnetic
        //      - Touch
        //      - Color
        //      - Distance
        //      - Webcams
        //
        // TODO: Update the timoeuts

        /************************************************
         * Initialize the Inertial Measurement Unit - IMU
         ************************************************/
        // Initialize the Inertial Measurement Unit (IMU) - "Gyro"
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Created by the Calibration OpMode

        // TODO: Review how the IMU is used and consider reving thew logcat information
        // Note that integration algorithm here just reports accelerations to the logcat log;
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
         * where the phone resides on the robot.  These specifications are in the form of transformation matrices.
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
         * coordinate system (the center of the field), facing up.
         *
         *           +----------------------- (Back) Space ------------------------+
         *           |                                                             |
         *           |  Blue                       +                      Red      |
         *           |  Crater                     ^                      Alliance |
         *           |                             |                               |
         *           |                             |                               |
         *           |              Q1                              Q2             |
         *           |                             X                               |
         *           |                                                             |
         *           |                             |                               |
         *         (Blue)                          |                              (Red)
         *         Rover      + <------- Y --------+ 0                         Footprint
         *           |                             0                               |
         *           |                                                             |
         *           |                                                             |
         *           |                                                             |
         *           |              Q4                              Q3             |
         *           |                                                             |
         *           |                                                             |
         *           |  Blue                                                Red    |
         *           |  Alliance                                            Crater |
         *           |                                                             |
         *           +---------------------- (Front) Craters ----------------------+
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
         *                +---------------------+
         *                |                     |
         *               0|                     |0
         *                |                     |
         *                |          cam        |
         *                |         ==========  |
         *                |                     |
         *               0|                     |0
         *                |                     |
         *                +---------------------+
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

        // TODO: Update with ACTUAL displacement once build has installed phone holder
        // TODO: Make sure we consider BOTH phone Models (Moto and Samsung galaxy S5)
        // TODO: Consider using the Samsung if it is wider screen so we can see the samples
        final int CAMERA_FORWARD_DISPLACEMENT  = 200;   // eg: Camera is 200 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 100;   // eg: Camera is 100 mm above ground
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

        // TODO: Enable ONLY during testing and if some of the sounds are not heard
        // telemetry.addData("lower resource", lowerFound ? "Found" : "NOT found\nAdd lower.wav to /src/main/res/raw" );
        // telemetry.addData("delatch resource", delatchFound ? "Found" : "Not found\nAdd delatch.wav to /src/main/res/raw" );
        // telemetry.addData("drive to sample resource", d2samplesFound ? "Found" : "Not found\nAdd d2samples.wav to /src/main/res/raw" );
        // telemetry.addData("sampling resource", samplingFound ? "Found" : "Not found\n dd sampling.wav to /src/main/res/raw" );
        // telemetry.addData("move silver resource", movegoldFound ? "Found" : "Not found\nAdd movesilver.wav to /src/main/res/raw" );
        // telemetry.addData("drive to alliance resource", d2allianceFound ? "Found" : "Not found\nAdd  d2alliance.wav to /src/main/res/raw" );
        // telemetry.addData("drop marker resource", dropmarkerFound ? "Found" : "Not found\nAdd dropmarker.wav to /src/main/res/raw" );
        // telemetry.addData("drive to crater resource", d2craterFound ? "Found" : "Not found\nAdd d2crater.wav to /src/main/res/raw" );
        // telemetry.addData("touch crater resource", touchcraterFound ? "Found" : "Not found\nAdd touchcrater.wav to /src/main/res/raw" );
        // telemetry.addData("waiting resource", waitingFound ? "Found" : "Not found\nAdd waiting.wav to /src/main/res/raw" );
        // telemetry.addData("silver resource", d2craterFound ? "Found" : "Not found\nAdd silver.wav to /src/main/res/raw" );
        // telemetry.addData("gold resource", touchcraterFound ? "Found" : "Not found\nAdd gold.wav to /src/main/res/raw" );
        // telemetry.addData("mineral resource", waitingFound ? "Found" : "Not found\nAdd mineral.wav to /src/main/res/raw" );
        // telemetry.update();

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

        // TODO: Store IMU parameter from initial position hanging on lander
        // What is our heading, roll and pitch while hanging?
        // Store these value and consider them for future navigation

        // TODO: Save and/or reset IMU readings
        // Enables us to track from here-on how the robot is moving on the playing field,
        // if we land uneven and/or with unexpected heading and if we are unable to gather
        // coordinates from Vuforia and the navigation targets

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

                        // Store the last valid location
                        lastLocation = robotLocationTransform;
                        VectorF translation = lastLocation.getTranslation();
                        xCoord = translation.get(0) / mmPerInch;
                        yCoord = translation.get(1) / mmPerInch;
                        zCoord = translation.get(2) / mmPerInch;
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        roll = rotation.firstAngle;
                        pitch = rotation.secondAngle;
                        heading = rotation.thirdAngle;

                        if (noPreviousCoord &&
                                (autoState == autoStates.LOWERING || autoState == autoStates.DELATCHING ||
                                        autoState == autoStates.MOVETO_SAMPLES || autoState == autoStates.IDENTIFY_GOLD)) {

                            // If we are in the first three states try to use the first valid coordinates
                            // to identify starting quadrant and from there determine movement patterns
                            // to/from the alliance zones and the crater
                            // Four different starting positions:
                            // 1. Red Crater
                            // 2. Red Alliance Zone
                            // 3. Blue Crater
                            // 4. Blue Alliance Zone

                            noPreviousCoord = false;
                            if (xCoord < 0 && yCoord < 0) {
                                // Playing RED alliance and started in from of "red" crater
                                // Quadrant 3: Negative X and Y
                                xFactor = -1.0;
                                yFactor = -1.0;

                            } else if (xCoord > 0 && yCoord < 0) {
                                // Playing RED alliance and started in front of red alliance zone
                                // Quadrant 2: Positive X and Negative Y
                                xFactor =  1.0;
                                yFactor = -1.0;

                            } else if (xCoord > 0 && yCoord > 0) {
                                // Playing BLUE alliance and started in front of "blue" crater
                                // Quadrant 1: Positive X and Y
                                xFactor = 1.0;
                                yFactor = 1.0;

                            } else {
                                // Playing BLUE alliance and started in front of blue alliance zone
                                // Quadrant 4: Negative X and Positive Y
                                xFactor = -1.0;
                                yFactor =  1.0;

                            }

                        }
                    }
                }
            }

            /**************************************************************************************
             * Aim to identify which out of three minerals is GOLD using Vuforia and Tensor Flow
             * in LOWERING, DELATCHING, MOVETO_SAMPLES and IDENTIFY_GOLD states.
             * Ideally we will have identified where the Gold is long before we have moved out to
             * the sample area.
             * Stop doing it once the position of the Gold mineral has been successfully identified
             **************************************************************************************/
            if ((tfod != null) && // Tensor Flow is running
                    (goldIndex == -1) &&  // Gold has not yet been identified
                    (autoState == autoStates.LOWERING || autoState == autoStates.DELATCHING ||
                         autoState == autoStates.MOVETO_SAMPLES || autoState == autoStates.IDENTIFY_GOLD)) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    // New information/recognitions found
                    // How hany different objects can we see?
                    objectsDetected = updatedRecognitions.size();

                    // If we recognize EXACTLY three objects
                    // Check how they are ordered from Left to Right
                    if (updatedRecognitions.size() == 3) {

                        // First reset out index variables
                        // Value of -1 means we havent identified this object yet
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;

                        // For each of the three recognitions
                        for (Recognition recognition : updatedRecognitions) {

                            // Is it a GOLD mineral?
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                // Set the of the Gold mineral to index of this object
                                // 0=LEFT, 1=CENTER, 2=RIGHT
                                goldMineralX = (int) recognition.getLeft();

                            } else if (silverMineral1X == -1) {

                                // It was NOT Gold so it must be SILVER.
                                // Is FIRST Silver still not identified (-1)?
                                // If so - this IS the first. Set its index to this object.
                                silverMineral1X = (int) recognition.getLeft();

                            } else {

                                // It must be a SILVER Mineral, but we had already found the first one
                                // so this is the second. Set its index to this object.
                                silverMineral2X = (int) recognition.getLeft();
                            }

                            // Note 1: If there are more than one Gold in the sample of three minerals
                            //         GoldMineralX will contain index to LAST (rightmost) one
                            //         GGS -> 1, GSG -> 2, SGG -> 2
                            // Note 2: If there are three Silver minerals in the sample
                            //         silverMineral1X will contain index to first (leftmost) one &
                            //         silverMineral2X will contain index ro
                        }

                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {

                            // TODO: Consider simplifying this test
                            // The above test demands that there are exactly 1 Gold and 2 Silver.
                            // Not sure if that is really needed?
                            // I suggest we replace by simply testing if "a" Gold mineral has been identified,
                            // Regardless of how many...like "if (goldMineralX != -1)"

                            // Remember where the gold is located
                            goldIndex = goldMineralX;

                            if (mineralFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mineralSoundID);
                        }
                    } else {

                        // TODO: This whole "else" branch can be disabled during competition
                        // Recognitions are made but more/less than 3
                        // Used for testing/calibration purposes to validated from what distance
                        // and under what conditions we can recognize the minerals

                        numGold = 0;
                        numSilver = 0;
                        for (Recognition recognition : updatedRecognitions) {

                            // Is it a GOLD mineral?
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                numGold++;
                                if (goldFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                            }

                            // Is it a SILVER mineral?
                            if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                numSilver++;
                                if (silverFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                            }

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

                        robot.encodeRack(LOWER_SPEED, 4.0);

                        timeOut = runtime.milliseconds() + 20000;

                    } else if (!robot.rackBusy() || (runtime.milliseconds()>timeOut)){

                        robot.encodeRackStop();

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

                        // Move robot sideways off the latch
                        robot.encodeDrive(TURN_SPEED, 5.0, -5.0, -5.0, 5.0);

                        timeOut = runtime.milliseconds() + 20000;

                    } else if (!robot.driveBusy() || (runtime.milliseconds()>timeOut)){

                        robot.encodeDriveStop();

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

                        // Drive up in from of samples
                        robot.encodeDrive(DRIVE_SPEED, 12, 12, 12, 12);

                        timeOut = runtime.milliseconds() + 20000;

                    } else if (!robot.driveBusy() || (runtime.milliseconds()>timeOut)){


                        robot.encodeDriveStop();

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
                        timeOut = runtime.milliseconds() + 20000;

                        // TODO: Determine if this state is really needed. Maybe we will have seen the gold before we get here?
                        // TODO: Or maybe we need to aim camera downwards to avoid seeing minerals in crater?
                    }
                    if (goldIndex != -1 || (runtime.milliseconds()>timeOut))  {

                        // We are done with mineral sampling and can stop Tensor Flow if still running
                         if (tfod != null) {
                            tfod.shutdown();
                        }

                        autoState = autoStates.REMOVE_GOLD;
                        newState = true;
                        if (goldIndex == -1) {

                            // We could NOT identify the gold so we will guess that it is in the middle position
                            goldIndex = 1;

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
                        timeOut = runtime.milliseconds() + 20000;

                         // Drive Forward enough to move gold mineral
                        leg = 1;

                    } else if (runtime.milliseconds()>timeOut || goldIndex == -1)  {

                       robot.encodeDriveStop();

                       autoState = autoStates.MOVETO_ALLIANCE_ZONE;
                       newState = true;


                    } else if (!robot.driveBusy()) {
                        switch(leg) {
                            case 1: // Make Appropriate Turn
                                if (goldIndex==0) {
                                    // Gold on the legt - turn left
                                    robot.encodeDrive(TURN_SPEED, -4, -4, 4, 4);
                                }
                                if (goldIndex==2){
                                    // Gold on the right - rurn right
                                    robot.encodeDrive(TURN_SPEED, 4, 4, -4, -4);
                                }
                                leg++;
                                break;

                            case 2: // Move forward to push gold mineral
                                robot.encodeDrive(DRIVE_SPEED, 8, 8, 8, 8);
                                leg++;
                                break;

                            case 3: // Back up
                                robot.encodeDrive(DRIVE_SPEED, -8, -8, -8, -8);
                                leg++;
                                break;

                            case 4: // Turn sideways
                                if (goldIndex==0) {
                                    // We are already turned 4 to the left, turn 8 more --> 12 total to the left
                                    robot.encodeDrive(TURN_SPEED, -8, -8, 8, 8);
                                }
                                if (goldIndex==1){
                                    // We are straight, turn 12 to the left
                                    robot.encodeDrive(TURN_SPEED, -12, -12, 12, 12);
                                }
                                if (goldIndex==2){
                                    // We are turned 4 to the right, turn 4+12=16 to the left --> 12 total to the left
                                    robot.encodeDrive(TURN_SPEED, -16, -16, 16, 16);
                                }
                                leg++;
                                break;

                            case 5: // Done turning
                                robot.encodeDriveStop();

                                autoState = autoStates.MOVETO_ALLIANCE_ZONE;
                                newState = true;

                                break;
                        }

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
                        timeOut = runtime.milliseconds() + 20000;

                        leg = 1;

                        // TODO: Move to alliance area...
                        // If we started in from of Alliance
                        //  -   adjust angle based on goldindex
                        //  -   drive into alliance depot
                        // If we started in front of crater
                        //  -   drive back
                        //  -   turn based on goldIndex
                        //  -   drive forward until close to wall
                        //  -   turn left (x degrees)
                        //  -   drive along wall into alliance depot
                    }
                    else if (runtime.milliseconds() > timeOut) {

                        robot.encodeDriveStop();

                        autoState = autoStates.DROP_MARKER;
                        newState = true;

                    }
                    else if (!robot.driveBusy()) {
                        if (xFactor==yFactor) {
                            // Started in front of crater
                            switch (leg) {
                                case 1: // Drive toward side of field
                                    robot.encodeDrive(DRIVE_SPEED, 50, 50, 50, 50);
                                    leg++;
                                    break;

                                case 2: // Turn left
                                    robot.encodeDrive(TURN_SPEED, -8, -8, 8, 8);
                                    leg++;
                                    break;

                                case 3: // Drive into alliance depot
                                    robot.encodeDrive(DRIVE_SPEED, 30, 30, 30, 30);
                                    leg++;
                                    break;

                                case 4: // We have reached alliance depot
                                    robot.encodeDriveStop();

                                    autoState = autoStates.DROP_MARKER;
                                    newState = true;

                                    break;
                            }
                        }
                        else {
                            //Started in front of alliance
                            switch (leg) {
                                case 1: // Drive to the left past the minerals
                                    robot.encodeDrive(DRIVE_SPEED, 20, 20, 20, 20);
                                    leg++;
                                    break;

                                case 2: // Turn right
                                    robot.encodeDrive(TURN_SPEED, 12, 12, -12, -12);
                                    leg++;
                                    break;

                                case 3: // Drive to alliance zone
                                    robot.encodeDrive(DRIVE_SPEED, 30, 30, 30, 30);
                                    leg++;
                                    break;

                                case 4: // Have arrived to crater
                                    robot.encodeDriveStop();

                                    autoState = autoStates.DROP_MARKER;
                                    newState = true;

                                    break;
                            }
                        }
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
                        timeOut = runtime.milliseconds() + 20000;

                        // TODO: Drop the team marker
                        // Consider that we have entered the alliance zone in different ways

                    } else if (runtime.milliseconds() > timeOut) {

                        // TODO: Stop all movement

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
                        timeOut = runtime.milliseconds() + 20000;

                        leg = 1;

                        // TODO: Drive to crater
                        //  -   turn based on where we came from - 180 if we started in front of crater
                        //      something less if we started in front of the alliance depot (130?)
                        //  -   drive forward based on encoder to the crater edge or until we detect the slope of the
                        //      crater edge

                    } else if (runtime.milliseconds() > timeOut) {

                        robot.encodeDriveStop();

                        autoState = autoStates.LOWER_ARM;
                        newState = true;

                    } else {
                        switch (leg) {
                            case 1: // Turn towards crater
                                if (xFactor==yFactor) {
                                    // We came from a start in front of the crater --> Turn around completely
                                    robot.encodeDrive(TURN_SPEED, 12, 12, -12, -12);
                                } else {
                                    // We came from a start in front of the alliance zone, turn just a little bit
                                    robot.encodeDrive(TURN_SPEED, 8, 8, -8, -8);
                                }
                                leg++;
                                break;

                            case 2: // Drive to crater
                                robot.encodeDrive(DRIVE_SPEED, 80,80,80,80);
                                leg++;
                                break;

                            case 3: // We have reached crater
                                robot.encodeDriveStop();

                                autoState = autoStates.LOWER_ARM;
                                newState = true;

                                break;
                        }
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
                        timeOut = runtime.milliseconds() + 20000;

                        // TODO: Lower the collections mechanism so that we are ready for manual mode and so we are sure to touch crater

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
                        timeOut = runtime.milliseconds() + 20000;
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
/**
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
**/
            telemetry.addData("Tensor Flow", "%d Obj: Silver %d, Gold %d (%s)", objectsDetected, numSilver, numGold,
                    goldIndex == 0 ? "LEFT" :
                    goldIndex == 1 ? "CENTER" :
                    goldIndex == 2 ? "RIGHT" : "Unknown");

            telemetry.addData("Power", "LF: %.1f, LB: %.1f, RF: %.1f, RB: %.1f", robot.LeftFrontPower(), robot.LeftBackPower(), robot.RightFrontPower(), robot.RightBackPower());
            telemetry.addData("Encoder", "Left: %d, Right: %d, Rack: %d", robot.ToLeftTarget(), robot.ToRightTarget(), robot.ToRackTarget());
            telemetry.update();
        }
    }
/**
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
 **/

}
