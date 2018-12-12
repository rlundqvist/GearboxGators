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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "left_front_drive"
 * Motor channel:  Right front drive motor:        "right_front_drive"
 */
public class HardwareRobot
{
    // Public OpMode members
    private static DcMotor leftFrontDrive = null;
    private static DcMotor leftBackDrive = null;
    private static DcMotor rightFrontDrive = null;
    private static DcMotor rightBackDrive = null;
    //Uprivate static DcMotor rackDrive = null;

    // Local OpMode members.
    HardwareMap hwMap  =  null;

    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hwMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class,"left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class,"right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class,"right_back_drive");
        //rackDrive = hwMap.get(DcMotor.class,"rack_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //rackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        //rackDrive.setPower(0);

        // Set all motors to run with encoders.
        // Need to use RUN_WITHOUT_ENCODERS if encoders are NOT installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void DriveStrafe(double drive, double strafe, double turn) {
        // Strafe Mode is similar to POV mode in that the movement is controlled by left stick and right stick turns.
        // Strafe Mode also supports moving sideways based on X movements of the left stick.
        double lf = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double lb = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double rf = Range.clip(drive - turn - strafe, -1.0, 1.0);
        double rb = Range.clip(drive - turn + strafe, -1.0, 1.0);

        Drive(lf, lb, rf, rb);
    }

    public static void DrivePOV (double drive, double turn) {
        // POV Mode uses left stick to go forward, and right stick to turn.
        // This mode is easier to drive straight.
        double lf = Range.clip(drive + turn, -1.0, 1.0);
        double lb = Range.clip(drive + turn, -1.0, 1.0);
        double rf = Range.clip(drive - turn, -1.0, 1.0);
        double rb = Range.clip(drive - turn, -1.0, 1.0);

        Drive(lf, lb, rf, rb);
    }

    public static void DriveTank (double left, double right) {
        // Tank Mode uses one stick to control each wheel.
        // This mode is hard to drive forward slowly and keep straight.
        Drive(left, left, right, right);
    }

    public static void DriveSTOP() {
        Drive(0,0,0,0);
    }

    private static void Drive (double lfPwr, double lbPwr, double rfPwr, double rbPwr) {
        // Send requested power to each wheel/motor.
        leftFrontDrive.setPower(lfPwr);
        leftBackDrive.setPower(lbPwr);
        rightFrontDrive.setPower(rfPwr);
        rightBackDrive.setPower(rbPwr);
    }

}

