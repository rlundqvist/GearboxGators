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
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:      "left_front_drive"
 * Motor channel:  Right front drive motor:     "right_front_drive"
 * Motor channel:  Left back drive motor        "left_back_drive"
 * Motor channel:  Right back drive motor       "right_back_drive"
 * Motor channel:  Rack drive moror:            "rack_drive"
 */
public class HardwareRobot
{
    // Public OpMode members
    private static DcMotor leftFrontDrive = null;
    private static DcMotor leftBackDrive = null;
    private static DcMotor rightFrontDrive = null;
    private static DcMotor rightBackDrive = null;
    private static DcMotor rackDrive = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // Example: One inch in forward motion = (1440 * 2) / (4 * 3.1415) = 229.2 encoder counts

    private static final double     RACK_GEAR_REDUCTION     = 0.33 ;
    private static final double     PINION_DIAMETER_INCHES  = 0.75 ;
    private static final double     RACK_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * RACK_GEAR_REDUCTION) / (PINION_DIAMETER_INCHES * 3.1415);
    // Example: One inch of movement of rack = (1440 * 0.33) / (0.75 * 3.1415) = 201.7 encoder counts

    private static double lfPower = 0.0;
    private static double lbPower = 0.0;
    private static double rfPower = 0.0;
    private static double rbPower = 0.0;


    // Local OpMode members.
    HardwareMap hwMap  =  null;

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
        rackDrive = hwMap.get(DcMotor.class,"rack_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rackDrive.setPower(0);

        // Set all motors to run with encoders.
        // Need to use RUN_WITHOUT_ENCODERS if encoders are NOT installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void DriveStrafe(double drive, double strafe, double turn) {
        // Strafe Mode is similar to POV mode in that the movement is controlled by left stick and right stick turns.
        // Strafe Mode also supports moving sideways based on X movements of the left stick.
        double lf = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double lb = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double rf = Range.clip(drive - turn - strafe, -1.0, 1.0);
        double rb = Range.clip(drive - turn + strafe, -1.0, 1.0);

        Drive(lf/3.0, lb, rf/3.0, rb);
    }

    public void DrivePOV (double drive, double turn) {
        // POV Mode uses left stick to go forward, and right stick to turn.
        // This mode is easier to drive straight.
        double lf = Range.clip(drive + turn, -1.0, 1.0);
        double lb = Range.clip(drive + turn, -1.0, 1.0);
        double rf = Range.clip(drive - turn, -1.0, 1.0);
        double rb = Range.clip(drive - turn, -1.0, 1.0);

        Drive(lf, lb, rf, rb);
    }

    public void DriveTank (double left, double right) {
        // Tank Mode uses one stick to control each wheel.
        // This mode is hard to drive forward slowly and keep straight.
        Drive(left, left, right, right);
    }

    public void DriveStop() {
        Drive(0,0,0,0);
    }

    private void Drive (double lfPwr, double lbPwr, double rfPwr, double rbPwr) {

        // If magnitude of any of the requested power values exceed 1 then divide all values with magnitude to normalize
        double lfAbs = Math.abs(lfPwr);
        double lbAbs = Math.abs(lbPwr);
        double rfAbs = Math.abs(rfPwr);
        double rbAbs = Math.abs(rbPwr);
        double magnitude = 1.0;
        if (lfAbs > magnitude) magnitude = lfAbs;
        if (lbAbs > magnitude) magnitude = lbAbs;
        if (rfAbs > magnitude) magnitude = rfAbs;
        if (rbAbs > magnitude) magnitude = rbAbs;

        lfPower = lfPwr/magnitude;
        lbPower = lbPwr/magnitude;
        rfPower = rfPwr/magnitude;
        rbPower = rbPwr/magnitude;

        // Send requested power to each wheel/motor.
        leftFrontDrive.setPower(lfPower);
        leftBackDrive.setPower(lbPower);
        rightFrontDrive.setPower(rfPower);
        rightBackDrive.setPower(rbPower);
    }

    public void RackDrive(double rPwr) {
        rackDrive.setPower(rPwr);
    }

    public void encodeDrive(double speed, double lfInches, double lbInches, double rfInches, double rbInches){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        int newlfTarget = leftFrontDrive.getCurrentPosition() + (int)(lfInches * COUNTS_PER_INCH);
        int newrfTarget = rightFrontDrive.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
        // TODO: Adjust when we have encoder cables and converters for back motors
        //double newlbTarget = leftBackDrive.getCurrentPosition() + (int)(lbInches * COUNTS_PER_INCH);
        //double newrbTarget = rightBackDrive.getCurrentPosition() + (int)(rbInches * COUNTS_PER_INCH);

        leftFrontDrive.setTargetPosition(newlfTarget);
        rightFrontDrive.setTargetPosition(newrfTarget);
        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setTargetPosition(newlbTarget);
        //rightBackDrive.setTargetPosition(newrbTarget);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));

        // TODO: Adjust when we have encoder cables and converters for back motors
        leftBackDrive.setPower(lbInches > 0.0 ? speed : -speed);
        rightBackDrive.setPower(rbInches> 0.0 ? speed : -speed);

    }

    public boolean driveBusy(){
        // TODO: Adjust when we have encoder cables and converters for back motors
        //return (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy());
        return (leftFrontDrive.isBusy() || rightFrontDrive.isBusy());
    }

    public void encodeDriveStop() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Adjust when we have encoder cables and converters for back motors
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encodeRack(double speed, double inches) {
        rackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        int newRackTarget = rackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        rackDrive.setTargetPosition(newRackTarget);

        // Turn On RUN_TO_POSITION
        rackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rackDrive.setPower(Math.abs(speed));
    }

    public void encodeRackStop (){
        rackDrive.setPower(0);

        rackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean rackBusy(){
        return rackDrive.isBusy();
    }

    public double LeftFrontPower() {
        return lfPower;
    }

    public double LeftBackPower() {
        return lbPower;
    }

    public double RightFrontPower() {
        return rfPower;
    }

    public double RightBackPower() {
        return rbPower;
    }

    public int ToLeftTarget(){
        return (leftFrontDrive.getTargetPosition() - leftFrontDrive.getCurrentPosition());
    }

    public int ToRightTarget(){
        return (rightFrontDrive.getTargetPosition() - rightFrontDrive.getCurrentPosition());
    }

    public int ToRackTarget() {
        return (rackDrive.getTargetPosition()-rackDrive.getCurrentPosition());
    }
}

