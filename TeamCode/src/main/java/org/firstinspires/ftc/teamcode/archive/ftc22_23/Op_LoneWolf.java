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

package org.firstinspires.ftc.teamcode.archive.ftc22_23;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//15055 is the best team

@TeleOp(name="Op_LoneWolf", group="Robot")
//@Disabled
public class Op_LoneWolf extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  leftbackDrive    = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  rightbackDrive   = null;
    public DcMotor  armleft = null;
    public DcMotor  armright = null;
    public DcMotor  wrist = null;
    private Servo claw = null;
    private Servo lock1 = null;





    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftbackDrive  = hardwareMap.get(DcMotor.class, "LBDrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "RBDrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        armleft = hardwareMap.get(DcMotor.class, "Larm");
        armright = hardwareMap.get(DcMotor.class, "Rarm");
        wrist = hardwareMap.get(DcMotor.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        lock1 = hardwareMap.get(Servo.class, "Lock1");




        leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);
        wrist.setDirection(DcMotor.Direction.FORWARD);









        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        lock1.setPosition(0);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double lift;
        double wrist1;
        double drive = -gamepad1.right_stick_y/2;
        double turn  =  gamepad1.right_stick_x/2;


        left  = Range.clip(drive + turn, -1.0, 1.0) ;
        right  = Range.clip(drive - turn, -1.0, 1.0) ;
        lift = -gamepad1.left_stick_y/1.5;
        wrist1 = -gamepad1.left_stick_x/1.5;


        leftbackDrive.setPower(left);
        leftfrontDrive.setPower(left);
        rightbackDrive.setPower(right);
        rightfrontDrive.setPower(right);
        armleft.setPower(lift);
        armright.setPower(lift);
        wrist.setPower(wrist1);

        //Boost mode
        if (gamepad1.right_bumper){
            leftbackDrive.setPower(left*2);
            leftfrontDrive.setPower(left*2);
            rightbackDrive.setPower(right*2);
            rightfrontDrive.setPower(right*2);
        }



        if (gamepad1.left_bumper) {
            claw.setPosition(.90);
        }
        else {
            claw.setPosition(0);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
