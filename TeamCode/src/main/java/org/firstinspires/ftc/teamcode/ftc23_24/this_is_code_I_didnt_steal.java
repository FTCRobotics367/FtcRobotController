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

package org.firstinspires.ftc.teamcode.ftc23_24;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="SEXY_CODE:BY_BEN", group="Robot")
//@Disabled
public class this_is_code_I_didnt_steal extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  LFDR   = null;
    //left front drive
    public DcMotor  LBDR    = null;
    //LEFT BACK DRIVE
    public DcMotor  RFDR  = null;
    //rightfrontDrive
    public DcMotor  JFK   = null;
    //rightbackDrive
    public DcMotor  spin = null;
    public DcMotor hold = null;
    //public DcMotor  armright = null;
    //public DcMotor  wrist = null;
    //private Servo claw = null;
    //private Servo lock1 = null;





    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        LBDR  = hardwareMap.get(DcMotor.class, "LBDrive");
        LFDR = hardwareMap.get(DcMotor.class, "LFDrive");
        JFK = hardwareMap.get(DcMotor.class, "RBDrive");
        RFDR = hardwareMap.get(DcMotor.class, "RFDrive");
        spin = hardwareMap.get(DcMotor.class, "spin");
        hold = hardwareMap.get(DcMotor.class, "hold");
        //armright = hardwareMap.get(DcMotor.class, "Rarm");
        //wrist = hardwareMap.get(DcMotor.class, "Wrist");
        //claw = hardwareMap.get(Servo.class, "Claw");
        //lock1 = hardwareMap.get(Servo.class, "Lock1");




        LBDR.setDirection(DcMotor.Direction.FORWARD);
        LFDR.setDirection(DcMotor.Direction.REVERSE);
        JFK.setDirection(DcMotor.Direction.REVERSE);
        RFDR.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotor.Direction.FORWARD);
        hold.setDirection(DcMotor.Direction.FORWARD);
        //armright.setDirection(DcMotor.Direction.REVERSE);
        //wrist.setDirection(DcMotor.Direction.FORWARD);









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
    //@Override



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double turn;
        double drive;
        double strafe;




        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        turn = -gamepad1.left_stick_x/2;
        drive = -gamepad1.right_stick_y/2;
        strafe = -gamepad1.right_stick_x/2;




        //driving
        LBDR.setPower(drive);
        LFDR.setPower(drive);
        JFK.setPower(drive);
        RFDR.setPower(drive);

        //turning
        LBDR.setPower(turn);
        LFDR.setPower(turn);
        JFK.setPower(-turn);
        RFDR.setPower(-turn);

        //probably strafing i have no clue man i need help
        if ((strafe > .5 )||(strafe < -.5)) {
            LBDR.setPower(strafe);
            LFDR.setPower(-strafe);
            JFK.setPower(-strafe);
            RFDR.setPower(strafe);
        }

        if (gamepad1.x){
            spin.setPower(100);
        }
        while (gamepad1.circle) {
            hold.setPower(100);
        }

        //zoomin vroomin button
        if (gamepad1.right_bumper){
            LBDR.setPower(drive*2);
            LFDR.setPower(drive*2);
            JFK.setPower(drive*2);
            RFDR.setPower(drive*2);
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
