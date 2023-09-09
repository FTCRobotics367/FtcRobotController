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
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="teleop_example", group="Robot")
//@Disabled
public class TeleopExample extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  armleft = null;
    public DcMotor  armright = null;
    public DcMotor  wrist = null;
    private Servo claw = null;

    double startclaw;
    double stopclaw;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "LDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "RDrive");
        armleft = hardwareMap.get(DcMotor.class, "Larm");
        armright = hardwareMap.get(DcMotor.class, "Rarm");
        wrist = hardwareMap.get(DcMotor.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        startclaw = -.50;
        stopclaw = 0;
        claw.setPosition(startclaw);


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armleft.setDirection(DcMotorSimple.Direction.FORWARD);
        armright.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(DcMotorSimple.Direction.FORWARD);






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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double aleft;
        double aright;
        double wrist1;



        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        aleft = -gamepad2.left_stick_y;
        aright = -gamepad2.left_stick_y;
        wrist1 = -gamepad2.right_stick_y;



        leftDrive.setPower(left);
        rightDrive.setPower(right);
        armleft.setPower(aleft);
        armright.setPower(aright);
        wrist.setPower(wrist1);


        if (gamepad2.right_bumper) {
            claw.setPosition(stopclaw);
        }
        else {
            claw.setPosition(startclaw);
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
