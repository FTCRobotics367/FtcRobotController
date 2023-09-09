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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Auto_Poll_Left", group="Robot")
//@Disabled
public class Auto_Poll_Left extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftfrontDrive = null;
    public DcMotor  armleft = null;
    public DcMotor  armright = null;
    public DcMotor  wrist = null;
    private Servo claw = null;
    private Servo lock1 = null;
    int num = 0;
    int startPosition;
    int currentPosition;
    int distanceTravelled;


    @Override
    public void runOpMode() {
        leftbackDrive = hardwareMap.get(DcMotor.class, "LBDrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "RBDrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        armleft = hardwareMap.get(DcMotor.class, "Larm");
        armright = hardwareMap.get(DcMotor.class, "Rarm");
        wrist = hardwareMap.get(DcMotor.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        lock1 = hardwareMap.get(Servo.class, "Lock1");

        leftbackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightbackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        armleft.setDirection(DcMotor.Direction.REVERSE);
        armright.setDirection(DcMotor.Direction.REVERSE);
        wrist.setDirection(DcMotor.Direction.FORWARD);

        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        lock1.setPosition(0);

        startPosition = rightbackDrive.getCurrentPosition();
        telemetry.addData("Before step 0 - Num=", num);
        telemetry.update();

        if (num==0) {
            //Drive forward
           telemetry.addData("Step 1 - Start Position", startPosition);
           telemetry.addData("Num=", num);
           telemetry.addData("Nothing",0);
           telemetry.update();

            claw.setPosition(0);

            rightfrontDrive.setTargetPosition(2700);
            rightbackDrive.setTargetPosition(2700);
            leftfrontDrive.setTargetPosition(2700);
            leftbackDrive.setTargetPosition(2700);

            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftbackDrive.setPower(0.37);
            rightbackDrive.setPower(0.35);
            leftfrontDrive.setPower(0.37);
            rightfrontDrive.setPower(0.35);

            while (opModeIsActive() && rightbackDrive.isBusy()) {
                   telemetry.addData("Forward",rightbackDrive.getCurrentPosition());
                   telemetry.update();
               }
            currentPosition = rightbackDrive.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;

           num++;
       }
        startPosition = rightbackDrive.getCurrentPosition();
        if (num==1) {
            claw.setPosition(0);

            leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftfrontDrive.setTargetPosition(-1600);
            leftbackDrive.setTargetPosition(-1600);
            rightfrontDrive.setTargetPosition(-1600);
            rightbackDrive.setTargetPosition(-1600);

            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfrontDrive.setPower(.35);
            leftbackDrive.setPower(.35);
            rightfrontDrive.setPower(.35);
            rightbackDrive.setPower(.35);

            while (opModeIsActive() && rightbackDrive.isBusy()) {
                telemetry.addData("Beep Beep Beep",rightbackDrive.getCurrentPosition());
                telemetry.update();
            }
            currentPosition = rightbackDrive.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;

            num++;
        }
        if (num==2) {
            //Lift up
            telemetry.addData("Step 2 - Start Position", startPosition);
            telemetry.addData("Num=", num);
            telemetry.addData("Nothing",0);
            telemetry.update();

            claw.setPosition(0);

            armright.setTargetPosition(700);
            armleft.setTargetPosition(700);

            armright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armright.setPower(.45);
            armleft.setPower(.45);

            while (opModeIsActive() && armleft.isBusy()) {
                telemetry.addData("Arm",armleft.getCurrentPosition());
                telemetry.update();
            }
            currentPosition = armright.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;

            num++;
        }
        if (num==3){
            //Turn
            rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("Step 4 - Start Position", startPosition);
            telemetry.addData("Num=", num);
            telemetry.update();
            currentPosition = rightbackDrive.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;

            claw.setPosition(0);

            rightfrontDrive.setTargetPosition(0);
            rightbackDrive.setTargetPosition(0);
            leftfrontDrive.setTargetPosition(1800);
            leftbackDrive.setTargetPosition(1800);

            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightfrontDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0.50);
            leftbackDrive.setPower(0.50);

            while (opModeIsActive() && leftbackDrive.isBusy()) {
                telemetry.addData("Nothing",0);
                telemetry.update();
            }
            currentPosition = rightbackDrive.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;
            num++;
        }
        startPosition = wrist.getCurrentPosition();
       if (num==4){
            //Drive back
            telemetry.addData("Step 1 - Start Position", startPosition);
            telemetry.addData("Num=", num);
            telemetry.addData("Nothing",0);
            telemetry.update();

            claw.setPosition(0);

           leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightfrontDrive.setTargetPosition(-750);
            rightbackDrive.setTargetPosition(-750);
            leftfrontDrive.setTargetPosition(-650);
            leftbackDrive.setTargetPosition(-650);

            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftbackDrive.setPower(0.35);
            rightbackDrive.setPower(0.35);
            leftfrontDrive.setPower(0.35);
            rightfrontDrive.setPower(0.35);

            while (opModeIsActive() && rightbackDrive.isBusy()) {
                telemetry.addData("Forward",rightbackDrive.getCurrentPosition());
                telemetry.update();
            }
            currentPosition = rightbackDrive.getCurrentPosition();
            distanceTravelled = currentPosition - startPosition;

            num++;
        }
       if (num==5){
           //Wrist out
           telemetry.addData("Step 3 - Start Position", startPosition);
           telemetry.addData("Num=", num);
           telemetry.update();
           currentPosition = wrist.getCurrentPosition();
           distanceTravelled = currentPosition - startPosition;
           sleep(3000);
           claw.setPosition(0);

           wrist.setTargetPosition(-350);

           wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           wrist.setPower(.35);

           while (opModeIsActive() && wrist.isBusy()) {
               telemetry.addData("wrist out",wrist.getCurrentPosition());
               telemetry.update();
           }
           currentPosition = wrist.getCurrentPosition();
           distanceTravelled = currentPosition - startPosition;
           num++;
       }
       startPosition = rightbackDrive.getCurrentPosition();

      if (num==6){
           //Open claw
           telemetry.addData("Step 5", 5);
           telemetry.addData("Num=", num);
           telemetry.update();

          sleep(3000);


           claw.setPosition(.95);

           sleep(200);

           wrist.setTargetPosition(100);
           wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           wrist.setPower(.45);

           armright.setTargetPosition(-200);
           armleft.setTargetPosition(-200);

           armright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           armleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           armright.setPower(0.35);
           armleft.setPower(0.35);

           sleep(200);

           rightfrontDrive.setTargetPosition(50);
           rightbackDrive.setTargetPosition(50);
           rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightfrontDrive.setPower(.20);
           rightbackDrive.setPower(.20);

           while (opModeIsActive() && armright.isBusy()) {
               telemetry.addData("Nothing",0);
               telemetry.update();
           }
           currentPosition = rightbackDrive.getCurrentPosition();
           distanceTravelled = currentPosition - startPosition;
           num++;
       }

        sleep(10000);
    }
}



