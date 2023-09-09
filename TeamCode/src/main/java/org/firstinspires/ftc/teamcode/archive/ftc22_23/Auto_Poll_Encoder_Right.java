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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Set;


@Autonomous(name="Auto_Poll_Encoder_Right", group="Robot")
@Disabled
public class Auto_Poll_Encoder_Right extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftfrontDrive = null;
    int num = 0;
    double startPosition;
    double currentPosition;
    double distanceTravelled;


    @Override
    public void runOpMode() {
        leftbackDrive = hardwareMap.get(DcMotor.class, "LBDrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "RBDrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "RFDrive");


        leftbackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightbackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //double startPosition;
        //double currentPosition;
        //double distanceTravelled;
        //int num = 0;
        /*startPosition = distanceTravelled;
        currentPosition = rightbackDrive.getCurrentPosition();
        distanceTravelled = currentPosition - startPosition;*/
        switch (num) {
            case 0:
                telemetry.addData("Case 0", startPosition);
                telemetry.addData("Num=", num);
                telemetry.update();
                startPosition = rightbackDrive.getCurrentPosition();
                num=1;
                telemetry.addData("num=", num);
                telemetry.update();

                break;

            case 1:
                telemetry.addData("Case 1", startPosition);
                telemetry.update();
                currentPosition = rightbackDrive.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 0) {

                    rightbackDrive.setTargetPosition(1000);

                    rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftbackDrive.setPower(0.35);
                    rightbackDrive.setPower(0.35);
                    leftfrontDrive.setPower(0.35);
                    rightfrontDrive.setPower(0.35);
                    num=2;
                }
                break;

            case 2:
                telemetry.addData("case 2", startPosition);
                telemetry.update();
                currentPosition = rightbackDrive.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 1000){
                    rightbackDrive.setTargetPosition(500);

                    rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftbackDrive.setPower(0);
                    rightbackDrive.setPower(0.35);
                    leftfrontDrive.setPower(0);
                    rightfrontDrive.setPower(0.35);
                    num++;
                }

            default:
                telemetry.addData("IDK", 1943);
                telemetry.update();
        }
        sleep(1000);

    }
}



