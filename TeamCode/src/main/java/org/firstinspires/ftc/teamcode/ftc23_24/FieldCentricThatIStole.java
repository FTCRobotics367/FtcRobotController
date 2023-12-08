package org.firstinspires.ftc.teamcode.ftc23_24;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

    @TeleOp
    public class FieldCentricThatIStole extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor FLDR = hardwareMap.dcMotor.get("FLDR");
            DcMotor BLDR = hardwareMap.dcMotor.get("BLDR");
            DcMotor FRDR = hardwareMap.dcMotor.get("FRDR");
            DcMotor JFK = hardwareMap.dcMotor.get("JFK");

            DcMotor rightSide = hardwareMap.dcMotor.get("rightSide");
            DcMotor leftSide = hardwareMap.dcMotor.get("leftSide");
            DcMotor lift = hardwareMap.dcMotor.get("lift");
            Servo hippo = hardwareMap.servo.get("hippo");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            //FLDR.setDirection(DcMotorSimple.Direction.REVERSE);
            BLDR.setDirection(DcMotorSimple.Direction.REVERSE);

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x;
                double rx = gamepad2.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad2.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                FLDR.setPower(frontLeftPower);
                BLDR.setPower(backLeftPower);
                FRDR.setPower(frontRightPower);
                JFK.setPower(backRightPower);

                //zoomin vroomin button
                if (gamepad2.left_stick_button) {
                    gamepad2.rumble(2000);
                    BLDR.setPower(backLeftPower * 2);
                    FLDR.setPower(frontLeftPower * 2);
                    JFK.setPower(backRightPower * 2);
                    FRDR.setPower(frontRightPower * 2);
                }
                //HOPEFULY SOME LIFT CODE!
                double cut = gamepad2.right_trigger;
                double closecut = -gamepad2.left_trigger;
                double number = 0;

                if(gamepad2.right_trigger >.01){
                    lift.setPower(cut);
                }else{
                    lift.setPower(0);
                }

                if(gamepad2.left_trigger >.01){
                    lift.setPower(closecut);
                }else{
                    lift.setPower(0);
                }

                //wrist thing probobly
                if(gamepad2.dpad_up){
                    leftSide.setPower(.75);
                    rightSide.setPower(-.75);
                }else{
                    leftSide.setPower(0);
                    rightSide.setPower(0);
                }
                if (gamepad2.dpad_down){
                    leftSide.setPower(-.75);
                    rightSide.setPower(.75);
                }else {
                    leftSide.setPower(0);
                    rightSide.setPower(0);
                }

                // claw for now
                if(gamepad2.right_bumper){
                    hippo.setPosition(.5);
                }

                if(gamepad2.left_bumper){
                    hippo.setPosition(0);
                }

            }
        }
    }
