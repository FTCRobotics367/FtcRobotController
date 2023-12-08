package org.firstinspires.ftc.teamcode.ftc23_24.automases;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BLUEFAR", group = "Robot")
//@Disabled
public class BLUEFAR extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {

        setup();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Started");

        sleep(1000000);
        driveForward(.75,-900);
        sleep(100);
        strafeLeft(.75,-5000);

    }

    private void jk(double speed){
        telemetry.addData("Status", "Auto");
        frontRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
    }

    private void driveForward(double speed, int position) {
        telemetry.addData("StrafeToPosition", position);
        telemetry.addData("speed", speed);

        nah1(position);

        jk(speed);
        while (opModeIsActive() &&
                (frontLeftMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  position);
            telemetry.addData("Currently at",
                    frontLeftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    private void strafeLeft(double speed, int position) {
        telemetry.addData("StrafeToPosition", position);
        telemetry.addData("speed", speed);

        nah2(position);

        jk(speed);
        while (opModeIsActive() &&
                (frontLeftMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  position);
            telemetry.addData("Currently at",
                    frontLeftMotor.getCurrentPosition());
            telemetry.update();
        }
    }


    private void setup() {
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("FLDR");
        backLeftMotor = hardwareMap.dcMotor.get("BLDR");
        frontRightMotor = hardwareMap.dcMotor.get("FRDR");
        backRightMotor = hardwareMap.dcMotor.get("JFK");


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    private void nah1(int position){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(position);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(position);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setTargetPosition(position);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setTargetPosition(position);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void nah2(int position){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(-position);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(position);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setTargetPosition(position);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setTargetPosition(-position);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}