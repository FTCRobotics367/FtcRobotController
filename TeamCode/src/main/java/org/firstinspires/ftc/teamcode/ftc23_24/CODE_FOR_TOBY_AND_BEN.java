
package org.firstinspires.ftc.teamcode.ftc23_24;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="SEXYER_CODE:BY_BEN 2 drivers", group="Robot")
//@Disabled
public class CODE_FOR_TOBY_AND_BEN extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  LFDR   = null;
    //left front drive
    public DcMotor  LBDR    = null;
    //LEFT BACK DRIVE
    public DcMotor  RFDR  = null;
    //rightfrontDrive
    public DcMotor  JFK   = null;
    //rightbackDrive
    public DcMotor lift  = null;
    //scissor lift

    public DcMotor rightSide = null;
    public DcMotor leftSide = null;

    private Servo hippo  = null;
    //claw


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        LBDR  = hardwareMap.get(DcMotor.class, "BLDR");
        LFDR = hardwareMap.get(DcMotor.class, "FLDR");
        JFK = hardwareMap.get(DcMotor.class, "JFK");
        RFDR = hardwareMap.get(DcMotor.class, "FRDR");

        lift = hardwareMap.get(DcMotor.class, "lift");
        hippo = hardwareMap.get(Servo.class, "hippo");
        rightSide = hardwareMap.get(DcMotor.class, "rightSide");
        leftSide = hardwareMap.get(DcMotor.class, "leftSide");

        
        LBDR.setDirection(DcMotor.Direction.FORWARD);
        LFDR.setDirection(DcMotor.Direction.REVERSE);
        JFK.setDirection(DcMotor.Direction.REVERSE);
        RFDR.setDirection(DcMotor.Direction.REVERSE);
        
        
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
        turn = gamepad1.left_stick_x/2;
        drive = -gamepad1.right_stick_y/2;
        strafe = -gamepad1.right_stick_x;




        //driving
        LBDR.setPower(drive*1.5);
        LFDR.setPower(drive*1.5);
        JFK.setPower(drive*1.5);
        RFDR.setPower(drive*1.5);

        //turning
        LBDR.setPower(turn*1.5);
        LFDR.setPower(turn*1.5);
        JFK.setPower(-turn*1.5);
        RFDR.setPower(-turn*1.5);

        //probably strafing i have no clue man i need help
        if ((strafe > .5 )||(strafe < -.5)) {
            LBDR.setPower(strafe);
            LFDR.setPower(-strafe);
            JFK.setPower(-strafe);
            RFDR.setPower(strafe);
        }



        //zoomin vroomin button
        if (gamepad1.right_bumper){
            LBDR.setPower(drive*2);
            LFDR.setPower(drive*2);
            JFK.setPower(drive*2);
            RFDR.setPower(drive*2);
        }
        //hopfully a lift code
        double cut = gamepad2.right_trigger;
        double closecut = -gamepad2.left_trigger;
        double number = 0;

            lift.setPower(cut);
            lift.setPower(closecut);

            // claw for now
        if(gamepad2.right_bumper){
            hippo.setPosition(1);
        }

        if(gamepad2.left_bumper){
            hippo.setPosition(0);
        }

            // claw code feat. sammi! (we love her!) (she is best) (im dyslexic tho so its hard to read her writing somitomes but thats okay)
        /*
        boolean temp =false;
        if(gamepad2.left_bumper) {
            hippo.setPosition(1);
            temp = true;
        }
            else if(gamepad2.left_bumper && temp){
                hippo.setPosition(1);
                temp = false;
            }
            else if(gamepad2.left_bumper && !temp){
                hippo.setPosition(0);
                temp = true;
        }

            if(gamepad2.dpad_up){
                rightSide.setPower(.25);
                leftSide.setPower(.25);
            }

        if(gamepad2.dpad_down){
            rightSide.setPower(-.25);
            leftSide.setPower(-.25);
        }
*/
        }




        //claw thing maybe? feat. jack and feat.jamies brain
/*
        if (gamepad2.left_bumper) {
            telemetry.addData("Status", "YESSSSS");
            if (number == 0) {
                number = 1;
                telemetry.addData("Status", number);
            } else {
                number = 0;
                telemetry.addData("Status", number);
            }
            telemetry.addData("Status", number);
        }
        hippo.setPosition(number);
        */



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
