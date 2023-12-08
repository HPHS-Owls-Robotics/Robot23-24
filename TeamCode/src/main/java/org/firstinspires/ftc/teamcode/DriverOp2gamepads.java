package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Driving2gamepads", group="12417")

public class DriverOp2gamepads extends LinearOpMode {

//declaration
    DcMotor LMotor;
    DcMotor RMotor;
    DcMotor Arm;
    DcMotor Sweeper;

//    CRServo sweepRight;
//    CRServo sweepLeft;

    Servo trapdoor;
    Servo drone;
    Servo rr;

    DigitalChannel breakBeam;


    @Override

    public void runOpMode() throws InterruptedException {

//initialization
        LMotor = hardwareMap.dcMotor.get("L_Motor");
        RMotor = hardwareMap.dcMotor.get("R_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        Sweeper = hardwareMap.dcMotor.get("Sweeper");

//        sweepRight = hardwareMap.crservo.get("Sweep_1");
//        sweepLeft = hardwareMap.crservo.get("Sweep_2");

        trapdoor = hardwareMap.servo.get("Trapdoor");
        drone = hardwareMap.servo.get("Drone");
        rr = hardwareMap.servo.get("RR");

        breakBeam = hardwareMap.digitalChannel.get("BreakBeam");

//speeds
        float MaxSpeed, SPwr=5f, LLPwr, LRPwr, RLPwr, RRPwr, APwr;
        trapdoor.setPosition(0.0); //check
        MaxSpeed = 0.7f;
        APwr = 0.3f;
//booleans
        boolean isBeamBroke;

        LMotor.setDirection(DcMotor.Direction.FORWARD);
        RMotor.setDirection(DcMotor.Direction.REVERSE);

//let's gooooooooooooooooooooooooooooooooooooooooo
        waitForStart();

        while (opModeIsActive()) {

            DcMotor[] motors = {LMotor, RMotor, Arm};
            for (int i = 0; i < 3; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
//            sweepRight.setDirection(DcMotor.Direction.FORWARD);
//            sweepLeft.setDirection(DcMotor.Direction.FORWARD);


            LLPwr = gamepad1.left_stick_y*MaxSpeed - gamepad1.left_stick_x*MaxSpeed;
            LRPwr = gamepad1.left_stick_y*MaxSpeed + gamepad1.left_stick_x*MaxSpeed;

            RLPwr = gamepad1.right_stick_y*MaxSpeed - gamepad1.right_stick_x*MaxSpeed;
            RRPwr = gamepad1.right_stick_y*MaxSpeed + gamepad1.right_stick_x*MaxSpeed;

            isBeamBroke = breakBeam.getState();

//gamepad 1 -- ooooooooooonnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee --
            if(gamepad1.x){
                //
            }
            if(gamepad1.y){
                //
            }
            if(gamepad1.b){
                //sweep mode 1
                if(!breakBeam.getState())
                {
                    rr.setPosition(2);
                    telemetry.addLine("Beam intact");
                } else
                {
                    rr.setPosition(0);
                    telemetry.addLine("Beam broke");
                }
            }
            if(gamepad1.a){
                //sweep mode 2
                rr.setPosition(0.52);
            }

            if(gamepad1.dpad_up){
                LMotor.setDirection(DcMotor.Direction.FORWARD);
                RMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            if(gamepad1.dpad_down){
                LMotor.setDirection(DcMotor.Direction.REVERSE);
                RMotor.setDirection(DcMotor.Direction.FORWARD);
            }

            if(gamepad1.left_bumper){
                //sweep out
                Sweeper.setPower(-SPwr);
            }
            if(gamepad1.right_bumper)
            {
                //sweep in
                Sweeper.setPower(SPwr);
            }
            if(gamepad1.left_bumper==gamepad1.right_bumper){
                Sweeper.setPower(0);
            }

//gamepad 2 --- tttttttwwwwwwwwwwwwwwoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo -------
            if(gamepad2.x)
            {
                //drone
                drone.setPosition(2);
            }
            if(gamepad2.y)
            {
                //
            }
            if(gamepad2.a)
            {
                //
            }
            if(gamepad2.b){
                if(trapdoor.getPosition()==0.0){
                    trapdoor.setPosition(1.0);
                    sleep(700);
                    trapdoor.setPosition(0.0);
                } else{
                    trapdoor.setPosition(0.0);
                }
                sleep(200);
            }

            if(gamepad2.dpad_up)
            {
                //suspension up
            }
            if(gamepad2.dpad_down)
            {
                //suspension down
            }

            if(gamepad2.left_bumper)
            {
                //Arm down
                Arm.setPower(-APwr);
            }
            if(gamepad2.right_bumper)
            {
                //Arm up
                Arm.setPower(APwr);
            }
            if(gamepad2.left_bumper==gamepad2.right_bumper){
                Arm.setPower(0);
            }


//            sweepRight.setPower(SPwr);
//            sweepLeft.setPower(-SPwr);
            LMotor.setPower(LLPwr + RLPwr);
            RMotor.setPower(LRPwr + RRPwr);

//            telemetry.addData("hello", sweepLeft.getPower());
//            telemetry.addData("hello", sweepRight.getPower());

            telemetry.addData("break beam state:", breakBeam.getState());
            telemetry.addData("break beam mode:", breakBeam.getMode());
            telemetry.update();





        }

    }

}
