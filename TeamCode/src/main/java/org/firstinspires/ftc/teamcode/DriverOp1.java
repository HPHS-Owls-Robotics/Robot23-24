package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Driving22", group="12417")

public class DriverOp1 extends LinearOpMode {

    DcMotor LMotor;
    DcMotor RMotor;

    DcMotor arm;

    CRServo sweepRight;
    CRServo sweepLeft;


    Servo trapdoor;
    //DigitalChannel breakBeam;
    float MaxSpeed = 0.7f;
    float ArmSpeed = 0.3f;


    @Override

    public void runOpMode() throws InterruptedException {
        LMotor = hardwareMap.dcMotor.get("L_Motor");
        RMotor = hardwareMap.dcMotor.get("R_Motor");
//        arm = hardwareMap.dcMotor.get("Arm_Motor");
//        sweepRight = hardwareMap.crservo.get("Sweep_1");
//        sweepLeft = hardwareMap.crservo.get("Sweep_2");
//        trapdoor = hardwareMap.servo.get("Trapdoor");
        float LPwr = 0, RPwr = 0, SPwr=0, LLPwr, LRPwr, RLPwr, RRPwr;
        //trapdoor.setPosition(0.0); //check
        waitForStart();
        while (opModeIsActive()) {

            //breakBeam = hardwareMap.digitalChannel.get("Beam");

            DcMotor[] motors = {LMotor, RMotor, arm};
            for (int i = 0; i < 2; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            LMotor.setDirection(DcMotor.Direction.FORWARD);
            RMotor.setDirection(DcMotor.Direction.REVERSE);
//            sweepRight.setDirection(DcMotor.Direction.FORWARD);
//            sweepLeft.setDirection(DcMotor.Direction.FORWARD);


            LLPwr = gamepad1.left_stick_y*MaxSpeed - gamepad1.left_stick_x*MaxSpeed;
            LRPwr = gamepad1.left_stick_y*MaxSpeed + gamepad1.left_stick_x*MaxSpeed;

            RLPwr =( gamepad1.right_stick_y*MaxSpeed - gamepad1.right_stick_x*MaxSpeed);
            RRPwr =( gamepad1.right_stick_y*MaxSpeed + gamepad1.right_stick_x*MaxSpeed);

//            if(gamepad1.b)
//            {
//
//                SPwr =3f;
//
//            }
//            else if(gamepad1.x)
//            {
//
//                SPwr =-3f;
//
//            }
//            else
//                SPwr=0f;
//
//            if(gamepad1.y){
//                if(trapdoor.getPosition()==0.0){
//                    trapdoor.setPosition(1.0);
//                    sleep(700);
//                    trapdoor.setPosition(0.0);
//                } else{
//                    trapdoor.setPosition(0.0);
//                }
//                sleep(200);
//            }
//
//            while(gamepad1.left_bumper)//down
//            {
//                arm.setPower(ArmSpeed);
//            }
//            while(gamepad1.right_bumper)//up
//            {
//                arm.setPower(-ArmSpeed);
//            }
//            if(!(gamepad1.left_bumper&&gamepad1.right_bumper))
//                arm.setPower(0);
//
//            sweepRight.setPower(SPwr);
//            sweepLeft.setPower(-SPwr);
            LMotor.setPower(LLPwr + RLPwr);
           RMotor.setPower(LRPwr + RRPwr);

            telemetry.addData("hello", LMotor.getCurrentPosition());
            telemetry.addData("hello", RMotor.getCurrentPosition());
            telemetry.update();





        }

    }

}