package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Driving222", group="12417")

public class DriverOp2 extends LinearOpMode {

    DcMotor LMotor;
    DcMotor RMotor;

    DcMotor arm;

    CRServo sweepRight;
    CRServo sweepLeft;

    CRServo intake;
    CRServo plane;
    Servo trapdoor;
    DigitalChannel breakBeam;
    float MaxSpeed = 0.7f;
    float ArmSpeed = 0.3f;


    @Override

    public void runOpMode() throws InterruptedException {
        LMotor = hardwareMap.dcMotor.get("L_Motor");
        RMotor = hardwareMap.dcMotor.get("R_Motor");
        arm = hardwareMap.dcMotor.get("Arm_Motor");
//        sweepRight = hardwareMap.crservo.get("Sweep_1");
//        sweepLeft = hardwareMap.crservo.get("Sweep_2");
        trapdoor = hardwareMap.servo.get("Trapdoor");
        float LPwr = 0, RPwr = 0, SPwr=0, LLPwr, LRPwr, RLPwr, RRPwr;
        trapdoor.setPosition(0.0); //check
        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.x)
            {
                telemetry.addData("hello", gamepad1.x);
                telemetry.update();
            }
            if(gamepad2.x)
            {
                telemetry.addData("goodbye", gamepad2.x);
                telemetry.update();

            }




        }

    }

}