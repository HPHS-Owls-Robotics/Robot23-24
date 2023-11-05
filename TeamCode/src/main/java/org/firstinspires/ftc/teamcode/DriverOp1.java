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

    //DigitalChannel breakBeam;
    float MaxSpeed = 1.0f;


    @Override

    public void runOpMode() throws InterruptedException {
        LMotor = hardwareMap.dcMotor.get("L_Motor");
        RMotor = hardwareMap.dcMotor.get("R_Motor");
        arm = hardwareMap.dcMotor.get("Arm_Motor");
        sweepRight = hardwareMap.crservo.get("Sweep_1");
        sweepLeft = hardwareMap.crservo.get("Sweep_2");

        float LPwr = 0, RPwr = 0, SPwr=0;
        waitForStart();
        while (opModeIsActive()) {

            //breakBeam = hardwareMap.digitalChannel.get("Beam");

            DcMotor[] motors = {LMotor, RMotor};
            for (int i = 0; i < 2; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            LMotor.setDirection(DcMotor.Direction.FORWARD);
            RMotor.setDirection(DcMotor.Direction.REVERSE);
            sweepRight.setDirection(DcMotor.Direction.FORWARD);
            sweepLeft.setDirection(DcMotor.Direction.FORWARD);



            LPwr = gamepad1.left_stick_y * MaxSpeed - gamepad1.left_stick_x * MaxSpeed;
                 RPwr = gamepad1.right_stick_y * MaxSpeed + gamepad1.right_stick_x * MaxSpeed;

            if(gamepad1.b)
            {

                SPwr =1f;

            }
            else
                SPwr=0f;



            while(gamepad1.left_bumper)//down
            {
                arm.setPower(MaxSpeed);
            }
            while(gamepad1.right_bumper)//up
            {
                arm.setPower(-MaxSpeed);
            }


            sweepRight.setPower(-SPwr);
            sweepLeft.setPower(SPwr);
            LMotor.setPower(LPwr);
            RMotor.setPower(RPwr);

            telemetry.addData("hello", sweepLeft.getPower());
            telemetry.addData("hello", sweepRight.getPower());
            telemetry.update();





        }

    }

}