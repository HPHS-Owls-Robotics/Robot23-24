package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Driving22", group="12417")

public class DriverOp1 extends LinearOpMode {
    
    DcMotor LMotor;
    DcMotor RMotor;  



    
    //DigitalChannel breakBeam;
    float MaxSpeed = 1.0f;
    @Override
    
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        while (opModeIsActive()) {
            LMotor = hardwareMap.dcMotor.get("motorLeft");
            RMotor = hardwareMap.dcMotor.get("motorRight");

            //breakBeam = hardwareMap.digitalChannel.get("Beam");

            DcMotor[] motors = {LMotor, RMotor};
            for (int i =0; i<2; i++)
            {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            LMotor.setDirection(DcMotor.Direction.REVERSE);
            RMotor.setDirection(DcMotor.Direction.FORWARD);


        float LPwr=0, RPwr=0;
        
        
        LPwr= gamepad1.left_stick_y*MaxSpeed+gamepad1.left_stick_x*MaxSpeed;
        RPwr = gamepad1.left_stick_y*MaxSpeed- gamepad1.left_stick_x*MaxSpeed;

        if(gamepad1.left_stick_y>0)
        {
            LPwr=MaxSpeed;
            RPwr=MaxSpeed;

        }
        if(gamepad1.left_stick_y<0)
        {
            LPwr=-MaxSpeed;
            RPwr=-MaxSpeed;

        }
        if(gamepad1.left_stick_x>0)
        {
            LPwr=MaxSpeed;
            RPwr=-MaxSpeed;

        }
        if(gamepad1.left_stick_x<0)
        {
            LPwr=-MaxSpeed;
            RPwr=MaxSpeed;
        }

        RMotor.setPower(RPwr);
        LMotor.setPower(LPwr);
        }

    }
}
