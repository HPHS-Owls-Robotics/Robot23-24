package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveSys {


    public DcMotor motorLeft;
    public DcMotor motorRight;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    float globalAngle;
    OpticSysTest opticSys;


    public DriveSys(HardwareMap hardwareMap)
    {
        opticSys = new OpticSysTest(hardwareMap);
    }
    private ElapsedTime runtime = new ElapsedTime();

    //will use 20:1 HD Hex motor (revrobotics) + 90 mm grip wheels
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.6f; //can adjust
    static final float     TURN_SPEED              = 0.2f; //can adjust
    public void drive( float Inches) {
        int newTarget;
            // Determine new target position, and pass to motor controller
            newTarget = motorLeft.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newTarget);
            motorRight.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(DRIVE_SPEED));
            motorRight.setPower(Math.abs(DRIVE_SPEED));


            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //This method reads the IMU getting the angle. It automatically adjusts the angle so that it is between -180 and +180.
    public float getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //The method turns the robot by a specific angle, -180 to +180.
    public void rotate(int degrees)
    {
        float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = TURN_SPEED;
            rightPower = -TURN_SPEED;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = -TURN_SPEED;
            rightPower = TURN_SPEED;
        }
        else return;

        //sets power to motors with negative signs properly assigned to make the robot go in the correct direction
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {

            while ( getAngle() > degrees) {}
        }
        else
            while (getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        motorLeft.setPower(0);
        motorRight.setPower(0);

        //sleep for a bit to make sure the robot doesn't over sh

        resetAngle();
    }


    //this method resets the angle so that the robot's heading is now 0
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public  int driveToLocation()
    {
        int location =0;

        if(location==2)
        {
            rotate(45);
            drive(10);
        }
        else if(location==3)
        {
            rotate(-45);
            drive(10);
        }
        else
        {
            drive(12);
        }
        return location;
    }


}
