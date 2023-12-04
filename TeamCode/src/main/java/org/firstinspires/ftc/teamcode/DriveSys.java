package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveSys {


    public DcMotor LMotor;
    public DcMotor RMotor;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    float globalAngle;
    OpticSys opticSys;
    AprilTag aprilTag;


    public DriveSys(HardwareMap hardwareMap)
    {
        opticSys = new OpticSys(hardwareMap,0);
        //Initialize the IMU and its parameters.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        LMotor = hardwareMap.dcMotor.get ("L_Motor"); //check with driver hub
        RMotor = hardwareMap.dcMotor.get("R_Motor"); //check with driver hub

        LMotor.setDirection(DcMotor.Direction.FORWARD); //to be tested with chassis
        RMotor.setDirection(DcMotor.Direction.REVERSE); //to be tested with chassis

        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //variable for how fast the robot will move
        float DRIVE_SPEED = 0.5f;

    }
    private ElapsedTime runtime = new ElapsedTime();

    //will use 20:1 HD Hex motor (revrobotics) + 90 mm grip wheels
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.6f; //can adjust
    static final float     TURN_SPEED              = 0.2f; //can adjust
    public void drive(float inches) {
        int newLeftTarget;
        int newRightTarget;


            // Determine new target position, and pass to motor controller
            newLeftTarget = LMotor.getCurrentPosition() -(int)(inches * COUNTS_PER_INCH);
            newRightTarget = RMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            LMotor.setTargetPosition(-newLeftTarget);
            RMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // reset the timeout time and start motion.
            //runtime.reset();
        LMotor.setPower(0.6);
        RMotor.setPower(0.6);

            // Turn off RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {

            while ( getAngle() > degrees) {}
        }
        else
            while (getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        LMotor.setPower(0);
        RMotor.setPower(0);

        //sleep for a bit to make sure the robot doesn't over sh

        resetAngle();
    }


    //this method resets the angle so that the robot's heading is now 0
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public  int driveToProp()
    {
        opticSys.startIt();
        int location = opticSys.run();
        if(location==1)
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
        return opticSys.run();
    }
    public int driveFromProp()
    {
        int location = opticSys.run();
        if(location==1)
        {
            drive(-10);
            rotate(-45);

        }
        else if(location==3)
        {
            drive(-10);
            rotate(45);

        }
        else
        {
            drive(-12);
        }
        return opticSys.run();
    }
    public int driveToTag()
    {
        aprilTag.initAprilTag();
        double x= aprilTag.getY();
        double y= aprilTag.getY();
        int angle=0;
        float distance=0;
        rotate(angle);
        drive(distance);
        return aprilTag.getTag();
    }
    public int getPos()
    {
        return LMotor.getCurrentPosition();
    }    public int getRPos()
    {
        return RMotor.getCurrentPosition();
    }


}
