package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU; //IMU thingy
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Sample Auto with IMU & encoders")

public class DemoAutoEncoderAndIMU extends LinearOpMode {

    public DcMotor motorLeft;
    public DcMotor motorRight;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    float globalAngle;
  
    private ElapsedTime runtime = new ElapsedTime();

    //will use 20:1 HD Hex motor (revrobotics) + 90 mm grip wheels
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.6f; //can adjust
    static final float     TURN_SPEED              = 0.2f; //can adjust

    @Override
    public void runOpMode() {

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

        motorLeft = hardwareMap.dcMotor.get ("motorLeft"); //check with driver hub
        motorRight = hardwareMap.dcMotor.get("motorRight"); //check with driver hub

        motorLeft.setDirection(DcMotor.Direction.REVERSE); //to be tested with chassis

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //The IMU does not initialize instantly. This makes it so the driver can see when they can push Play without errors.
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }



        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition());
        telemetry.update();

        //Tells the driver it is ok to start.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //variable for how fast the robot will move
        float DRIVE_SPEED = 0.5;

        waitForStart();

//motion code:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);
        rotate(90, TURN_SPEED);
        encoderDrive(DRIVE_SPEED,  -12,  -12, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    // //Method for driving forward by TIME, not Encoder
    // public void driveByTime (float power, long time){

    //     motorLeft.setPower(power);
    //     motorRight.setPower(power);

    //     sleep(time);

    //     motorLeft.setPower(0.0);
    //     motorRight.setPower(0.0);
    // }

    // //Method for turning right, by TIME, not IMU
    // public void TurnRightByTime(float power, long time)
    // {
    //     motorLeft.setPower(power);
    //     motorRight.setPower(-power);
    //     sleep(time);

    //     motorLeft.setPower(0);
    //     motorRight.setPower(0);
    //     sleep(250);
    // }

    // //Method for turning left, by TIME, not IMU
    // public void TurnLeftByTime(float power, long time)
    // {
    //     motorLeft.setPower(-power);
    //     motorRight.setPower(power);
    //     sleep(time);

    //     motorLeft.setPower(0);
    //     motorRight.setPower(0);
    //     sleep(250);
    // }

    //Method for driving with encoder
    public void encoderDrive(float speed, float leftInches, float rightInches, float timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


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
    public void rotate(int degrees, float power)
    {
        float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = -power;
            rightPower = power;
        }
        else return;

        //sets power to motors with negative signs properly assigned to make the robot go in the correct direction
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else
            while (opModeIsActive() && getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        motorLeft.setPower(0);
        motorRight.setPower(0);

        //sleep for a bit to make sure the robot doesn't over shoot
        sleep(1000); //can adjust

        resetAngle();
    }


    //this method resets the angle so that the robot's heading is now 0
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }




}
