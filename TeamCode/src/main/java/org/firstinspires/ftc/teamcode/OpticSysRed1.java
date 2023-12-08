package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Autonomous
public class OpticSysRed1  extends LinearOpMode {
    private OpenCvCamera webcam1;
    private OpenCvCamera webcam2;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);
    ContourPipeline myPipeline1;
    ContourPipeline myPipeline2;
    int color;


    public DcMotor LMotor;
    public DcMotor RMotor;

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


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    List<AprilTagDetection> currentDetections;
    @Override
    public void runOpMode()
    {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
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

        LMotor.setDirection(DcMotor.Direction.REVERSE); //to be tested with chassis

        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //The IMU does not initialize instantly. This makes it so the driver can see when they can push Play without errors.
        telemetry.addData("Mode", "calibrating...");


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                LMotor.getCurrentPosition(),
                RMotor.getCurrentPosition());
        telemetry.update();

        //Tells the driver it is ok to start.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //variable for how fast the robot will move
        float DRIVE_SPEED = 0.5f;





        ////////////////////////////////////////////
        color=0;
        myPipeline1 = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY);
        myPipeline2 = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY);
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        //OpenCV Pipeline

        // Configuration of Pipeline
        myPipeline1.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline2.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline1.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);        myPipeline1.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline2.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.setPipeline( myPipeline1);
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.setPipeline( myPipeline2);
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();
//                encoderDrive(1,-24,-24,5);
        if(opModeIsActive())
        {
            //encoderDrive(DRIVE_SPEED,  -36,  -36, 5);
            sleep(200);
            double go = run();
            telemetry.addData("location", run());
            telemetry.update();
            sleep(3000);
           go= run();
            telemetry.addData("location", go);
            telemetry.update();
            if(go==3)
            {
                telemetry.addData("straight ahead",go);
                telemetry.update();
                encoderDrive(1,-24,-24,5);
                encoderDrive(1,24,24,5);

            }
            else
            {
                encoderDrive(1,-24,-24,5);
                sleep(3000);
                go= run();
                telemetry.addData("straight ahead",go);
                telemetry.update();
                sleep(3000);

            }


//            if(myPipeline2.getRectMidpointY()>50)
//            {
//                telemetry.addData("rectangle midpoint", myPipeline1.getRectMidpointX());
//                telemetry.update();
//                encoderDrive(DRIVE_SPEED,  -12,  -12, 5);
//                encoderDrive(DRIVE_SPEED,  12,  12, 5);
//            }
//            else if(myPipeline1.getRectMidpointY()>50)
//            {
//                telemetry.addData("rectangle midpoint", myPipeline2.getRectMidpointX());
//                telemetry.update();
//                rotate(45,0.5f);
//
//                encoderDrive(DRIVE_SPEED,  -12,  -12, 5);
//                encoderDrive(DRIVE_SPEED,  12,  12, 5);
//                rotate(-90,0.5f);
//            }
//            else
//            {
//                rotate(-45,0.5f);
//
//                encoderDrive(DRIVE_SPEED,  -12,  -12, 5);
//                encoderDrive(DRIVE_SPEED,  12,  12, 5);
//                rotate(-45,0.5f);
////            }
//
//            telemetry.addData("rectangle midpoint", myPipeline1.getRectMidpointX());
//            telemetry.addData("rectangle midpoint", myPipeline2.getRectMidpointX());
            telemetry.update();

        }
    }

    public void initAprilTag( ) {


        currentDetections = aprilTag.getDetections();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                int i = detection.id;
            }
        }

    }
    public double run()
    {
        myPipeline1.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        myPipeline2.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if(myPipeline1.error){
            return -1;
        }
        // Only use this line of the code when you want to find the lower and upper values
        //testing(myPipeline1);
        if(myPipeline1.getRectHeight() > 50 || myPipeline2.getRectHeight() > 50 ){

            if(myPipeline1.getRectArea()>myPipeline2.getRectArea()){
                return 2;
            }
            if(myPipeline2.getRectArea()>myPipeline1.getRectArea())
            {
                return 3;

            }
        }
        return 1;
    }

    public double getA1()
    {
        return myPipeline1.getRectArea();
    }
    public double getA2()
    {
        return myPipeline2.getRectArea();
    }
    public void testing(ContourPipeline myPipeline){
        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }


    public void encoderDrive(float speed, float leftInches, float rightInches, float timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LMotor.setTargetPosition(newLeftTarget);
            RMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LMotor.setPower(Math.abs(speed));
            RMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LMotor.isBusy() && RMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        LMotor.getCurrentPosition(),
                        RMotor.getCurrentPosition());
                telemetry.update();
            }



            LMotor.setPower(0);
            RMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else
            while (opModeIsActive() && getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        LMotor.setPower(0);
        RMotor.setPower(0);

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
