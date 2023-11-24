package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueCloseFull extends LinearOpMode {

    //OpticSys test;
    DriveSys driveSys;

    int color =0;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSys = new DriveSys(hardwareMap);

        waitForStart();
        telemetry.update();
        if (opModeIsActive()) {
            int hello=driveSys.drive(25);
            telemetry.addData("goodBye", hello);
            telemetry.addData("hello", driveSys.getPos());
            telemetry.update();
            sleep(1000);
        }
    }

}
