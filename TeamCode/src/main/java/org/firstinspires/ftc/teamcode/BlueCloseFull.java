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
        driveSys = new DriveSys(this.hardwareMap);

        waitForStart();
        telemetry.update();
        while (opModeIsActive()) {
            driveSys.drive(12);
            telemetry.addData("hello", driveSys.getPos());
            telemetry.addData("hello", driveSys.getRPos());

        }
    }

}
