package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueCloseFull extends LinearOpMode {

    OpticSys test;
    DriveSys driveSys;

    int color =0;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSys = new DriveSys(this.hardwareMap);

        waitForStart();
        telemetry.update();
        if (opModeIsActive()) {
            driveSys.drive(12);
            driveSys.driveToProp();
            telemetry.update();
            driveSys.driveFromProp();
            driveSys.rotate(90);
            driveSys.drive(24);
            telemetry.addData("tag",driveSys.driveToTag());
            telemetry.update();
        }
    }

}
