/**
 *
 * Created by Maddie and Bria!, FTC Team 4962, The Rockettes
 * version 1.0 Aug 22, 2016
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Sample program to read data from a generic switch.
 * The sensor must be plugged into an digital port
 * configured on driver hub as "switch".
 *
 * Ex:
 * Hall effect sensor: See https://www.adafruit.com/product/158
 * Break Beam sensor: See https://www.amazon.com/Adafruit-Accessories-Break-Beam-Sensor/dp/B01BU6YBWU
 */
@TeleOp(name = "Sensor: Generic Switch", group = "12417")
//@Disabled
public class BreakBeam extends LinearOpMode {

    /*
     * Main loop
     */
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

        DigitalChannel genericSwitch;

        genericSwitch = hardwareMap.digitalChannel.get("switch");

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // is it on or off?

            boolean isIntercepted = genericSwitch.getState();

            String switchState;
            if (isIntercepted) {
                switchState = "Object Absent";
            } else {
                switchState = "Object Present";
            }
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("state", ":  " + switchState);
            telemetry.update();
        }
    }

}
