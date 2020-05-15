package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ReneBase;

@Autonomous
public class ReneAuto extends ReneBase {

    @Override
    public void runOpMode()
    {
        telemetry.addData("Initializing: ", "Started");
        initMotors();
        initLocation(0, 0, 0);
        initOdometry();
        telemetry.addData("Initializing: ", "Done");
        telemetry.update();

        waitForStart();
        time.reset();

        /*
        while(opModeIsActive())
        {
            updateOdometry();
            telemetry.addData("X: ", odo.getX());
            telemetry.addData("Y: ", odo.getY());
            telemetry.addData("A: ", odo.getAngle());
            telemetry.update();
        }

         */

        driveToPointLoop(0, 30, .4, 1);
        driveToPointLoop(30, 30, .4, 1);
        driveToPointLoop(30, 0, .4, 1);
        driveToPointLoop(0, 0, .4, 1);
    }


}
