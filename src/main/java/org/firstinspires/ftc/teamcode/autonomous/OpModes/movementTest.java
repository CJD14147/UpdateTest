package org.firstinspires.ftc.teamcode.autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configuration.hardware.driveHardware;

@Autonomous(name = "Movement Test", group = "Test")
public class movementTest extends LinearOpMode {

    /* Declare OpMode members. */
    driveHardware robot = new driveHardware();
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        //void setMsTransmissionInterval();

        // Wait for the game to start (driver presses PLAY)
        /* Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
//        telemetry.update();
        waitForStart();

        robot.reset();


        if (opModeIsActive()) {

            telemetry.addData("left Y encoder Position:", robot.bl.getCurrentPosition());
            telemetry.addData("X encoder Position:", robot.fr.getCurrentPosition());
            telemetry.update();


            robot.rotate(90, 0.5);

            robot.forwardByEncoder(0.25, 3000, this);
            sleep(1000);

            robot.leftByEncoder(0.25, 3000, this);
            sleep(100);

            robot.backwardByEncoder(0.25, 0, this);
            sleep(100);

            robot.rightByEncoder(0.25, 0, this);
            sleep(100);

        }
    }
}
