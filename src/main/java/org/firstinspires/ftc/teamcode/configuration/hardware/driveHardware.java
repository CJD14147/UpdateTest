
package org.firstinspires.ftc.teamcode.configuration.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class driveHardware {

    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .3;

    /* local OpMode members. */
    HardwareMap dHwMap = null;
    private ElapsedTime period = new ElapsedTime();

    final int OFF = 0;


    /* Constructor */
    public driveHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap adtHwMap) {
        // Save reference to Hardware map
        dHwMap = adtHwMap;

        // Define and Initialize Motors
        fl = dHwMap.get(DcMotor.class, "fl");
        fr = dHwMap.get(DcMotor.class, "fr");
        bl = dHwMap.get(DcMotor.class, "bl");
        br = dHwMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        // Set all motors to RUN WITHOUT ENCODER
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = dHwMap.get(BNO055IMU.class, "imu0");

        imu.initialize(parameters);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Movement Methods by encoder FORWARD////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void forwardByEncoder(double speed, double distance, LinearOpMode op) {
        // reset();
        while (((bl.getCurrentPosition() < distance) && op.opModeIsActive())) {
            fl.setPower(speed);
            fr.setPower(speed);
            bl.setPower(speed);
            br.setPower(speed);
        }
        fl.setPower(OFF);
        fr.setPower(OFF);
        bl.setPower(OFF);
        br.setPower(OFF);
    }

    // Movement Methods by encoder BACKWARD
    public void backwardByEncoder(double speed, double distance, LinearOpMode op) {
        //reset();
        while (((bl.getCurrentPosition() > distance) && op.opModeIsActive())) {
            fl.setPower(-speed);
            fr.setPower(-speed);
            bl.setPower(-speed);
            br.setPower(-speed);
        }
        fl.setPower(OFF);
        fr.setPower(OFF);
        bl.setPower(OFF);
        br.setPower(OFF);
    }

    // Movement Methods by encoder RIGHT
    public void rightByEncoder(double speed, double distance, LinearOpMode op) {
        // reset();
        while (((fr.getCurrentPosition() > distance) && op.opModeIsActive())) {
            fl.setPower(speed);
            fr.setPower(-speed);
            bl.setPower(-speed);
            br.setPower(speed);
        }
        fl.setPower(OFF);
        fr.setPower(OFF);
        bl.setPower(OFF);
        br.setPower(OFF);
    }

    // Movement Methods by encoder LEFT
    public void leftByEncoder(double speed, double distance, LinearOpMode op) {
        //reset();
        ///// switching > to < for test
        while (((fr.getCurrentPosition() < distance) && op.opModeIsActive())) {
            fl.setPower(-speed);
            fr.setPower(speed);
            bl.setPower(speed);
            br.setPower(-speed);
        }
        fl.setPower(OFF);
        fr.setPower(OFF);
        bl.setPower(OFF);
        br.setPower(OFF);
    }

    // Method for driving forwards -- select speed
    public void forward(double speed) {
        fl.setPower(Math.abs(speed));
        fr.setPower(Math.abs(speed));
        bl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
    }

    // Method for driving backwards -- select speed
    public void backward(double speed) {
        fl.setPower(-Math.abs(speed));
        fr.setPower(-Math.abs(speed));
        bl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
    }

    // Method for driving left -- select speed
    public void left(double speed) {
        fl.setPower(-Math.abs(speed));
        fr.setPower(Math.abs(speed));
        bl.setPower(Math.abs(speed));
        br.setPower(-Math.abs(speed));
    }

    // Method for driving right -- select speed
    public void right(double speed) {
        fl.setPower(Math.abs(speed));
        fr.setPower(-Math.abs(speed));
        bl.setPower(-Math.abs(speed));
        br.setPower(Math.abs(speed));
    }

    // Method to stop moving
    public void stop() {
        fl.setPower(OFF);
        fr.setPower(OFF);
        bl.setPower(OFF);
        br.setPower(OFF);
    }


    public void reset() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////// Methods for Rotations///////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Resets the cumulative angle tracking to zero.
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    // Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
    // @param degrees Degrees to turn, + is left - is right
    public void rotate(int degrees, double power) {
        double turnPower;
        double angle = getAngle();
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < angle) {   // turn right.
            turnPower = -power;
        } else if (degrees > angle) {
            turnPower = power;
        } else return;

        // set power to rotate.
        fl.setPower(-turnPower);
        bl.setPower(-turnPower);
        fr.setPower(turnPower);
        br.setPower(turnPower);

        // rotate until turn is completed.
        if (degrees < angle) {
            // On right turn we have to get off zero first.
            while (getAngle() == angle) {
            }

            while (getAngle() > degrees) {
            }
        } else    // left turn.
            while (getAngle() < degrees) {
            }

        // rotate until turn is completed.
        if (degrees < angle) {
            // On right turn we have to get off zero first.
            while (getAngle() == angle) {
            }

            while (getAngle() > degrees) {
            }
        } else    // left turn.
            while (getAngle() < degrees) {
            }

        // turn the motors off.
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

}
