package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChassisSystem {
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx leftRearDrive = null;
    public DcMotorEx rightRearDrive = null;

    private Telemetry telemetry;
    public double speedFactor = 1.0;
    public ChassisSystem(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        leftFrontDrive = hwMap.get(DcMotorEx.class, "lf");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "rf");
        leftRearDrive = hwMap.get(DcMotorEx.class, "lr");
        rightRearDrive = hwMap.get(DcMotorEx.class, "rr");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobotVel(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        double magnitude = Math.abs(lfPower);
        magnitude = Math.max(magnitude, Math.abs(lrPower));
        magnitude = Math.max(magnitude, Math.abs(rfPower));
        magnitude = Math.max(magnitude, Math.abs(rrPower));

        if (magnitude > 1){
            lfPower /= magnitude;
            lrPower /= magnitude;
            rfPower /= magnitude;
            rrPower /= magnitude;
        }


        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontDrive.setPower(rfPower);
        rightRearDrive.setPower(rrPower);
        leftRearDrive.setPower(lrPower);
        leftFrontDrive.setPower(lfPower);
    }


    public void brakeAll (boolean isLockPosition, boolean isBrakeOn){

        if (isLockPosition)
        {
            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
            leftRearDrive.setTargetPosition(leftRearDrive.getCurrentPosition());
            rightRearDrive.setTargetPosition(rightRearDrive.getCurrentPosition());
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            leftRearDrive.setPower(0.5);
            rightRearDrive.setPower(0.5);
        } else if (isBrakeOn)
        {
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);
        }
    }

    public void resetEncoders (){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveRobot(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        double magnitude = Math.abs(lfPower);
        magnitude = Math.max(magnitude, Math.abs(lrPower));
        magnitude = Math.max(magnitude, Math.abs(rfPower));
        magnitude = Math.max(magnitude, Math.abs(rrPower));

        if (magnitude > 1){
            lfPower /= magnitude;
            lrPower /= magnitude;
            rfPower /= magnitude;
            rrPower /= magnitude;
        }


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setPower(rfPower);
        rightRearDrive.setPower(rrPower);
        leftRearDrive.setPower(lrPower);
        leftFrontDrive.setPower(lfPower);
    }

    public void stopRobot()
    {
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
    }

    public void chassisTeleOp(Gamepad gamepad)
    {
        if(gamepad.right_bumper ){
            speedFactor = 1.0;
        }
        else{
            speedFactor = 0.3;
        }

        double leftStickXPos = -gamepad.left_stick_x * speedFactor;
        double leftStickYPos = -gamepad.left_stick_y * speedFactor;
        double rightStickXPos = -gamepad.right_stick_x * speedFactor;

        double denominator = Math.max(Math.abs(leftStickYPos) + Math.abs(leftStickXPos) + Math.abs(rightStickXPos), 1);

        double lfPower = (leftStickYPos - leftStickXPos - rightStickXPos) / denominator;
        double lrPower = (leftStickYPos + leftStickXPos - rightStickXPos) / denominator;
        double rfPower = (leftStickYPos + leftStickXPos + rightStickXPos) / denominator;
        double rrPower = (leftStickYPos - leftStickXPos + rightStickXPos) / denominator;

      //  telemetry.addData("power", "%1.2f,  %1.2f, %1.2f, %1.2f", lfPower, lrPower, rfPower, rrPower);

       // driveRobotVel(lfPower, lrPower, rfPower, rrPower);
        driveRobot(lfPower, lrPower, rfPower,rrPower);
    }


    /*
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
     * from its angle or rotation rate.
     *
     * @param f The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param s The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param r The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *     positive.
     * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this
     *     to implement field-oriented controls.
     */
    public void driveCartesian(Gamepad gamepad, double robotAngle)
    {
        if(gamepad.right_bumper ){
            speedFactor = 1.0;
        }
        else{
            speedFactor = 0.3;
        }
        double f = -speedFactor * Math.pow(gamepad.left_stick_y, 3);
        double s = -speedFactor * Math.pow(gamepad.left_stick_x , 3);
        double r = -speedFactor * Math.pow(gamepad.right_stick_x, 3);

        double lfPower = 0;
        double rfPower = 0;
        double lrPower = 0;
        double rrPower = 0;
        f = Range.clip(f, -1.0, 1.0);
        s = Range.clip(s, -1.0, 1.0);
        r = Range.clip(r, -1.0,1.0);
        double robot_radians = Math.toRadians(robotAngle);

        double temp = f * Math.cos(robot_radians) +  s * Math.sin(robot_radians);
        s = -f * Math.sin(robot_radians) + s * Math.cos(robot_radians);
        f = temp;
        driveRobotFSR(f,s,r);
    }



    public void setChassisBrakeMode(DcMotor.ZeroPowerBehavior mode)
    {
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftFrontDrive.setZeroPowerBehavior(mode);
        rightRearDrive.setZeroPowerBehavior(mode);
        leftRearDrive.setZeroPowerBehavior(mode);
    }

    /* set the robot command out, if using odometry tracking wheel, */
    public void driveRobotFSR(double fPower, double sPower, double rPower){
        //translate to each motor power
        double lfP = fPower - sPower - rPower;
        double lrP = fPower + sPower - rPower;
        double rfP = fPower + sPower + rPower;
        double rrP = fPower - sPower + rPower;


      //  driveRobotVel(lfP,lrP,rfP,rrP);
        driveRobot(lfP, lrP, rfP,rrP);
    }


}
