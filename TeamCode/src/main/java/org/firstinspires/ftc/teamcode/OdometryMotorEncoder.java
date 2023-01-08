package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.opencv.core.Mat;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import android.os.Build;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;



/**
 * /*
 * x: â¬‡ï¸ negative, â¬†ï¸ positive
 * y: âž¡ï¸ negative, â¬…ï¸ positive
 * a: â†ªï¸ positive, â†©ï¸ negative
 *
 */
public class OdometryMotorEncoder {

    private final boolean USE_IMU = true;  // angle estimation is by IMU not by left/right encoder
    public static final double  COUNTS_PER_MOTOR_REV    = 8192   ;// 435RPM-383.6, 1150RPM--145.6, 8192 for REV encoder
    public static final double  DRIVE_GEAR_REDUCTION    = 1.0 ;     // the new one is 1:1, mecanum drive chassis
    public static final double  WHEEL_DIAMETER_M   = 0.072; //0.072;//0.096;     // for actually odometry tracking wheel
    public static final double  ENC_COUNTS_TO_M         =  (WHEEL_DIAMETER_M * Math.PI) / (COUNTS_PER_MOTOR_REV );  // for tracking wheel no * DRIVE_GEAR_REDUCTION
    public static final double  BASE_WIDTH_M = 0.269;          // left-right wheels to center distance
    public static final double  MIDDLE_CENTER_DIS_M = 0.15;//0.055;      // back tracking wheel to robot rotation center 0.056, "+" for front, "-" for back

    //make sure positive reading when robot move forward and move to left
    public static final int LEFT_ENCODER_DIR = 1;
    public static final int RIGHT_ENCODER_DIR = -1;
    public static final int MIDDLE_ENCODER_DIR = 1;

    private final int PERIOD = 10 ;     //10ms


    // for thread control
    private volatile Orientation currentAngles;        //used for control
    private volatile double yawReading ;
    private volatile double lastYawReading;
    private volatile double headingAngle ;

    private RobotPosition robotPos = new RobotPosition(0,0,0);
    private RobotVel robotLocalVel;

    private double wheelEstimateAngle = 0;

    public DcMotorEx leftEnc = null;
    public DcMotorEx rightEnc = null;
    public DcMotorEx middleEnc = null;

    //IMU
    public BNO055IMU imu;

    private boolean runFlag = false;

    private LinearOpMode mainTask = null;
    private ChassisSystem chassisSystem = null;
    private EstimationThread estimationThread = null;
    private Telemetry telemetry;

    public OdometryMotorEncoder(LinearOpMode mainTask, HardwareMap hwMap, ChassisSystem chassis, RobotPosition pos, Telemetry telemetry, boolean auto){
        this.mainTask = mainTask;
        this.chassisSystem = chassis;
        this.robotPos = pos;   // set starting the robot position
        this.robotLocalVel = new RobotVel();
        this.telemetry = telemetry;
        //IMU
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //map for odometry, please check the wire connection
        leftEnc = chassis.leftFrontDrive;//hwMap.get(DcMotorEx.class, "LF");;
        rightEnc = chassis.rightFrontDrive;//hwMap.get(DcMotorEx.class, "RF");
        middleEnc = chassis.leftRearDrive;//hwMap.get(DcMotorEx.class, "LR");

        // set current angle first
        currentAngles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawReading =  AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);
        lastYawReading = yawReading;
        headingAngle = yawReading;
        wheelEstimateAngle = yawReading / 180 * Math.PI;
        estimationThread = new EstimationThread();


        //start the robot position
        estimationThread.start();


    }

    public RobotPosition getRobotPosition(){
        return robotPos;
    }

    public double getAngle()
    {
        return robotPos.angle;
    }


    public RobotVel getRobotLocalVel(){
        return this.robotLocalVel;
    }

    public void setRobotPosition(RobotPosition pos){
        this.robotPos = pos;
    }


    /**
     *  will translate the IMU reading from -180 ~ 180 to absolute angle
     * @return absolute angle reading from IMU
     */

    public double getHeadingAngle(){
        double heading = 0;
        currentAngles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawReading = AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);

        double delta = yawReading - lastYawReading;
        if (abs(delta) >= 180){
            //it should be over +180 to -180 point
            if (lastYawReading < 0) {
                delta = yawReading - 180 - (180 + lastYawReading);
            }
            if (lastYawReading > 0){
                delta = 180 - lastYawReading + 180 + yawReading;
            }

        }
        headingAngle = headingAngle + delta;
        lastYawReading = yawReading;
        return headingAngle;
    }


    private class EstimationThread extends Thread
    {
        private double leftLastPos = 0, rightLastPos = 0, backLastPos = 0, lastIMUAngle = 0;



        private double accelerateDis = 0;  // accelerate distance
        public EstimationThread()
        {
            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            backLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            lastIMUAngle = getHeadingAngle();       //read back current IMU reading
            runFlag = true;
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            double leftEncPos, rightEncPos, backEncPos, imuHeading;  // now the robot sensor reading
            double deltaL, deltaR, deltaMiddle, deltaTheta, deltaThetaIMU;

            double deltaTime = 0;
            double v1,v2;
            double last_time = 0;
            double angleLast = 0;
            double robotAngle= 0;

            double deltaXLocal, deltaYLocal;

            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            backLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            imuHeading = getHeadingAngle();       //read back current IMU reading
            lastIMUAngle = imuHeading;
            angleLast = robotPos.angle;



            ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            last_time = runtime.milliseconds();
            runFlag = true;
            while(!mainTask.isStopRequested() && !isInterrupted() && runFlag)
            {
                try {
                    leftEncPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
                    rightEncPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
                    backEncPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();


                    imuHeading = getHeadingAngle();
                    deltaL = (leftEncPos - leftLastPos) * ENC_COUNTS_TO_M * 1.0; //translate to meter distance
                    deltaR = (rightEncPos - rightLastPos) * ENC_COUNTS_TO_M * 1.0;  //translate to meter distance

                    //based on encoder reading, (deltaR - deltaL)  -- turn to Left are positive, counter-clockwise is positive, so we use deltaR - deltaL
                    //original using encoder to estimate the angle
                    if (!USE_IMU) {
                        deltaTheta = (deltaR - deltaL) / BASE_WIDTH_M; // radian,
                    } else {
                        //if using gyro(IMU)
                        deltaTheta = Math.toRadians(imuHeading - lastIMUAngle);  // translate to radian
                    }

                    //middle back or front tracking wheel encoder
                    deltaMiddle = (backEncPos - backLastPos) * ENC_COUNTS_TO_M * 1.0;


                    deltaXLocal = 0.5 * (deltaL + deltaR ) ;

                    deltaYLocal =  deltaMiddle - deltaTheta * MIDDLE_CENTER_DIS_M;

                    if (Math.abs(deltaTheta) > 0.000001)
                    {
                        double a = Math.sin(deltaTheta)/deltaTheta;
                        double b = (Math.cos(deltaTheta)-1)/deltaTheta;

                        v1 = a * deltaXLocal + b * deltaYLocal;
                        v2=  -b * deltaXLocal + a * deltaYLocal;
                    } else {
                        v1 = deltaXLocal;
                        v2 = deltaYLocal;
                    }

                    robotAngle = Math.toRadians(robotPos.angle) + deltaTheta;  // to radian
                    //Using IMU instead estimated angle
                    double deltaXGlobal = Math.cos(robotAngle) * v1 - Math.sin(robotAngle) * v2;
                    double deltaYGlobal = Math.sin(robotAngle) * v1 + Math.cos(robotAngle) * v2;
                    robotPos.x += deltaXGlobal;
                    robotPos.y += deltaYGlobal;
                    robotPos.angle += Math.toDegrees(deltaTheta);


                    /******************    To numerically calculate the velocity of the robot ***************/
                    double samplePeriod = runtime.milliseconds() - last_time;
                    last_time += samplePeriod;

                    if (samplePeriod != 0) {
                        robotLocalVel.velX = deltaXLocal / samplePeriod * 1000.0;
                        robotLocalVel.velY = deltaYLocal / samplePeriod * 1000.0;


                        robotLocalVel.velAngle = 0.8 * robotLocalVel.velAngle + 0.2 * (robotPos.angle - angleLast) / samplePeriod * 1000.0;
                        angleLast = robotPos.angle;
                    }



                    //update sensor reading here
                    lastIMUAngle = imuHeading; // already is new data
                    leftLastPos = leftEncPos;
                    rightLastPos = rightEncPos;
                    backLastPos = backEncPos;
                    Thread.sleep(PERIOD);

                }
                // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
                // or by the interrupted exception thrown from the sleep function.
                //catch (InterruptedException e) {goStraightEndFlag = true;}
                // an error occurred in the run loop.
                catch (Exception e) {}
            }
            runFlag = false;
        }
    }

    public void stopThread()
    {
        runFlag = false;
    }



}
