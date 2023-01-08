package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumXYTurnDriveLib {

    private LinearOpMode mainTask;
    private ChassisSystem chassis;
    private OdometryMotorEncoder odometry;
    private Telemetry telemetry ;

    public double publickP;
    public double publicVmin;


    private RobotPosition robotPosition;
    private boolean Chassis_motorEnabled = false;
    GoXYnTurnThread goXYnTurnThread = null;
    boolean goXYnTurnFinished = true;

    public MecanumXYTurnDriveLib(LinearOpMode mainTask, ChassisSystem chassis, OdometryMotorEncoder odometry, Telemetry telemetry)
    {
        this.mainTask = mainTask;
        this.chassis = chassis;
        this.odometry = odometry;
        this.telemetry = telemetry;
        goXYnTurnFinished = true;
        Chassis_motorEnabled = false;
    }


    private class GoXYnTurnThread extends Thread        {
        double heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn,turn_resolution;
        boolean isLockPosition;
        public boolean isgoXYnTurn_finished = false;
        int delayMS;
        boolean isBrakeOn;

        public GoXYnTurnThread( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, boolean isLockPosition, boolean isBrakeOn,  int delayMS)
        {
            this.heading = heading;
            this.targetX= targetX;          this.targetY= targetY;
            this.V_max= V_max;                this.Tracking_resolution= Tracking_resolution;
            this.V_max_Turn = V_max_Turn;         this.turn_resolution = turn_resolution;
            this.isLockPosition = isLockPosition;
            this.delayMS = delayMS;
            this.isBrakeOn = isBrakeOn;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                isgoXYnTurn_finished =  goXYnTurn(heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn, turn_resolution, isLockPosition, isBrakeOn);
            }
            catch (Exception e) {;}
        }
    }


    private boolean goXYnTurn ( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, boolean isLockPosition, boolean isBrakeOn){
        Chassis_motorEnabled = true;
        double drive_powerX = 0, drive_powerY=0;
        //resetEncoders();
        robotPosition = odometry.getRobotPosition();
        double x0 = robotPosition.x, y0= robotPosition.y;
        double V_XLocal=0, V_YLocal=0, Turn_power=0;
        double distance2Target = Math.hypot((targetX - robotPosition.x),(targetY-robotPosition.y));
        double distance2Go;
        boolean isTargetPosArrived = false, isHeadingAchieved = false;
        double traveledDistance =0;
        double Vp;
        double pathAngle =0;
        double robotHeading =0;
        double Kp = publickP;//0.003; // Proportional gain for turning control 0.01 means full power(speed) at 100 degree turn
        double V_turn_min = publicVmin;//0.08;
        double angle2turn;

        while (Chassis_motorEnabled  && !mainTask.isStopRequested() && (!isHeadingAchieved || !isTargetPosArrived)) {
            robotPosition = odometry.getRobotPosition();
            distance2Go = Math.hypot((targetX - robotPosition.x),(targetY-robotPosition.y));
            angle2turn = heading - odometry.getAngle();

            if (distance2Go  > Tracking_resolution && traveledDistance < distance2Target)
            {//distance to target spot
                traveledDistance = Math.hypot((robotPosition.x - x0), (robotPosition.y - y0));
                Vp = speed_planning(V_max, traveledDistance, distance2Target  - Tracking_resolution);
                pathAngle = Math.atan2(targetY - robotPosition.y, targetX - robotPosition.x);
                robotHeading = Math.toRadians(odometry.getAngle());
                V_XLocal = Vp * Math.cos(robotHeading - pathAngle);
                V_YLocal = -Vp * Math.sin(robotHeading - pathAngle);
                isTargetPosArrived = false;
            } else
            {V_XLocal =0;V_YLocal =0; isTargetPosArrived = true;}

            /*//just for testing
            telemetry.addData("Position", "%.2f %.2f %.2f", robotPosition.x, robotPosition.y, robotPosition.angle);
            if (isTargetPosArrived)
            {
                telemetry.addLine("stop ");
            }
            else{
                telemetry.addLine("run");
            }
            telemetry.update();
*/

            if (Math.abs(angle2turn) > turn_resolution)
            {//Turning to target heading
                Turn_power = Kp * angle2turn;

                if (Math.abs(Turn_power) < V_turn_min) {
                    Turn_power = V_turn_min * Math.signum(Turn_power);
                }
                if (Math.abs(Turn_power) > V_max_Turn) {
                    Turn_power = V_max * Math.signum(Turn_power);
                }
                isHeadingAchieved = false;
            } else {Turn_power =0; isHeadingAchieved = true;}


            double LFPower = V_XLocal - V_YLocal - Turn_power;
            double RFPower = V_XLocal + V_YLocal + Turn_power;
            double LRPower = V_XLocal + V_YLocal - Turn_power;
            double RRPower = V_XLocal - V_YLocal + Turn_power;

            chassis.driveRobot(LFPower, LRPower, RFPower, RRPower);
            try {
                Thread.sleep(10);
            }
            catch (Exception e)
            {

            }

        }
        chassis.brakeAll(isLockPosition, isBrakeOn);
        goXYnTurnFinished = true;
        return true;
    }

    private double speed_planning ( double V_max, double current_d, double target_d)
    {
        double V1, V3;
        double V_start = 0.5; //starting power without getting wheel slip
        double V_end = 0.1; //end speed to approach destination

        double Kup = 6, Kdown = 4 / (Math.abs(target_d) + 1.0); // Acc and de-acc rate  Power/count 2.5
        if (target_d > 0) {
            V1 = V_start + Kup * current_d;
            if (V1 < V_start) V1= V_start;
            V3 = V_end + Kdown * (target_d - current_d);
            if (V3<V_end) V3 = V_end;

            return Math.min(V1, Math.min(V_max, V3));
        } else {
            V1 = -V_start + Kup * current_d;
            if (V1 > V_start) V1= V_start;
            V3 = -V_end + Kdown * (target_d - current_d);
            if (V3>V_end) V3 = V_end;
            return Math.max(V1, Math.max(-V_max, V3));
        }
    }

    public void goXYnTurnTask( double heading, double targetX, double targetY, double V_max,
                               double Tracking_resolution, double V_max_Turn, double turn_resolution, boolean isLockPosition, boolean isBrakeOn, int delayMS)
    {
        stopGoXYnTurn();
        try {
            Thread.sleep(10);
        }
        catch (Exception e){

        }
        if (mainTask.opModeIsActive()) {
            goXYnTurnThread = new GoXYnTurnThread(heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn, turn_resolution, isLockPosition, isBrakeOn, delayMS);
            goXYnTurnThread.start();
            goXYnTurnFinished = false;
        }
    }

    public boolean isGoXYnTurnFinished()
    {
        return goXYnTurnFinished;
    }

    public void stopGoXYnTurn()
    {
        //Chassis_motorEnabled = false;
        chassis.stopRobot();
    }

    public void setTurnPID(double kP, double vMin)
    {
        publickP = kP;
        publicVmin = vMin;
    }




}
