/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Left Middle High")
//@Disabled

public class AutoLeftMidHigh extends LinearOpMode {
    //CameraObjectDetector cameraObjectDetector = null;
    ChassisSystem chassis = null;
    OdometryMotorEncoder odometry = null;
    MecanumXYTurnDriveLib mecanumDrive = null;

    HSliderSystem hSliderSystem = null;
    VSliderSystem vSliderSystem = null;
    RobotPosition robotPos = new RobotPosition(0,0,0);

    RobotPosition parkPosition = new RobotPosition(0,0,0);
    public static double targetX = 1.32;
    public static double targetY = 0;
    public static double targetA = 0;
    public static double targetVel = 0.5;

    public static double turnAngle = -99.94;
    public static double turnVel = 0.2;


    private double vSliderHDropSliderLen = 0.418;
    private double vSliderHDropTiltAngle = 59;
    private double vSliderHDropPanAngle = 9.01;
    private double vSliderDropTiltAngle = 90;

    private double hSliderPickPanAngle = -12;


    private double VPICKUP_LEN = 0.23;
    private double VPICKUP_TILT_ANGLE = -134;
    private double VPICKUP_PAN_ANGLE = 0;

    private double HMIDDLE_TILT_ANGLE = 90;
    private double HHANDOVER_TILT_ANGLE = 134;


    private double VSLIDER_HANDOVER_LEN = 0.16;
    private double HSLIDER_HANDOVER_LEN = 0.056;



    private int pickLevel = 1;

    double vcheckPos = 0;
    double hCheckPos = 0;
    ElapsedTime timer = new ElapsedTime();

    int parkingPos = 1;  // for parking location, by camera
    boolean foundPole = false;
    boolean foundCone = false;
    double deltaDis = 0.015;
    @Override
    public void runOpMode(){
        //cameraObjectDetector = new CameraObjectDetector(this, hardwareMap,telemetry);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(hardwareMap, telemetry);


        robotPos = new RobotPosition(0,0,0);
        odometry = new OdometryMotorEncoder(this, hardwareMap, chassis,robotPos, telemetry, true);

        sleep(1000);
        telemetry.addLine("odometry is setup");
        telemetry.update();

        mecanumDrive = new MecanumXYTurnDriveLib(this, chassis, odometry,telemetry);

        hSliderSystem = new HSliderSystem(this, hardwareMap, telemetry, true);
        sleep(1000);

        vSliderSystem = new VSliderSystem(this, hardwareMap, telemetry, true);


        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        hSliderSystem.enableTouchCheck();
        vSliderSystem.enableTouchCheck();

        timer.reset();
        pickLevel = 5;
        //parkingPos = cameraObjectDetector.identifyTeamObject();// will take some time here

        if (parkingPos == 1){
            parkPosition.x = 0.64;
            parkPosition.y = 0.6;
            parkPosition.angle = 0;
        }else if(parkingPos == 2){
            parkPosition.x = 0.64;
            parkPosition.y = 0.0;
            parkPosition.angle = 0;
        }else if(parkingPos == 3){
            parkPosition.x = 0.64;
            parkPosition.y = -0.6;
            parkPosition.angle = 0;
        }
        writeSDCardFile(parkPosition);

        vSliderSystem.setClawAngle(vSliderSystem.CLAW_CLOSE_ANGLE);
        vSliderSystem.sliderLenCtrl(vSliderHDropSliderLen, 0.5);
        vSliderSystem.setTiltAngle(vSliderHDropTiltAngle);
        // reset claw to right position first, only for this tele demo program

        robotPos = odometry.getRobotPosition();

        //go forward

        mecanumDrive.setTurnPID(0.01, 0.3);

        hSliderSystem.sliderLenCtrl(0.01, 0.1);//keep arm in

        mecanumDrive.goXYnTurnTask(0,0.04,-0.44,targetVel + 0.3,0.04,0.6,10,true,true,0);
        while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 2250){
            updateTelemetry();
            sleep(10);
        }
        mecanumDrive.stopGoXYnTurn();
        sleep(250);

        mecanumDrive.setTurnPID(0.004, 0.1);


        mecanumDrive.goXYnTurnTask(0,1.08,-0.44,targetVel,0.01,0.2,1,true,true,0);
        while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 4000){
            updateTelemetry();
            sleep(10);
        }
        mecanumDrive.stopGoXYnTurn();

        sleep(2000);

        /*mecanumDrive.goXYnTurnTask(0,1.19,-0.31,targetVel,0.01,0.2,1,true,true,0);
        while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 6250){
            updateTelemetry();
            sleep(10);
        }
        mecanumDrive.stopGoXYnTurn();

        sleep(150);*/


        vSliderSystem.setPanAngle(vSliderHDropPanAngle);


        //turn

        mecanumDrive.setTurnPID(0.01, 0.2);

        mecanumDrive.goXYnTurnTask(turnAngle,1.08,-0.44,targetVel,0.01,0.2,1,true,true,0);
        while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 12000){
            sleep(10);
        }
        mecanumDrive.stopGoXYnTurn();


        //vSliderSystem.scanToFindJunction();


        while(opModeIsActive() && timer.milliseconds() < 290000)
        {
            sleep(10);
        }


        /*
        //scan and fix error
        ElapsedTime timeout = new ElapsedTime();
        while(opModeIsActive() && !vSliderSystem.isFoundJunction() && timeout.milliseconds() < 2000)
        {
            sleep(10);
        }

        if(vSliderSystem.isFoundJunction())
        {
            vSliderHDropPanAngle = vSliderSystem.getPanAngle();
        }
        vSliderSystem.stopScan();

        double junctionDiff = (vSliderSystem.middleSensor.getDistance(DistanceUnit.MM) - 185)/1000;

        robotPos = odometry.getRobotPosition();

        telemetry.addData("x", robotPos.x);
        telemetry.addData("y", robotPos.y);
        telemetry.addData("angle", robotPos.angle);
        telemetry.addData("junctionDiff", junctionDiff);
        telemetry.update();

        if(Math.abs(junctionDiff) > 0.01)
        {
            double theta = robotPos.angle - vSliderSystem.getPanAngle();
            double x = junctionDiff * Math.cos(Math.toRadians(theta)) + robotPos.x;
            double y = junctionDiff * Math.sin(Math.toRadians(theta)) + robotPos.y;

            mecanumDrive.goXYnTurnTask(turnAngle, x, y,0.2,0.01,0.2,1,true,true,0);
            timeout.reset();
            while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timeout.milliseconds() < 3000)
            {
                sleep(10);
            }
            mecanumDrive.stopGoXYnTurn();
        }
        //stop scan
        */






        hSliderGotoPickUpPos(pickLevel,false);
        //pick up cone
        boolean finishFlag = false;
        robotPos = odometry.getRobotPosition();
        if (opModeIsActive() && Math.abs(robotPos.x - targetX) < 0.05 && Math.abs(robotPos.y - targetY) < 0.05 && Math.abs(robotPos.angle - turnAngle) < 5) {
            while (opModeIsActive() && !finishFlag && timer.milliseconds() < 25000) {
                vSliderDropCone();
                //suppose the hSlider already at position
                gotoHandOverPos();
                handOverCone();
                pickLevel--;
                hSliderGotoPickUpPos(pickLevel, false);
                vSliderGotoDropPos(3, true);
                if (pickLevel <= 0) {
                    finishFlag = true;
                }
            }

            vSliderDropCone();
        }


        vSliderSystem.setTiltAngle(0);
        vSliderSystem.setPanAngle(0);
        vSliderSystem.sliderLenCtrl(0.15, 1.0);


        hSliderSystem.sliderLenCtrl(0.01, 1.0);
        hSliderSystem.setTiltAngle(134);
        hSliderSystem.setPanAngle(0);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);


        /*
        if (parkingPos == 1 && opModeIsActive()) {
            mecanumDrive.goXYnTurnTask(90, 1.2, 0.6, targetVel, 0.02, 0.8, 1, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 29000) {
                updateTelemetry();
                sleep(10);
            }

            mecanumDrive.goXYnTurnTask(0, 1.2, 0.6, targetVel, 0.02, 0.8, 5, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }

            mecanumDrive.goXYnTurnTask(parkPosition.angle, parkPosition.x, parkPosition.y, 1.0, 0.05, 0.8, 1, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }
        }
        else if(parkingPos == 2 && opModeIsActive()){

            //turn first
            mecanumDrive.goXYnTurnTask(0, 1.2, 0.0, targetVel, 0.02, 0.8, 5, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }

            mecanumDrive.goXYnTurnTask(parkPosition.angle, parkPosition.x, parkPosition.y, 1.0, 0.05, 0.6, 1, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }
        }else if(opModeIsActive()){

            mecanumDrive.goXYnTurnTask(90, 1.2, -0.6, targetVel, 0.02, 0.3, 1, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 29000) {
                updateTelemetry();
                sleep(10);
            }
            mecanumDrive.goXYnTurnTask(0, 1.2, -0.6, targetVel, 0.02, 0.8, 5, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }

            mecanumDrive.goXYnTurnTask(parkPosition.angle, parkPosition.x, parkPosition.y, 1.0, 0.05, 0.3, 1, true, true, 0);
            while (opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && timer.milliseconds() < 30000) {
                updateTelemetry();
                sleep(10);
            }
        }

         */

        updateTelemetry();

        //finish, stop all
        mecanumDrive.stopGoXYnTurn();
        chassis.stopRobot();
        //write current position, may only use with tracking wheel
        robotPos = odometry.getRobotPosition();
        writeSDCardFile(robotPos);

        odometry.stopThread();

    }

    public void updateTelemetry()
    {
        telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
        telemetry.addData("Timer:", "%f", timer.milliseconds());
        telemetry.addData("ParkingPos", " %d", parkingPos);
        telemetry.update();
    }

    private void hSliderGotoPickUpPos(int level, boolean wait)
    {

        if (opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            if (level == 5) {
                //h5
                hCheckPos = 0.85 + deltaDis;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(12);
                hSliderSystem.setPanAngle(hSliderPickPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 4) {
                //h4
                hCheckPos = 0.85 + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(7.96);
                hSliderSystem.setPanAngle(hSliderPickPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 3) {
                //h3
                hCheckPos = 0.85 + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(1.05);
                hSliderSystem.setPanAngle(hSliderPickPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 2) {
                //h2
                hCheckPos = 0.85 + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-7);
                hSliderSystem.setPanAngle(hSliderPickPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            } else if (level == 1) {
                //h1
                hCheckPos = 0.85 + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-13.00);
                hSliderSystem.setPanAngle(hSliderPickPanAngle);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 0) {
                //h1
                hCheckPos = 0.85;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(90);
                hSliderSystem.setPanAngle(0);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            }
            if (wait) {
                if (level != 0) {
                    timeOut.reset();
                    while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hCheckPos) > HSLIDER_HANDOVER_LEN && timeOut.milliseconds() < 1500) {
                        sleep(10);
                    }
                    sleep(300);
                }
            }
        }
    }

    public void gotoHandOverPos(){

        // pick up cone
        if (opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            vSliderSystem.sliderLenCtrl(VPICKUP_LEN, 1.0);
            vSliderSystem.setTiltAngle(VPICKUP_TILT_ANGLE);
            vSliderSystem.setPanAngle(VPICKUP_PAN_ANGLE);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);

            // take the cone first


            if (!foundCone) {
                hSliderSystem.scanToFindCone();
                timeOut.reset();
                while(opModeIsActive() && !hSliderSystem.isFoundCone() && timeOut.milliseconds() < 1000){
                    sleep(10);
                }

                foundCone = true;
                hSliderPickPanAngle = hSliderSystem.getPanAngle();

                hSliderSystem.stopScan();
            }
            double disSensor = hSliderSystem.leftSensor.getDistance(DistanceUnit.MM) > hSliderSystem.rightSensor.getDistance(DistanceUnit.MM) ?
                    hSliderSystem.rightSensor.getDistance(DistanceUnit.MM) : hSliderSystem.leftSensor.getDistance(DistanceUnit.MM);
            //55mm may be need change based on different level cone
            if (Math.abs(disSensor - 55) > 10 && Math.abs(disSensor - 55) < 80){

                hSliderSystem.sliderLenCtrl(hCheckPos + (( disSensor - 55) / 1000),1.0);
                sleep(200);
            }



            hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
            sleep(200);
            hSliderSystem.setTiltAngle(HMIDDLE_TILT_ANGLE);
            sleep(200);

            hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);
            timeOut.reset();
            while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - 0.01) > HSLIDER_HANDOVER_LEN && timeOut.milliseconds() < 3000){
                if(hSliderSystem.getSliderLen() < 0.2) hSliderSystem.setPanAngle(0);
                sleep(10);
            }

            //  sleep(200);
            hSliderSystem.setTiltAngle(HHANDOVER_TILT_ANGLE);
            sleep(300);


            hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
            timeOut.reset();
            while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() -VSLIDER_HANDOVER_LEN ) > 0.05 && timeOut.milliseconds() < 3000){
                sleep(10);
            }
            sleep(100);
            hSliderSystem.sliderLenCtrl(0.1, 1.0);

        }
    }

    public void handOverCone(){
        if(opModeIsActive()) {
            sleep(100);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
            sleep(200);
        }
    }

    public void vSliderGotoDropPos(int level, boolean wait){
        int vSliderHeight = 0;
        if(opModeIsActive()) {


            ElapsedTime timeOut = new ElapsedTime();
            vSliderSystem.setTiltAngle(vSliderHDropTiltAngle);
            sleep(100);
            vSliderSystem.sliderLenCtrl(vSliderHDropSliderLen, 1.0);
            timeOut.reset();
            while(opModeIsActive() && vSliderSystem.getSliderLen() < 0.23 && timeOut.milliseconds() < 1500){
                sleep(10);
            }
            vSliderSystem.setPanAngle(vSliderHDropPanAngle);
            timeOut.reset();
            while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - vSliderHDropSliderLen ) > 0.02 && timeOut.milliseconds() < 2000)
            {
                sleep(10);
            }
            sleep(200);

        }
    }

    public void vSliderDropCone()
    {
        if(opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            if (!foundPole){
                vSliderSystem.scanToFindJunction();
                timeOut.reset();
                while (opModeIsActive() && !vSliderSystem.isFoundJunction() && timeOut.milliseconds() < 2000){
                    sleep(10);
                }
                if (vSliderSystem.isFoundJunction()){
                    vSliderHDropPanAngle = vSliderSystem.getPanAngle();
                }
                else{
                    vSliderSystem.setPanAngle(vSliderHDropPanAngle);
                    sleep(200);
                }

                foundPole = true;

                vSliderSystem.stopScan();
            }

            vSliderSystem.setTiltAngle(vSliderDropTiltAngle);
            sleep(150);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
            sleep(150);
            vSliderSystem.setTiltAngle(vSliderHDropTiltAngle);
            sleep(100);
        }
    }

    private void writeSDCardFile(RobotPosition currentPos)
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        dir.mkdirs();
        File file = new File(dir, "parkingPos.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);


            printStream.println(Double.toString(currentPos.x));
            printStream.println(Double.toString(currentPos.y));
            printStream.println(Double.toString(currentPos.angle));
            //printStream.println("R");   // Right
            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }



}
