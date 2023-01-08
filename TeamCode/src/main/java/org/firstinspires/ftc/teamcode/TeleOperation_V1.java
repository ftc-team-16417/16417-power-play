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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;


@TeleOp(name="TeleOp_V1")
//@Disabled

public class TeleOperation_V1 extends LinearOpMode {
    ChassisSystem chassis = null;
    OdometryMotorEncoder odometry = null;
    MecanumXYTurnDriveLib mecanumDrive = null;

    HSliderSystem hSliderSystem = null;
    VSliderSystem vSliderSystem = null;
    RobotPosition robotPos = null;
    private double vPanDropAngle = -57; // left/right will be different

    //--------------------------------------------------------------------------------
    //init

    private double HPICKUP_LEN = 0.283;
    private double HPICKUP_TILT_ANGLE = -104.45;
    private double HPICKUP_PAN_ANGLE = 13.21;
    private double VPICKUP_LEN = 0.09;
    private double VPICKUP_TILT_ANGLE = -116.5;
    private double VPICKUP_PAN_ANGLE = -4.5;

    private double VDROP_LEN = 0.45;
    private double VDROP_PAN_ANGLE = -49.10;

    private double HPREPICKUP_LEN = 0.1;


    //-------------------------------------------------------------------------------------
    //Don't change Bf Variables (they store base values for poles)
    //Middle pole
    private double BfMidHPICKUP_LEN = 0.283;
    private double BfMidHPICKUP_TILT_ANGLE = -104.45;
    private double BfMidHPICKUP_PAN_ANGLE = 13.21;
    private double BfMidVPICKUP_LEN = 0.09;//test
    private double BfMidVPICKUP_TILT_ANGLE = -116.5;//test
    private double BfMidVPICKUP_PAN_ANGLE = -4.5;//test

    private double BfMidVDROP_LEN = 0.442;
    private double BfMidVDROP_PAN_ANGLE = -44.74;
    private double BfMidHPREPICKUP_LEN = 0.1;

    //Left pole
    //NA for now

    //Right pole

    private double BfRightHPICKUP_LEN = 0.900;
    private double BfRightHPICKUP_TILT_ANGLE = -106.01;
    private double BfRightHPICKUP_PAN_ANGLE = 0;
    private double BfRightVPICKUP_LEN = 0.09;//test
    private double BfRightVPICKUP_TILT_ANGLE = -116.5;//test
    private double BfRightVPICKUP_PAN_ANGLE = -4.5;//test


    private double BfRightVDROP_LEN = 0.434;
    private double BfRightVDROP_PAN_ANGLE = 36.94;
    private double BfRightHPREPICKUP_LEN = 0.83;
    //-------------------------------------------------------------------------------------
    //universal values
    private double HMIDDLE_TILT_ANGLE = 17.72;
    private double HHANDOVER_TILT_ANGLE = 44.44;
    private double HHANDOVER_PAN_ANGLE = 5.11;

    private double VSLIDER_HANDOVER_LEN = 0.02;
    private double HSLIDER_HANDOVER_LEN = 0.056;

    private double VPREDROP_TILT_ANGLE = 74.92;

    private double VDROP_TILT_ANGLE = 90;

    private double VMIDDLEPOL_DROP_LEN = 0.14;

    //---------------------------------------------------------------------------------

    private int pickLevel = 1;
    private int dropLevel = 3;  //high junction

    private int polePosition = 2; //middle pole

    String posFlag = "R";
    private double MidTargetX = 0.68;
    private double MidTargetY = 0.65;
    private double MidTargetA = 1.6;

    private double LeftTargetX = 0;
    private double LeftTargetY = 0;
    private double LeftTargetA = 0;

    private double RightTargetX = 0;
    private double RightTargetY = 0;
    private double RightTargetA = 0;

    @Override
    public void runOpMode(){


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(hardwareMap, telemetry);

        robotPos = new RobotPosition(0.0,0.0,0);
        //read back the robot position from autonomouse
        /*if (!readSDCardFile()){
            robotPos = new RobotPosition(0,0,0);
        }*/


        if (posFlag.startsWith("R")){
            MidTargetX = 0.68;
            MidTargetY = 0.65;
            MidTargetA = 1.6;
            vPanDropAngle = -49.10;
        }
        else{
            MidTargetX = 0.65;
            MidTargetY = -0.7;
            MidTargetA = 0;
            vPanDropAngle = 49.10;   //need test
        }
        VDROP_PAN_ANGLE = -49.10;//vPanDropAngle;

        telemetry.addData("Pos ( ", "%2.2f, %2.2f, %2.2f )", robotPos.x, robotPos.y, robotPos.angle);


        odometry = new OdometryMotorEncoder(this, hardwareMap, chassis,robotPos, telemetry, false);
        sleep(1000);
        telemetry.addLine("odometry is setup");

        mecanumDrive = new MecanumXYTurnDriveLib(this, chassis, odometry,telemetry);


        hSliderSystem = new HSliderSystem(this, hardwareMap, telemetry, false);
        sleep(200);

        vSliderSystem = new VSliderSystem(this, hardwareMap, telemetry, false);


        telemetry.addData("Working Mode", "waiting for start " + posFlag);
        telemetry.update();

        waitForStart();

        // reset claw to right position first, only for this tele demo program
        // vSliderSystem.scanToFindCone();
        while (opModeIsActive()){
            robotPos = odometry.getRobotPosition();
            if (mecanumDrive.isGoXYnTurnFinished()) {
                chassis.chassisTeleOp(gamepad1);
            }

            hSliderSystem.joystickCtrl(gamepad1, gamepad2);

            vSliderSystem.joystickCtrl(gamepad1, gamepad2);


            if (gamepad1.left_bumper){
                //move to prepare position
                mecanumDrive.goXYnTurnTask(MidTargetA, MidTargetX, MidTargetY, 1.0, 0.02, 0.8, 2, true, true, 0);
                while(opModeIsActive() && !mecanumDrive.isGoXYnTurnFinished() && gamepad1.right_trigger < 0.5){
                    sleep(10);
                }
                mecanumDrive.stopGoXYnTurn();
            }

            if (gamepad1.dpad_left){
                polePosition = 1;
                poleValue(polePosition);
            }
            else if (gamepad1.dpad_down){
                polePosition = 2;
                poleValue(polePosition);
            }
            else if (gamepad1.dpad_right){
                polePosition = 3;
                poleValue(polePosition);
            }

            //for preset position test
            if (gamepad2.dpad_up)
            {
                //high junction

                vSliderSystem.sliderLenCtrl(VDROP_LEN, 1.0);
                vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
                vSliderSystem.setPanAngle(0);


            }else if(gamepad2.dpad_right)
            {
                //middle junction
                vSliderSystem.sliderLenCtrl(VMIDDLEPOL_DROP_LEN, 1.0);
                vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
                vSliderSystem.setPanAngle(0);


            }
            else if(gamepad2.dpad_down)
            {
                //low junction
                vSliderSystem.sliderLenCtrl(0, 1.0);
                vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
                vSliderSystem.setPanAngle(0);

            }
            else if(gamepad2.dpad_left)
            {
                //reset all the default value
                /////////////////////////////////////////////////////////////////////
                HPICKUP_LEN = 0.38;
                HPICKUP_TILT_ANGLE = -10;
                HPICKUP_PAN_ANGLE = 0;
                VPICKUP_LEN = 0.23;
                VPICKUP_TILT_ANGLE = -134;
                VPICKUP_PAN_ANGLE = 0;

                HMIDDLE_TILT_ANGLE = 90;
                HHANDOVER_TILT_ANGLE = 134;


                VPREDROP_TILT_ANGLE = 59;
                VDROP_LEN = 0.41;
                VDROP_PAN_ANGLE = vPanDropAngle;  //-57
                VDROP_TILT_ANGLE = 80;
                VSLIDER_HANDOVER_LEN = 0.16;
            }

            if (gamepad2.y)
            {

                // get ready to pickup , the claw is outside the substation
                getReadyPickup();

            }

            if (gamepad2.b) {
                pickupToUnicorn(true);
            }

            if(gamepad2.a)
            {
                gotoDropPos(true);
            }
            if(gamepad2.x){
                dropCone(true);
            }




            if (gamepad2.start){

                for (int i = 0; i < 20 && !gamepad2.back; i ++)
                {
                    if (opModeIsActive() && !gamepad2.back)
                    {
                        getReadyPickup();
                        sleep(150);
                        pickupToUnicorn(false);
                        sleep(200);
                       // sleep(1000);
                        if (!gamepad2.back){
                            gotoDropPos(false);
                            sleep(300);
                            dropCone(false);
                            //sleep(100);
                        }
                        else{
                            i = 20;
                        }
                    }

                }

            }

            if (gamepad1.start){

                vSliderSystem.scanToFindJunction();

            }
            else{
                vSliderSystem.stopScan();
            }
            if (gamepad1.back){
                hSliderSystem.scanToFindCone();
            }
            else{
                hSliderSystem.stopScan();
            }


            robotPos = odometry.getRobotPosition();

            telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
            telemetry.addData("encoder", "Left:%d Right:%d Middle:%d",
                    odometry.leftEnc.getCurrentPosition(),
                    odometry.rightEnc.getCurrentPosition(),
                    odometry.middleEnc.getCurrentPosition());

            telemetry.addData("HSliderLen", "%2.3f", hSliderSystem.getSliderLen());
            telemetry.addData("HSliderTiltA", "%2.2f", hSliderSystem.getTiltAngle());
            telemetry.addData("HSliderPanA", "%2.2f", hSliderSystem.getPanAngle());
            telemetry.addData("HSliderClawA", "%2.2f", hSliderSystem.getClawAngle());


            telemetry.addData("VSliderLen", "%2.3f", vSliderSystem.getSliderLen());
            telemetry.addData("VSliderTiltA", "%2.2f", vSliderSystem.getTiltAngle());
            telemetry.addData("VSliderPanA", "%2.2f", vSliderSystem.getPanAngle());
            telemetry.addData("VSliderClawA", "%2.2f", vSliderSystem.getClawAngle());

            telemetry.addData("PickUpLevel", "%d", pickLevel );

            telemetry.addData("hLeftDis", "%.2f", hSliderSystem.leftSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("hRightDis", "%.2f", hSliderSystem.rightSensor.getDistance(DistanceUnit.MM));

            telemetry.addData("vLeftDis", "%.2f", vSliderSystem.leftSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("vRightDis", "%.2f", vSliderSystem.rightSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("vMiddleDis", "%.2f", vSliderSystem.middleSensor.getDistance(DistanceUnit.MM));

            telemetry.update();

            sleep(10);
            //idle();
        }

        mecanumDrive.stopGoXYnTurn();
        chassis.stopRobot();

        odometry.stopThread();

    }

    private void getReadyPickup()   //Y
    {
        // h

        hSliderSystem.sliderLenCtrl(HPICKUP_LEN, 1.0);
        hSliderSystem.setTiltAngle(HPICKUP_TILT_ANGLE);
        hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

        //v

        vSliderSystem.sliderLenCtrl(VPICKUP_LEN, 1.0);
        vSliderSystem.setTiltAngle(VPICKUP_TILT_ANGLE);
        vSliderSystem.setPanAngle(VPICKUP_PAN_ANGLE);
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
    }

    private void pickupToUnicorn(boolean save)   //B
    {
        //h
        //save position first
        if(save) {
            HPICKUP_LEN = hSliderSystem.getSliderLen();
            HPICKUP_TILT_ANGLE = hSliderSystem.getTiltAngle();
            HPICKUP_PAN_ANGLE = hSliderSystem.getPanAngle();
        }

        hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
        sleep(100);
        hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
        hSliderSystem.setTiltAngle(HMIDDLE_TILT_ANGLE);
        hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);
        while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - 0.01) > HSLIDER_HANDOVER_LEN){
            sleep(10);
        }
        hSliderSystem.setTiltAngle(HHANDOVER_TILT_ANGLE);
        sleep(150);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
        vSliderSystem.sliderLenCtrl(VSLIDER_HANDOVER_LEN, 1.0);
        while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() -VSLIDER_HANDOVER_LEN ) > HSLIDER_HANDOVER_LEN){
            sleep(10);
        }
        hSliderSystem.sliderLenCtrl(0.1, 1.0);
    }




    void gotoDropPos(boolean save){  // A

        hSliderSystem.sliderLenCtrl(HPREPICKUP_LEN, 1.0);
        hSliderSystem.setTiltAngle(HPICKUP_TILT_ANGLE);
        hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
        if(save){
            VSLIDER_HANDOVER_LEN = vSliderSystem.getSliderLen();
            VPICKUP_TILT_ANGLE = vSliderSystem.getTiltAngle();
            VPICKUP_PAN_ANGLE = vSliderSystem.getPanAngle();

        }
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
        sleep(150);
        vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
        sleep(100);
        vSliderSystem.sliderLenCtrl(VDROP_LEN, 1.0);
        while(vSliderSystem.getSliderLen() < 0.25){
            sleep(10);
        }
        vSliderSystem.setPanAngle(VDROP_PAN_ANGLE);


    }

    private void dropCone(boolean save)  // X
    {

        if (save){
            VDROP_LEN = vSliderSystem.getSliderLen();
            VPREDROP_TILT_ANGLE = vSliderSystem.getTiltAngle();
            VDROP_PAN_ANGLE = vSliderSystem.getPanAngle();
        }
        vSliderSystem.setTiltAngle(VDROP_TILT_ANGLE);
        sleep(150);
        vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
        sleep(150);
        vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
        sleep(75);
        hSliderSystem.sliderLenCtrl(HPICKUP_LEN, 1.0);
        hSliderSystem.setTiltAngle(HPICKUP_TILT_ANGLE);
        hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
    }



    private boolean readSDCardFile()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"parkingPos.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    robotPos.x = Double.parseDouble(line);
                }else if(lineCnt == 1){
                    robotPos.y = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    robotPos.angle = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    posFlag = line;
                }

                lineCnt ++;
            }
            telemetry.update();
            br.close();
            return true;
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
            return false;
        }
    }

    private void poleValue(int _polePosition){
        // 1 = left, 2 = middle, 3 = right
        if (_polePosition == 1){
            //Change these when left pole values are found
            //Is set as middle pole right now
            HPICKUP_LEN = BfMidHPICKUP_LEN;
            HPICKUP_TILT_ANGLE = BfMidHPICKUP_TILT_ANGLE;
            HPICKUP_PAN_ANGLE = BfMidHPICKUP_PAN_ANGLE;

            VDROP_LEN = BfMidVDROP_LEN;
            VDROP_PAN_ANGLE = BfMidVDROP_PAN_ANGLE;
            HPREPICKUP_LEN = BfMidHPREPICKUP_LEN;
        }
        else if (_polePosition == 2){
            HPICKUP_LEN = BfMidHPICKUP_LEN;
            HPICKUP_TILT_ANGLE = BfMidHPICKUP_TILT_ANGLE;
            HPICKUP_PAN_ANGLE = BfMidHPICKUP_PAN_ANGLE;

            VDROP_LEN = BfMidVDROP_LEN;
            VDROP_PAN_ANGLE = BfMidVDROP_PAN_ANGLE;
            HPREPICKUP_LEN = BfMidHPREPICKUP_LEN;
        }
        else{
            HPICKUP_LEN = BfRightHPICKUP_LEN;
            HPICKUP_TILT_ANGLE = BfRightHPICKUP_TILT_ANGLE;
            HPICKUP_PAN_ANGLE = BfRightHPICKUP_PAN_ANGLE;

            VDROP_LEN = BfRightVDROP_LEN;
            VDROP_PAN_ANGLE = BfRightVDROP_PAN_ANGLE;
            HPREPICKUP_LEN = BfRightHPREPICKUP_LEN;
        }
    }
}
