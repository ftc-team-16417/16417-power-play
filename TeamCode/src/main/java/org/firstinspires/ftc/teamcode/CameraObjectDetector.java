package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class CameraObjectDetector {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AUQob4f/////AAABmQ82GhD2+k9wr5jVo5UQGnBH/bCXN5Wrz779hXv0waiwot5yzzqtV6VN/g5J4CNGkuQZBL5n2OHp/Gaxs9iBlCm+/gnzFueYuzatB1DORan9mh1PlF89vFtvWfJ1Bz0TUJHNMUv8zlbwpiHSJlZld6pwdyMxfs9UNp/F2CE0hCJ/sCU3dYh7crxnPmOpFJnekTtOWiNZnWx3xk0ZJIm7VCTF+6bsOIrUZYU/X0XHRnkRkGLiqJ1+4a2VSqR+lq6An8m3qKfXuC5nSiGAJFoEWcLdHLPNE+2LcUZHaJQYDLfa5XVl8JQBbQytwIhEOGBeqWMSAWyhCCVTobBNus1/wSy5oz9szorVMRgJBp6XVH9V";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private HardwareMap hwMap;
    private Telemetry telemetry;

    private int teamObjectPosition;
    private LinearOpMode mainTask = null;
    public CameraObjectDetector(LinearOpMode mainTask, HardwareMap hwMap, Telemetry telemetry)
    {
        this.mainTask = mainTask;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.8, 1.5 / 1);

        }

    }
    private void initVuforia () {
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.*/
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod () {
        //     Initialize the TensorFlow Object Detection engine.
        //     0: Ball,  1: Cube, 2: Duck, 3: Marker (duck location tape marker)
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minResultConfidence = 0.3f;

        tfodParameters.isModelTensorFlow2 = true;

        tfodParameters.inputSize = 300;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public int identifyTeamObject( ) {

        int trialCount = 0;
        int boltCnt = 0;
        int bulbCnt = 0;
        int panelCnt = 0;


        // only detect 3 frame, is it enough?
        while (mainTask.opModeIsActive() && trialCount < 3) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    trialCount++;

                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        if (recognition.getLabel().equals("1 Bolt")) {
                            boltCnt++;
                        } else if (recognition.getLabel().equals("2 Bulb")) {
                            bulbCnt++;
                        } else if (recognition.getLabel().equals("3 Panel")) {
                            panelCnt++;
                        }
                        telemetry.addData("- Cnt", "%d / %d / %d", boltCnt, bulbCnt, panelCnt);
                    }
                    telemetry.update();

                }
            }
        }

        if (panelCnt >= boltCnt && panelCnt >= bulbCnt)
        {
            return 3; /// location 3
        }else if (boltCnt >= panelCnt && boltCnt >= bulbCnt)
        {
            return 1; /// location 1
        }else // if (bulbCnt >= panelCnt && bulbCnt >= boltCnt)
        {
            return 2; /// location 3
        }


    }



}
