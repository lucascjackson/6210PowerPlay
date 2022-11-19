package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import com.qualcomm.robotcore.hardware.HardwareMap;



import java.util.ArrayList;
import java.util.Collections;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Bitmap;
import android.graphics.Color;

public class VuforiaBitMap {

    private LinearOpMode robot;
    private VuforiaLocalizer vuforia;

    private int redValue;
    private int blueValue;
    private int greenValue;
    double expectedRatio1 = 0.7214285714;
    double expectedRatio2 = 0.736;
   // double ratio1;
    double ratio;

    public VuforiaBitMap(LinearOpMode robot) {
        this.robot = robot;
        //Create vuforia object
        this.vuforia = null;

        //Add initialization
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        //localizer for webcam
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AcwUcZv/////AAABmcZmJOhJO0lDmuWB65t6j8YfeflGlsIu2d3qoec9xPdFe3HCmmfVbCJhV5xlLc6hwP47x69a0BxYDrvsJfv7L5Cjgf3daHpsyuahKkROo1ptoXfmMuJpLd1QFj8/DF0FIcmM9gxWpevecLMBnPVorHrE+JyVUiafGWGENxnEAb9HW+IxZ9eXIFjrZTbcUv7jTnD358PlITDeMouxj/pI7tegVuksVUlNhYBg420Oo1eGvqWB9b8Ikwy5VAahwIn2IDNj74q5Lo64MgLLPTDFDJN+BKwo5XERrU8ONXrGiPLSWBSjvOvM/joQVGqn7Z5bw6ApGT7r2b2VELqI45AZIXGaux8QAmk6JOf7ttdmyL9V";
        //paramWC.cameraName = LogitechC310;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        com.vuforia.Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
    }

    int midH = 0;

    int midW = 0;

    public Bitmap getBitmap() throws InterruptedException {
        Bitmap bm = null;
        Image rgb = null;

        // Grab frame
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        // Iterate through all the images
        long num = frame.getNumImages();
        for(int i = 0; i < num; i++){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                rgb = frame.getImage(i);
            }
        }

        midH = rgb.getHeight() / 2;

        midW = rgb.getWidth() / 2;

        // Create bitmap
        bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;

    }


    public int LeftPostionVision() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        blueValue = blue(bm.getPixel(midW, midH));
        greenValue = green(bm.getPixel(midW, midH));
        redValue = red(bm.getPixel(midW, midH));
        boolean posOne = false;
        boolean posTwo = false;

        ratio = (double)greenValue /(blueValue + redValue);


        double percentError1 = Math.abs((ratio - expectedRatio1)/ expectedRatio1);
        double percentError2 = Math.abs((ratio - expectedRatio2)/ expectedRatio2);

        if (percentError1 < .1) {
            posOne = true;
        }
        if (percentError2 < .1) {
            posTwo = true;
        }

        if (posOne && posTwo) {
            if (percentError1 < percentError2) {
                return 1;
            } else return 2;
        }

        if (posOne) {
            return 1;
        }

        if (posTwo) {
            return 2;
        }

        return  3;
    }

    public String colorFeedBack() {

        return "red: " + redValue + " green: " + greenValue + " blue: " + blueValue;

    }

    public String ratioFeedBack() {
        return "ratio1: " + ratio;
    }

}
