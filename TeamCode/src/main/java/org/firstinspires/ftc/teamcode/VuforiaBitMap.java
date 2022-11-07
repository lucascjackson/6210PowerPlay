package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

    public VuforiaBitMap(LinearOpMode robot) {
        this.robot = robot;
        //Create vuforia object
        this.vuforia = null;

        //Add initialization
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        //LogitechC310 = hardwareMap.get(WebcamName.class, "Logitech C310");

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

    //returns 1, 2, or 3 for the three possible capstone locations
    // 1 : left || 2 : mid || 3 : right
    // Resolution : 1280 X 720
    // RED CLOSE TO CAROUSEL COORDINATES : RIGHT (3) - 698, 600 || MID (2) - 232, 600
    // BLUE CLOSE TO CAROUSEL COORDINATES : LEFT (1) - || MID (2) -

    public int LeftPostionVision() throws InterruptedException{

        Bitmap bm = this.getBitmap();

        int blueValue = blue(bm.getPixel(midW, midH));
        int greenValue = green(bm.getPixel(midW, midH));
        int redValue = red(bm.getPixel(midW, midH));


        if (blueValue < 100 && greenValue < 100 && redValue < 100) {
            return 1;
        }
        if (blueValue < 120 && greenValue > 140 && redValue < 120 ) {
            return 2;
        }
        if (blueValue > 200 && greenValue > 200 && redValue < 100) {
            return 3;
        }

        return 3;
    }

    public void SetIndicatorPixel() throws InterruptedException {

        Bitmap bm = this.getBitmap();

        bm.setPixel(midW, midH, Color.rgb(255, 0, 255));

    }

}
