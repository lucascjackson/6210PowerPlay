package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;


import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;

public class VuforiaBitMap {

    private LinearOpMode robot;
    private VuforiaLocalizer vuforia;

    private int redValue;
    private int blueValue;
    private int greenValue;
    private int redValue2;
    private int blueValue2;
    private int greenValue2;
    private int redValue3;
    private int blueValue3;
    private int greenValue3;

    double expectedRatio1 = 134;
    double expectedRatio2 = 7;
    double expectedRatio3 = 19;

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

    int detectH = 0;
    int detectW = 0;

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

        detectH = (rgb.getHeight() / 2)+120;

        detectW = (rgb.getWidth() / 2);

        // Create bitmap
        bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;

    }


    public int LeftPostionVision() throws InterruptedException {

        Bitmap bm = this.getBitmap();

        int detectRange = 5;
        blueValue = blue(bm.getPixel(detectW, detectH))
                + blue(bm.getPixel(detectW, detectH + detectRange))
                + blue(bm.getPixel(detectW, detectH - detectRange));
        greenValue = green(bm.getPixel(detectW, detectH))
                + green(bm.getPixel(detectW, detectH + detectRange))
                + green(bm.getPixel(detectW, detectH - detectRange));
        redValue = red(bm.getPixel(detectW, detectH))
                + red(bm.getPixel(detectW, detectH + detectRange))
                + red(bm.getPixel(detectW, detectH - detectRange));


        blueValue /= 3;
        greenValue /= 3;
        redValue /= 3;

        boolean posOne = false;
        boolean posTwo = false;
        boolean posThree = false;

        if ((greenValue - redValue/ redValue < 0.08)) {

            return 3;
        }

        else if (greenValue > redValue) {

            return 1;
        }

        else {

            return 2;
        }

    }


        //ratio = (double) (blueValue*greenValue)/((redValue) + 1);

        /*
        double percentError2 = Math.abs((ratio - expectedRatio2)/ expectedRatio2);
        double percentError3 = Math.abs((ratio - expectedRatio3)/ expectedRatio3);


        if (percentError2 < .3) {
            posTwo = true;
        }
        if (percentError3 < .3) {
            posThree = true;
        }

        if (posTwo && posThree) {
            if (percentError2 < percentError3) {
                return 2;
            } else return 3;
        }

        if (posTwo) {
            return 2;
        }

        if (posThree) {
            return 3;
        }

        return  1;
    }

*/

    public String colorFeedBack() {

        return "red: " + redValue + " green: " + greenValue + " blue: " + blueValue;
        //return "midW: " + midW + " midH: " + midH + " useless: " + blueValue;
    }

    public String ratioFeedBack() {
        return "ratio: ";
    }



}
