/*package org.firstinspires.ftc.teamcode;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaBitMap {

    private LinearOpMode robot;
    private VuforiaLocalizer vuforia;

    //public VuforiaBitMap(LinearOpMode robot) {
        //this.robot = robot;
        //Create vuforia object
        //this.vuforia = null;

        //Add initialization
        //int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        //LogitechC310 = hardwareMap.get(WebcamName.class, "Logitech C310");

        //localizer for webcam
        //VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQt2xVL/////AAABmXIVKUnTcEJbqvVBjp/Sw/9SqarohYyKotzRjT/Xl1/S8KDwsFHv/zYw6rXqXTjKrnjk92GfBA4hbZaQP17d1N6BiBuXO2W/hFNoMGxiF+fWlnvtDmUM1H/MF9faMOjZcPNjnQ7X8DVwdDDha3A3aqaoegefkKxb4A5EjP8Xcb0EPJ1JA4RwhUOutLbCDJNKUq6nCi+cvPqShvlYTvXoROcOGWSIrPxMEiOHemCyuny7tJHUyEg2FTd2upiQygKAeD+LN3P3cT02aK6AJbQ0DlQccxAtoo1+b//H6/eGro2s0fjxA2dH3AaoHB7qkb2K0Vl7ReFEwX7wmqJleamNUG+OZu7K3Zm68mPudzNuhAWQ";
        //paramWC.cameraName = LogitechC310;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        com.vuforia.Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
    }

    public Bitmap getBitmap() throws InterruptedException {
        Bitmap bm = null;
        Image rgb = null;

        // Grab frame
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        // Iterate through all the images
        long num = frame.getNumImages();
        for (int i = 0; i < num; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
            }
        }

        // Create bitmap
        bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;

    }

    // IDENTIFIES AND CHECKS COLOR
    public int right() throws InterruptedException {

        Bitmap bm = this.getBitmap();

        //checks for the RED value of three selected pixels on the camera
        int p3Red = red(bm.getPixel(0, 0));
        int p2Red = red(bm.getPixel(0, 0));
        int p1Red = red(bm.getPixel(0, 0));
        //checks for the GREEN value of three selected pixels on the camera
        int p3Green = green(bm.getPixel(0, 0));
        int p2Green = green(bm.getPixel(0, 0));
        int p1Green = green(bm.getPixel(0, 0));


        //threshold values for capstone color
        if (p3Red < 0 && p3Green < 0) return 3;
        if (p2Red < 1 && p2Green < 1) return 2;
        if (p1Red < 2 && p1Green < 2) return 1;
        else return 1;
    }
        // IDENTIFIES AND CHECKS COLOR
    public int left() throws InterruptedException {

        Bitmap bm = this.getBitmap();
        //checks for the RED value of three selected pixels on the camera
        int p3Red = red(bm.getPixel(0, 0));
        int p2Red = red(bm.getPixel(0, 0));
        int p1Red = red(bm.getPixel(0, 0));
        //checks for the GREEN value of three selected pixels on the camera
        int p3Green = green(bm.getPixel(0, 0));
        int p2Green = green(bm.getPixel(0, 0));
        int p1Green = green(bm.getPixel(0, 0));


        //threshold values for capstone color, returning integer based on the colour value detected
        if (p3Red < 0 && p3Green < 0) return 3;
        if (p2Red < 1 && p2Green < 1) return 2;
        if (p1Red < 2 && p1Green < 2) return 1;
        else return 1;
    }

}
*/


