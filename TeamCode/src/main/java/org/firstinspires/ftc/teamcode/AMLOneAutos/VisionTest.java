package org.firstinspires.ftc.teamcode.AMLOneAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.VuforiaBitMap2;

@Config
@Autonomous(name = "VisionTest", group = "Autonomous")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{

        VuforiaBitMap2 vuforia = new VuforiaBitMap2(this);

        telemetry.addData("Position Detected: ", vuforia.LeftPostionVision());
        telemetry.addData("Color Feedback: ", vuforia.colorFeedBack());
        telemetry.update();

        waitForStart();

    }
}

