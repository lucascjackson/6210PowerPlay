package org.firstinspires.ftc.teamcode.AMLOneAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VuforiaBitMap3;

@Config
@Autonomous(name = "VisionTest", group = "Autonomous")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        VuforiaBitMap3 vuforia = new VuforiaBitMap3(this);

        telemetry.addData("Position Detected: ", vuforia.leftPositionVision());
        telemetry.addData("Color Feedback: ", vuforia.colorFeedBack());
        telemetry.update();

        waitForStart();

    }
}

