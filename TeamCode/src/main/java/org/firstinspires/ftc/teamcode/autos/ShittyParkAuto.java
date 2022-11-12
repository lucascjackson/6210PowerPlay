package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;

@Config
@Autonomous(name = "ShittyParkAuto", group = "Autonomous")
public class ShittyParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Movement move = new Movement(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        int pos = vuforia.LeftPostionVision();

        telemetry.addData("",vuforia.colorFeedBack());
        telemetry.addData("pos: ", pos);
        telemetry.addData("", vuforia.ratioFeedBack());
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            move.AML1Park(2);
        }
    }
}
