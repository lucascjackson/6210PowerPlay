package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "LeftPosition", group = "Autonomous")
public class LeftPosition extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Movement move = new Movement(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        vuforia.SetIndicatorPixel();

        waitForStart();

        int pos = vuforia.LeftPostionVision();

        move.AML1Park(pos);
    }
}
