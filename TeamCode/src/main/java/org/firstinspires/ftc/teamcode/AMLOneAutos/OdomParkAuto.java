package org.firstinspires.ftc.teamcode.AMLOneAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "OdomParkAuto", group = "Autonomous")
public class OdomParkAuto extends LinearOpMode {

    int pos2X = 36;
    int pos2Y = 0;
    double pos2Angle = Math.toRadians(270);

    int pos1X = 36;
    int pos1Y = -24;
    double pos1Angle = Math.toRadians(270);

    int pos3X = 36;
    int pos3Y = 24;
    double pos3Angle = Math.toRadians(270);

    Pose2d startPose = new Pose2d(9, 0, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // TODO create manipulotors
        VuforiaBitMap vuforia = new VuforiaBitMap(this);
        int pos = vuforia.LeftPostionVision();


        drive.setPoseEstimate(startPose);

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory pos2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(pos2X, pos2Y, pos2Angle))
                .build();

        Trajectory pos1 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(pos1X, pos1Y, pos1Angle))
                .build();

        Trajectory pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(pos3X, pos3Y, pos1Angle))
                .build();


        drive.followTrajectoryAsync(pos2);

        if (pos == 1) {
            drive.followTrajectoryAsync(pos1);
        } else if (pos == 3) {
            drive.followTrajectoryAsync(pos3);
        }

        drive.update();
    }
}
