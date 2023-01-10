package org.firstinspires.ftc.teamcode.AMLTresAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "LeftAutoPushPark", group = "Autonomous")

public class LeftAutoPushPark extends LinearOpMode {

    enum State {

        IDLE,
        WAIT,
        PUSH_CONE,
        ALIGN,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double pushConeX = 0;
    public static double pushConeY = -40;
    public static double pushConeAngle = 0;

    public static double align1x = 12;
    public static double align1y = -40;
    public static double align1angle = 0;

    public static double align2x = 31;
    public static double align2y = -40;
    public static double align2angle = Math.toRadians(-258);




    public static double startWait = 0;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        int pos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory pushCone = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(pushConeX, pushConeY,pushConeAngle ))
                .build();


        Trajectory align1 = drive.trajectoryBuilder(pushCone.end())
                .lineToLinearHeading(new Pose2d(align1x, align1y, align1angle))
                .build();


        Trajectory align2 = drive.trajectoryBuilder(align1.end())
                .lineToLinearHeading(new Pose2d(align2x, align2y, align2angle))
                .build();


        Trajectory pos_dos = drive.trajectoryBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(align2x, (align2y+20), align2angle))
                .build();

        Trajectory pos_tres = drive.trajectoryBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(align2x, (align2y+(23*2)), align2angle))
                .build();


        currentState = State.WAIT;



        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = State.PUSH_CONE;
                    }

                    break;

                case PUSH_CONE:

                    drive.followTrajectory(pushCone);
                    currentState = State.ALIGN;


                    break;

                case ALIGN:
                    drive.followTrajectory(align1);
                    drive.followTrajectory(align2);


                    currentState = State.PARK;


                    break;

                case PARK:
                    if (pos==1){
                        currentState = State.IDLE;
                    }
                    else if (pos==2){
                        drive.followTrajectory(pos_dos);
                        currentState = State.IDLE;
                    }
                    else{
                        drive.followTrajectory(pos_tres);
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:

                    break;
            }
        }
    }
}