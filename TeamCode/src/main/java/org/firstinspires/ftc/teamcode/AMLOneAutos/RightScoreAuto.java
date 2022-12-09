package org.firstinspires.ftc.teamcode.AMLOneAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "RightScoreAuto", group = "Autonomous")

public class RightScoreAuto extends LinearOpMode {

    enum State {
        WAIT,
        STRAIGHT,
        ALIGN,
        POS_DOS,
        POS_UNO,
        POS_TRES,
        IDLE
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;
    public static double pos2X = 28;
    public static int pos2Y = 0;
    public static double pos2Angle = Math.toRadians(0);

    public static double pos1X = 28;
    public static double pos1Y = -22.43 * 2;
    public static double pos1Angle = Math.toRadians(0);

    public static double pos3X = 28;
    public static  double pos3Y = 24.51 * 2;
    public static double pos3Angle = Math.toRadians(0);

    public static double pushConeX = 60;
    public static double pushConeY = 0;
    public static double pushConeAngle = Math.toRadians(0);

    public static double alignStackX = 0.1;
    public static double alignStackY = 0.1;
    public static double getAlignStackAngle=Math.toRadians(270);
    public static double startWait = 0;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();
        Trajectory pushCone = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(pushConeX, pushConeY, pushConeAngle))
                .build();

        Trajectory align_stack = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(alignStackX, alignStackY, getAlignStackAngle))
                .build();

        Trajectory pos2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(pos2X, pos2Y, pos2Angle))
                .build();

        /*Trajectory pos1 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(pos1X, pos1Y, pos1Angle))
                .build();

        Trajectory pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(pos3X, pos3Y, pos1Angle))
                .build();*/
        currentState = State.WAIT;

        waitTimer.reset();

        int pos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ",pos);
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:
                    if (waitTimer.seconds() > startWait) {
                        currentState = State.STRAIGHT;
                    }
                    break;

                case STRAIGHT:
                    telemetry.addLine("Going Straight");
                    telemetry.update();
                    drive.followTrajectory(pushCone);
                    currentState = State.ALIGN;
                    break;

                case ALIGN:
                    telemetry.addLine(String.valueOf(posEstimate.getX()));
                    telemetry.update();
                    drive.followTrajectory(align_stack);
                    currentState = State.IDLE;
                    break;

/*                case POS_DOS:

                    drive.followTrajectory(pos2);

                    if (pos == 1) {

                        currentState = State.POS_UNO;
                    }

                    else if (pos == 3) {

                        currentState = State.POS_TRES;
                    }

                    else {

                        currentState = State.IDLE;
                    }

                    break;

                case POS_UNO:

                    drive.followTrajectory(pos1);

                    currentState = State.IDLE;

                    break;

                case POS_TRES:

                    drive.followTrajectory(pos3);

                    currentState = State.IDLE;

                    break;
*/
                case IDLE:

                    break;
            }
        }

    }

}




