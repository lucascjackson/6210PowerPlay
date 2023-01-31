package org.firstinspires.ftc.teamcode.Almost;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AMLTresAuto.BigConeAutoRight;
import org.firstinspires.ftc.teamcode.AMLTresAuto.LeftAutoPushPark;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "lowJunctionRed", group = "Autonomous")

public class lowJunctionRed extends LinearOpMode{

    Manipulators manip;
    private DcMotor lift2;
    public DigitalChannel liftSensor;

    enum State {

        IDLE,
        WAIT,
        GO_TO_STACK,
        GET_CONE,
        SCORE_CONE,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double goToWallX = 70;
    public static double goToWallY = 0;
    public static double goToWallAngle = 0;

    public static double moveAlongWallX = 70;
    public static double moveAlongWallY = 0;
    public static double moveAlongWallAngle = 0;

    public static double turnToAlignX = 70;
    public static double turnToAlignY = 0;
    public static double turnToAlignAngle = 0;

    public static double moveToPoleX = 70;
    public static double moveToPoleY = 0;
    public static double moveToPoleAngle = 0;

    public static double parkUnoX = 70;
    public static double parkUnoY = 0;
    public static double parkUnoAngle = 0;

    public static double parkDosX = 70;
    public static double parkDosY = 0;
    public static double parkDosAngle = 0;

    public static double parkTresX = 70;
    public static double parkTresY = 0;
    public static double parkTresAngle = 0;

    public static double startWait = 0;

    lowJunctionRed.State currentState = lowJunctionRed.State.IDLE;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        manip = new Manipulators(hardwareMap);
        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory goToWall = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY, goToWallAngle))
                .build();
        telemetry.addLine("T1");
        telemetry.update();

        Trajectory moveAlongWall = drive.trajectoryBuilder(goToWall.end())
                .lineToLinearHeading(new Pose2d(moveAlongWallX, moveAlongWallY, moveAlongWallAngle))
                .build();
        telemetry.addLine("T2");
        telemetry.update();

        Trajectory turnToAlign = drive.trajectoryBuilder(moveAlongWall.end())
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();
        telemetry.addLine("T3");
        telemetry.update();

        Trajectory moveToPole = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(moveToPoleX, moveToPoleY, moveToPoleAngle))
                .build();
        telemetry.addLine("T4");
        telemetry.update();

        Trajectory parkUno = drive.trajectoryBuilder(moveToPole.end())
                .lineToLinearHeading(new Pose2d(parkUnoX, parkUnoY, parkUnoAngle))
                .build();
        telemetry.addLine("T5");
        telemetry.update();

        Trajectory parkDos = drive.trajectoryBuilder(moveToPole.end())
                .lineToLinearHeading(new Pose2d(parkDosX, parkDosY, parkDosAngle))
                .build();
        telemetry.addLine("T6");
        telemetry.update();

        Trajectory parkTres = drive.trajectoryBuilder(moveToPole.end())
                .lineToLinearHeading(new Pose2d(parkTresX, parkTresY, parkTresAngle))
                .build();
        telemetry.addLine("T7");
        telemetry.update();

        currentState = lowJunctionRed.State.WAIT;
        telemetry.addData("trajectories", "built");
        waitTimer.reset();

        int pos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = lowJunctionRed.State.GO_TO_STACK;
                    }

                    break;

                case GO_TO_STACK:

                    drive.followTrajectory(goToWall);
                    drive.followTrajectory(moveAlongWall);
                    currentState = lowJunctionRed.State.GET_CONE;
                    break;

                case GET_CONE:

                    drive.followTrajectory(turnToAlign);
                    currentState = lowJunctionRed.State.SCORE_CONE;
                    break;

                case SCORE_CONE:

                    drive.followTrajectory(moveToPole);
                    currentState = lowJunctionRed.State.PARK;
                    break;

                case PARK:

                    if (pos==1){
                        drive.followTrajectory(parkUno);
                        currentState = lowJunctionRed.State.IDLE;
                    }
                    else if (pos==2){
                        drive.followTrajectory(parkDos);
                        currentState = lowJunctionRed.State.IDLE;
                    }
                    else{
                        drive.followTrajectory(parkTres);
                        currentState = lowJunctionRed.State.IDLE;
                    }
                    break;

                case IDLE:

                    break;


            }
        }
    }
}