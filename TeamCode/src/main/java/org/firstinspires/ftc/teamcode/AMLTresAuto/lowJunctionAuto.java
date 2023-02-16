package org.firstinspires.ftc.teamcode.AMLTresAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap3;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        GO_TO_WALL,
        GO_TO_STACK,
        TURN_TO_ALIGN_STACK,
        GO_FORWARD,
        SCORE_LOW,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    Pose2d startPose2 = new Pose2d(0, 0, Math.toRadians(180));
    Pose2d startPose3 = new Pose2d(0, 0, Math.toRadians(0));

    Pose2d posEstimate;

    public static double accuracyVar = 0.5;

    public static double goToWallX = -3;
    public static double goToWallY = -70;
    public static double gotoWallAngle = Math.toRadians(180);

    public static double goToStackX =-38;
    public static double goToStackY = -70;
    public static double goToStackAngle = Math.toRadians(180);

    public static double turnToAlignX = 2;
    public static double turnToAlignY = 5;
    public static double turnToAlignAngle = 3.55;

    public static double faceWallX = 0;
    public static double faceWallY = 10;
    public static double faceWallAngle = 5.236;

    public static double pickUpConeX = -2;
    public static double pickUpConeY = 23;
    public static double pickUpConeAngle = 5.29;

    public static double moveBack = 4;

    public static double scoreLowX = -4;
    public static double scoreLowY = 4;
    public static double scoreLowAngle = Math.toRadians(2.18166);

    public static double turnBackX = -4;
    public static double turnBackY = 0;
    public static double turnBackAngle= 0;

    public static double moveForward = 4;

    public static double turnFromScoreX = 0;
    public static double turnFromScoreY = 0;
    public static double turnFromScoreAngle = Math.toRadians(0);

    public static double parkUnoX = 0;
    public static double parkUnoY = 0;
    public static double parkUnoAngle = Math.toRadians(0);

    public static double parkDosX = 0;
    public static double parkDosY = 0;
    public static double parkDosAngle = Math.toRadians(0);

    public static double parkTresX = 0;
    public static double parkTresY = 0;
    public static double parkTresAngle = Math.toRadians(0);

    public static double startWait = 0;

    int cycleCount;

    lowJunctionAuto.State currentState = lowJunctionAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap3 vuforia = new VuforiaBitMap3(this);
        Movement move = new Movement(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        int pos = (int) vuforia.leftPositionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory goToWall = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY, gotoWallAngle))
                .build();


        Trajectory goToStack = drive.trajectoryBuilder(goToWall.end())
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();

//fortnte battlepass
        Trajectory turnToAlign = drive.trajectoryBuilder(startPose2)
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory faceWall = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(faceWallX, faceWallY, faceWallAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(faceWall.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        Trajectory scoreLow = drive.trajectoryBuilder(startPose3)
                .back(moveBack)
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        Trajectory turnFromScore = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(turnFromScoreX, turnFromScoreY, turnFromScoreAngle))
                .forward(moveForward)
                .build();
/*
        Trajectory pos_uno = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkUnoX, (parkUnoY), parkUnoAngle))
                .build();

        Trajectory pos_dos = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkDosX, (parkDosY), parkDosAngle))
                .build();

        Trajectory pos_tres = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkTresX, (parkTresY), parkTresAngle))
                .build();
        */

        currentState = lowJunctionAuto.State.WAIT;



        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = lowJunctionAuto.State.GO_TO_WALL;
                    }

                    break;

                case GO_TO_WALL:

                    drive.followTrajectory(goToWall);

                    currentState = State.GO_TO_STACK;

                    break;

                case GO_TO_STACK:
                    drive.followTrajectory(goToStack);
                    currentState = State.TURN_TO_ALIGN_STACK;


                    break;

                case TURN_TO_ALIGN_STACK:
                    drive.setPoseEstimate(startPose2);

                    manip.clawClose();

                    drive.followTrajectory(turnToAlign);

                    while (!manip.colorIsActive()) {
                        move.setPowers(0.375,0.3,0.3,0.34999995);
                    }

                    move.setPowers(0,0,0,0);

                    manip.moveLift(-800);

                    manip.clawOpen();

                    drive.followTrajectory(faceWall);

                    move.setPowers(0,0,0,0);

                    currentState = State.GO_FORWARD;


                    break;

                case GO_FORWARD:

                    drive.followTrajectory(pickUpCone);


                    manip.clawClose();

                    sleep(1000);

                    currentState = State.SCORE_LOW;

                    cycleCount++;

                    break;

                case SCORE_LOW:

                    manip.moveLift(-1000);

                    drive.followTrajectory(scoreLow);

                    //drive.followTrajectory(turnFromScore);

                    manip.clawOpen();

                    //manip.returnLiftToDefault();

                    currentState = State.IDLE;

                    break;

                case PARK:
                    /*if (pos == 1){
                        drive.followTrajectory(pos_uno);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (pos == 2){
                        drive.followTrajectory(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else{
                        drive.followTrajectory(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }*/
                    break;

                case IDLE:

                    break;
            }
        }
    }
}
