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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        POS_ON_WALL,
        GO_FORWARD,
        CYCLE_LOW,
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
    public static double pickUpConeY = 21;
    public static double pickUpConeAngle = 5.29;

    public static double moveBack = 15 ;

    public static double scoreLowX = 20 ;
    public static double scoreLowY = 0;
    public static double scoreLowAngle = Math.toRadians(-125);

    public static double turnBackX = -4;
    public static double turnBackY = 10;
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

        TrajectorySequence posOnWall = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY, gotoWallAngle))
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();

        Trajectory turnToAlign = drive.trajectoryBuilder(startPose2)
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory faceWall = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(faceWallX, faceWallY, faceWallAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(faceWall.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        TrajectorySequence scoreLow = drive.trajectorySequenceBuilder(startPose3)
                .forward(moveBack)
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        TrajectorySequence turnFromScore = drive.trajectorySequenceBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(turnFromScoreX, turnFromScoreY, turnFromScoreAngle))
                .back(moveForward)
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

                        currentState = State.POS_ON_WALL;
                    }

                    break;

                case POS_ON_WALL:

                    drive.followTrajectorySequence(posOnWall);

                    manip.clawClose();

                    drive.followTrajectory(turnToAlign);

                    while (!manip.colorIsActive()) {
                        move.setPowers(0.375,0.3,0.3,0.34999995);
                    }

                    move.setPowers(0,0,0,0);
                    manip.moveLift(-800);
                    manip.clawOpen();

                    drive.followTrajectory(faceWall);

                    currentState = State.GO_FORWARD;

                    break;

                case GO_FORWARD:

                    drive.followTrajectory(pickUpCone);

                    manip.clawClose();

                    sleep(800);

                    currentState = State.CYCLE_LOW;

                    break;

                case CYCLE_LOW:

                    if (cycleCount > 0) {

                        manip.clawClose();

                        sleep(800);
                    }

                    manip.moveLift(-1000);

                    drive.followTrajectorySequence(scoreLow);

                    manip.clawOpen();
                    sleep(500);
                    manip.moveLift(1000);

                    drive.followTrajectorySequence(turnFromScore);

                    if (cycleCount == 2) {

                        currentState = State.PARK;
                    }

                    cycleCount++;

                    currentState = State.IDLE;

                    break;
/*
                case PARK:
                    if (pos == 1) {
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (pos == 2){
                        drive.followTrajectory(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else{
                        drive.followTrajectory(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }

                    break;
*/
                case IDLE:

                    break;
            }
        }
    }
}
