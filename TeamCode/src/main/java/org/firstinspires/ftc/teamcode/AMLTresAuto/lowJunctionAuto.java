package org.firstinspires.ftc.teamcode.AMLTresAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AMLOneAutos.BetterOdomPark;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        GO_TO_WALL,
        GO_TO_STACK,
        TURN_TO_ALIGN_STACK,
        GO_FORWARD_PICK_UP_CONE,
        SCORE_LOW,
        PARK
    }

    int cycleCount = 0;


    Manipulators manip;
    Movement move;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double goToWallX = 0;
    public static double goToWallY = -40;
    public static double goToWallAngle = 0;

    public static double goToStackX = 12;
    public static double goToStackY = -40;
    public static double goToStackAngle = 0;

    public static double turnToAlignX = 31;
    public static double turnToAlignY = -40;
    public static double turnToAlignAngle = Math.toRadians(-258);

    public static double pickUpConeX = 31;
    public static double pickUpConeY = -40;
    public static double pickUpConeAngle = Math.toRadians(-258);

    public static double scoreLowX = 31;
    public static double scoreLowY = -40;
    public static double scoreLowAngle = Math.toRadians(-258);



    public static double posUnoX = 0;
    public static double posUnoY = 0;
    public static double posUnoAngle = Math.toRadians(0);

    public static double posDosX = 0;
    public static double posDosY = 0;
    public static double posDosAngle = Math.toRadians(0);

    public static double posTresX = 0;
    public static double posTresY = 0;
    public static double posTresAngle = Math.toRadians(0);




    public static double startWait = 0;

    lowJunctionAuto.State currentState = lowJunctionAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        int parkPos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", parkPos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory goToWall = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY,goToWallAngle ))
                .build();


        Trajectory goToStack = drive.trajectoryBuilder(goToWall.end())
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();


        Trajectory turnToAlign = drive.trajectoryBuilder(goToStack.end())
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory alignWithWall = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(turnToAlign.end().getX(), turnToAlign.end().getY()+.01, pickUpConeAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        Trajectory scoreLow = drive.trajectoryBuilder(pickUpCone.end())
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        Trajectory pos_uno = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(posUnoX, posUnoY, posUnoAngle))
                .build();

        Trajectory pos_dos = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(posDosX, posDosY, posDosAngle))
                .build();

        Trajectory pos_tres = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(posTresX, posTresY, posTresAngle))
                .build();


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

                    boolean isAligned = false;

                    drive.followTrajectory(turnToAlign);

                    while (!isAligned) {
                        while (!manip.colorBackIsActive() || !manip.colorFrontIsActive()) {
                            move.setPowers(-1, 0.8, 0.8, -0.9333333333);
                        }
                        if (manip.colorBackIsActive() && manip.colorFrontIsActive()) {
                            isAligned = true;
                        } else {
                            drive.followTrajectory(alignWithWall);
                        }
                    }

                    /*
                    while (!manip.colorIsActive()) {
                        move.setPowers(-1, 0.8, 0.8, -0.9333333333);
                    }

                    drive.followTrajectory(alignWithWall);
                     */

                    currentState = State.GO_FORWARD_PICK_UP_CONE;
                    break;

                case GO_FORWARD_PICK_UP_CONE:
                    drive.followTrajectory(pickUpCone);
                    currentState = State.SCORE_LOW;

                    break;

                case SCORE_LOW:
                    drive.followTrajectory(scoreLow);

                    if (cycleCount > 4) {
                        currentState = State.PARK;
                    }

                    cycleCount++;
                    currentState = State.GO_FORWARD_PICK_UP_CONE;

                    break;


                case PARK:
                    if (parkPos==1){
                        drive.followTrajectory(pos_uno);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (parkPos==2){
                        drive.followTrajectory(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else {
                        drive.followTrajectory(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    break;

                case IDLE:

                    break;
            }
        }
    }
}
