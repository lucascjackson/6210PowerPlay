package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

@Config
public class Manipulators {

    private HardwareMap robot = null;
    //Lowest and Highest Encoder Positions
    public int lowest = 0;
    public int highest = 0;
    //lift
    public DcMotor lift1;
    public DcMotor lift2;

    //claw
    public Servo claw;
    public int startPos;
    public int pos1;
    public int pos2;
    public int pos3;

    //Hardware Map
    public Manipulators(HardwareMap robot) {
        this.robot = robot;
        //lift mapping
        lift1 = robot.get(DcMotor.class, "lift1");
        lift2 = robot.get(DcMotor.class, "lift2");

        //claw mapping
        claw = robot.get(Servo.class, "claw");

        //lift Encoders
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkPosition() {
        if (lift1.getCurrentPosition() < lowest) {
            lowest = lift1.getCurrentPosition();
        }
        if (lift1.getCurrentPosition() > highest) {
            highest = lift1.getCurrentPosition();
        }
    }

    public void powerLift(double leftStickY){
        lift1.setPower(leftStickY);
        lift2.setPower(leftStickY);
    }

    public String getpowers() {
        return "1:" + lift1.getPower() + " 2: " + lift2.getPower();
    }

    public void clawOpen(){
        claw.setPosition(0.03);
    }

    public void clawClose(){
        claw.setPosition(0.27);
    }

//Macro for the lift height.
//NUMBERS ARE PLACEHOLDERS
    public void liftToHeight(int[] positions, String height){
        if (Objects.equals(height, "low")) {
            lift1.setTargetPosition(positions[0]);
            lift2.setTargetPosition(positions[0]);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (Objects.equals(height, "mid")) {
            lift1.setTargetPosition(positions[1]);
            lift2.setTargetPosition(positions[1]);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (Objects.equals(height, "high")) {
            lift1.setTargetPosition(positions[2]);
            lift2.setTargetPosition(positions[2]);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int[] setStartPos() {
        startPos = lift1.getCurrentPosition();
        pos1 = startPos + 1000;
        pos2 = startPos + 2000;
        pos3 = startPos + 3000;

        int[] poses = {pos1, pos2, pos3};

        return poses;

    }
}

