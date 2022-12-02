package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Manipulators {

    private HardwareMap robot = null;
    //Lowest and Highest Encoder Positions
    public int lowest = 0;
    public int highest = 0;
    //lift
    public DcMotor lift1;
    public DcMotor lift2;
    public DigitalChannel liftSensor;
    public int defaultPos;
    public int pos1;
    public int pos2;
    public int pos3;

    //claw
    public Servo claw;
    public boolean open;


    //Hardware Map
    public Manipulators(HardwareMap robot) {
        this.robot = robot;
        //lift mapping
        lift1 = robot.get(DcMotor.class, "lift1");
        lift2 = robot.get(DcMotor.class, "lift2");
        liftSensor = robot.get(DigitalChannel.class, "liftSensor");

        //claw mapping
        claw = robot.get(Servo.class, "claw");

        //lift Encoders
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //LIFT METHODS

    public void powerLift(double power){
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public int getLiftPosition() {
        return lift1.getCurrentPosition();
    }

    public boolean liftIsDefault () {
        return !liftSensor.getState();
    }

    //CLAWS METHODS

    public void clawOpen(){
        claw.setPosition(.6);
        open = true;
    }

    public void clawClose(){
        claw.setPosition(1.1);
        open = false;
    }

    public boolean clawIsOpen() {
       return open;
    }

//Macro for the lift height
//NUMBERS ARE PLACEHOLDERS
    public void liftToHeight(int[] positions, String height){
        switch(height) {
            case "default":
                lift1.setTargetPosition(positions[0]);
                lift2.setTargetPosition(positions[0]);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "low":
                lift1.setTargetPosition(positions[1]);
                lift2.setTargetPosition(positions[1]);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "mid":
                lift1.setTargetPosition(positions[2]);
                lift2.setTargetPosition(positions[2]);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "high":
                lift1.setTargetPosition(positions[3]);
                lift2.setTargetPosition(positions[3]);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public int[] setPositions() {
        defaultPos = lift1.getCurrentPosition();
        pos1 = defaultPos - 1000;
        pos2 = defaultPos - 2000;
        pos3 = defaultPos - 4500;

        int[] positions = {defaultPos, pos1, pos2, pos3};

        return positions;

    }
}

