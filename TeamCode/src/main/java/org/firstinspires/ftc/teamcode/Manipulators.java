package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    //claw
    public Servo claw;

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

    public void clawOpen(){
        claw.setPosition(0);
    }

    public void clawClose(){
        claw.setPosition(1);
    }
}
