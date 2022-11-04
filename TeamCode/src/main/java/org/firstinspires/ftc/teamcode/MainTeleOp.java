package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

    Manipulators manip;
    Movement move;
    ElapsedTime waitTimer = new ElapsedTime();

    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    double[] motorPower = {0, 0, 0, 0};
    boolean open = true;

    public void init() {
        manip = new Manipulators(hardwareMap);
        move = new Movement(hardwareMap);

        //manip.clawOpen();

        telemetry.addData("init", "completed");
        telemetry.update();
    }

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    @Override
    public void loop()
    {
       double leftY = 0;
       double leftX = 0;
       double rightX = 0;

        if (Math.abs(gamepad2.left_stick_y) > 0.1){
          manip.powerLift(gamepad2.left_stick_y);
        }
        else {
          manip.powerLift(0);
        }


        if (isPressed("rightBumper1", gamepad2.right_bumper)){
            if (open){
                telemetry.addData("position", manip.claw.getPosition());
                manip.clawClose();
                open = false;

            } else {
                telemetry.addData("claw", "open");
                manip.clawOpen();
                open = true;
            }
        }


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
            Math.abs(gamepad1.left_stick_x)  > 0.1 ||
            Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            motorPower = move.holonomicDrive(leftX, leftY, rightX);

        }
        else {
            motorPower = new double[]{0, 0, 0, 0};
        }

        move.setPowers(motorPower);

        //Lift MACROS

        if (isPressed("2y", gamepad2.y)){
            manip.liftToHeight(manip.setStartPos(), "low");
        }
        if (isPressed("2b", gamepad2.b)){
            manip.liftToHeight(manip.setStartPos(), "mid");
        }
        if (isPressed("2a", gamepad2.a)) {
            manip.liftToHeight(manip.setStartPos(), "high");
        }

    }

}
