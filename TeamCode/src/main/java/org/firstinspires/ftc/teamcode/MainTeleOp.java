package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    private int halfspeedDivider;

    String stringPos = "default";

    int[] positions;

    public void init() {
        manip = new Manipulators(hardwareMap);
        move = new Movement(hardwareMap);

        manip.clawOpen();

        positions = manip.setPositions();
        halfspeedDivider = 1;

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


        if (!manip.liftIsDefault()) {
            positions = manip.setPositions();
        }

        if (manip.clawIsOpen() && gamepad2.right_stick_y < -0.1)
        {
            gamepad2.rumble(0.5, 0.5, 100);
        }

        if (gamepad2.right_stick_y < -0.1 && !manip.clawIsOpen()) {
            manip.powerLift(gamepad2.right_stick_y);
        }
        else if (gamepad2.right_stick_y > 0.1 && !manip.liftIsDefault()) {
            manip.powerLift(gamepad2.right_stick_y);
        }
        else {
            manip.powerLift(0);
        }


        if (isPressed("rightBumper2", gamepad2.right_bumper)){
            if (open){
                manip.clawClose();
                open = false;

            } else {
                manip.clawOpen();
                open = true;
            }
        }

        if (gamepad1.right_bumper) {
            halfspeedDivider = 2;
        } else {
            halfspeedDivider= 1;
        }


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
            Math.abs(gamepad1.left_stick_x)  > 0.1 ||
            Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y/1.5;
            leftX = gamepad1.left_stick_x;
            rightX = (-gamepad1.right_stick_x)/2;

            motorPower = move.holonomicDrive(leftX/halfspeedDivider, leftY/halfspeedDivider, rightX/halfspeedDivider);

        }

        else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        move.setPowers(motorPower);

        //Lift MACROS

/*
         if (isPressed("2y", gamepad2.y) && stringPos.equals("default") || !stringPos.equals("high")){

            manip.liftToHeight(positions, "high");
            stringPos = "high";
        }
        else if (isPressed("2b", gamepad2.b) && stringPos.equals("default") || !stringPos.equals("mid")){

            manip.liftToHeight(positions, "mid");
            stringPos = "mid";
        }
        else if (isPressed("2a", gamepad2.a) && stringPos.equals("default") || !stringPos.equals("low")){

            manip.liftToHeight(positions, "low");
            stringPos = "low";
        }
        else if (isPressed("2a", gamepad2.a) ||
                 isPressed("2b", gamepad2.b) ||
                 isPressed("2x", gamepad2.x) ||
                 isPressed("2y", gamepad2.y)) {

            manip.liftToHeight(positions, "default");
            stringPos = "default";
        }

 */

        telemetry.addData("Lift1 Encoder Value: ", manip.getLiftPosition());
        telemetry.addData("Lift Position: ",stringPos );
        telemetry.addData("StickY: ", gamepad2.right_stick_y);
        telemetry.addData("isInDefault: ", manip.liftIsDefault());
        telemetry.update();


        //Write a story about rishi the software guy

    }

}
