package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp
public class BirdBotsTeleOp extends LinearOpMode {

    private static final double LOW_POWER = 0.175;
    private static final double LOW_POWER_TURN = 0.3;
    private Gyroscope imu;
    private DcMotor motorR;
    private DcMotor motorL;
    private Servo kitkatServo;
    private Servo twixServo;
    private Servo jellyServo;
    private Servo checkersServo;
    private Gamepad gamepad;

    private boolean clawClosed = true;
    private boolean plowUp = true;

    private double motorLeftPower = 0, motorRightPower = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorR = hardwareMap.get(DcMotor.class, "motorR");
        motorL = hardwareMap.get(DcMotor.class, "motorL");

        kitkatServo = hardwareMap.get(Servo.class, "KitKatServo");
        twixServo = hardwareMap.get(Servo.class, "TwixServo");

        jellyServo = hardwareMap.get(Servo.class, "JellyServo");
        checkersServo = hardwareMap.get(Servo.class, "CheckersServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gamepad = gamepad1;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorLeftPower = motorRightPower = 0;

            moveFast();
            moveSlow();
            turnFast();
            moveClawsAndPlow();

            setMotors();
            setClaws();
            setPlow();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    /**
     * Move fast using the joystick
     */
    double x, y, powerL, powerR;

    private void moveFast() {
        y = gamepad.left_stick_y;
        x = gamepad.left_stick_x;
        powerL = x - y;
        powerR = -x - y;
        double max = abs(x);
        if (max < abs(y)) {
            max = abs(y);
        }
        if (max > 1.0) {
            powerL /= max;
            powerR /= max;
        }
        motorLeftPower = powerL;
        motorRightPower = powerR;
    }

    private void turnFast() {
        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;

        if (leftTrigger > 0.05) {
            motorLeftPower = -leftTrigger;
            motorRightPower = leftTrigger;
        } else if (rightTrigger > 0.05) {
            motorLeftPower = rightTrigger;
            motorRightPower = -rightTrigger;
        }
    }

    /**
     * Move slow using the dpad
     */
    private void moveSlow() {
        if (gamepad.dpad_up) {
            motorLeftPower = LOW_POWER;
            motorRightPower = LOW_POWER;
        } else if (gamepad.dpad_down) {
            motorLeftPower = -LOW_POWER;
            motorRightPower = -LOW_POWER;
        } else if (gamepad.dpad_left) {
            motorLeftPower = -LOW_POWER_TURN;
            motorRightPower = LOW_POWER_TURN;
        } else if (gamepad.dpad_right) {
            motorLeftPower = LOW_POWER_TURN;
            motorRightPower = -LOW_POWER_TURN;
        }
    }

    /**
     * Claws open and close using A/B X/Y and bumpers
     */
    private void moveClawsAndPlow() {
        if (gamepad1.b) clawClosed = true;
        if (gamepad1.a) clawClosed = false;

        if (gamepad1.x) plowUp = true;
        if (gamepad1.y) plowUp = false;

        if (gamepad1.left_bumper) {
            plowUp = true;
            clawClosed = false;
        }

        if (gamepad1.right_bumper) {
            plowUp = false;
            clawClosed = true;
        }
    }

    /**
     * Sets the motor powers
     */
    private void setMotors() {
        motorLeftPower = -motorLeftPower; // because motor is mounted backwards

        motorR.setPower(motorRightPower);
        motorL.setPower(motorLeftPower);

        telemetry.addData("Target power: ", motorLeftPower + " | " + motorRightPower);
        telemetry.addData("Motors: ", motorL.getPower() + " | " + motorR.getPower());
    }

    /**
     * Sets the claw servos
     */
    private void setClaws() {
        double pos = clawClosed ? 0 : 0.42;
        kitkatServo.setDirection(Servo.Direction.FORWARD);
        kitkatServo.setPosition(pos);
        twixServo.setDirection(Servo.Direction.REVERSE);
        twixServo.setPosition(pos);

        telemetry.addData("Claws: ", kitkatServo.getPosition() + " | " + twixServo.getPosition());
    }

    /**
     * Sets the plow servos
     */
    private void setPlow() {
        double pos = plowUp ? 0.5 : 0.08;
        jellyServo.setDirection(Servo.Direction.FORWARD);
        jellyServo.setPosition(pos);
        checkersServo.setDirection(Servo.Direction.REVERSE);
        checkersServo.setPosition((pos));

        telemetry.addData("Plow: ", jellyServo.getPosition() + " | " + checkersServo.getPosition());
    }
}
