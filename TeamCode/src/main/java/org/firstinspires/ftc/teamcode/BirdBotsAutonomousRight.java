package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.AutonomousConstants.FWD_DURATION;
import static org.firstinspires.ftc.teamcode.AutonomousConstants.PARK_DURATION;
import static org.firstinspires.ftc.teamcode.AutonomousConstants.TURN_DURATION;

@Autonomous
public class BirdBotsAutonomousRight extends LinearOpMode {

    private static final double LOW_POWER = 0.175;
    private Gyroscope imu;
    private DcMotor motorR;
    private DcMotor motorL;
    private Servo kitkatServo;
    private Servo twixServo;
    private Servo jellyServo;
    private Servo checkersServo;
    private Gamepad gamepad;

    private boolean clawClosed = false;
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

        setClaws();
        setPlow();

        move(0.8, 0.8, FWD_DURATION);
        move(-0.8, 0.8, TURN_DURATION);
        move(0.8, 0.8, PARK_DURATION);

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    private void move(double L, double R, double duration) {
        telemetry.addLine("Moving forward for " + duration + "s");
        motorLeftPower = L;
        motorRightPower = R;
        setMotors();
        wait(duration);
        motorLeftPower = 0;
        motorRightPower = 0;
        setMotors();
        wait(0.5);
    }

    private void wait(double duration) {
        sleep((int) (duration * 1000));
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
