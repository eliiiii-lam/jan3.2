package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous (name = "PID AUTO")
public class PIDautotest extends LinearOpMode {
    private DcMotor fL = null;
    private DcMotor fR = null;

    private DcMotor bL = null;

    private DcMotor bR = null;


    private DcMotorEx elbow;
    private DcMotorEx elbow2;







    Servo clawL;
    Servo clawR;

    Servo  wrist;






    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77952;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.85;
    static final double TURN_SPEED = 0.5;
    double diameter = 15.4;
    double arc90 = Math.PI * diameter / 2;


    public static double p = 0.025, i = 0.15, d = 0.000;




    private static final double ENCODER_CPR = 5281.1;
    private static final double GEAR_RATIO = 1.0;
    private static final double ARM_TICKS_PER_DEGREE = ENCODER_CPR * GEAR_RATIO / 360.0;
    private static final double MAX_ARM_HOLDING_POWER = 2;
    private static final double ZERO_OFFSET = 45.0;
    private double targetPosInDegrees;
    private double powerLimit;

    private PIDController controller;


    static final double ARM_LOADING_POS = 50.0;

    PIDautotest arm;



    @Override
    public void runOpMode() throws InterruptedException {

           controller = new PIDController(p, i, d);

        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");







        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");


        wrist = hardwareMap.servo.get("wrist");

        arm = new ();


        waitForStart();



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);


        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);




        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);







        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),

                bR.getCurrentPosition());


        telemetry.update();

        telemetry.addData("Ending at", "%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        telemetry.addData("Starting motion", "%7d :%7d");

        telemetry.update();
        clawL.setPosition(0.4);
        clawR.setPosition(0);


        wrist.setPosition(0.5);

        //encoderDrive(0.6, -72,-72,2);

        arm.setPosition(ARM_LOADING_POS, 0.5);


        wrist.setPosition(0);

        openClaw();







        telemetry.addData("Ending at", "%7d :%7d");
        telemetry.update();


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //place pixels on backstage here**************

        telemetry.addData("Ending at", "%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





    public void openClaw() {
        clawL.setPosition(0);
        clawR.setPosition(0.4);
    }



    public void armTask()
    {
        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = elbow.getCurrentPosition();
        double currPosInTicks2 = elbow2.getCurrentPosition();
        double pidOutput = controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        double ff = MAX_ARM_HOLDING_POWER * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : power > powerLimit ? powerLimit : power;
        elbow.setPower(power);
        elbow2.setPower(power);
    }

    public boolean isOnTarget(double toleranceInDegrees)
    {
        double currPosInDegrees = getPosition();
        return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
    }

    public void setPosition(double targetPosInDegrees, double powerLimit)
    {
        this.targetPosInDegrees = targetPosInDegrees;
        this.powerLimit = Math.abs(powerLimit);
    }

    public double ticksToRealWorldDegrees(double ticks)
    {
        return ticks / ARM_TICKS_PER_DEGREE + ZERO_OFFSET;
    }

    public double getPosition()
    {
        return ticksToRealWorldDegrees(elbow.getCurrentPosition());
    }












    public void encoderDrive(double speed,
                             double leftDrive, double rightDrive, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////











    public void strafeRight(double speed, double frontLeft,
                            double frontRight, double backLeft,
                            double backRight, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() - (int) (frontRight * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() - (int) (backLeft * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }


    public void strafeLeft(double speed, double frontLeft,
                           double frontRight, double backLeft,
                           double backRight, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() + (int) (frontRight * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() + (int) (backLeft * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(-speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }


    public void elbowmove(double speed,
                          double elbowenco, double timeoutS) {

        int newElbow;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newElbow = elbow.getCurrentPosition() + (int) (elbowenco * COUNTS_PER_INCH);

            elbow.setTargetPosition(newElbow);

            // Turn On RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            elbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elbow.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newElbow);
                telemetry.addData("Currently at", " at %7d",

                        elbow.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            elbow.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }
}
///////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
