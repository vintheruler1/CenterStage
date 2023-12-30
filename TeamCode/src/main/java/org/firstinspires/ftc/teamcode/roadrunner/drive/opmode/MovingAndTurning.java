package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.drive.AStarAlgorithm;
import org.firstinspires.ftc.teamcode.common.drive.Pair;


import java.util.ArrayList;

public class MovingAndTurning extends LinearOpMode{
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;

    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double TICKS_PER_INCH = 1120.0 / (WHEEL_DIAMETER_INCHES * Math.PI);
    public void runOpMode() {

        // Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        Pair[][] grid2 = new Pair[6][6];

        for (int i = 0; i < grid2.length; i++) {
            for (int j = 0; j < grid2[0].length; j++) {
                grid2[i][j] = new Pair(i, j, 1);
            }
        }

        grid2[0][1] = new Pair(0, 1, 0);
        grid2[0][4] = new Pair(0, 4, 0);
        grid2[3][2] = new Pair(3, 2, 0);
        grid2[3][3] = new Pair(3, 3, 0);
        AStarAlgorithm test = new AStarAlgorithm(grid2, 0, 0, 5, 5);

        ArrayList<Pair> path = test.tracePath();
        System.out.println(path);
        for (int i=0; i<path.size()-1; i++){
            Pair a = path.get(i);
            Pair b = path.get(i+1);
            double y_diff = b.getSecond()-a.getSecond();
            double x_diff = b.getFirst()-a.getFirst();
            double distance = Math.sqrt(Math.pow(x_diff, 2) + Math.pow(y_diff, 2));

            if (distance == Math.sqrt(2) && getDistanceFromBlocked(a, grid2) ==1 &&
                getDistanceFromBlocked(b, grid2)==1){
                moveForward(x_diff*18, 0.5);
                if (y_diff == 1){
                    setRobotHeading(180, 0.5);
                }else if (y_diff == -1){
                    setRobotHeading(0, 0.5);
                }
                moveForward(y_diff*18, 0.5);
            }else {
                setRobotHeading(Math.atan(y_diff / x_diff), 0.5);

                moveForward(18 * distance, 0.5);
            }
        }
    }
    public double getDistance(Pair a, Pair b){
        int x_diff = a.getSecond()-b.getSecond();
        int y_diff = a.getFirst()-b.getSecond();
        return Math.sqrt(Math.pow(x_diff, 2)+ Math.pow(y_diff, 2));
    }
    public double getDistanceFromBlocked(Pair a, Pair[][] grid){
        double minimum = 1000;
        for (int i=0; i< grid.length; i++){
            for (int j=0; j<grid[0].length; j++){
                if (!(grid[i][j].isUnblocked())){
                    if (getDistance(a, grid[i][j])<minimum){
                        minimum = getDistance(a, grid[i][j]);
                    }
                }
            }
        }
        return minimum;
    }

    private void moveForward(double inches, double power) {

        // Reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Calculate target encoder ticks for the distance
        int targetEncoderTicks = (int) (inches * TICKS_PER_INCH);
        // Set target positions
        frontLeftMotor.setTargetPosition(targetEncoderTicks);
        backLeftMotor.setTargetPosition(targetEncoderTicks);
        frontRightMotor.setTargetPosition(targetEncoderTicks);
        backRightMotor.setTargetPosition(targetEncoderTicks);

        // Set mode to RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);


        // Wait until the motors reach the target position
        while ( opModeIsActive() && frontLeftMotor.isBusy() && backLeftMotor.isBusy()
        && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            // You can add additional logic or actions here if needed
        }

        // Stop the motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set mode back to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setRobotHeading(double targetHeading, double power) {
        double currentHeading;

        // Get current heading
        Orientation angles = imu.getAngularOrientation();
        currentHeading = angles.firstAngle;

        // Calculate the difference in headings
        double headingDifference = targetHeading - currentHeading;

        // Adjust the heading
        int encoderTicksPerRevolution = 1120; // Replace with your motor's actual value
        int degreesPerRevolution = 360; // Replace with your robot's actual value

        int ticksPerRotation = (360 * encoderTicksPerRevolution) / degreesPerRevolution;
        // Reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setTargetPosition((int)headingDifference/ticksPerRotation);
        backLeftMotor.setTargetPosition((int)headingDifference/ticksPerRotation);
        frontRightMotor.setTargetPosition((int)headingDifference/ticksPerRotation);
        backRightMotor.setTargetPosition((int)headingDifference/ticksPerRotation);
        // Set mode to RUN_TO_POSITION
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);


        // Wait until the motors reach the target position
        while ( opModeIsActive() && frontLeftMotor.isBusy() && backLeftMotor.isBusy()
                && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            // You can add additional logic or actions here if needed
        }
        // Stop the robot after reaching the target heading
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        // Set mode back to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}
