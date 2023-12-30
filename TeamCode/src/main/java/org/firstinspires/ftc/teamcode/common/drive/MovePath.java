package org.firstinspires.ftc.teamcode.WIP.common.drive;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import java.util.ArrayList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MovePath {
    static final int MOTOR_TICK_COUNTS = 1120;
    private static BNO055IMU imu;

    private static DcMotor frontLeftMotor;
    private static DcMotor backLeftMotor;
    private static DcMotor frontRightMotor;
    private static DcMotor backRightMotor;

    public static void main(String[] args){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //example
        Pair[][] grid1 = new Pair[6][6];

        for (int i = 0; i < grid1.length; i++) {
            for (int j = 0; j < grid1[0].length; j++) {
                grid1[i][j] = new Pair(i, j, 1);
            }
        }

        grid1[0][1] = new Pair(0, 1, 0);
        grid1[0][4] = new Pair(0, 4, 0);
        grid1[3][2] = new Pair(3, 2, 0);
        grid1[3][3] = new Pair(3, 3, 0);

        AStarAlgorithm test = new AStarAlgorithm(grid1, 5, 1, 1, 4);
        ArrayList<Pair> path =  test.tracePath();
        for (int i=0; i<path.size()-1; i++){
            int x_diff = (path.get(i+1).getSecond()-path.get(i).getSecond());
            int y_diff = path.get(i+1).getFirst()-path.get(i).getSecond();
            double angle  =Math.atan(y_diff/x_diff);
            double distance = Math.sqrt(Math.pow(x_diff, 2)+ Math.pow(y_diff, 2));
            setHeading(angle);
            moveForward(distance*18);
        }

    }
    public static void moveForward(double inches){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheels are 18 inches
        double circumference = 3.14*2.938; //pi*diameter
        double rotationsNeeded = inches/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*MOTOR_TICK_COUNTS);
        frontLeftMotor.setTargetPosition(encoderDrivingTarget);
        frontRightMotor.setTargetPosition(encoderDrivingTarget);
        backLeftMotor.setTargetPosition(encoderDrivingTarget);
        backRightMotor.setTargetPosition(encoderDrivingTarget);
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public static void rotate(double degrees){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheels are 18 inches
        double circumference = 3.14*2.938; //pi*diameter
        double inches = degrees/360*66.47;
        double rotationsNeeded = inches/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*MOTOR_TICK_COUNTS);
        frontLeftMotor.setTargetPosition(encoderDrivingTarget);
        frontRightMotor.setTargetPosition(-encoderDrivingTarget);
        backLeftMotor.setTargetPosition(-encoderDrivingTarget);
        backRightMotor.setTargetPosition(encoderDrivingTarget);
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public static void setHeading(double target){
        double k = imu.getAngularOrientation().thirdAngle;
        rotate(target-k); //or k-target
    }

}
