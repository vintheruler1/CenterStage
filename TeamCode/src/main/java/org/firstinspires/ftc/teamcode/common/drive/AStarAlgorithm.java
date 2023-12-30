package org.firstinspires.ftc.teamcode.common.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class AStarAlgorithm {
    private Pair[][] grid;
    private ArrayList<Pair> path = new ArrayList<>();
    private Pair start;
    private Pair dest;
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public AStarAlgorithm(Pair[][] gr, int x1, int y1, int x2, int y2) {
        grid = gr;
        start = grid[x1][y1];
        dest = grid[x2][y2];
        path.add(start);
    }

    public boolean equals(Pair a, Pair b) {
        return a.getFirst() == b.getFirst() && a.getSecond() == b.getSecond();
    }

    public double getHValue(Pair a) {
        double diff1 = a.getFirst() - dest.getFirst();
        double diff2 = a.getSecond() - dest.getSecond();
        return Math.sqrt(Math.pow(diff1, 2) + Math.pow(diff2, 2));
    }

    public double getGValue(Pair a) {
        double g = 0;

        if (path.isEmpty()) {
            return 0;
        }

        for (int i = 0; i < path.size() - 1; i++) {
            Pair current = path.get(i);
            Pair next = path.get(i + 1);
            double diff1 = current.getFirst() - next.getFirst();
            double diff2 = current.getSecond() - next.getSecond();
            g += Math.sqrt(Math.pow(diff1, 2) + Math.pow(diff2, 2));
        }

        Pair last = path.get(path.size() - 1);
        double diff1 = last.getFirst() - a.getFirst();
        double diff2 = last.getSecond() - a.getSecond();
        g += Math.sqrt(Math.pow(diff1, 2) + Math.pow(diff2, 2));

        return g;
    }
    public double getFValue(Pair a) {
        return getGValue(a)+getHValue(a);
    }
    public boolean isValid(Pair a) {
        return (a.getFirst() >= 0 && a.getSecond() >= 0 &&
                a.getFirst() < grid.length && a.getSecond() < grid[0].length);
    }

    public boolean isUnblockedPair(Pair a) {
        return isValid(a) && a.isUnblocked();
    }

    public boolean isDestination(Pair a) {
        return a.equals(dest);
    }
    public double getDistance(Pair a, Pair b){
        int x_diff = a.getSecond()-b.getSecond();
        int y_diff = a.getFirst()-b.getSecond();
        return Math.sqrt(Math.pow(x_diff, 2)+ Math.pow(y_diff, 2));
    }
    public double getDistanceFromBlocked(Pair a){
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

    public void makeNextMove() {
        Pair nextMove = new Pair(100, 100, -1);
        double min = getHValue(nextMove) + getGValue(nextMove);

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i != 0 || j != 0) {
                    int nextX = start.getFirst() + i;
                    int nextY = start.getSecond() + j;

                    if (nextX >= 0 && nextY >= 0 && nextX < 6 && nextY < 6) {
                        System.out.println(grid[nextX][nextY]);
                        System.out.println(isUnblockedPair(
                                grid[nextX][nextY]));
                        System.out.println();

                        if (isUnblockedPair(grid[nextX][nextY]) ) {
                            Pair candidate = grid[nextX][nextY];
                            telemetry.addLine("" +getGValue(candidate));
                            telemetry.addLine(""+ getHValue(candidate));
                            telemetry.addLine(""+ getFValue(candidate));
                            telemetry.addLine();
                            double candidateCost = getHValue(candidate) + getGValue(candidate);

                            if (candidateCost < min&& !candidate.equals(start)) {
                                nextMove = candidate;
                                min = candidateCost;
                            }
                        }
                    }
                }
            }
        }

        path.add(nextMove);
        start = nextMove;

        double totalCost = getHValue(nextMove) + getGValue(nextMove);
        telemetry.addLine("Next Move: ");
        telemetry.addLine(""+ nextMove);
        telemetry.addLine(""+ getGValue(nextMove));
        telemetry.addLine("" +getHValue(nextMove));
        telemetry.addLine("" + totalCost);

    }

    public ArrayList<Pair> tracePath() {
        while (!start.equals(dest)) {
            makeNextMove();
        }
        telemetry.addLine(""+ path);
        return path;
    }

    public static void main(String[] args) {
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

        AStarAlgorithm test = new AStarAlgorithm(grid1, 0, 0, 5, 5);
        test.tracePath();
    }
}