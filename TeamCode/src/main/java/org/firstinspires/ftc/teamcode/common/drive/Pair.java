package org.firstinspires.ftc.teamcode.common.drive;

public class Pair {
    private int first;
    private int second;
    private int unblocked;
    public Pair(int f, int s, int u){
        first = f;
        second = s;
        unblocked = u;
    }
    public int getFirst(){
        return first;
    }
    public int getSecond(){
        return second;
    }
    public boolean isUnblocked(){
        if (unblocked ==1 ){
            return true;
        }else{
            return false;
        }
    }
    public String toString() {
        return "first: " + first + " second: " + second+ " unblocked: " + isUnblocked();
    }
}

