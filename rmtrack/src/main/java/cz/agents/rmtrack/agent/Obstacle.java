package cz.agents.rmtrack.agent;

import java.util.ArrayList;

public class Obstacle {

    public int robot_i; // id of robot i
    public int robot_j; // id of robot j
    public int label; // it should be robot_i or robot_j
    public boolean checkedForFlip; // Check the flip only once. Not every time in the waiting state.
    public int robot_iLeft, robot_iRight, robot_jTop, robot_jBottom; // when robots reach and clear the obstacle
    public ArrayList<Integer> lowestObstacle = new ArrayList<>(); // lowest y's of the obstacle
    public ArrayList<Integer> highestObstacle = new ArrayList<>(); // highest y's of the obstacle
    public Obstacle twin; // Every obstacle has a symmetric twin.

    @Override
    public String toString() {
        return "Obstacle{" +
                "robot_i=" + robot_i +
                ", robot_j=" + robot_j +
                ", label=" + label +
                ", robot_iLeft=" + robot_iLeft +
                ", robot_iRight=" + robot_iRight +
                ", robot_jBottom=" + robot_jBottom +
                ", robot_jTop=" + robot_jTop +
                //", lowestObstacle=" + lowestObstacle +
                //", highestObstacle=" + highestObstacle +
                '}';
    }

}