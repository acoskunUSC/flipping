package cz.agents.rmtrack.agent;

import com.mxgraph.layout.mxCircleLayout;
import com.mxgraph.layout.mxIGraphLayout;
import com.mxgraph.util.mxCellRenderer;
import cz.agents.rmtrack.ScenarioCreator;
import cz.agents.rmtrack.util.Disturbance;
import org.jgrapht.DirectedGraph;
import org.jgrapht.alg.CycleDetector;
import org.jgrapht.ext.JGraphXAdapter;
import org.jgrapht.graph.DefaultDirectedGraph;
import smile.regression.KernelMachine;
import tt.euclid2d.Point;
import tt.euclid2i.Trajectory;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Queue;
import java.util.*;

public class TrackingAgent extends Agent {

    private int currentTime;

    public enum TrackingMethod {ALLSTOP, RMTRACK, RMTRACK_TFF, RMTRACK_TFA}

    public enum ActionStatus {STOPS_FOR_LABEL, STOPS_FOR_OBSTACLE, MOVES}

    private final Trajectory traj;

    private final TrackingMethod trackingMethod;

    private int planPos = 0;

    private boolean currentlyDisturbed = false;

    private boolean currentlyWaiting = false;

    Queue<Integer> isDisturbedTrack = new LinkedList<>();

    double previousObservedProbability = 0;

    public ArrayList<Obstacle> obstacles = new ArrayList<>();

    public int endPosition;

    int deltaT = 100;

    Obstacle currentObstacle;

    static boolean isOnce = false;

    public TrackingAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, float radius, float maxSpeed, Trajectory traj, Disturbance disturbance, TrackingMethod trackingMethod) {
        super(id, start, goal, radius, maxSpeed, disturbance);
        this.traj = traj;
        this.trackingMethod = trackingMethod;

        for (int time = traj.getMaxTime(); time >= traj.getMinTime(); time -= deltaT) {
            if (traj.get(time).toPoint2d().epsilonEquals(goal, 0.9)) {
                this.endPosition = time;
            } else {
                break;
            }
        }
    }

    @Override
    public Point getPosition() {
        return this.traj.get(planPos).toPoint2d();
    }

    @Override
    public void update(int t_current_ms, int t_next_ms, List<Agent> agents) {

        //////////////////////////////////////////

        if (!isOnce && ScenarioCreator.visualizeGraph) {
            Agent agent1 = agents.get(0);
            Agent agent2 = agents.get(7);
            Agent agent3 = agents.get(8);

            List<Agent> threeAgents = new ArrayList<Agent>();
            threeAgents.add(agent1);
            threeAgents.add(agent2);
            threeAgents.add(agent3);

            visualizeSmallGraph(threeAgents, "initial");
            isOnce = true;
            System.out.println("initial graph is visualized!");
        }


        //////////////////////////////////////////

        this.currentTime = t_current_ms;
        if (!isAtGoal()) {
            travelTime = t_current_ms;
        }

        int deltaT = t_next_ms - t_current_ms;

        boolean proceed = true;

        if (trackingMethod == TrackingMethod.ALLSTOP) {
            proceed = !someoneDisturbed(agents, t_current_ms);
        } else if (trackingMethod == TrackingMethod.RMTRACK) {
            proceed = true;
            for (int j = 0; j < agents.size(); j++) {
                if (j != id) {
                    TrackingAgent otherAgent = (TrackingAgent) agents.get(j);
                    if (this.getPlanPos() > otherAgent.getPlanPos()) {
                        if (spatialCollision(this.getTrajectory(), this.getPlanPos(), this.getPlanPos() + deltaT,
                                otherAgent.getTrajectory(), otherAgent.getPlanPos(), this.getPlanPos() + deltaT,
                                deltaT / 2, (int) (this.getRadius() + otherAgent.getRadius()))) {
                            proceed = false;
                            break;
                        }
                    }
                }
            }
        } else if (trackingMethod == TrackingMethod.RMTRACK_TFF || trackingMethod == TrackingMethod.RMTRACK_TFA) {
            proceed = true;
            for (Obstacle obstacle : this.obstacles) {
                if (this.planPos + deltaT >= obstacle.robot_iLeft && this.planPos + deltaT <= obstacle.robot_iRight) {
                    TrackingAgent otherAgent = (TrackingAgent) agents.get(obstacle.robot_j);

                    ActionStatus action = action(this.planPos, otherAgent.getPlanPos(), obstacle, otherAgent);

                    if (action == ActionStatus.STOPS_FOR_OBSTACLE) {
                        currentObstacle = obstacle;
                        proceed = false;
                        currentStatus = Status.WAITS_FOR_OBSTACLE;
                        break;
                    } else if (action == ActionStatus.STOPS_FOR_LABEL) {
                        currentObstacle = obstacle;
                        proceed = false;
                        currentStatus = Status.WAITS_FOR_LABEL;
                        if (obstacle.checkedForFlip) {
                            break;
                        } else {
                            obstacle.checkedForFlip = true;
                            obstacle.twin.checkedForFlip = true;
                            // check if it is flippable or not
                            if (this.endPosition != obstacle.robot_iRight && obstacle.robot_jBottom != 0 && !(otherAgent.planPos >= obstacle.robot_jBottom && otherAgent.planPos<=obstacle.robot_iRight)) {
                                numberOfCheckingFlip++;
                                if (ScenarioCreator.debug) {
                                    System.out.println("checking flipping between robot " + this.id + " and " + otherAgent.id);
                                }
                                if (ScenarioCreator.isAvoidingDeadlock) {
                                    // temporarily change the label
                                    obstacle.label = this.id;
                                    obstacle.twin.label = this.id;

                                    if (ScenarioCreator.visualizeGraph) {
                                        if ((this.id == 0 && otherAgent.id == 7) || (this.id == 0 && otherAgent.id == 7)) {
                                            Agent agent1 = agents.get(0);
                                            Agent agent2 = agents.get(7);
                                            Agent agent3 = agents.get(8);

                                            List<Agent> threeAgents = new ArrayList<Agent>();
                                            threeAgents.add(agent1);
                                            threeAgents.add(agent2);
                                            threeAgents.add(agent3);

                                            visualizeSmallGraph(threeAgents, "flipped");

                                            System.out.println("graph is visualized after flipping");
                                        }
                                    }

                                    if (doesGraphContainCycle(agents)) {
                                        // back to the original
                                        obstacle.label = otherAgent.id;
                                        obstacle.twin.label = otherAgent.id;
                                        if (ScenarioCreator.debug) {
                                            System.out.println("Flipping between " + this.id + " and " + otherAgent.id + " makes a cycle!");
                                            System.out.println("----------------");
                                        }
                                        break;
                                    }

                                    if (ScenarioCreator.debug) {
                                        System.out.println("Flipping between " + this.id + " and " + otherAgent.id + " does NOT make a cycle!");
                                    }

                                    obstacle.label = otherAgent.id;
                                    obstacle.twin.label = otherAgent.id;

                                }
                                KernelMachine<double[]> gpr = ScenarioCreator.trainModel();
                                if (trackingMethod == TrackingMethod.RMTRACK_TFF) {
                                    checkTFF(otherAgent, obstacle, gpr);
                                } else if (trackingMethod == TrackingMethod.RMTRACK_TFA) {
                                    checkTFA(otherAgent, obstacle, gpr);
                                }
                            }
                            ActionStatus updatedAction = action(this.planPos, otherAgent.getPlanPos(), obstacle, otherAgent);
                            if (updatedAction == ActionStatus.MOVES) {
                                proceed = true;
                            } else if (updatedAction == ActionStatus.STOPS_FOR_OBSTACLE) {
                                proceed = false;
                                currentStatus = Status.WAITS_FOR_OBSTACLE;
                                break;
                            } else if (updatedAction == ActionStatus.STOPS_FOR_LABEL) {
                                proceed = false;
                                currentStatus = Status.WAITS_FOR_LABEL;
                                break;
                            }
                        }
                    }
                }
            }
        }

        if (proceed && trackingMethod != TrackingMethod.RMTRACK) {
            for (int j = 0; j < agents.size(); j++) {
                if (j != id) {
                    TrackingAgent otherAgent = (TrackingAgent) agents.get(j);
                    double distance = this.traj.get(getPlanPos() + deltaT).distance(otherAgent.traj.get(otherAgent.getPlanPos()));
                    boolean collision = distance < (this.radius + otherAgent.radius) * 0.95;
                    if (collision) {
                        proceed = false;
                        currentStatus = Status.WAITS_FOR_OBSTACLE;
                    }
                }
            }
        }

        if (proceed && trackingMethod != TrackingMethod.RMTRACK) {
            currentStatus = Status.MOVES;
            if (planPos >= endPosition) {
                currentStatus = Status.REACHED_THE_GOAL;
            }
            for (int j = 0; j < agents.size(); j++) {
                if (j != id) {
                    TrackingAgent otherAgent = (TrackingAgent) agents.get(j);
                    double distance = this.traj.get(getPlanPos() + deltaT).distance(otherAgent.traj.get(otherAgent.getPlanPos()));
                    boolean collision = distance < (this.radius + otherAgent.radius) * 0.95;
                    assert !collision : "collision between robot " + this.id + "-" + this.planPos + " and " + otherAgent.id + "-" + otherAgent.planPos + " with the distance " + distance + " - " + trackingMethod;
                }
            }
        }

        if (trackingMethod == TrackingMethod.RMTRACK) {
            if (proceed) {
                currentStatus = Status.MOVES;
            }
        }

        currentlyWaiting = !proceed;

        if (isDisturbed(this.traj.get(this.planPos))) {
            // disturbed, no progress
            currentlyDisturbed = true;
            isDisturbedTrack.add(1);
        } else {
            // disturbed, progress
            currentlyDisturbed = false;
            isDisturbedTrack.add(0);
            if (proceed) {
                planPos += deltaT;

//                Point oldPos = traj.get(planPos).toPoint2d();
//                Point newPos = traj.get(planPos+deltaT).toPoint2d();
//
//                // print current velocity
//                Vector vel = new Vector(newPos);
//                vel.sub(oldPos);
//                LOGGER.debug(id + ": velocity " + (vel.length() / deltaT));

            }
        }

        if (isDisturbedTrack.size() == 10) {

            int sum = isDisturbedTrack.stream().mapToInt(e -> e).sum();
            double currentObservedProbability = (double) sum / 10;
            if (Math.abs(currentObservedProbability - previousObservedProbability) > 0.05) {
                ScenarioCreator.observedProbabilities.add(currentObservedProbability);
                ScenarioCreator.observedPositions.add(this.traj.get(planPos));
                previousObservedProbability = currentObservedProbability;
            }

            isDisturbedTrack.remove();

        }

        if (ScenarioCreator.verbose) {
            if (currentStatus == Status.WAITS_FOR_LABEL || currentStatus == Status.WAITS_FOR_OBSTACLE) {

                System.out.println("Agent " + this.id + ": " + currentStatus +"(" + currentObstacle.toString() + ")" + " at " + planPos);
            } else {
                System.out.println("Agent " + this.id + ": " + currentStatus + " at " + planPos);
            }

        }

    }

    private void checkTFF(TrackingAgent otherAgent, Obstacle obstacle, KernelMachine<double[]> gpr) {
        long startTime = System.currentTimeMillis();
        //KernelMachine<double[]> gpr = ScenarioCreator.trainModel();
        if (expectedTravelTime(this.planPos, obstacle.robot_iRight, gpr) < otherAgent.expectedTravelTime(otherAgent.planPos, obstacle.robot_jBottom, gpr)) {
            numberOfSuccessfulFlip++;
            obstacle.label = this.id;
            obstacle.twin.label = this.id;
            if (ScenarioCreator.debug) {
                System.out.println("Agent " +this.id + " flipped the label of the obstacle with agent " + otherAgent.id);
                System.out.println("----------------");
            }
        } else {
            if (ScenarioCreator.debug) {
                System.out.println("Agent " +this.id + " did NOT flip the label of the obstacle with agent " + otherAgent.id);
                System.out.println("----------------");
            }
        }
        timeSpentForFlips += (System.currentTimeMillis() - startTime);

        //System.out.println("Checking TFF between " + this.id + " and " + otherAgent.id + " with " + ScenarioCreator.observedProbabilities.size() + " samples took " + (System.currentTimeMillis() - startTime));

    }

    private void checkTFA(TrackingAgent otherAgent, Obstacle obstacle, KernelMachine<double[]> gpr) {
        long startTime = System.currentTimeMillis();
        //KernelMachine<double[]> gpr = ScenarioCreator.trainModel();
        double thisAgentCurrentLabel = expectedTravelTime(this.planPos, obstacle.robot_iRight, otherAgent.planPos, otherAgent, obstacle,gpr);
        double otherAgentCurrentLabel = otherAgent.expectedTravelTime(otherAgent.planPos, obstacle.robot_jTop, this.planPos, this, obstacle.twin,gpr);

        // temporarily change the label
        obstacle.label = this.id;
        obstacle.twin.label = this.id;
        double thisAgentFlippedLabel = expectedTravelTime(this.planPos, obstacle.robot_iRight, otherAgent.planPos, otherAgent, obstacle,gpr);
        double otherAgentFlippedLabel = otherAgent.expectedTravelTime(otherAgent.planPos, obstacle.robot_jTop, this.planPos, this, obstacle.twin,gpr);
        // back to the original
        obstacle.label = otherAgent.id;
        obstacle.twin.label = otherAgent.id;

        if ((thisAgentFlippedLabel+otherAgentFlippedLabel) < (thisAgentCurrentLabel+otherAgentCurrentLabel)) {
            numberOfSuccessfulFlip++;
            obstacle.label = this.id;
            obstacle.twin.label = this.id;
            if (ScenarioCreator.debug) {
                System.out.println("Agent " +this.id + " flipped the label of the obstacle with agent " + otherAgent.id);
                System.out.println("----------------");
            }
        } else {
            if (ScenarioCreator.debug) {
                System.out.println("Agent " +this.id + " did NOT flip the label of the obstacle with agent " + otherAgent.id);
                System.out.println("----------------");
            }
        }
        timeSpentForFlips += (System.currentTimeMillis() - startTime);

        //System.out.println("Checking TFA between " + this.id + " and " + otherAgent.id + " with " + ScenarioCreator.observedProbabilities.size() + " samples took " + (System.currentTimeMillis() - startTime));

    }

    private ActionStatus action(int thisPlanPos, int otherPlanPos, Obstacle obstacle,TrackingAgent otherAgent) {

        int nextPos = thisPlanPos + deltaT;

        /*
        if (thisPlanPos == otherPlanPos) {
            return ActionStatus.MOVES;
        }

         */

        if (obstacle.label != this.id) {
            if (nextPos == obstacle.robot_iLeft) {
                if (otherPlanPos <= obstacle.highestObstacle.get(nextPos / deltaT) * deltaT) {
                    return ActionStatus.STOPS_FOR_LABEL;
                }
            }
        }

        /*
        if (nextPos < this.endPosition && otherPlanPos < otherAgent.endPosition) {
            if (this.traj.get(nextPos).distance(otherAgent.traj.get(otherPlanPos)) < (this.radius + otherAgent.radius) * 0.95) {
                return ActionStatus.STOPS_FOR_OBSTACLE;
            }
        }

         */

        if (nextPos >= obstacle.robot_iLeft && nextPos <= obstacle.robot_iRight) {
            if (otherPlanPos <= obstacle.highestObstacle.get(nextPos / deltaT) * deltaT) {
                if (otherPlanPos >= obstacle.lowestObstacle.get(nextPos / deltaT) * deltaT) {
                    return ActionStatus.STOPS_FOR_OBSTACLE;
                }
            }
        }



        /*
        if (nextPos >= obstacle.robot_iLeft && nextPos <= obstacle.robot_iRight) {
            if (otherPlanPos <= obstacle.highestObstacle.get(nextPos / deltaT) * deltaT) {
                if (otherPlanPos >= obstacle.lowestObstacle.get(nextPos / deltaT) * deltaT) {
                    return ActionStatus.STOPS_FOR_OBSTACLE;
                }
            }
        }

         */


        /*
        if (thisPlanPos != otherPlanPos) {
            if (obstacle.label != this.id) {
                if (otherPlanPos <= obstacle.highestObstacle.get(nextPos / deltaT) * deltaT) {
                    if (nextPos == obstacle.robot_iLeft) {
                        return ActionStatus.STOPS_FOR_LABEL;
                    } else if (nextPos > obstacle.robot_iLeft && nextPos <= obstacle.robot_iRight) {
                        return ActionStatus.STOPS_FOR_OBSTACLE;
                    }
                }
            }
        }

         */

        return ActionStatus.MOVES;

    }

    private double expectedTravelTime(int start, int end, KernelMachine<double[]> gpr) {

        double sum = 0.0;
        int cost = deltaT;
        //KernelMachine<double[]> gpr = ScenarioCreator.trainModel();

        for (int i=start; i<end; i+=deltaT) {
            double prediction = gpr.predict(new double[] {this.traj.get(i).toPoint2d().x, this.traj.get(i).toPoint2d().y});
            if (prediction >= 1) prediction = 0.9999999;
            else if (prediction <= 0) prediction = 0.0000001;
            sum += cost/(1-prediction);
        }

        return sum;

    }

    private double expectedTravelTime(int start_i, int end_i, int start_j, TrackingAgent otherAgent, Obstacle obstacle, KernelMachine<double[]> gpr) {

        double sum = 0.0;
        int cost = deltaT;
        int current_i = start_i;
        int current_j = start_j;
        //KernelMachine<double[]> gpr = ScenarioCreator.trainModel();

        while(current_i < end_i) {

            // if this robot can move, then add up the expected travel time
            if (action(current_i, current_j, obstacle, otherAgent) == ActionStatus.MOVES) {

                double predictionForRobot_i = gpr.predict(new double[] {this.traj.get(current_i).toPoint2d().x, this.traj.get(current_i).toPoint2d().y});
                if (predictionForRobot_i >= 1) predictionForRobot_i = 0.9999999;
                else if (predictionForRobot_i <= 0) predictionForRobot_i = 0.0000001;

                sum += cost/(1-predictionForRobot_i);

                if (otherAgent.action(current_j, current_i, obstacle.twin, this) == ActionStatus.MOVES) {

                    current_j +=deltaT;

                }

                current_i += deltaT;

            } else {

                // if this robot can not move, then add up the expected waiting time
                // the expected waiting time = the expected travel time for other robot so that this robot can move
                while (action(current_i, current_j, obstacle, otherAgent) != ActionStatus.MOVES) {

                    double predictionForRobot_j = gpr.predict(new double[] {otherAgent.traj.get(current_j).toPoint2d().x, otherAgent.traj.get(current_j).toPoint2d().y});
                    if (predictionForRobot_j >= 1) predictionForRobot_j = 0.95;
                    else if (predictionForRobot_j <= 0) predictionForRobot_j = 0.05;

                    sum += cost/(1-predictionForRobot_j);

                    current_j += deltaT;

                }

            }

        }

        return sum;

    }

    private boolean someoneDisturbed(List<Agent> agents, int timeMs) {
        for (Agent agent : agents) {
            if (agent.isDisturbed(agent.getPosition().toPoint2i())) {
                return true;
            }
        }
        return false;
    }

    private boolean spatialCollision(Trajectory traj1, int start1, int end1, Trajectory traj2, int start2, int end2, int sampling, int separation) {
        for (int t1 = start1 + deltaT; t1 <= end1; t1 += sampling) {
            for (int t2 = start2; t2 <= end2; t2 += sampling) {
                if (traj1.get(t1).distance(traj2.get(t2)) < separation) {
                    return true;
                }
            }
        }

        return false;
    }

    public Trajectory getTrajectory() {
        return traj;
    }

    @Override
    public boolean wasDisturbed() {
        return currentlyDisturbed;
    }

    public int getPlanPos() {
        return planPos;
    }

    public boolean isCurrentlyWaiting() {
        return currentlyWaiting;
    }

    @Override
    public Point getPlannedPosition() {
        return traj.get(this.currentTime).toPoint2d();
    }

    private void visualizeSmallGraph(final List<Agent> threeAgents, String name) {

        DirectedGraph<PathSegment, RelationshipEdge> segmentGraph = new DefaultDirectedGraph<>(RelationshipEdge.class);

        ArrayList<ArrayList<PathSegment> > allPathSegments = new ArrayList<ArrayList<PathSegment> >(threeAgents.size());

        for (int i=0; i<threeAgents.size(); i++) {

            TrackingAgent agent = (TrackingAgent) threeAgents.get(i);

            // using Set to avoid duplications
            Set<Integer> partition = new HashSet<>();
            for (Obstacle obstacle : agent.obstacles) {
                partition.add(obstacle.robot_iLeft);
                partition.add(obstacle.robot_iRight);
            }
            partition.add(0);
            partition.add(agent.endPosition);

            // need to have List to use sort() method
            ArrayList<Integer> sortedPartition = new ArrayList<>(partition);
            for (Obstacle obstacle : agent.obstacles) {
                if (obstacle.robot_iLeft == obstacle.robot_iRight) {
                    sortedPartition.add(obstacle.robot_iRight);
                }
            }
            Collections.sort(sortedPartition);

            // adding vertices
            ArrayList<PathSegment> pathSegments = new ArrayList<>();

            for (int j=0; j<sortedPartition.size()-1; j++) {

                PathSegment temp = new PathSegment(threeAgents.get(i).id, sortedPartition.get(j), sortedPartition.get(j+1));

                pathSegments.add(temp);

                segmentGraph.addVertex(temp);

            }

            allPathSegments.add(pathSegments);

            // adding path sequence edges
            for (int k=0; k<pathSegments.size()-1; k++) {

                segmentGraph.addEdge(pathSegments.get(k), pathSegments.get(k+1), new RelationshipEdge(RelationshipEdge.EdgeType.PATH));

            }

        }

        // adding obstacle-label edges
        for (int i=0; i<threeAgents.size(); i++) {

            TrackingAgent sourceAgent = (TrackingAgent) threeAgents.get(i);

            for (int o=0; o<sourceAgent.obstacles.size(); o++) {

                if (sourceAgent.obstacles.get(o).label == threeAgents.get(i).id) {

                    int robotIdOfSourceVertex = i;
                    int robotIdOfTargetVertex = sourceAgent.obstacles.get(o).robot_j;

                    if (robotIdOfTargetVertex != threeAgents.get(0).id && robotIdOfTargetVertex != threeAgents.get(1).id && robotIdOfTargetVertex != threeAgents.get(2).id) {
                        continue;
                    }

                    // initialize sourceVertex and targetVertex
                    PathSegment sourceVertex = new PathSegment(robotIdOfSourceVertex,-1,-1);
                    PathSegment targetVertex = new PathSegment(robotIdOfTargetVertex,-1,-1);

                    for (int v=0; v<allPathSegments.get(robotIdOfSourceVertex).size(); v++) {

                        if (allPathSegments.get(robotIdOfSourceVertex).get(v).start == sourceAgent.obstacles.get(o).robot_iRight) {

                            sourceVertex = allPathSegments.get(robotIdOfSourceVertex).get(v);

                            break;

                        }

                    }

                    assert (sourceVertex.start != -1 || sourceVertex.finish != -1) : "Source vertex could not detected!\n" + sourceAgent.obstacles.get(o);

                    if (robotIdOfTargetVertex == threeAgents.get(0).id) {
                        robotIdOfTargetVertex = 0;
                    }
                    if (robotIdOfTargetVertex == threeAgents.get(1).id) {
                        robotIdOfTargetVertex = 1;
                    }
                    if (robotIdOfTargetVertex == threeAgents.get(2).id) {
                        robotIdOfTargetVertex = 2;
                    }

                    for (int v=0; v<allPathSegments.get(robotIdOfTargetVertex).size(); v++) {

                        if (allPathSegments.get(robotIdOfTargetVertex).get(v).start == sourceAgent.obstacles.get(o).robot_jBottom) {

                            targetVertex = allPathSegments.get(robotIdOfTargetVertex).get(v);

                            break;

                        }

                    }

                    assert (targetVertex.start != -1 || targetVertex.finish != -1) : "Target vertex could not detected!\n" + sourceAgent.obstacles.get(o);

                    segmentGraph.addEdge(sourceVertex, targetVertex, new RelationshipEdge(RelationshipEdge.EdgeType.OBSTACLE));

                }

            }

        }





        try {

            JGraphXAdapter<PathSegment, RelationshipEdge> graphAdapter = new JGraphXAdapter<PathSegment, RelationshipEdge>(segmentGraph);
            mxIGraphLayout layout = new mxCircleLayout(graphAdapter);
            layout.execute(graphAdapter.getDefaultParent());

            BufferedImage image = mxCellRenderer.createBufferedImage(graphAdapter, null, 1, Color.WHITE, true, null);
            File imgFile = new File(name + ".png");
            ImageIO.write(image, "PNG", imgFile);

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    private boolean doesGraphContainCycle(final List<Agent> agents) {

        DirectedGraph<PathSegment, RelationshipEdge> segmentGraph = new DefaultDirectedGraph<>(RelationshipEdge.class);

        ArrayList<ArrayList<PathSegment> > allPathSegments = new ArrayList<ArrayList<PathSegment> >(agents.size());

        for (int i=0; i<agents.size(); i++) {

            TrackingAgent agent = (TrackingAgent) agents.get(i);

            // use Set to avoid duplications
            Set<Integer> partition = new HashSet<>();

            for (Obstacle obstacle : agent.obstacles) {
                partition.add(obstacle.robot_iLeft);
                partition.add(obstacle.robot_iRight);
            }

            partition.add(0);
            partition.add(agent.endPosition);

            // need to have List to use sort() method
            ArrayList<Integer> sortedPartition = new ArrayList<>(partition);
            for (Obstacle obstacle : agent.obstacles) {
                if (obstacle.robot_iLeft == obstacle.robot_iRight) {
                    sortedPartition.add(obstacle.robot_iRight);
                }
            }
            Collections.sort(sortedPartition);

            // adding vertices
            ArrayList<PathSegment> pathSegments = new ArrayList<>();

            for (int j=0; j<sortedPartition.size()-1; j++) {

                PathSegment temp = new PathSegment(i, sortedPartition.get(j), sortedPartition.get(j+1));

                pathSegments.add(temp);

                segmentGraph.addVertex(temp);

            }

            allPathSegments.add(pathSegments);

            // adding path sequence edges
            for (int k=0; k<pathSegments.size()-1; k++) {

                segmentGraph.addEdge(pathSegments.get(k), pathSegments.get(k+1), new RelationshipEdge(RelationshipEdge.EdgeType.PATH));

            }

        }

        // adding obstacle-label edges
        for (int i=0; i<agents.size(); i++) {

            TrackingAgent sourceAgent = (TrackingAgent) agents.get(i);

            for (int o=0; o<sourceAgent.obstacles.size(); o++) {

                if (sourceAgent.obstacles.get(o).label == i) {

                    int robotIdOfSourceVertex = i;
                    int robotIdOfTargetVertex = sourceAgent.obstacles.get(o).robot_j;

                    // initialize sourceVertex and targetVertex
                    PathSegment sourceVertex = new PathSegment(robotIdOfSourceVertex,-1,-1);
                    PathSegment targetVertex = new PathSegment(robotIdOfTargetVertex,-1,-1);

                    for (int v=0; v<allPathSegments.get(robotIdOfSourceVertex).size(); v++) {

                        // Instead of allPathSegments.get(robotIdOfSourceVertex).get(v).finish
                        // Instaad of finish. it is changed to start
                        if (allPathSegments.get(robotIdOfSourceVertex).get(v).start == sourceAgent.obstacles.get(o).robot_iRight) {

                            sourceVertex = allPathSegments.get(robotIdOfSourceVertex).get(v);

                            break;

                        }

                    }

                    assert (sourceVertex.start != -1 || sourceVertex.finish != -1) : "Source vertex could not detected!\n" + sourceAgent.obstacles.get(o);

                    for (int v=0; v<allPathSegments.get(robotIdOfTargetVertex).size(); v++) {

                        if (allPathSegments.get(robotIdOfTargetVertex).get(v).start == sourceAgent.obstacles.get(o).robot_jBottom) {

                            targetVertex = allPathSegments.get(robotIdOfTargetVertex).get(v);

                            break;

                        }

                    }

                    assert (targetVertex.start != -1 || targetVertex.finish != -1) : "Target vertex could not detected!\n" + sourceAgent.obstacles.get(o);

                    segmentGraph.addEdge(sourceVertex, targetVertex, new RelationshipEdge(RelationshipEdge.EdgeType.OBSTACLE));

                }

            }

        }

        if (false) {

            try {

                JGraphXAdapter<PathSegment, RelationshipEdge> graphAdapter = new JGraphXAdapter<PathSegment, RelationshipEdge>(segmentGraph);
                mxIGraphLayout layout = new mxCircleLayout(graphAdapter);
                layout.execute(graphAdapter.getDefaultParent());

                BufferedImage image = mxCellRenderer.createBufferedImage(graphAdapter, null, 1, Color.WHITE, true, null);
                File imgFile = new File("graph.png");
                ImageIO.write(image, "PNG", imgFile);

            } catch (IOException e) {
                e.printStackTrace();
            }

        }

        CycleDetector<PathSegment, RelationshipEdge> cycleDetector = new CycleDetector<PathSegment, RelationshipEdge>(segmentGraph);

        return cycleDetector.detectCycles();

    }


}
