package cz.agents.rmtrack;

import cz.agents.alite.simulation.vis.SimulationControlLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.rmtrack.agent.Agent;
import cz.agents.rmtrack.agent.ORCAAgent;
import cz.agents.rmtrack.agent.Obstacle;
import cz.agents.rmtrack.agent.TrackingAgent;
import cz.agents.rmtrack.util.Disturbance;
import org.apache.commons.lang3.ArrayUtils;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;
import smile.clustering.KMeans;
import smile.math.MathEx;
import smile.math.kernel.GaussianKernel;
import smile.regression.GaussianProcessRegression;
import smile.regression.KernelMachine;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.Trajectory;
import tt.euclid2i.region.Circle;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.vis.RegionsLayer;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.TrajectoryCoordinationProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.vis.FastTrajectoriesLayer;
import tt.vis.FastTrajectoriesLayer.ColorProvider;
import tt.vis.FastTrajectoriesLayer.TrajectoriesProvider;
import tt.vis.LabeledCircleLayer;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.awt.*;
import java.io.*;
import java.util.List;
import java.util.*;

public class ScenarioCreator {

    /*
     * Units:
     * time: 1 time unit = 1ms;
     * distance: 1 distance unit = depending on the map, 2cm typically.
     * speed: in du/ms (distance units / millisecond), typically 0.05 du/ms represents roughly 1m/1s
     */

    ////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    enum Method {
        ALLSTOP,
        RMTRACK,
        RMTRACK_TFF,
        RMTRACK_TFA,
        ALLMETHODS,
        ORCA_T,
        ORCA
    }

    private static boolean isItAllMethods = false;

    private static EarliestArrivalProblem problem;

    public static List<Region> disturbanceRegions;

    public static List<Float> disturbanceProbabilities = new ArrayList<>();

    public static ArrayList<Point> observedPositions = new ArrayList<>();

    public static ArrayList<Double> observedProbabilities = new ArrayList<>();

    public static float lambda;

    public static double optimalLambda;

    public static boolean verbose = false;

    public static boolean debug = false;

    public static boolean visualizeGraph = false;

    static int fileIndex;

    public static boolean isAvoidingDeadlock = false;

    public static void createFromArgs(String[] args) {

        isAvoidingDeadlock = Args.isArgumentSet(args, "-avoidDeadlock");

        lambda = Float.parseFloat(Args.getArgumentValue(args, "-lambda", false, "0.1"));

        Parameters params = new Parameters();

        String xml = Args.getArgumentValue(args, "-problemfile", true);
        if (xml.length() > 9) {
            String fileIndexStr;
            char tempChar = xml.charAt(xml.length()-6);
            char c = xml.charAt(xml.length() - 5);
            if (tempChar == '/') {
                fileIndexStr = String.valueOf(c);
            } else {
                fileIndexStr = String.valueOf(tempChar) + c;
            }
            fileIndex = Integer.parseInt(fileIndexStr);
            System.out.println(fileIndexStr);
        }
        String methodStr = Args.getArgumentValue(args, "-method", true);

        String maxTimeStr = Args.getArgumentValue(args, "-maxtime", true);
        params.maxTime = Integer.parseInt(maxTimeStr);

        String timeStepStr = Args.getArgumentValue(args, "-timestep", true);
        params.timeStep = Integer.parseInt(timeStepStr);

        params.showVis = Args.isArgumentSet(args, "-showvis");

        params.verbose = Args.isArgumentSet(args, "-verbose");

        verbose = params.verbose;

        debug = Args.isArgumentSet(args, "-debug");

        visualizeGraph = Args.isArgumentSet(args, "-visualizeGraph");

        if (!verbose) {
            Logger.getRootLogger().setLevel(Level.OFF);
        }

        String timeoutStr = Args.getArgumentValue(args, "-timeout", false);

        params.summaryPrefix = Args.getArgumentValue(args, "-summaryprefix", false, "");

        params.activityLogFile = Args.getArgumentValue(args, "-activitylog", false, null);

        String bgImgFileName = Args.getArgumentValue(args, "-bgimg", false, null);

        String simSpeedStr = Args.getArgumentValue(args, "-simspeed", false, "1");
        params.simSpeed = Double.parseDouble(simSpeedStr);

        String disturbanceProbStr = Args.getArgumentValue(args, "-dprob", false, "2");
        String[] parts = disturbanceProbStr.split("s", -1);
        params.disturbanceProbs = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            params.disturbanceProbs[i] = Integer.parseInt(parts[i]) / 100.0;
        }

        String disturbanceSeedStr = Args.getArgumentValue(args, "-dseed", false, "1");
        params.disturbanceSeed = Integer.parseInt(disturbanceSeedStr);

        String disturbanceQuantStr = Args.getArgumentValue(args, "-dquant", false, "1000");
        params.disturbanceQuantum = Integer.parseInt(disturbanceQuantStr);

        File file = new File(xml);
        params.fileName = file.getName();

        // Load the PNG image as a background, if provided
        if (bgImgFileName != null) {
            File bgImgFile = new File(bgImgFileName);
            if (bgImgFile.exists()) {
                params.bgImageFile = bgImgFile;
            }
        }

        try {
            problem = TrajectoryCoordinationProblemXMLDeserializer.deserialize(new FileInputStream(file));
            parse(new FileInputStream(file));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        Method method = Method.valueOf(methodStr);
        if (method == Method.ALLMETHODS) {
            isItAllMethods = true;
        }

        params.runtimeDeadlineMs = 3600 * 1000; /* default timeout is 1 hour */
        if (timeoutStr != null) {
            int timeout = Integer.parseInt(timeoutStr);
            params.runtimeDeadlineMs = timeout;
            killAt(System.currentTimeMillis() + timeout, params.summaryPrefix, params.noOfClusters);
        }

        switch (method) {
            case ALLSTOP:
            case RMTRACK:
            case RMTRACK_TFF:
            case RMTRACK_TFA:
            case ALLMETHODS:
            case ORCA_T:
                solveTracking(problem, method, params);
                break;
            case ORCA:
                solveORCA(problem, params);
                break;
            default:
                throw new RuntimeException("Unknown method");

        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static void parse(InputStream stream) {
        Document doc = null;

        try {
            DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder docBuilder = docFactory.newDocumentBuilder();
            doc = docBuilder.parse(stream);
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        NodeList disturbanceNL = doc.getElementsByTagName("disturbances");

        if (disturbanceNL.getLength() == 1) {
            NodeList disturbanceChildren = disturbanceNL.item(0).getChildNodes();
            disturbanceRegions = parseDisturbances(disturbanceChildren);
        }
    }

    private static List<Region> parseDisturbances(NodeList disturbanceElements) {

        final List<Region> disturbance = new ArrayList<Region>();

        for (int i = 0; i < disturbanceElements.getLength(); i++) {
            Region region = null;
            if (disturbanceElements.item(i).getNodeName().equals("polygon")) {
                region = parsePolygon((Element) disturbanceElements.item(i));
            } else if (disturbanceElements.item(i).getNodeName().equals("circle")) {
                region = parseCircle((Element) disturbanceElements.item(i));
            }
            if (region != null) {
                disturbance.add(region);
            }
        }

        return disturbance;
    }

    private static tt.euclid2i.region.Polygon parsePolygon(Element disturbance) {

        try {

            boolean forceFilledInside = false;
            if (disturbance.hasAttribute("filledinside")) {
                forceFilledInside = true;
            }

            if (disturbance.hasAttribute("probability")) {
                disturbanceProbabilities.add(Float.parseFloat(disturbance.getAttribute("probability")));
            }

            NodeList pointElements = disturbance.getElementsByTagName("POINT");

            tt.euclid2i.Point[] pointsArray;
            if (pointElements.getLength() == 0) {
                // Polygon is defined using the simple syntax
                String text = disturbance.getTextContent();
                pointsArray = stringToPointArray(text);
            } else {
                // Polygon is defined using the verbose syntax
                List<tt.euclid2i.Point> points = new ArrayList<tt.euclid2i.Point>();

                for (int i = 0; i < pointElements.getLength(); i++) {
                    Node pointElement = pointElements.item(i);
                    points.add(parsePoint((Element) pointElement));
                }

                pointsArray = points.toArray(new tt.euclid2i.Point[points.size()]);
            }

            if (forceFilledInside) {
                if (!tt.euclid2i.region.Polygon.isClockwise(pointsArray)) {
                    ArrayUtils.reverse(pointsArray);
                }
            }

            return new Polygon(pointsArray);

        } catch (Exception ex) {
            throw new RuntimeException("Error while parsing" + formatOutput(disturbance));
        }
    }

    private static Region parseCircle(Element disturbance) {

        if (disturbance.hasAttribute("probability")) {
            disturbanceProbabilities.add(Float.parseFloat(disturbance.getAttribute("probability")));
        }

        tt.euclid2i.Point center = stringToPoint(disturbance.getAttribute("center"));
        int radius = Integer.parseInt(disturbance.getAttribute("radius"));
        return new Circle(center, radius);
    }

    private static tt.euclid2i.Point parsePoint(Element pointElement) {
        NodeList xElement = pointElement.getElementsByTagName("X");
        NodeList yElement = pointElement.getElementsByTagName("Y");

        int x = Integer.valueOf(xElement.item(0).getTextContent());
        int y = Integer.valueOf(yElement.item(0).getTextContent());

        return new tt.euclid2i.Point(x, y);
    }

    private static tt.euclid2i.Point[] stringToPointArray(String pointListStr) {

        String[] pointsStrArray = pointListStr.split(" ");
        LinkedList<tt.euclid2i.Point> points = new LinkedList<tt.euclid2i.Point>();
        for (String pointStr : pointsStrArray) {
            if (!pointStr.isEmpty()) {
                tt.euclid2i.Point point = stringToPoint(pointStr);
                points.add(point);
            }
        }
        return points.toArray(new tt.euclid2i.Point[points.size()]);
    }

    private static tt.euclid2i.Point stringToPoint(String verticeStr) {
        String[] xyStr = verticeStr.split(",");
        tt.euclid2i.Point point = new Point(Integer.parseInt(xyStr[0]), Integer.parseInt(xyStr[1]));
        return point;
    }

    public static String formatOutput(Node node) {
        try {
            Transformer transformer = TransformerFactory.newInstance().newTransformer();
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");

            StreamResult result = new StreamResult(new StringWriter());
            DOMSource source = new DOMSource(node);

            transformer.transform(source, result);
            return result.getWriter().toString();
        } catch (TransformerException e) {

            e.printStackTrace();
            return "";
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static KernelMachine<double[]> trainModel() {

        // We need to convert ArrayList to array because kMeans and gaussian regression methods are required arrays.
        int size = observedProbabilities.size();
        double[][] trainPos = new double[size][2];
        double[] trainProb = new double[size];
        for (int k = 0; k < size; k++) {
            trainPos[k][0] = observedPositions.get(k).x;
            trainPos[k][1] = observedPositions.get(k).y;
            trainProb[k] = observedProbabilities.get(k);
        }

        KMeans kmeans = KMeans.fit(trainPos, 30);
        double[][] centers = kmeans.centroids;

        double r0 = 0.0;
        for (int l = 0; l < centers.length; l++) {
            for (int j = 0; j < l; j++) {
                r0 += MathEx.distance(centers[l], centers[j]);
            }
        }
        r0 /= (2 * centers.length);

        /*

        KernelMachine<double[]> gpr;

        double[] p = new double[size];

        double minRMSE = Double.MAX_VALUE;

        for (double l = 0.1; l <= 100; l *= 10) {

            gpr = GaussianProcessRegression.fit(trainPos, trainProb, new GaussianKernel(r0), l);

            for (int i = 0; i < size; i++) {
                p[i] = gpr.predict(new double[]{trainPos[i][0], trainPos[i][1]});
            }

            double rmse = RMSE.of(trainProb, p);

            System.out.println("l: " + l);

            System.out.println("rmse: " + rmse);

            System.out.println("=====================");

            if (rmse <= minRMSE) {
                minRMSE = rmse;
                optimalLambda = l;
            }
        }

        System.out.println("final lambda: " + optimalLambda);

        return GaussianProcessRegression.fit(trainPos, trainProb, new GaussianKernel(r0), optimalLambda);

        */

        //return GaussianProcessRegression.fit(trainPos, trainProb, centers, new GaussianKernel(r0), 0.1);
        return GaussianProcessRegression.fit(trainPos, trainProb, new GaussianKernel(r0), 0.1);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static void killAt(final long killAtMs, final String summaryPrefix, final int clusters) {
        Thread t = new Thread() {
            @Override
            public void run() {
                while (System.currentTimeMillis() < killAtMs) {
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                    }
                }
                printSummary(summaryPrefix, Status.TIMEOUT, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
                System.exit(0);
            }
        };
        t.start();
    }

    private static void solveTracking(final EarliestArrivalProblem problem, Method method, final Parameters params) {

        // find minimum lengths
        //int[] durationsOverShortestPath = Util.computeDurationOverShortestPath(problem);

        // find trajectories
        final EvaluatedTrajectory[] trajs = Util.findCollisionFreeTrajs(problem, params.timeStep, params.maxTime);

        for (int m=0; m<3; m++) {

            observedPositions.clear();

            observedProbabilities.clear();

            if (isItAllMethods) {
                if (m==0) {
                    method = Method.RMTRACK;
                } else if (m==1) {
                    method = Method.RMTRACK_TFF;
                } else if (m==2) {
                    method = Method.RMTRACK_TFA;
                }
            } else {
                m = 3;
            }



            if (params.showVis) {

                String title = method.toString();
                if (method == Method.RMTRACK_TFF || method == Method.RMTRACK_TFA) {
                    if (isAvoidingDeadlock) {
                        title += " with Deadlock Detection";
                    } else {
                        title += " without Deadlock Detection";
                    }
                }

                VisUtil.initVisualization(problem.getEnvironment(), title, params.bgImageFile, params.timeStep / 2);
                VisUtil.visualizeEarliestArrivalProblem(problem);
                visualizeTrajectories(trajs, params.timeStep);
            }

            Disturbance disturbance = new Disturbance((float) params.disturbanceProbs[0], params.disturbanceSeed, disturbanceRegions, disturbanceProbabilities);
            //final int[] lowerBoundDurations = Util.findLowerBoundDuration(problem, trajs, disturbance);
            //final int[] d0Durations = Util.findLowerBoundDuration(problem, trajs, new Disturbance(0, 1000, 1, problem.nAgents()));

            List<Agent> agents = new LinkedList<>();
            for (int i = 0; i < problem.nAgents(); i++) {
                Agent agent = null;

                switch (method) {
                    case ALLSTOP:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.ALLSTOP);
                        break;

                    case RMTRACK:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.RMTRACK);
                        break;

                    case RMTRACK_TFF:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.RMTRACK_TFF);
                        break;

                    case RMTRACK_TFA:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.RMTRACK_TFA);
                        break;

                    case ORCA_T:
                        agent = new ORCAAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getEnvironment(),
                                trajs[i],
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                disturbance,
                                new Random(params.disturbanceSeed),
                                params.showVis);
                }

                agents.add(i, agent);
            }

            findAllObstacles(agents);
            if (verbose) {
                for (final Agent agent : agents) {
                    TrackingAgent thisAgent = (TrackingAgent) agent;
                    for (int o = 0; o < thisAgent.obstacles.size(); o++) {
                        System.out.println(thisAgent.obstacles.get(o).toString());
                    }
                }
            }

            // it is for drawing coordination space obstacle
            if (false) {
                System.out.println("draw coordination space obstacle");
                TrackingAgent thisAgent = (TrackingAgent) agents.get(8);
                TrackingAgent otherAgent = (TrackingAgent) agents.get(19);
                for (int thisAgentPos = 22000; thisAgentPos <= 23400; thisAgentPos+=100) {
                    for (int otherAgentPos = 29000; otherAgentPos <=30400; otherAgentPos+=100) {
                        double separation = (thisAgent.getRadius() + otherAgent.getRadius());
                        if (thisAgentPos == 22500 && otherAgentPos == 30300) {
                            System.out.print("@");
                        } else {
                            if (thisAgent.getTrajectory().get(thisAgentPos).distance(otherAgent.getTrajectory().get(otherAgentPos)) < separation*0.95) {
                                System.out.print("*");
                            } else {
                                System.out.print("#");
                            }
                        }
                    }
                    System.out.println();
                }

            }


            // simulate execution
            simulate(problem, agents, params);
            //computeStatistics(agents, disturbanceProb,  durationsOverShortestPath, d0Durations, lowerBoundDurations, params);
            VisManager.unregisterLayers();


            if (allDone(agents)) {
                double totalTravelTime = 0;
                for (int i = 0; i < agents.size(); i++) {
                    if (agents.get(i).isAtGoal()) {
                        totalTravelTime += (double) agents.get(i).travelTime / 1000.0;
                        if (verbose) {
                            System.out.println("robot " + i + "'s travel time is " + (double) agents.get(i).travelTime / 1000.0);
                        }
                    }
                }
                System.out.println(totalTravelTime);
                System.out.println(TrackingAgent.numberOfCheckingFlip);
                System.out.println(TrackingAgent.numberOfSuccessfulFlip);
                System.out.println((double) TrackingAgent.timeSpentForFlips/1000);
            } else if (isDeadlock(agents)){
                System.out.println(-1);
                System.out.println(TrackingAgent.numberOfCheckingFlip);
                System.out.println(TrackingAgent.numberOfSuccessfulFlip);
                System.out.println((double) TrackingAgent.timeSpentForFlips/1000);
            }

            if (method == Method.RMTRACK) {
                trainModel();
            }

        }

        if (!params.showVis) {
            System.exit(0);
        }

    }

    private static void findAllObstacles(final List<Agent> agents) {

        for (int i = 0; i < agents.size(); i++) {

            TrackingAgent thisAgent = (TrackingAgent) agents.get(i);

            for (int j = i + 1; j < agents.size(); j++) {

                TrackingAgent otherAgent = (TrackingAgent) agents.get(j);

                // Find all the collisions between robot_i and robot_j.
                int deltaT = 100;
                double separation = (thisAgent.getRadius() + otherAgent.getRadius());
                boolean[][] space = new boolean[(thisAgent.endPosition / deltaT) + 1][(otherAgent.endPosition / deltaT) + 1];
                for (int ti = 0; ti < space.length; ti++) {
                    for (int tj = 0; tj < space[0].length; tj++) {
                        if (thisAgent.getTrajectory().get(ti * deltaT).distance(otherAgent.getTrajectory().get(tj * deltaT)) < separation*0.95) {
                            space[ti][tj] = true;
                        } else {
                            space[ti][tj] = false;
                        }
                    }
                }

                ArrayList<Integer> obstacle_x = new ArrayList<>();
                ArrayList<Integer> obstacle_y = new ArrayList<>();
                // Now, separate the obstacles.
                for (int x = 0; x < space.length; x++) {
                    for (int y = 0; y < space[0].length; y++) {

                        if (space[x][y] == true) {

                            obstacle_x.clear();
                            obstacle_y.clear();
                            dfs(x, y, space, obstacle_x, obstacle_y);

                            int minX = Collections.min(obstacle_x);
                            int maxX = Collections.max(obstacle_x);
                            int minY = Collections.min(obstacle_y);
                            int maxY = Collections.max(obstacle_y);

                            Obstacle o = new Obstacle();
                            o.robot_i = i;
                            o.robot_j = j;
                            o.checkedForFlip = false;
                            o.robot_iLeft = minX * deltaT;
                            o.robot_iRight = maxX * deltaT;
                            o.robot_jBottom = minY * deltaT;
                            o.robot_jTop = maxY * deltaT;

                            o.twin = new Obstacle();
                            o.twin.robot_i = j;
                            o.twin.robot_j = i;
                            o.twin.checkedForFlip = false;
                            o.twin.robot_iLeft = minY * deltaT;
                            o.twin.robot_iRight = maxY * deltaT;
                            o.twin.robot_jBottom = minX * deltaT;
                            o.twin.robot_jTop = maxX * deltaT;

                            if (o.robot_iLeft <= o.robot_jBottom) {
                                o.label = i;
                                o.twin.label = i;
                            } else {
                                o.label = j;
                                o.twin.label = j;
                            }

                            for (int k = 0; k <= thisAgent.endPosition / deltaT; k++) {
                                o.lowestObstacle.add(Integer.MAX_VALUE);
                                o.highestObstacle.add(Integer.MIN_VALUE);
                            }

                            for (int z = 0; z < obstacle_x.size(); z++) {
                                int x_ = obstacle_x.get(z);
                                int y_ = obstacle_y.get(z);
                                if (y_ < o.lowestObstacle.get(x_)) {
                                    o.lowestObstacle.set(x_, y_);
                                }
                                if (y_ > o.highestObstacle.get(x_)) {
                                    o.highestObstacle.set(x_, y_);
                                }
                            }

                            for (int k = 0; k <= otherAgent.endPosition / deltaT; k++) {
                                o.twin.lowestObstacle.add(Integer.MAX_VALUE);
                                o.twin.highestObstacle.add(Integer.MIN_VALUE);
                            }

                            for (int z = 0; z < obstacle_y.size(); z++) {
                                int x_ = obstacle_y.get(z);
                                int y_ = obstacle_x.get(z);
                                if (y_ < o.twin.lowestObstacle.get(x_)) {
                                    o.twin.lowestObstacle.set(x_, y_);
                                }
                                if (y_ > o.twin.highestObstacle.get(x_)) {
                                    o.twin.highestObstacle.set(x_, y_);
                                }
                            }

                            thisAgent.obstacles.add(o);
                            otherAgent.obstacles.add(o.twin);
                            o.twin.twin = o;

                            /*
                            assert o.robot_jBottom == Collections.min(o.lowestObstacle)*deltaT:"robot " + o.robot_j + "'s bottom, " + o.robot_jBottom + ", is NOT equal to min of lowestObstacle," + Collections.min(o.lowestObstacle)*deltaT + " of robot " + o.robot_i;

                            assert o.robot_jTop == Collections.max(o.highestObstacle)*deltaT:"robot " + o.robot_j + "'s top, " + o.robot_jTop + ", is NOT equal to max of highestObstacle," + Collections.max(o.highestObstacle)*deltaT + " of robot " + o.robot_i;

                            assert o.robot_iLeft == Collections.min(o.twin.lowestObstacle)*deltaT:"robot " + o.robot_i + "'s left, " + o.robot_iLeft + ", is NOT equal to min of lowestObstacle of twin," + Collections.min(o.twin.lowestObstacle)*deltaT + " of robot " + o.robot_j;

                            assert o.robot_iRight == Collections.max(o.twin.highestObstacle)*deltaT:"robot " + o.robot_i + "'s right, " + o.robot_iRight + ", is NOT equal to max of highestObstacle of twin," + Collections.max(o.twin.highestObstacle)*deltaT + " of robot " + o.robot_j;
                            */
                        }
                    }
                }
            }
        }

    }

    private static void dfs(int x, int y, boolean[][] space, ArrayList<Integer> obstacle_x, ArrayList<Integer> obstacle_y) {

        if (!space[x][y]) return;

        obstacle_x.add(x);
        obstacle_y.add(y);
        space[x][y] = false;

        if (x < space.length - 1) {
            dfs(x + 1, y, space, obstacle_x, obstacle_y);
        }

        if (x > 0) {
            dfs(x - 1, y, space, obstacle_x, obstacle_y);
        }

        if (y < space[0].length - 1) {
            dfs(x, y + 1, space, obstacle_x, obstacle_y);
        }

        if (y > 0) {
            dfs(x, y - 1, space, obstacle_x, obstacle_y);
        }

    }

    private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
        for (double disturbanceProb : params.disturbanceProbs) {

            int[] durationsOverShortestPath = Util.computeDurationOverShortestPath(problem);

            if (params.showVis) {
                VisUtil.initVisualization(problem.getEnvironment(), "RMTRACK", params.bgImageFile, params.timeStep / 2);
                VisUtil.visualizeEarliestArrivalProblem(problem);
            }

            Disturbance disturbance = new Disturbance((float) disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed, problem.nAgents());

            List<Agent> agents = new LinkedList<>();
            for (int i = 0; i < problem.nAgents(); i++) {
                agents.add(i,
                        new ORCAAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getEnvironment(),
                                problem.getPlanningGraph(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                disturbance,
                                new Random(params.disturbanceSeed),
                                false/*params.showVis*/));
            }

            // simulate execution
            simulate(problem, agents, params);

            int[] zeros = new int[problem.nAgents()];

            computeStatistics(agents, disturbanceProb, durationsOverShortestPath, zeros, zeros, params);
            VisManager.unregisterLayers();
        }

        System.exit(0);
    }

    private static void simulate(final EarliestArrivalProblem problem, List<Agent> agents, final Parameters params) {

        initAgentVisualization(agents, params.timeStep);

        int SIMULATION_STEP_MS = 100;
        int SIMULATE_UNTIL = 600000;

        class SimulationControl {
            public int simulatedTimeMs = 0;
            public float simSpeed = 1;
            public boolean running = true;
        }

        final SimulationControl simControl = new SimulationControl();

        // Simulation Control Layer
        VisManager.registerLayer(SimulationControlLayer.create(new SimulationControlLayer.SimulationControlProvider() {

            @Override
            public void setSpeed(float f) {
                simControl.simSpeed = f;
            }

            @Override
            public void setRunning(boolean running) {
                simControl.running = running;
            }

            @Override
            public boolean isRunning() {
                return simControl.running;
            }

            @Override
            public double getTime() {
                return (simControl.simulatedTimeMs / 1000.0);
            }

            @Override
            public float getSpeed() {
                return simControl.simSpeed;
            }
        }));

//        try {
//            System.in.read();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        while (!allDone(agents) && !isDeadlock(agents)) {

            if (simControl.running == true) {

                for (Agent agent : agents) {
                    agent.update(simControl.simulatedTimeMs, simControl.simulatedTimeMs + SIMULATION_STEP_MS, agents);
                }

                if (verbose) {
                    System.out.println("---");
                }

                if (params.showVis) {
                    try {
                        Thread.sleep((int) ((double) SIMULATION_STEP_MS * (double) simControl.simSpeed));
                    } catch (InterruptedException e) {
                    }
                }

                simControl.simulatedTimeMs += SIMULATION_STEP_MS;
            } else {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                }
            }
        }
    }

    private static void computeStatistics(List<Agent> agents, double disturbance, int[] shortestPathDurations, int[] d0Durations, int[] lbDurations, Parameters params) {
        long spSum = 0;
        long d0Sum = 0;
        long lbSum = 0;

        long travelTimeSum = 0;
        long travelTimeSumSq = 0;

        long prolongSpSum = 0;
        long prolongSpSumSq = 0;

        long prolongD0Sum = 0;
        long prolongD0SumSq = 0;

        long prolongLBSum = 0;
        long prolongLBSumSq = 0;

        long makespanAbs = 0;
        long makespanSPProlong = 0;
        long makespanD0Prolong = 0;
        long makespanLBProlong = 0;


        for (int i = 0; i < agents.size(); i++) {
            Agent agent = agents.get(i);
            spSum += shortestPathDurations[i];
            d0Sum += d0Durations[i];
            lbSum += lbDurations[i];

            LOGGER.debug("Agent " + i + " finished at " + agent.travelTime);

            travelTimeSum += agent.travelTime;
            travelTimeSumSq += travelTimeSum * travelTimeSum;

            long prolongSp = agent.travelTime - shortestPathDurations[i];
            long prolongD0 = agent.travelTime - d0Durations[i];
            long prolongLB = agent.travelTime - lbDurations[i];

            prolongSpSum += prolongSp;
            prolongD0Sum += prolongD0;
            prolongLBSum += prolongLB;

            prolongSpSumSq += prolongSp * prolongSp;
            prolongD0SumSq += prolongD0 * prolongD0;
            prolongLBSumSq += prolongLB * prolongLB;

            if (agent.travelTime > makespanAbs)
                makespanAbs = agent.travelTime;

            if ((agent.travelTime - shortestPathDurations[i]) > makespanSPProlong)
                makespanSPProlong = (agent.travelTime - shortestPathDurations[i]);

            if ((agent.travelTime - d0Durations[i]) > makespanD0Prolong)
                makespanD0Prolong = (agent.travelTime - d0Durations[i]);

            if ((agent.travelTime - lbDurations[i]) > makespanLBProlong)
                makespanLBProlong = (agent.travelTime - lbDurations[i]);
        }

        long n = agents.size();

        Status status;
        if (allDone(agents)) {
            status = Status.SUCCESS;
        } else {
            status = Status.FAIL;
        }

        // status;dprob;dquant;dseed;spSum;d0Sum;lbSum;travelTimeSum;travelTimeSumSq;prolongSpSum;prolongSpSumSq;prolongD0Sum;prolongD0SumSq;prolongLBSum;prolongLBSumSq;makespanAbs;makespanSPProlong;makespanD0Prolong;makespanLBProlong

        printSummary(params.summaryPrefix, status, disturbance, params.disturbanceQuantum, params.disturbanceSeed,
                spSum, d0Sum, lbSum, travelTimeSum, travelTimeSumSq, prolongSpSum, prolongSpSumSq,
                prolongD0Sum, prolongD0SumSq, prolongLBSum, prolongLBSumSq, makespanAbs, makespanSPProlong, makespanD0Prolong, makespanLBProlong);
    }

    private static boolean allDone(List<Agent> agents) {
        for (final Agent agent : agents) {
            agent.isAtGoal();
            if (!agent.isAtGoal()) {
                return false;
            }
        }
        return true;
    }

    private static boolean isDeadlock(List<Agent> agents) {

        boolean isOneWaitingForLabel = false;

        for (final Agent agent : agents) {

            if (agent.currentStatus == Agent.Status.MOVES) {
                return false;
            } else if (agent.currentStatus == Agent.Status.WAITS_FOR_LABEL) {
                isOneWaitingForLabel = true;
            }
        }

        if (isOneWaitingForLabel) {
            return true;
        } else {
            return false;
        }

    }

    private static void visualizeTrajectories(Trajectory[] trajs, int timestep) {

        // disturbance areas
        VisManager.registerLayer(
                KeyToggleLayer.create("d", false,
                        RegionsLayer.create(new RegionsLayer.RegionsProvider() {
                            @Override
                            public Collection<? extends Region> getRegions() {
                                return disturbanceRegions;
                            }
                        }, new Color(255,69,0), new Color(255,69,0)))
        );

        // trajectories
        VisManager.registerLayer(
                KeyToggleLayer.create("t", true,
                        FastTrajectoriesLayer.create(new TrajectoriesProvider() {

                            @Override
                            public Trajectory[] getTrajectories() {
                                return trajs;
                            }
                        }, new ColorProvider() {

                            @Override
                            public Color getColor(int i) {
                                return AgentColors.getColorForAgent(i);
                            }
                        }, 6, timestep)));
    }

    private static void initAgentVisualization(final List<Agent> agents, int timeStep) {

        // goals
        VisManager.registerLayer(
                KeyToggleLayer.create("g", true,
                        LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

                            @Override
                            public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                                LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                                for (int i = 0; i < agents.size(); i++) {
                                    tt.euclid2d.Point pos = agents.get(i).getGoal();

                                    list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                                            (int) agents.get(i).getRadius(), "", AgentColors.getColorForAgent(i).darker(),
                                            null,
                                            null));
                                }

                                return list;
                            }

                        }, new tt.euclid2d.vis.ProjectionTo2d())));

        // positions
        VisManager.registerLayer(
                KeyToggleLayer.create("b", true,
                        LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

                            @Override
                            public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                                LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                                for (int i = 0; i < agents.size(); i++) {
                                    tt.euclid2d.Point pos = agents.get(i).getPosition();


                                    Color textColor = Color.WHITE;
                                    Color fillColor = AgentColors.getColorForAgent(i);

                                    if (agents.get(i) instanceof TrackingAgent) {
                                        TrackingAgent agent = (TrackingAgent) agents.get(i);
                                        if (agent.isCurrentlyWaiting()) {
                                            fillColor = Color.LIGHT_GRAY;
                                            textColor = AgentColors.getColorForAgent(i);
                                        }
                                    }

                                    if (agents.get(i).wasDisturbed()) {
                                        fillColor = Color.black;
                                        textColor = AgentColors.getColorForAgent(i);
                                    }

                                    String text = Integer.toString(i + 1) ; //+ i;

                                    if (agents.get(i).isAtGoal()) {
                                        text = String.format("%.0fs", (double) agents.get(i).travelTime / 1000.0);
                                    }

                                    list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                                            (int) agents.get(i).getRadius(),
                                            text,
                                            AgentColors.getColorForAgent(i),
                                            fillColor,
                                            textColor));
                                }

                                return list;
                            }

                        }, new tt.euclid2d.vis.ProjectionTo2d())));

        // planned positions
        VisManager.registerLayer(
                KeyToggleLayer.create("x", false,
                        LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

                            @Override
                            public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                                LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                                for (int i = 0; i < agents.size(); i++) {
                                    tt.euclid2d.Point pos = agents.get(i).getPlannedPosition();

                                    list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                                            (int) agents.get(i).getRadius(), "", AgentColors.getColorForAgent(i),
                                            null,
                                            Color.DARK_GRAY));
                                }

                                return list;
                            }

                        }, new tt.euclid2d.vis.ProjectionTo2d())));

    }

    enum Status {SUCCESS, FAIL, TIMEOUT}

    static long avg(long sum, long n) {
        return sum / n;
    }

    private static void printSummary(String prefix, Status status, double dprob, int dquant, int dseed, long spSum, long d0Sum, long lbSum, long travelTimeSum, long travelTimeSumSq, long prolongSpSum, long prolongSpSumSq, long prolongD0Sum, long prolongD0SumSq, long prolongLBSum, long prolongLBSumSq, long makespanAbs, long makespanSPProlong, long makespanD0Prolong, long makespanLBProlong) {
        System.out.println(prefix +
                status.toString() + ";" +
                dprob + ";" +
                dquant + ";" +
                dseed + ";" +
                spSum + ";" +
                d0Sum + ";" +
                lbSum + ";" +
                travelTimeSum + ";" +
                travelTimeSumSq + ";" +
                prolongSpSum + ";" +
                prolongSpSumSq + ";" +
                prolongD0Sum + ";" +
                prolongD0SumSq + ";" +
                prolongLBSum + ";" +
                prolongLBSumSq + ";" +
                makespanAbs + ";" +
                makespanSPProlong + ";" +
                makespanD0Prolong + ";" +
                makespanLBProlong + ";");

    }

    static long sd(long sumSq, long mean, long n) {
        double var = (sumSq / n) - (mean * mean);
        return (long) Math.round(Math.sqrt(var));
    }

}
