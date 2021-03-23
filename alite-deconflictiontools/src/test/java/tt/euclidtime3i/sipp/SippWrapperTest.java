package tt.euclidtime3i.sipp;

import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.ExpansionListener;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.ZeroHeuristic;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import tt.euclid2i.*;
import tt.euclid2i.Point;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.probleminstance.RandomEnvironment;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.trajectory.SegmentedTrajectoryFactory;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclidtime3i.L1Heuristic;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.discretization.WaitToMaxTimeExtension;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.sipp.SippEdge;
import tt.euclidtime3i.sipp.SippGoal;
import tt.euclidtime3i.sipp.SippHeuristic;
import tt.euclidtime3i.sipp.SippNode;
import tt.euclidtime3i.sipp.SippUtils;
import tt.euclidtime3i.sipp.SippWrapper;
import tt.euclidtime3i.sipprrts.DynamicObstacles;
import tt.euclidtime3i.sipprrts.DynamicObstaclesImpl;
import tt.euclidtime3i.vis.TimeParameter;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.generator.ConflictGenerator;
import tt.jointeuclid2ni.probleminstance.generator.exception.ProblemNotCreatedException;
import tt.util.AgentColors;
import tt.util.Common;
import tt.vis.*;

import javax.vecmath.Point2d;

import java.awt.*;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;


/**
 * This is only rought test. It is impossible to compare the SIPP exactly to the Full-space-time search! Sometimes there
 * might not even exist a solution while using current implementation of the SIPP algorithm which uses safe intervals for both
 * edges and verties of the graph.
 */

public class SippWrapperTest {

    private static final int MAX_SPEED = 1;
    private static final int MAX_TIME = 3000;
    private static final int REGION_SIZE = 1000;
    private static final int OBSTACLES = 10;
    private static final int OBSTACLE_SIZE = 200;
    private static final int BODY_RADIUS = 50;
    private static final int GRID_STEP = 50;

    @Before
    public void setUp() throws Exception {
        //printStream = new PrintStream(new FileOutputStream(new File("sipp.out")));
        //System.setOut(printStream);
        //initVisualization();
        //Verbose.setVerbose(true);
    }

    @After
    public void tearDown() throws Exception {
        //Thread.sleep(Long.MAX_VALUE);
        //printStream.flush();
        //printStream.close();
    }

    //@Test
    public void testAgainstPP() {
        //interesting case: 103 - no solution can be find by the SIPP algorithm
        for (int seed = 1; seed <= 10; seed++) {
            try {
                testOnSingleInstance(seed);
            } catch (ProblemNotCreatedException e) {
                //System.out.println("Problem could not be created");
                //fail();
            }
        }
    }

    static int i = 0;

    private void testOnSingleInstance(int seed) throws ProblemNotCreatedException {
        EarliestArrivalProblem problem = generateProblem(seed);
        DirectedGraph<Point, Line> graph = createGrid(problem.getEnvironment(), LazyGrid.PATTERN_4_WAY_WAIT);

        //visualizeProblem(problem, graph);

        SegmentedTrajectory firstTrajectory = solveFirstMission(problem, graph);

        SegmentedTrajectory expected = solveInFullSpaceTime(problem, firstTrajectory, graph);
        SegmentedTrajectory sipp = solveBySIPP(problem, firstTrajectory, graph);

        solution[0] = firstTrajectory;
        solution[1] = expected;
        solution[2] = sipp;

        assertEquals(expected == null, sipp == null);
        assertEquals(expected != null, sipp != null);

        //if (expected.getCost() != sipp.getCost())
        //    System.out.printf("%d %d %f %f %n", i++, seed, expected.getCost(), sipp.getCost());

        assertEquals(expected.getCost(), sipp.getCost(), 2 * GRID_STEP + 1);
    }

    private EarliestArrivalProblem generateProblem(int seed) throws ProblemNotCreatedException {
        RandomEnvironment environment = new RandomEnvironment(REGION_SIZE, REGION_SIZE, OBSTACLES, OBSTACLE_SIZE, 100);
        int[] bodyRadiuses = Common.prefillArray(2, BODY_RADIUS);
        float[] maxSpeeds = {1f,1f};
        int[][] pattern = LazyGrid.PATTERN_4_WAY;
        Rectangle bounds = environment.getBoundary().getBoundingBox();

        int maxTime = 1000;
		return ConflictGenerator.generateInstance(environment, bodyRadiuses, maxSpeeds, pattern, GRID_STEP, bounds, maxTime, true, false, false, new Random(seed));
    }

    private DirectedGraph<Point, Line> createGrid(Environment environment, int[][] pattern) {
        Point initialPoint = new Point(0, 0);
        Collection<Region> obstacles = environment.getObstacles();
        Rectangle bounds = environment.getBoundary().getBoundingBox();

        return new LazyGrid(initialPoint, obstacles, bounds, pattern, GRID_STEP).generateFullGraph();
    }

    private SegmentedTrajectory solveFirstMission(EarliestArrivalProblem problem, DirectedGraph<Point, Line> graph) {
        GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(graph, new ZeroHeuristic<Point>()
                , problem.getStart(0), problem.getTarget(0));
        return SegmentedTrajectoryFactory.createConstantSpeedTrajectory(path, 0, problem.getMaxSpeed(0), MAX_TIME, path.getWeight());
    }

    private SegmentedTrajectory solveInFullSpaceTime(EarliestArrivalProblem problem, SegmentedTrajectory firstTrajectory, DirectedGraph<Point, Line> graph) {
        MovingCircle dynamicObstacle = new MovingCircle(firstTrajectory, BODY_RADIUS * 2);
        List<MovingCircle> dynamicObstacles = Collections.singletonList(dynamicObstacle);

        ConstantSpeedTimeExtension timeExtension = new ConstantSpeedTimeExtension(graph, MAX_TIME, new float[]{MAX_SPEED},
                dynamicObstacles, 50);

        //DirectedGraph<tt.euclidtime3i.Point, Straight> motionGraph = new FreeOnTargetWaitExtension(timeExtension, problem.getTarget(1));
        WaitToMaxTimeExtension motionGraph = new WaitToMaxTimeExtension(timeExtension, Collections.singletonList(dynamicObstacle), problem.getTarget(1), MAX_TIME);

        HeuristicToGoal<tt.euclidtime3i.Point> heuristic = new L1Heuristic(problem.getTarget(1));
        tt.euclidtime3i.Point start = new tt.euclidtime3i.Point(problem.getStart(1), 0);
        tt.euclidtime3i.Point goal = new tt.euclidtime3i.Point(problem.getTarget(1), MAX_TIME);

        GraphPath<tt.euclidtime3i.Point, Straight> path =
                AStarShortestPathSimple.findPathBetween(motionGraph, heuristic, start, goal);

        return new StraightSegmentTrajectory(path, MAX_TIME);
    }

    private SegmentedTrajectory solveBySIPP(final EarliestArrivalProblem problem, SegmentedTrajectory firstTrajectory, DirectedGraph<Point, Line> graph) {
        DynamicObstacles environment = new DynamicObstaclesImpl(new SegmentedTrajectory[]{firstTrajectory}, new int[]{BODY_RADIUS}, MAX_TIME);

        SippWrapper wrapper = new SippWrapper(graph, environment, problem.getBodyRadius(1), problem.getMaxSpeed(1), 2, MAX_TIME);
        SippGoal goal = new SippGoal(problem.getTarget(1), MAX_TIME);
        SippNode start = wrapper.wrapNode(problem.getStart(1), 0);

        HeuristicToGoal<SippNode> heuristic = new SippHeuristic(new tt.euclid2i.discretization.L2Heuristic(problem.getTarget(1)), 1);
        AStarShortestPathSimple<SippNode, SippEdge> astar = new AStarShortestPathSimple<SippNode, SippEdge>(wrapper, heuristic, start, goal);
        astar.addExpansionListener(new ExpansionListener<SippNode>() {
            @Override
            public void exapanded(SippNode node) {
                node.getSafeInterval();
            }
        });
        GraphPath<SippNode, SippEdge> path = astar.findPath(Integer.MAX_VALUE);
        //GraphPath<SippNode, SippEdge> path = AStarShortestPathSimple.findPathBetween(wrapper, heuristic, start, goal);
        return SippUtils.parseTrajectory(path, MAX_TIME);
    }

    private static Trajectory[] solution = new Trajectory[3];

    private void initVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 200, 200);
        VisManager.setSceneParam(new VisManager.SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(500, 500);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.5;
            }
        });
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

    }

    private void visualizeProblem(final EarliestArrivalProblem problem, final Graph<Point, Line> graph) {

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsLayer.RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();
                        list.add(problem.getEnvironment().getBoundary());
                        return list;
                    }

                }, Color.BLACK, Color.WHITE
        ));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsLayer.RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        return problem.getObstacles();
                    }

                }, Color.GRAY, Color.GRAY
        ));

        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointLayer.LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPointLayer.LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPointLayer.LabeledPoint<Point>> list = new LinkedList<LabeledPointLayer.LabeledPoint<Point>>();

                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getStart(0), "s" + 0));
                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getStart(1), "s" + 1));
                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getStart(1), "s" + 2));

                return list;
            }

        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.BLUE));


        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointLayer.LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPointLayer.LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPointLayer.LabeledPoint<Point>> list = new LinkedList<LabeledPointLayer.LabeledPoint<Point>>();

                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getTarget(0), "g" + 0));
                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getTarget(1), "g" + 1));
                list.add(new LabeledPointLayer.LabeledPoint<Point>(problem.getTarget(1), "g" + 2));

                return list;
            }

        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.RED));

        final TimeParameter time = new TimeParameter(10);
        VisManager.registerLayer(ParameterControlLayer.create(time));

        // visualize the graph
        VisManager.registerLayer(GraphLayer.create(new GraphLayer.GraphProvider<Point, Line>() {

            @Override
            public Graph<Point, Line> getGraph() {
                return graph;
            }
        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.GRAY, Color.GRAY, 1, 4));

        KeyToggleLayer bKeyLayer = KeyToggleLayer.create("b");
        bKeyLayer.addSubLayer(ColoredTrajectoriesLayer.create(
                new ColoredTrajectoriesLayer.TrajectoriesProvider<Point>() {
                    @Override
                    public tt.discrete.Trajectory<Point>[] getTrajectories() {
                        return solution;
                    }
                }, new ColoredTrajectoriesLayer.ColorProvider() {
                    @Override
                    public Color getColor(int i) {
                        return AgentColors.getColorForAgent(i);
                    }
                }, new tt.euclid2i.vis.ProjectionTo2d(), 10, MAX_TIME, 6, 's'
        ));
        bKeyLayer.addSubLayer(FastAgentsLayer.create(
                new FastAgentsLayer.TrajectoriesProvider() {

                    @Override
                    public Trajectory[] getTrajectories() {
                        return solution;
                    }

                    @Override
                    public int[] getBodyRadiuses() {
                        return new int[]{BODY_RADIUS, BODY_RADIUS, BODY_RADIUS};
                    }

                }, new FastAgentsLayer.ColorProvider() {
                    @Override
                    public Color getColor(int i) {
                        return AgentColors.getColorForAgent(i);
                    }
                }, time
        ));

        VisManager.registerLayer(bKeyLayer);
        VisManager.registerLayer(VisInfoLayer.create());
        VisManager.init();
    }
}
