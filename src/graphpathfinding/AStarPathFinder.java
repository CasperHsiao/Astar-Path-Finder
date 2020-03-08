package graphpathfinding;

import priorityqueues.DoubleMapMinPQ;
import priorityqueues.ExtrinsicMinPQ;
import timing.Timer;

import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @see ShortestPathFinder for more method documentation
 */
public class AStarPathFinder<VERTEX> extends ShortestPathFinder<VERTEX> {
    private final AStarGraph<VERTEX> graph;
    public static final double START_DIST = 0.0;


    /**
     * Creates a new AStarPathFinder that works on the provided graph.
     */
    public AStarPathFinder(AStarGraph<VERTEX> graph) {
        this.graph = graph;
    }

    @Override
    public ShortestPathResult<VERTEX> findShortestPath(VERTEX start, VERTEX end, Duration timeout) {
        Timer timer = new Timer(timeout);
        ExtrinsicMinPQ<VERTEX> fringe = new DoubleMapMinPQ<VERTEX>();
        Map<VERTEX, Double> distTo = new HashMap<VERTEX, Double>();
        Map<VERTEX, VERTEX> edgeTo = new HashMap<VERTEX, VERTEX>();
        Map<VERTEX, Double> priorities = new HashMap<VERTEX, Double>();
        List<VERTEX> solution = new ArrayList<>();
        double solutionWeight = START_DIST;
        int numStatesExplored = 0;


        double heuristic = this.graph.estimatedDistanceToGoal(start, end);
        fringe.add(start, START_DIST + heuristic);
        distTo.put(start, START_DIST);
        priorities.put(start, START_DIST + heuristic);
        solution.add(start);

        if (start.equals(end)) {
            return new ShortestPathResult.Solved<>(solution, solutionWeight, numStatesExplored,
                timer.elapsedDuration());
        }

        while (!fringe.isEmpty() && !timer.isTimeUp()) {
            VERTEX current = fringe.removeMin();
            if (current.equals(end)) {
                VERTEX temp = end;
                while (!temp.equals(start)) {
                    solution.add(1, temp);
                    temp = edgeTo.get(temp);
                }
                solutionWeight = distTo.get(end);
                return new ShortestPathResult.Solved<>(solution, solutionWeight, numStatesExplored,
                    timer.elapsedDuration());
            }
            numStatesExplored++;
            double currentDistance = distTo.get(current);
            for (WeightedEdge<VERTEX> e : this.graph.neighbors(current)) {
                VERTEX neighbor = e.to();
                if (!distTo.containsKey(neighbor)) {
                    distTo.put(neighbor, Double.POSITIVE_INFINITY);
                }
                double neighborDistance = distTo.get(neighbor);
                if (neighborDistance < currentDistance + e.weight()) {
                    continue;
                } else {
                    neighborDistance = currentDistance + e.weight();
                    distTo.put(neighbor, neighborDistance);
                    edgeTo.put(neighbor, current);
                    heuristic = this.graph.estimatedDistanceToGoal(neighbor, end);
                    priorities.put(neighbor, neighborDistance + heuristic);
                    if (!fringe.contains(neighbor)) {
                        fringe.add(neighbor, neighborDistance + heuristic);
                    } else {
                        fringe.changePriority(neighbor, neighborDistance + heuristic);
                    }
                }
                // if (neighbor.equals(end)) {
                //     solved = true;
                //     solutionWeight = neighborDistance;
                //     double minWeight = -1.0;
                //     if (!fringe.isEmpty()) {
                //         minWeight = priorities.get(fringe.peekMin());
                //     }
                //     if (minWeight >= solutionWeight) {
                //         while (!neighbor.equals(start)) {
                //             solution.add(1, neighbor);
                //             neighbor = edgeTo.get(neighbor);
                //         }
                //         return new ShortestPathResult.Solved<>(solution, solutionWeight, numStatesExplored,
                //             timer.elapsedDuration());
                //     }
                // }
            }
        }
        // if (solved) {
        //     VERTEX current = end;
        //     while (!current.equals(start)) {
        //         solution.add(1, current);
        //         current = edgeTo.get(current);
        //     }
        //     solutionWeight = distTo.get(end);
        //     return new ShortestPathResult.Solved<>(solution, solutionWeight, numStatesExplored,
        //         timer.elapsedDuration());
        // }
        if (timer.isTimeUp()) {
            return new ShortestPathResult.Timeout<>(numStatesExplored, timer.elapsedDuration());
        }
        return new ShortestPathResult.Unsolvable<>(numStatesExplored, timer.elapsedDuration());
    }

    @Override
    protected AStarGraph<VERTEX> graph() {
        return this.graph;
    }
}
