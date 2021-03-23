package cz.agents.rmtrack.util;

import tt.euclid2i.Point;
import tt.euclid2i.Region;

import java.util.List;
import java.util.Random;

public class Disturbance {

    private final boolean[][] disturbances;
    private final int[] offsets;
    float probability;
    int timeQuantum;
    int seed;

    List<Region> disturbanceRegions;
    List<Float> disturbanceProbabilities;
    Random random;


    public Disturbance(float probability, int timeQuantum, int seed, int nAgents) {
        this.probability = probability;
        this.timeQuantum = timeQuantum;
        this.seed = seed;

        Random random = new Random(seed);

        int len = 10000;
        disturbances = new boolean[nAgents][len];
        for (int i=0; i<nAgents; i++) {
            for (int t=0; t<len; t++)
            disturbances[i][t] = random.nextFloat() < probability;
        }

        offsets = new int[nAgents];
        for (int i=0; i<nAgents; i++) {
            offsets[i] = -random.nextInt(timeQuantum);
        }

    }

    public Disturbance(float defaultProbability, int seed, List<Region> disturbanceRegions, List<Float> disturbanceProbabilities) {
        this.probability = defaultProbability;
        this.seed = seed;
        this.disturbanceRegions = disturbanceRegions;
        this.disturbanceProbabilities = disturbanceProbabilities;
        random = new Random(seed);
        disturbances = null;
        offsets = null;
    }

    public boolean isDisturbed(Point position) {
        for (int i=0; i<disturbanceRegions.size(); i++) {
            if (disturbanceRegions.get(i).isInside(position)) {
                return random.nextFloat() < disturbanceProbabilities.get(i);
            }
        }
        return random.nextFloat() < probability;
    }

    public boolean isDisturbed(int i, int time){
        int tq = (time + offsets[i])/ timeQuantum;
        return disturbances[i][tq];
    }

    public float getProbability() {
        return probability;
    }
}
