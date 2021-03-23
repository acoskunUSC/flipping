package cz.agents.rmtrack.agent;

import org.jgrapht.graph.DefaultEdge;

public class RelationshipEdge extends DefaultEdge {

    enum EdgeType {PATH, OBSTACLE};

    private EdgeType edgeType;

    public RelationshipEdge(EdgeType edgeType) {
        this.edgeType = edgeType;
    }

    @Override
    public String toString()
    {
        return "edgeType = " + edgeType;
    }
}
