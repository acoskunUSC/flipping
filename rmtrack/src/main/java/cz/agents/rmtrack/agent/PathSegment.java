package cz.agents.rmtrack.agent;

public class PathSegment {

    public int id; // robot's id
    public int start; // the number of path step when robot reaches the path segment
    public int finish; // the number of path step when robot clears the path segment

    // succeeding path segment's start is equal to preceding path segment's final

    public PathSegment(int id, int start, int finish) {
        this.id = id;
        this.start = start;
        this.finish = finish;
    }

    @Override
    public String toString() {
        return "PathSegment{" +
                "id=" + id +
                ", start=" + start +
                ", finish=" + finish +
                '}';
    }

}
