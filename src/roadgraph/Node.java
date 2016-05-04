package roadgraph;

import java.util.HashSet;
import java.util.Set;

/**
 * A class modeling a Vertex in the Graph. Maintains a set of edges starting from this node
 * and reaching other nodes. Essentially every edge corresponds to a neighbor
 * @author Manos
 *
 */
public class Node {
	private Set<Edge> neighbors;
	
	/**
	 * Default constructor, initialize an empty list of neighbors
	 */
	public Node() {
		neighbors = new HashSet<Edge>();
	}

	public Set<Edge> getNeighbors() {
		return new HashSet<Edge>(neighbors);
	}	
	
	/**
	 * add an edge coming out of this Node.
	 * @param e the edge to be added
	 * @return true if the edge did not already exist and is therefore successfully added
	 */
	public boolean addNeighbor(Edge e) {
		return neighbors.add(e);
	}

}
