package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import geography.GeographicPoint;

public class BfsPathFinder extends ShortestPathFinder {

	public BfsPathFinder(Map<GeographicPoint, Node> nodeMap) {
		super(nodeMap);
	}

	@Override
	public List<GeographicPoint> search(GeographicPoint start, GeographicPoint goal) {
		// Initialization
		Queue<Node> q = new Queue<Node>();
		HashMap<Node,Node> parent = new HashMap<Node,Node>();
		Set<Node> visited = new HashSet<Node>();
		Node s = nodeMap.get(start);
		Node g = nodeMap.get(goal);
		Node curr = s;
		q.enqueue(curr);

		// Loop
		while(!q.isEmpty()) {
			curr = q.dequeue();
			visited.add(curr);
			if(curr == g) {
				return getBfsPath(parent,g,s);
			}

			for(Edge e : curr.getNeighbors()) {
				Node dest = e.getDestination();
				if(!visited.contains(dest)) {
					q.enqueue(dest);
					parent.put(dest, curr);
				}	
			}
		}
		return null;	
	}

	/** Construct the path leading from start to goal
	 * 
	 * @param parent a collection mapping each Vertex to the Vertex that led to it
	 * @param goal The final vertex in the path
	 * @param start The starting vertex
	 * @return
	 */
	private List<GeographicPoint> getBfsPath(HashMap<Node,Node> parent,Node goal,Node start) {
		List<Node> path = new LinkedList<Node>();
		path.add(goal);
		Node next = null;
		// Iterate generations backwards starting from goal until we reach the starting point.
		while(next != start) {
			next = parent.get(path.get(path.size() - 1));
			path.add(next);
		}
		// Client code expects the path from start to finish, not the opposite.
		Collections.reverse(path);
		return getBfsPath(path);
	}

	/** Transform the inner representation of the path to the form expected by client code
	 * 
	 * @param nPath the inner representation of the path
	 * @return path the path in terms of Geographic points
	 */
	private List<GeographicPoint> getBfsPath(List<Node> nPath) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		for(Node i : nPath) {
			path.add(reverseMapping(i));
		}
		return path;
	}
}
