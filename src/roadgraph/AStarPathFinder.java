package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import geography.GeographicPoint;

public class AStarPathFinder extends ShortestPathFinder{
	
	// boolean field to determine the desired search algorithm. True for Djikstra's, false for a*.
	private boolean pureDjikstra;
	
	public AStarPathFinder(Map<GeographicPoint,Node> nodeMap,boolean pureDjikstra) {
		super(nodeMap);
		this.pureDjikstra = pureDjikstra;
	}
	
	/** A helper function aiming to remove code duplication by performing
	 * either Djikstra's algorithm or Astar Search on the graph. 
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param pureDjikstra true for djikstra's, false for AstarSearch
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> djikstraHelper(GeographicPoint start, 
			GeographicPoint goal,boolean pureDjikstra) {

		// TODO: Implement this method in WEEK 3
		PriorityQueue<SearchNode> pr = new PriorityQueue<SearchNode>();
		Set<Node> visited = new HashSet<Node>();
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		double startDistance = pureDjikstra ? 0 : start.distance(goal);
		SearchNode s = new SearchNode(nodeMap.get(start),startDistance,null);
		SearchNode g = new SearchNode(nodeMap.get(goal));
		SearchNode curr = s;
		pr.add(curr);
		// Loop
		while(!pr.isEmpty()) {
			curr = pr.poll();
			if(visited.contains(curr.node)) {
				continue;
			}
			visited.add(curr.node);
			if(curr.node == g.node) {
				//return getPath(parent,g,s);
				return dijkstraPath(curr);
			}
			for(Edge e : curr.node.getNeighbors()) {
				double newDistance = curr.distance + e.getDistance();
				if(!pureDjikstra) {
					newDistance += curr.estimateDistance(g);
				}
				// I don't need to check for the previous distance, the shorter one 
				// is guaranteed to be dequeued first.
				pr.add(new SearchNode(e.getDestination(),newDistance,curr));
			}
		}
		return null;
	}
	
	/** Recreate the shortest path from found by the algorithm by iterating backwards.
	 *  Each SearchNode holds a reference to its successor.
	 * @param goal the final SearchNode of the path
	 * @return The shortest path from start to goal
	 */
	private List<GeographicPoint> dijkstraPath(SearchNode goal) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(reverseMapping(goal.node));
		SearchNode previous = goal;
		while(previous.parent != null) {
			previous = previous.parent;
			path.add(reverseMapping(previous.node));
		}
		Collections.reverse(path);
		return path;
	}

	/** A wrapper class around my Node object.
	 *  This class is used by Djikstra's algorithm as it also contains a cost field,
	 *  distance as well as the parent Node leading to the current one. The class implements 
	 *  Combarable in order to facilitate priorityQueye entries.
	 *  
	 * @author manos
	 */
	private class SearchNode implements Comparable<SearchNode>{
		public double distance;
		public Node node;
		public SearchNode parent;
		public SearchNode(Node node) {
			this(node,Double.POSITIVE_INFINITY,null);
		}

		public SearchNode(Node node,double distance,SearchNode parent) {
			this.node = node;
			this.distance = distance;
			this.parent = parent;
		}
		public double estimateDistance(SearchNode other) {
			GeographicPoint start = reverseMapping(this.node);
			GeographicPoint goal = reverseMapping(other.node);
			return start.distance(goal);
		}
		@Override
		public int compareTo(SearchNode other) {
			// TODO Auto-generated method stub
			if(this.distance < other.distance) {
				return -1;
			}
			else return 1;
		}
	}

	@Override
	public List<GeographicPoint> search(GeographicPoint start, GeographicPoint goal) {
		// TODO Auto-generated method stub
		return djikstraHelper(start,goal,pureDjikstra);
	}

}
