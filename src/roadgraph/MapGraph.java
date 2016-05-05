/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;



import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	private int numEdges;
	private int numVertices;
	// It makes no sense for a neighbor to be included twice.
	private Map<GeographicPoint,Node> nodeMap;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodeMap = new HashMap<GeographicPoint,Node>();
		numEdges = 0;
		numVertices = 0;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return nodeMap.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}



	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(nodeMap.containsKey(location) || location == null) {
			return false;
		}

		nodeMap.put(location,new Node());
		numVertices++;
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {	

		if(length < 0 || !nodeMap.containsKey(from) || !nodeMap.containsKey(to) || from == null || to == null) {
			throw new IllegalArgumentException();
		}
		Edge e = new Edge(nodeMap.get(to),roadName,roadType,length);
		boolean added = nodeMap.get(from).addNeighbor(e);
		if(added) {
			numEdges++;
		}
	}


	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
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
				return getPath(parent,g,s);
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
	private List<GeographicPoint> getPath(HashMap<Node,Node> parent,Node goal,Node start) {
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
		return getPath(path);
	}

	/** Transform the inner representation of the path to the form expected by client code
	 * 
	 * @param nPath the inner representation of the path
	 * @return path the path in terms of Geographic points
	 */
	private List<GeographicPoint> getPath(List<Node> nPath) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		for(Node i : nPath) {
			path.add(reverseMapping(i));
		}
		return path;

	}

	/** Find the map's key given its value.
	 *   @param n The map value
	 *   @return p The key mapping to i
	 */
	private GeographicPoint reverseMapping(Node n) {
		for(Map.Entry<GeographicPoint, Node> entry : nodeMap.entrySet()) {
			if(entry.getValue() == n) {
				return entry.getKey();
			}
		}
		throw new IllegalArgumentException("no point has an id of " + n);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<SearchNode> pr = new PriorityQueue<SearchNode>();
		Set<Node> visited = new HashSet<Node>();
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		SearchNode s = new SearchNode(nodeMap.get(start),0,null);
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
			if(curr == g) {
				//return getPath(parent,g,s);
				return dijkstraPath(g);
			}
			for(Edge e : curr.node.getNeighbors()) {
				double newDistance = curr.distance + e.getDistance();
				// I don't need to check for the previous distance, the shorter one 
				// is guaranteed to be dequeued first.
				pr.add(new SearchNode(e.getDestination(),newDistance,curr));
				//parent.put(dest, curr);
			}
		}
		return null;
	}
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
		@Override
		public int compareTo(SearchNode other) {
			// TODO Auto-generated method stub
			if(this.distance < other.distance) {
				return -1;
			}
			else return 1;
		}


	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return null;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		// You can use this method for testing.  

		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		 */

	}

}