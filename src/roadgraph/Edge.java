package roadgraph;

/**
 * A class modeling an immutable edge between 2 vertices. Each edge belongs to the Node it starts from,
 * therefore only the destination and additional properties should be fields.
 * @author Manos
 *
 */
public class Edge {
	private Node destination;
	private String type;
	private String name;
	private double distance;
	
	/**
	 * Constructor using all fields
	 * @param destination the GeographicPoint the edge leads to
	 * @param name the road's name
	 * @param type the road's type
	 * @param distance the distance in km
	 */
	public Edge(Node destination, String name, String type, double distance) {
		super();
		this.destination = destination;
		this.type = type;
		this.name = name;
		this.distance = distance;
	}

	public Node getDestination() {
		return destination;
	}

	public String getType() {
		return type;
	}

	public String getName() {
		return name;
	}

	public double getDistance() {
		return distance;
	}	

}
