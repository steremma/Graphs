package roadgraph;

import java.util.List;
import java.util.Map;

import geography.GeographicPoint;

public abstract class ShortestPathFinder {
	
	protected Map<GeographicPoint,Node> nodeMap;
	
	public ShortestPathFinder(Map<GeographicPoint,Node> nodeMap) {
		this.nodeMap = nodeMap;
	}
	
	/** Find the map's key given its value.
	 *   @param n The map value
	 *   @return p The key mapping to i
	 */
	protected GeographicPoint reverseMapping(Node n) {
		for(Map.Entry<GeographicPoint, Node> entry : nodeMap.entrySet()) {
			if(entry.getValue() == n) {
				return entry.getKey();
			}
		}
		throw new IllegalArgumentException("no point has an id of " + n);
	}
	
	public abstract List<GeographicPoint> search(GeographicPoint start, GeographicPoint goal);
}
