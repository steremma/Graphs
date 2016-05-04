/**
 * A minimal implementation of a Queue, using a LinkedList for maximum efficiency.
 * The functionality is pretty straight-forward
 */
package roadgraph;

import java.util.LinkedList;

public class Queue<T> {
	
	private LinkedList<T> lst;
	
	/**
	 * Construct an empty Queue
	 */
	public Queue() {
		lst = new LinkedList<T>();
	}
	
	/**
	 * Check if queue is empty
	 * @return true if queue is empty
	 */
	public boolean isEmpty() {
		return lst.size() == 0;
	}
	
	/**
	 * Insert an element at the back of the queue
	 * @param element to be inserted
	 * @return true if element was successfully inserted
	 */
	public boolean enqueue(T element) {
		if(element == null) {
			return false;
		}
		lst.add(element);
		return true;	
	}
	
	/**
	 * Extract the front element, throw exception if queue is empty.
	 */
	public T dequeue() {
		if(this.isEmpty()) {
			throw new IllegalStateException("the Queue is empty!");
		}
		return lst.poll();
	}
	
	/**
	 * Check the queue's current size
	 * @return size
	 */
	public int size() {
		return lst.size();
	}
}
