package edu.hendrix.ev3.ai.bsoc;

import java.util.ArrayList;

import edu.hendrix.ev3.ai.cluster.Clusterable;
import edu.hendrix.ev3.ai.cluster.Clusterer;
import edu.hendrix.ev3.ai.cluster.DistanceFunc;
import edu.hendrix.ev3.util.DeepCopyable;
import edu.hendrix.ev3.util.FixedSizeArray;
import edu.hendrix.ev3.util.Util;

import java.util.TreeSet;
import java.util.function.Function;

// This data structure is an adaptation of the idea of Agglomerative Clustering.
// 
// Traditional agglomerative clustering is concerned with finding a hierarchical relationship
// among the data elements.
//
// Our goal here is to create an online learning algorithm with fast and predictable 
// runtime performance, suitable for both supervised and unsupervised learning.

public class BoundedSelfOrgCluster<T extends Clusterable<T> & DeepCopyable<T>> implements Clusterer<T>, DeepCopyable<BoundedSelfOrgCluster<T>> {
	// Object state
	private FixedSizeArray<Node<T>> nodes;
	private ArrayList<TreeSet<Edge<T>>> nodes2edges;
	private TreeSet<Edge<T>> edges;
	
	public int maxNumNodes() {return nodes.capacity() - 1;}
	
	// Higher-order function
	private DistanceFunc<T> dist;
	
	// Notification
	private ArrayList<BSOCListener> listeners = new ArrayList<>();

	@Override
	public BoundedSelfOrgCluster<T> deepCopy() {
		BoundedSelfOrgCluster<T> result = new BoundedSelfOrgCluster<>(size(), dist);
		deepCopyHelp(result);
		return result;
	}
	
	protected void deepCopyHelp(BoundedSelfOrgCluster<T> result) {
		for (Edge<T> edge: this.edges) {
			result.edges.add(edge.deepCopy());
		}
		result.nodes = this.nodes.deepCopy();
	}

	public BoundedSelfOrgCluster(int maxNumNodes, DistanceFunc<T> dist) {
		setupBasic(dist);
		setupAvailable(maxNumNodes);
	}
	
	private void setupBasic(DistanceFunc<T> dist) {
		this.dist = dist;
		this.edges = new TreeSet<>();		
		this.nodes2edges = new ArrayList<>();
	}
	
	private void setupAvailable(int maxNumNodes) {
		this.nodes = FixedSizeArray.make(maxNumNodes + 1);
		Util.assertState(size() == 0, "size() should be zero, but is " + size());
	}
	
	public BoundedSelfOrgCluster(String src, Function<String,T> extractor, DistanceFunc<T> dist) {
		setupBasic(dist);
		ArrayList<String> topLevel = Util.debrace(src);
		fromStringHelp(topLevel, extractor);
	}
	
	protected void fromStringHelp(ArrayList<String> topLevel, Function<String,T> extractor) {
		rebuildAvailable(topLevel.get(0));
		if (topLevel.size() > 1) {
			rebuildNodes(topLevel.get(1), extractor);
		}
		if (topLevel.size() > 2) {
			rebuildEdges(topLevel.get(2));
		}
	}
	
	@Override
	public String toString() {
		StringBuilder result = new StringBuilder();
		result.append("{");
		result.append(maxNumNodes());
		result.append("}\n{");
		nodes.doAll((i, v) -> {
			result.append('{');
			result.append(v);
			result.append('}');
		});
		result.append("}\n{");
		for (Edge<T> edge: edges) {
			result.append('{');
			result.append(edge.toString());
			result.append('}');
		}
		result.append("}");
		return result.toString();
	}
	
	public boolean nodeExists(int node) {
		return nodes.containsKey(node);
	}
	
	public void addListener(BSOCListener listener) {
		listeners.add(listener);
	}
	
	private void rebuildAvailable(String availStr) {
		ArrayList<String> availability = Util.debrace(availStr);
		int maxNumNodes = Integer.parseInt(availability.get(0));
		setupAvailable(maxNumNodes);
	}
	
	private void rebuildNodes(String nodeStr, Function<String,T> extractor) {
		for (String node: Util.debrace(nodeStr)) {
			Node<T> newNode = new Node<>(node, extractor);
			nodes.put(newNode.getID(), newNode);
			nodes2edges.add(new TreeSet<>());
		}
	}
	
	private void rebuildEdges(String edgeStr) {
		for (String edge: Util.debrace(edgeStr)) {
			Edge<T> newEdge = new Edge<>(edge);
			edges.add(newEdge);
			nodes2edges.get(newEdge.getNode1()).add(newEdge);
			nodes2edges.get(newEdge.getNode2()).add(newEdge);
		}
	}
	
	public int size() {return nodes.size();}
	
	public int getStartingLabel() {return 0;}
	
	public long distanceToClosestMatch(T example) {
		return getNodeRanking(example).get(0).getSecond();
	}
	
	private long distance(Node<T> n1, Node<T> n2) {
		return Math.max(n1.getNumInputs(), n2.getNumInputs()) * dist.distance(n1.getCluster(), n2.getCluster());
	}
	
	private void removeAllEdgesFor(int node) {
		for (Edge<T> edge: nodes2edges.get(node)) {
			edges.remove(edge);
			nodes2edges.get(edge.getOtherNode(node)).remove(edge);
		}
		nodes2edges.get(node).clear();
	}
	
	private void createEdgesFor(int node) {
		for (int i = nodes.getLowestInUse(); i < nodes.capacity(); i = nodes.nextInUse(i)) {
			if (i != node) {
				long distance = distance(nodes.get(i), nodes.get(node));
				Edge<T> edge = new Edge<>(Math.min(i, node), Math.max(i, node), distance);
				edges.add(edge);
				nodes2edges.get(node).add(edge);
				nodes2edges.get(i).add(edge);
			}
		}
	}

	@Override
	public int train(T example) {
		int where = nodes.getLowestAvailable();
		insert(new Node<>(where, example));
		notifyAdd(where);
		if (nodes.size() > maxNumNodes()) {
			where = removeAndMerge();
		}
		Util.assertState(nodes.getHighestInUse() == nodes.size() - 1, "Not compact");
		return getClosestMatchFor(example);
	}
	
	private void insert(Node<T> example) {
		nodes.put(example.getID(), example);
		Util.assertState(example == nodes.get(example.getID()), "Went to the wrong place");
		Util.assertState(nodes2edges.size() >= example.getID(), String.format("nodes2edges mismatch! exampleID: %d nodes2edges.size: %d", example.getID(), nodes2edges.size()));
		if (example.getID() == nodes2edges.size()) {
			nodes2edges.add(new TreeSet<>());
		}
		createEdgesFor(example.getID());
	}

	private int removeAndMerge() {
		Edge<T> smallest = edges.first();
		Node<T> removedNode = removeNode(smallest.getNode2());
		Node<T> absorberNode = removeNode(smallest.getNode1());
		
		Node<T> merged = absorberNode.mergedWith(removedNode);
		insert(merged);
		notifyReplace(removedNode.getID(), absorberNode.getID());
		
		int unused = removedNode.getID();
		if (unused > nodes.getHighestInUse()) {
			return absorberNode.getID();
		} else {
			Node<T> tooHighNode = removeNode(nodes.getHighestInUse());
			tooHighNode.renumber(unused);
			insert(tooHighNode);
			return tooHighNode.getID();
		}
	}
	
	private Node<T> removeNode(int target) {
		removeAllEdgesFor(target);
		return nodes.remove(target);
	}

	private void notifyAdd(int added) {
		for (BSOCListener listener: listeners) {
			listener.addingNode(added);
		}
	}
	
	private void notifyReplace(int original, int replacement) {
		for (BSOCListener listener: listeners) {
			listener.replacingNode(original, replacement);
		}
	}
	
	public boolean edgeRepresentationConsistent() {
		for (int i = 0; i < nodes2edges.size(); i++) {
			if (nodeExists(i)) {
				for (Edge<T> edge: nodes2edges.get(i)) {
					if (!edges.contains(edge)) {
						return false;
					}
				}
			} else {
				if (!nodes2edges.get(i).isEmpty()) {
					return false;
				}
			}
		}
		
		for (Edge<T> edge: edges) {
			if (!nodeExists(edge.getNode1())) {return false;}
			if (!nodes2edges.get(edge.getNode1()).contains(edge)) {return false;}
			if (!nodeExists(edge.getNode2())) {return false;}
			if (!nodes2edges.get(edge.getNode2()).contains(edge)) {return false;}
		}
		
		return true;
	}
	
	@Override
	public T getIdealInputFor(int node) {
		Util.assertArgument(nodes.containsKey(node), "Node " + node + " not present");
		return nodes.get(node).getCluster();
	}
	
	public int getNumMergesFor(int node) {
		Util.assertArgument(nodes.containsKey(node), "Node " + node + " not present");
		return nodes.get(node).getNumInputs();
	}
	
	public int getTotalSourceInputs() {
		int total = 0;
		for (Node<T> node: nodes.values()) {
			total += node.getNumInputs();
		}
		return total;
	}

	@Override
	public ArrayList<Integer> getClusterIds() {
		return nodes.indices();
	}
	
	public ArrayList<T> getIdealInputs() {
		ArrayList<T> result = new ArrayList<T>();
		for (Node<T> n: nodes.values()) {
			result.add(n.getCluster());
		}
		return result;
	}

	@Override
	public DistanceFunc<T> getDistanceFunc() {
		return dist;
	}
	
	@Override
	public boolean equals(Object other) {
		return toString().equals(other.toString());
	}
	
	@Override
	public int hashCode() {
		return toString().hashCode();
	}
}
