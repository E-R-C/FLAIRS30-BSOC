package edu.hendrix.ev3.ai.bsoc;

import java.util.ArrayList;
import java.util.function.Function;

import edu.hendrix.ev3.ai.cluster.Clusterable;
import edu.hendrix.ev3.util.DeepCopyable;
import edu.hendrix.ev3.util.Util;

public class Node<T extends Clusterable<T> & DeepCopyable<T>> implements DeepCopyable<Node<T>> {
	private T cluster;
	private int numInputSources, id;
	
	Node(int id, T cluster) {
		this(id, cluster.deepCopy(), 1);
	}
	
	private Node(int id, T cluster, int ancestors) {
		this.id = id;
		this.cluster = cluster;
		this.numInputSources = ancestors;
	}
	
	void renumber(int newID) {
		this.id = newID;
	}
	
	Node(String src, Function<String,T> extractor) {
		ArrayList<String> parts = Util.debrace(src);
		id = Integer.parseInt(parts.get(0));
		numInputSources = Integer.parseInt(parts.get(1));
		cluster = extractor.apply(parts.get(2));
	}
	
	Node<T> mergedWith(Node<T> other) {
		return mergedWith(other.cluster, other.numInputSources);
	}
	
	Node<T> mergedWith(T example) {
		return mergedWith(example, 1);
	}

	private Node<T> mergedWith(T example, int otherInputs) {
		return new Node<>(this.id, this.cluster.weightedCentroidWith(example, this.numInputSources, otherInputs),
				this.numInputSources + otherInputs);
	}

	@Override
	public String toString() {
		return "{" + id + "}{" + numInputSources + "}{" + cluster + "}";
	}

	@Override
	public Node<T> deepCopy() {
		return new Node<>(id, cluster.deepCopy(), numInputSources);
	}
	
	T getCluster() {return cluster;}
	int getID() {return id;}
	int getNumInputs() {return numInputSources;}
}
