package edu.hendrix.ev3.ai.cluster;

public interface BinaryFoldingOperator<T> {
	public T apply(int x, int y, T start);
}
