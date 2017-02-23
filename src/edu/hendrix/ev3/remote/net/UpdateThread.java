package edu.hendrix.ev3.remote.net;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.function.Function;

public class UpdateThread<T> extends Thread {
	private ArrayList<UpdateListener<T>> listeners = new ArrayList<>();
	private boolean quit = false;
	private DatagramSocket sock;
	private Function<byte[],T> converter;
	private int expectedBytes;
	
	public UpdateThread(DatagramSocket sock, Function<byte[],T> converter, int expectedBytes) {
		this.sock = sock;
		this.converter = converter;
		this.expectedBytes = expectedBytes;
	}
	
	public void addListener(UpdateListener<T> listener) {
		listeners.add(listener);
	}
	
	@Override
	public void run() {
		try {
			while (!quit) {
				byte[] bytes = new byte[expectedBytes];
				DatagramPacket incoming = new DatagramPacket(bytes, bytes.length);
				sock.receive(incoming);
				T reported = converter.apply(bytes);
				for (UpdateListener<T> listener: listeners) {
					listener.report(reported);
				}
			}
		} catch (SocketException socky) {
			socky.printStackTrace();
		} catch (IOException ioe) {
			ioe.printStackTrace();
		} catch (Exception exc) {
			exc.printStackTrace();
		}
	}
}
