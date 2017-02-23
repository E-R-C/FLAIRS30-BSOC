package edu.hendrix.ev3.remote.net;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public interface TaggedMessage {
	public byte getTag();
	public byte[] toBytes();
	
	default public void ship(DatagramSocket sock) throws IOException {
		ship(sock, InetAddress.getByAddress(RobotConstants.addr));
	}
	
	default public void ship(DatagramSocket sock, InetAddress target) throws IOException {
		byte[] bytes = toBytes();
		DatagramPacket info = new DatagramPacket(bytes, bytes.length, target, RobotConstants.MSG_PORT);
		sock.send(info);
	}
}
