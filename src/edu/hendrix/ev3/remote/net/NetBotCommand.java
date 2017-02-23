package edu.hendrix.ev3.remote.net;

import edu.hendrix.ev3.remote.Move;

public class NetBotCommand {
	private Move move;
	private byte[] message;
	private boolean quit;
	
	public NetBotCommand(Move move) {
		this(move, new byte[0], false);
	}
	
	public NetBotCommand(Move move, byte[] message, boolean quit) {
		this.move = move;
		this.message = message;
		this.quit = quit;
	}
	
	public static NetBotCommand makeQuit() {
		return new NetBotCommand(Move.STOP, new byte[0], true);
	}
	
	public Move getMove() {return move;}
	
	public boolean hasMessage() {return message.length > 0;}
	
	public byte[] getMessage() {return message;}
	
	public boolean shouldQuit() {return quit;}
}
