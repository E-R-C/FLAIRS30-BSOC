package edu.hendrix.ev3.remote.net;

import java.net.InetAddress;

abstract public class ReturnableTaggedMessage implements TaggedMessage {
	private InetAddress sender;
	
	public ReturnableTaggedMessage() {sender = null;}
	
	abstract public boolean keepGoing();
	
	public InetAddress getSender() {return sender;}
	public void setSender(InetAddress sender) {this.sender = sender;}
	public boolean senderKnown() {return getSender() != null;}
}
