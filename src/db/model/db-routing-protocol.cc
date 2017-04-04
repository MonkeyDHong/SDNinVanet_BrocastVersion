/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Haoliang Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Haoliang Chen <chl41993@gmail.com>
 */


///
/// \brief Implementation of DB agent on car side 
/// and related classes.
///
/// This is the main file of this software because DB's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "db-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"

#include <ostream>
#include <algorithm>
/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))

/********** Miscellaneous constants **********/

/// Maximum allowed jitter.
#define DB_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Random number between [0-DB_MAXJITTER] used to jitter DB packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, DB_MAXJITTER)))

#define DB_MAX_SEQ_NUM        65535

//For CCH
#define DB_PORT_NUMBER 419
/// Maximum number of messages per packet.
#define DB_MAX_MSGS    64

#define INFHOP 2147483647

namespace ns3 {
namespace db {

NS_LOG_COMPONENT_DEFINE ("DbRoutingProtocol");

//Double ABS
double dabs(double x)
{
	return x > 0 ? x : -x;
}

std::string Ipv4toString(const Ipv4Address& address)
{
	std::ostringstream buffer("");
	address.Print(buffer);
	return buffer.str();
}

/********** DB controller class **********/

NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);

TypeId RoutingProtocol::GetTypeId()
{
	static TypeId tid = TypeId("ns3::db::RoutingProtocol")
			.SetParent<Ipv4RoutingProtocol>()
			.AddConstructor<RoutingProtocol>();
	return tid;
}


RoutingProtocol::RoutingProtocol ()
  :
    m_packetSequenceNumber (DB_MAX_SEQ_NUM),
    m_messageSequenceNumber (DB_MAX_SEQ_NUM),
    m_ipv4 (0),
    m_SCHinterface (0),
    m_CCHinterface (0),
	//period function timer
    m_helloInterval (Seconds(0.5)),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
	//algorithm parameter
    m_nodetype (CAR),
	//some variable setting
    m_road_length (814),//MagicNumber
    m_signal_range (419),
    m_safety_raito (0.9),
	//statistic
	m_numofhm (0),
	m_numofapp (0),
	m_numofackhello (0),
	m_numofdontforward (0),
	m_numoflc2lc (0),
	m_numofdatapacket (0)
{
	m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol()
{

}

void
RoutingProtocol::SetCCHInterface(uint32_t interface)
{
	m_CCHinterface = interface;
	m_CCHAddress = m_ipv4->GetAddress(m_CCHinterface, 0).GetLocal();
}

void
RoutingProtocol::SetSCHInterface (uint32_t interface)
{
	m_SCHinterface = interface;
	m_SCHAddress = m_ipv4->GetAddress(m_SCHinterface, 0).GetLocal();
}


int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
	NS_LOG_FUNCTION(this << stream);
	m_uniformRandomVariable->SetStream(stream);
	return 1;
}

void
RoutingProtocol::DoInitialize ()
{

	Ipv4Address loopback("127.0.0.1");

	bool canRunDb = false;
	//Install RecvDB  Only on CCH channel.
	if (m_CCHAddress != Ipv4Address::GetZero())
	{
		// Create a socket to listen only on this interface
		Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(),
				UdpSocketFactory::GetTypeId());
		// TRUE
		socket->SetAllowBroadcast(true);
		InetSocketAddress inetAddr(
				m_ipv4->GetAddress(m_CCHinterface, 0).GetLocal(),
				DB_PORT_NUMBER);
		socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvDB, this));
		if (socket->Bind(inetAddr))
		{
			NS_FATAL_ERROR("Failed to bind() DB socket");
		}
		socket->BindToNetDevice(m_ipv4->GetNetDevice(m_CCHinterface));
		m_socketAddresses[socket] = m_ipv4->GetAddress(m_CCHinterface, 0);

		canRunDb = true;
		m_allforwardcar.clear();
	}

	if (canRunDb)
	{
		HelloTimerExpire();
		NS_LOG_DEBUG("DB on node (Car) " << m_CCHAddress << " started");
	}
}
void
RoutingProtocol::SetMobility (Ptr<MobilityModel> mobility)
{
	m_mobility = mobility;
}
void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{

}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
	for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
			m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
	{
		Ipv4InterfaceAddress iface = j->second;
		if (a == iface.GetLocal())
		{
			return true;
		}
	}
	if (a == m_SCHAddress)
	{
		return true;
	}
	return false;
}
Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
             const Ipv4Header &header,
             Ptr<NetDevice> oif,
             Socket::SocketErrno &sockerr)
{
	NS_LOG_FUNCTION(this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
	Ptr<Ipv4Route> rtentry;

    uint32_t interfaceIdx = m_SCHinterface;
    Ipv4InterfaceAddress ifAddr;
    ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
    rtentry = Create<Ipv4Route> ();
    rtentry->SetSource (ifAddr.GetLocal ());
    rtentry->SetGateway (Ipv4Address::GetBroadcast());
    //rtentry->SetGateway (Ipv4Address::GetZero());
    rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
	return rtentry;
}

bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header &header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,
                            ErrorCallback ecb)
{
	NS_LOG_FUNCTION(
			this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());
	// if(header.GetDestination ().Get ()%256 != 255)
	//std::cout<<"RouteInput "<<header.GetSource ().Get ()%256<< ", "<<m_SCHmainAddress.Get () %256<< ",Dest:"<<header.GetDestination ().Get ()%256<<std::endl;
	//bool lcb_status = false;
	Ipv4Address dest = header.GetDestination();
	Ipv4Address sour = header.GetSource();
	int iden = header.GetIdentification();
	UdpHeader uheader;
	p->PeekHeader(uheader);
	//std::cout<<"iden:"<<iden<<std::endl;
	if (m_DD.CheckThis(p)|| !m_PPM.CheckThis(uheader.GetDestinationPort(), m_mobility->GetPosition()))
		return true;
	double distance = 0;
	Vector3D thispos = m_mobility->GetPosition();
	std::map<Ipv4Address, Vector3D>::const_iterator iiit = IP2Pos.find(sour);
	if (iiit != IP2Pos.end() && (iiit->second.x > thispos.x))
		distance = CalculateDistance(iiit->second, thispos);
	// Consume self-originated packets
	if (IsMyOwnAddress(sour) == true)
	{
		return true;
	}

	if (header.GetTtl() == 0)	  //avoid broadcast storm
	{
		return true;
	}
	// Local delivery
	NS_ASSERT(m_ipv4->GetInterfaceForDevice(idev) >= 0);
	uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);	  //SCH dev!
	if (m_ipv4->IsDestinationAddress(dest, iif))
	{
		//Local delivery
		if (!lcb.IsNull())
		{
			NS_LOG_LOGIC("Broadcast local delivery to " << dest);
			//std::cout<<"Broadcast local delivery to "<<std::endl;
			lcb(p, header, iif);
			/*if ((m_SCHmainAddress.Get ()%256 == 53)&&(iif=m_SCHinterface))
			 {
			 std::cout<<m_SCHmainAddress.Get ()%256<<" "<<header.GetDestination ().Get () %256<<std::endl;
			 std::cout<<"YES!"<<std::endl;
			 }*/
			return true;
		}
		else
		{
			NS_LOG_ERROR(
					"Unable to deliver packet locally due to null callback");
			ecb(p, header, Socket::ERROR_NOROUTETOHOST);
			return false;
		}

	}
	//Broadcast forwardfind(numBitmapIp.begin(),numBitmapIp.end(),citi->second)-

	if ((iif == m_SCHinterface) && (m_nodetype == CAR))	//todo to do sth to guarantee forward once
	{
		NS_LOG_LOGIC("Forward broadcast");
		Ptr<Ipv4Route> broadcastRoute = Create<Ipv4Route>();
		// broadcastRoute->SetGateway (dest);//broadcast
		broadcastRoute->SetGateway(Ipv4Address::GetBroadcast());
		broadcastRoute->SetOutputDevice(m_ipv4->GetNetDevice(m_SCHinterface));
		Ipv4Header ipHeader = header;
		ipHeader.SetSource(m_ipv4->GetAddress(m_SCHinterface, 0).GetLocal()); //To Prevent Brocast Storm, m_CCHmainAddress is for CCH.
		ipHeader.SetTtl(ipHeader.GetTtl() - 1);
		if (ipHeader.GetTtl() != 0)
		{
			Time sendInterval;
			if (distance > 0)
			{
				sendInterval = Seconds(1 / (distance * 100));
				Simulator::Schedule(sendInterval, &RoutingProtocol::Senducb,
						this, ucb, broadcastRoute, p, ipHeader);
			}
			else
				ucb(broadcastRoute, p, ipHeader);
			// std::cout<<"FORWARD,UCB,TTL:"<<int(ipHeader.GetTtl())<<std::endl;
		}
	}
	return true;
}
void RoutingProtocol::Senducb (UnicastForwardCallback ucb,Ptr<Ipv4Route> broadcastRoute,Ptr<const Packet> p,Ipv4Header ipHeader)
{
	ucb (broadcastRoute, p, ipHeader);
}
void
RoutingProtocol::SetType (NodeType nt)
{
	m_nodetype = nt;
}

NodeType
RoutingProtocol::GetType () const
{
	return m_nodetype;
}
void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created db::RoutingProtocol");
  m_helloTimer.SetFunction
    (&RoutingProtocol::HelloTimerExpire, this);
  m_queuedMessagesTimer.SetFunction
    (&RoutingProtocol::SendQueuedMessages, this);

  m_packetSequenceNumber = DB_MAX_SEQ_NUM;
  m_messageSequenceNumber = DB_MAX_SEQ_NUM;

  m_ipv4 = ipv4;
}



void RoutingProtocol::DoDispose ()
{
	m_ipv4 = 0;

	for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter =
			m_socketAddresses.begin(); iter != m_socketAddresses.end(); ++iter)
	{
		iter->first->Close();
	}
	m_socketAddresses.clear();

	Ipv4RoutingProtocol::DoDispose();
}

// DB packets actually send here.
void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
	NS_LOG_DEBUG("DB node " << m_CCHAddress << " sending a DB packet");
	// Add a header
	db::PacketHeader header;
	header.originator = m_CCHAddress; //CCH Address
	header.SetPacketLength(header.GetSerializedSize() + packet->GetSize());
	header.SetPacketSequenceNumber(GetPacketSequenceNumber());
	packet->AddHeader(header);

	// Trace it
	m_txPacketTrace(header, containedMessages);

	// Send it
	for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
			m_socketAddresses.begin(); i != m_socketAddresses.end(); ++i)
	{
		Ipv4Address bcast = i->second.GetLocal().GetSubnetDirectedBroadcast(
				i->second.GetMask());
		i->first->SendTo(packet, 0, InetSocketAddress(bcast, DB_PORT_NUMBER));
	}
}

uint16_t
RoutingProtocol::GetPacketSequenceNumber ()
{
	m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (DB_MAX_SEQ_NUM + 1);
	return m_packetSequenceNumber;
}

uint16_t
RoutingProtocol::GetMessageSequenceNumber ()
{
	m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (DB_MAX_SEQ_NUM + 1);
	return m_messageSequenceNumber;
}

//
// \brief Processes an incoming %DB packet (Car Side).
void
RoutingProtocol::RecvDB (Ptr<Socket> socket)
{
	Ptr<Packet> receivedPacket;
	Address sourceAddress;
	receivedPacket = socket->RecvFrom(sourceAddress);

	InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(
			sourceAddress);
	Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4();
	Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal();
	NS_ASSERT(receiverIfaceAddr != Ipv4Address());
	NS_LOG_DEBUG(
			"DB node " << m_CCHAddress << " received a DB packet from " << senderIfaceAddr << " to " << receiverIfaceAddr);

	// All routing messages are sent from and to port DB_PORT_NUMBER,
	// so we check it.
	NS_ASSERT(inetSourceAddr.GetPort () == DB_PORT_NUMBER);

	Ptr<Packet> packet = receivedPacket;

	db::PacketHeader dbPacketHeader;
	packet->RemoveHeader(dbPacketHeader);
	NS_ASSERT(
			dbPacketHeader.GetPacketLength()
					>= dbPacketHeader.GetSerializedSize());
	uint32_t sizeLeft = dbPacketHeader.GetPacketLength()
			- dbPacketHeader.GetSerializedSize();

	MessageList messages;

	while (sizeLeft)
	{
		MessageHeader messageHeader;
		if (packet->RemoveHeader(messageHeader) == 0)
			NS_ASSERT(false);

		sizeLeft -= messageHeader.GetSerializedSize();

		NS_LOG_DEBUG(
				"DB Msg received with type " << std::dec << int (messageHeader.GetMessageType ()) << " TTL=" << int (messageHeader.GetTimeToLive ()) << " SeqNum=" << messageHeader.GetMessageSequenceNumber ());
		messages.push_back(messageHeader);
	}

	m_rxPacketTrace(dbPacketHeader, messages);

	for (MessageList::const_iterator messageIter = messages.begin();
			messageIter != messages.end(); ++messageIter)
	{
		const MessageHeader &messageHeader = *messageIter;
		// If ttl is less than or equal to zero, or
		// the receiver is the same as the originator,
		// the message must be silently dropped
		if ((messageHeader.GetTimeToLive() == 0)
				|| (IsMyOwnAddress(dbPacketHeader.originator)))
		{
			// ignore it
			packet->RemoveAtStart(messageHeader.GetSerializedSize());
			continue;
		}

		switch (messageHeader.GetMessageType())
		{
		case db::MessageHeader::HELLO_MESSAGE:
			NS_LOG_DEBUG(Simulator::Now ().GetSeconds () << "s DB node " << m_CCHAddress << " received Routing message of size " << messageHeader.GetSerializedSize ());
			//Car Node should discare Hello_Message
				ProcessHM(messageHeader);
			break;
		default:
			NS_LOG_DEBUG("DB message type " << int (messageHeader.GetMessageType ()) << " not implemented");
		}
	}
}// End of RecvDB

void
RoutingProtocol::HelloTimerExpire ()
{
	SendHello();
	m_helloTimer.Schedule(m_helloInterval);
}

void
RoutingProtocol::QueueMessage (const db::MessageHeader &message, Time delay)
{
	m_queuedMessages.push_back(message);
	if (not m_queuedMessagesTimer.IsRunning())
	{
		m_queuedMessagesTimer.SetDelay(delay);
		m_queuedMessagesTimer.Schedule();
	}
}

// NS3 is not multithread, so mutex is unnecessary.
// Here, messages will queue up and send once numMessage is equl to DB_MAX_MSGS.
// This function will NOT add a header to each message
void
RoutingProtocol::SendQueuedMessages ()
{
	Ptr<Packet> packet = Create<Packet>();
	int numMessages = 0;

	NS_LOG_DEBUG("DB node " << m_CCHAddress << ": SendQueuedMessages");
	MessageList msglist;

	for (std::vector<db::MessageHeader>::const_iterator message =
			m_queuedMessages.begin(); message != m_queuedMessages.end();
			++message)
	{
		Ptr<Packet> p = Create<Packet>();
		p->AddHeader(*message);
		packet->AddAtEnd(p);
		msglist.push_back(*message);
		if (++numMessages == DB_MAX_MSGS)
		{
			SendPacket(packet, msglist);
			msglist.clear();
			// Reset variables for next packet
			numMessages = 0;
			packet = Create<Packet>();
		}
	}

	if (packet->GetSize())
	{
		SendPacket(packet, msglist);
	}

	m_queuedMessages.clear();
}

void
RoutingProtocol::SendHello ()
{
	NS_LOG_FUNCTION(this);
	db::MessageHeader msg;
	Time now = Simulator::Now();
	msg.SetVTime(m_helloInterval);
	msg.SetTimeToLive(41993);		//Just MY Birthday.
	msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
	msg.SetMessageType(db::MessageHeader::HELLO_MESSAGE);

	db::MessageHeader::Hello &hello = msg.GetHello();
	hello.ID = m_SCHAddress;
	Vector pos = m_mobility->GetPosition();
	Vector vel = m_mobility->GetVelocity();
	hello.SetPosition(pos.x, pos.y, pos.z);
	hello.SetVelocity(vel.x, vel.y, vel.z);

	NS_LOG_DEBUG("DB HELLO_MESSAGE sent by node: " << hello.ID
			<< "   at " << now.GetSeconds() << "s");
	QueueMessage(msg, JITTER);
	//statistic
	m_numofhm++;
}


void RoutingProtocol::ProcessHM(const db::MessageHeader &msg)
{
	Ipv4Address ID = msg.GetHello().ID;		//should be SCH address
	if (IP2Pos.find(ID) == IP2Pos.end())
		IP2Pos.insert(
				std::map<Ipv4Address, Vector3D>::value_type(ID,
						msg.GetHello().GetPosition()));
	else
		IP2Pos.find(ID)->second = msg.GetHello().GetPosition();
}



void
RoutingProtocol::SetSignalRange (double signal_range)
{
	m_signal_range = signal_range;
}

bool
RoutingProtocol::Comp (const std::pair<double, Ipv4Address> &p1, const std::pair<double, Ipv4Address> &p2)
{
	return p1.first > p2.first;
}

double
RoutingProtocol::CalcDist (const Vector3D &pos1, const Vector2D &pos2)
{
	return CalculateDistance(Vector2D(pos1.x, pos1.y), pos2);
}




} // namespace db
} // namespace ns3


