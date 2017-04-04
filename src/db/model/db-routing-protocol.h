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

#ifndef DB_IMPL_H
#define DB_IMPL_H

#include "db-header.h"
#include "db-duplicate-detection.h"
#include "db-port-position-match.h"

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/udp-header.h"
#include "ns3/tcp-header.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-module.h"

#include <vector>
#include <map>
#include <set>

namespace ns3 {
namespace db {

std::string
Ipv4toString (const Ipv4Address& address);

enum NodeType {CAR, LOCAL_CONTROLLER, OTHERS};
enum Algo {Yangs_Algo, Binary_Search};


class RoutingProtocol;

/// \brief DB routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId (void);

  RoutingProtocol ();
  virtual ~RoutingProtocol ();

  ///
  /// \brief Set the DB CCH and SCH Interface to the indicated interface
  /// \param interface IPv4 interface index
  ///
  void SetSCHInterface (uint32_t interface);
  void SetCCHInterface (uint32_t interface);

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);

protected:
  virtual void DoInitialize (void);
private:
	
  /// Packets sequence number counter.
  uint16_t m_packetSequenceNumber;
  /// Messages sequence number counter.
  uint16_t m_messageSequenceNumber;

  /// HELLO messages' emission interval.
  Time m_helloInterval;

  Ptr<Ipv4> m_ipv4;


  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                      const Ipv4Header &header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);
  virtual bool RouteInput (Ptr<const Packet> p,
                           const Ipv4Header &header,
                           Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb,
                           MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb,
                           ErrorCallback ecb);
  // Inherited from Ipv4Routing. Void Methods.
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);

  // Initialized the System.
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;
  void DoDispose ();

  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);

  /// Increments packet sequence number and returns the new value.
  inline uint16_t GetPacketSequenceNumber ();
  /// Increments message sequence number and returns the new value.
  inline uint16_t GetMessageSequenceNumber ();

  void RecvDB (Ptr<Socket> socket);

  // Timer handlers
  Timer m_helloTimer;
  void HelloTimerExpire ();

  /// A list of pending messages which are buffered awaiting for being sent.
  db::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; // timer for throttling outgoing messages

  void QueueMessage (const db::MessageHeader &message, Time delay);
  void SendQueuedMessages ();
  void SendHello ();

  void ProcessHM (const db::MessageHeader &msg);

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;

private:
  // The following two address can get by Calling
  // m_ipv4->GetAddress(m_CCHinterface, 0).GetLocal() OR
  // m_ipv4->GetAddress(m_SCHinterface, 0).GetLocal(),
  // but for my conveniences.
  Ipv4Address m_CCHAddress;
  Ipv4Address m_SCHAddress;
  uint32_t m_SCHinterface;
  uint32_t m_CCHinterface;
  // One socket per interface, each bound to that interface's address
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;

  TracedCallback <const PacketHeader &,
                  const MessageList &> m_rxPacketTrace;
  TracedCallback <const PacketHeader &,
                  const MessageList &> m_txPacketTrace;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;  

  // Mobility module for Vanet
  Ptr<MobilityModel> m_mobility;

public:
  void SetMobility (Ptr<MobilityModel> mobility);
private:

  double m_road_length;
  double m_signal_range;
  double m_safety_raito;

public:
  void SetSignalRange (double signal_range);

private:
  static bool Comp (const std::pair<double, Ipv4Address> &p1, const std::pair<double, Ipv4Address> &p2);
  static double CalcDist (const Vector3D &pos1, const Vector2D &pos2);

  Port_Position_Match m_PPM;
  Duplicate_Detection m_DD;
  std::map<Ipv4Address, Vector3D> IP2Pos;
  void Senducb (UnicastForwardCallback ucb,Ptr<Ipv4Route> broadcastRoute,Ptr<const Packet> p,Ipv4Header ipHeader);
private:
  NodeType m_nodetype;

public:
  void SetType (NodeType nt);
  NodeType GetType () const;
//statistic tools //todo change to private and wirte get
public:
  int m_numofhm, m_numofapp, m_numofackhello,m_numofdontforward,m_numoflc2lc;
  int m_numofdatapacket;
  std::set<Ipv4Address> m_allforwardcar;
};


}
}  // namespace ns3

#endif /* DB_IMPL_H */
