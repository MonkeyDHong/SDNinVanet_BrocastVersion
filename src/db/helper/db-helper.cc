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
 * Author: Haoliang Chen <chl41993@gmail.com>
 */
#include "db-helper.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {

DbHelper::DbHelper ()
  : m_sr (814)
{
  m_agentFactory.SetTypeId ("ns3::db::RoutingProtocol");
}

DbHelper::DbHelper (const DbHelper &o)
  : m_agentFactory (o.m_agentFactory),
    m_sr (o.m_sr)
{
  m_ntmap = o.m_ntmap;
}

DbHelper*
DbHelper::Copy () const
{
  return new DbHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
DbHelper::Create (Ptr<Node> node) const
{
  Ptr<db::RoutingProtocol> agent = m_agentFactory.Create<db::RoutingProtocol> ();

  Ptr<MobilityModel> temp = node -> GetObject<MobilityModel> ();
  agent->SetMobility (temp);

  std::map< Ptr<Node>, db::NodeType >::const_iterator it3 = m_ntmap.find (node);
  if (it3 != m_ntmap.end ())
    {
      agent->SetType (it3->second);
    }
  else
    {
      agent->SetType (db::OTHERS);
    }
  agent->SetSignalRange (m_sr);


  node->AggregateObject (agent);
  return agent;
}

void
DbHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

int64_t
DbHelper::AssignStreams (NodeContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<Node> node;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      node = (*i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      NS_ASSERT_MSG (ipv4, "Ipv4 not installed on node");
      Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol ();
      NS_ASSERT_MSG (proto, "Ipv4 routing not installed on node");
      Ptr<db::RoutingProtocol> db = DynamicCast<db::RoutingProtocol> (proto);
      if (db)
        {
          currentStream += db->AssignStreams (currentStream);
          continue;
        }
      // Db may also be in a list
      Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting> (proto);
      if (list)
        {
          int16_t priority;
          Ptr<Ipv4RoutingProtocol> listProto;
          Ptr<db::RoutingProtocol> listDb;
          for (uint32_t i = 0; i < list->GetNRoutingProtocols (); i++)
            {
              listProto = list->GetRoutingProtocol (i, priority);
              listDb = DynamicCast<db::RoutingProtocol> (listProto);
              if (listDb)
                {
                  currentStream += listDb->AssignStreams (currentStream);
                  break;
                }
            }
        }
    }
  return (currentStream - stream);

}

void
DbHelper::SetNodeTypeMap (Ptr<Node> node, db::NodeType nt)
{
  std::map< Ptr<Node> , db::NodeType >::iterator it = m_ntmap.find(node);

  if (it != m_ntmap.end() )
    {
      std::string temp = "Duplicate NodeType on Node: " + std::to_string (node->GetId());
      NS_ASSERT_MSG (false, temp);
    }
  m_ntmap[node] = nt;
}

void
DbHelper::SetSR(double signal_range)
{
  m_sr = signal_range;
}

} // namespace ns3
