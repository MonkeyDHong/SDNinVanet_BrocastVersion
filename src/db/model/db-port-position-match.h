/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Da Hong
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
 * Author: Da Hong <bh.dong@foxmail.com>
 */

#ifndef DB_PORT_POSITION_MATCH__H
#define DB_PORT_POSITION_MATCH__H
#include <list>
#include <unordered_map>
#include <set>
#include "ns3/hash.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

#define ControlAreaList std::list<std::pair<Vector2D, Vector2D>>
#define ControlArea std::pair<Vector2D, Vector2D>

namespace ns3
{
namespace db
{

//std::pair<Vector2D, Vector2D> ControlArea;

class Port_Position_Match
{
public:
	uint32_t m_port1;
	uint32_t m_port2;
	uint32_t m_port3;
	uint32_t m_port4;
	uint32_t m_port5;
	uint32_t m_port6;
	uint32_t m_port7;
	uint32_t m_port8;

	Port_Position_Match();

	bool CheckThis(uint16_t port, Vector position);
	bool IsInTheArea(Vector position, ControlArea area);
private:
	std::unordered_map<uint16_t, ControlAreaList> m_controlAreaMap;
};

}
}


#endif //DB_PORT_POSITION_MATCH__H
