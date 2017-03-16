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

#include "sdn-port-position-match.h"

//#define ControlArea std::pair<Vector2D, Vector2D>

namespace ns3{
namespace sdn{

Port_Position_Match::Port_Position_Match ()
{
	m_port1 = 65419;
	m_port2 = 65420;
	m_port3 = 65421;
	m_port4 = 65422;
	m_port5 = 65423;
	m_port6 = 65424;
	m_port7 = 65425;
	m_port8 = 65426;


	m_controlAreaMap.clear();
	ControlAreaList templist;
	ControlArea temp;

	ControlAreaList crossroadlist;
	ControlArea crossroad;
	/*crossroad.first = Vector2D(990, 1990), crossroad.second = Vector2D(1010, 2010);
	crossroadlist.push_back(crossroad);
	crossroad.first = Vector2D(990, 990), crossroad.second = Vector2D(1010, 1010);
	crossroadlist.push_back(crossroad);
	crossroad.first = Vector2D(1990, 990), crossroad.second = Vector2D(2010, 1010);
	crossroadlist.push_back(crossroad);
	crossroad.first = Vector2D(1990, 1990), crossroad.second = Vector2D(2010, 2010);
	crossroadlist.push_back(crossroad);*/

	templist.clear();
	templist= crossroadlist;
	temp.first=Vector2D (990,3000),temp.second=Vector2D (1000,2010);
	templist.push_back(temp);
	temp.first=Vector2D (990,1990),temp.second=Vector2D (1000,1010);
	templist.push_back(temp);
	temp.first=Vector2D (1010,990),temp.second=Vector2D (1990,1000);
	templist.push_back(temp);
	temp.first=Vector2D (1990,990),temp.second=Vector2D (2000,0);
	templist.push_back(temp);
	m_controlAreaMap[m_port1] = templist;

	templist.clear();
	templist= crossroadlist;
	temp.first=Vector2D (0,1990),temp.second=Vector2D (990,2000);
	templist.push_back(temp);
	temp.first=Vector2D (1010,1990),temp.second=Vector2D (1990,2000);
	templist.push_back(temp);
	temp.first=Vector2D (1990,1990),temp.second=Vector2D (2000,1010);
	templist.push_back(temp);
	temp.first=Vector2D (2010,990),temp.second=Vector2D (3000,1000);
	templist.push_back(temp);
	m_controlAreaMap[m_port2] = templist;

	templist.clear();
	templist= crossroadlist;
	temp.first=Vector2D (0,990),temp.second=Vector2D (990,1000);
	templist.push_back(temp);
	temp.first=Vector2D (1010,990),temp.second=Vector2D (1990,1000);
	templist.push_back(temp);
	temp.first=Vector2D (2000,1010),temp.second=Vector2D (2010,1990);
	templist.push_back(temp);
	temp.first=Vector2D (2010,1990),temp.second=Vector2D (3000,2000);
	templist.push_back(temp);
	m_controlAreaMap[m_port3] = templist;

	templist.clear();
	templist = crossroadlist;
	temp.first=Vector2D (1000,0),temp.second=Vector2D (1010,990);
	templist.push_back(temp);
	temp.first=Vector2D (1000,1010),temp.second=Vector2D (1010,1990);
	templist.push_back(temp);
	temp.first=Vector2D (1010,1990),temp.second=Vector2D (1990,2000);
	templist.push_back(temp);
	temp.first=Vector2D (2000,2010),temp.second=Vector2D (2010,3000);
	templist.push_back(temp);
	m_controlAreaMap[m_port4] = templist;

	templist.clear();
	templist = crossroadlist;
	temp.first=Vector2D (2000,0),temp.second=Vector2D (2010,990);
	templist.push_back(temp);
	temp.first=Vector2D (2000,1010),temp.second=Vector2D (2010,1990);
	templist.push_back(temp);
	temp.first=Vector2D (1990,2010),temp.second=Vector2D (1010,2000);
	templist.push_back(temp);
	temp.first=Vector2D (1000,2010),temp.second=Vector2D (1010,3000);
	templist.push_back(temp);
	m_controlAreaMap[m_port5] = templist;

	templist.clear();
	templist = crossroadlist;
	temp.first=Vector2D (3000,1010),temp.second=Vector2D (2010,990);
	templist.push_back(temp);
	temp.first=Vector2D (1990,1010),temp.second=Vector2D (1010,1000);
	templist.push_back(temp);
	temp.first=Vector2D (1000,1010),temp.second=Vector2D (1010,1990);
	templist.push_back(temp);
	temp.first=Vector2D (990,2010),temp.second=Vector2D (0,2000);
	templist.push_back(temp);
	m_controlAreaMap[m_port6] = templist;

	templist.clear();
	templist = crossroadlist;
	temp.first=Vector2D (3000,2010),temp.second=Vector2D (2010,2000);
	templist.push_back(temp);
	temp.first=Vector2D (1990,2010),temp.second=Vector2D (1010,2000);
	templist.push_back(temp);
	temp.first=Vector2D (990,1990),temp.second=Vector2D (1000,1010);
	templist.push_back(temp);
	temp.first=Vector2D (990,1010),temp.second=Vector2D (0,1000);
	templist.push_back(temp);
	m_controlAreaMap[m_port7] = templist;

	templist.clear();
	templist = crossroadlist;
	temp.first=Vector2D (1990,3000),temp.second=Vector2D (2000,2010);
	templist.push_back(temp);
	temp.first=Vector2D (1990,1990),temp.second=Vector2D (2000,1010);
	templist.push_back(temp);
	temp.first=Vector2D (1990,1010),temp.second=Vector2D (1010,990);
	templist.push_back(temp);
	temp.first=Vector2D (990,990),temp.second=Vector2D (1000,0);
	templist.push_back(temp);
	m_controlAreaMap[m_port8] = templist;

};

bool
Port_Position_Match::CheckThis (uint16_t port, Vector position)
{
	if(m_controlAreaMap.find(port) != m_controlAreaMap.end())
	{
		ControlAreaList templist;
		templist = m_controlAreaMap[port];
		for(std::list<std::pair<Vector2D, Vector2D>>::iterator it = templist.begin();it != templist.end();++it)
		{
			if(IsInTheArea(position,*it))
			{
				 std::cout<<"pass?"<<port<<std::endl;
				return true;
			}
		}
		return false;
	}
	 std::cout<<"what?"<<port<<std::endl;
	return true;
}

bool
Port_Position_Match::IsInTheArea(Vector position, ControlArea area)
{
	double lx = (area.first.x < area.second.x) ? area.first.x : area.second.x;
	double rx = (area.first.x < area.second.x) ? area.second.x : area.first.x;
	double ly = (area.first.y < area.second.y) ? area.first.y : area.second.y;
	double ry = (area.first.y < area.second.y) ? area.second.y : area.first.y;

	if(position.x >= lx && position.x <= rx
			&& position.y >= ly && position.y <= ry)
	{
		return true;
	}
	else
		return false;
}

}
}

