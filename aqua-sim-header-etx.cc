/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Universidade Federal de Minas Gerais
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
 * Author: Igor Castejon F. e Castro <igorcastejon@dcc.ufmg.br>
 */

#include "ns3/buffer.h"
#include "ns3/log.h"

#include "aqua-sim-header-etx.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ETXHeader");
NS_OBJECT_ENSURE_REGISTERED(ETXHeader);
/*
* UW-GRE
*/

ETXHeader::ETXHeader() : m_messType(0)
{
}

ETXHeader::~ETXHeader()
{
}

TypeId
ETXHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ETXHeader")
                            .SetParent<Header>()
                            .AddConstructor<ETXHeader>();
    return tid;
}

uint32_t
ETXHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_messType = i.ReadU8();
    m_pkNum = i.ReadU32();
    m_srcAddr = (AquaSimAddress)i.ReadU16();
    m_senderAddr = (AquaSimAddress)i.ReadU16();
    m_destAddr = (AquaSimAddress)i.ReadU16();
    m_ts = ((uint32_t)i.ReadU32()) / 1000.0;

    m_expectedToReceive = i.ReadU8();
    m_received = i.ReadU8();

    m_position_Sender.x = ((double)i.ReadU32()) / 1000.0;
    m_position_Sender.y = ((double)i.ReadU32()) / 1000.0;
    m_position_Sender.z = ((double)i.ReadU32()) / 1000.0;

    return GetSerializedSize();
}

uint32_t
ETXHeader::GetSerializedSize(void) const
{
    //reserved bytes for header
    return (1 + 4 + 2 + 2 + 2 + 2 + 4 + 12);
}

void ETXHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i.WriteU8(m_messType);
    i.WriteU32(m_pkNum);

    i.WriteU16(m_srcAddr.GetAsInt());
    i.WriteU16(m_senderAddr.GetAsInt());

    i.WriteU16(m_destAddr.GetAsInt());
    i.WriteU32((uint32_t)(m_ts * 1000.0));

    i.WriteU8(m_expectedToReceive);
    i.WriteU8(m_received);

    i.WriteU32((uint32_t)(m_position_Sender.x * 1000.0 + 0.5));
    i.WriteU32((uint32_t)(m_position_Sender.y * 1000.0 + 0.5));
    i.WriteU32((uint32_t)(m_position_Sender.z * 1000.0 + 0.5));
}

void ETXHeader::Print(std::ostream &os) const
{
    os << "ETXHeader is: messType=";
    switch (m_messType) {
        case NB_DISCOVER_REQ:
            os << "NB_DISCOVER_REQ";
            break;
        case NB_DISCOVER_REPLY:
            os << "NB_DISCOVER_REPLY";
            break;
    }
}

TypeId
ETXHeader::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

void ETXHeader::SetMessType(uint8_t messType)
{
    m_messType = messType;
}
void ETXHeader::SetPkNum(uint32_t pkNum)
{
    m_pkNum = pkNum;
}

void ETXHeader::SetSrcAddr(AquaSimAddress srcAddr)
{
    m_srcAddr = srcAddr;
}

void ETXHeader::SetSenderAddr(AquaSimAddress senderAddr)
{
    m_senderAddr = senderAddr;
}

void ETXHeader::SetDestAddr(AquaSimAddress destAddr)
{
    m_destAddr = destAddr;
}

void ETXHeader::SetSenderPosition(Vector3D position_Sender)
{
    m_position_Sender = position_Sender;
}

void ETXHeader::SetExpectedToReceive(uint8_t expectedToReceive)
{
    m_expectedToReceive = expectedToReceive;
}

void ETXHeader::SetReceived(uint8_t received)
{
    m_received = received;
}

void ETXHeader::SetTs(uint32_t ts)
{
    m_ts = ts;
}

uint8_t
ETXHeader::GetMessType()
{
    return m_messType;
}
uint32_t
ETXHeader::GetPkNum()
{
    return m_pkNum;
}

uint8_t ETXHeader::GetExpectedToReceive()
{
    return m_expectedToReceive;
}

uint8_t ETXHeader::GetReceived()
{
    return m_received;
}

AquaSimAddress
ETXHeader::GetSrcAddr()
{
    return m_srcAddr;
}

AquaSimAddress
ETXHeader::GetSenderAddr()
{
    return m_senderAddr;
}

AquaSimAddress
ETXHeader::GetDestAddr()
{
    return m_destAddr;
}

uint32_t
ETXHeader::GetTs()
{
    return m_ts;
}

Vector3D ETXHeader::GetSenderPosition()
{
    return m_position_Sender;
}
