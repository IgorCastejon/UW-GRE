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

#include "aqua-sim-header-uwgre.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UWGREHeader");
NS_OBJECT_ENSURE_REGISTERED(UWGREHeader);
/*
* UW-GRE
*/

UWGREHeader::UWGREHeader() : m_messType(0)
{
    m_sizeDelaunayNeighbors = 0;
    m_IsPosition_SrcNX = false;
    m_IsPosition_SrcNY = false;
    m_IsPosition_SrcNZ = false;
    m_IsPosition_SenderNX = false;
    m_IsPosition_SenderNY = false;
    m_IsPosition_SenderNZ = false;
    m_IsPosition_DestNX = false;
    m_IsPosition_DestNY = false;
    m_IsPosition_DestNZ = false;
    m_IsPosition_RelayNX = false;
    m_IsPosition_RelayNY = false;
    m_IsPosition_RelayNZ = false;
    m_dataType = 0;
    m_range = 0;
}

UWGREHeader::~UWGREHeader()
{
}

TypeId
UWGREHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::UWGREHeader")
                            .SetParent<Header>()
                            .AddConstructor<UWGREHeader>();
    return tid;
}

uint32_t
UWGREHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_messType = i.ReadU8();
    m_pkNum = i.ReadU32();
    m_srcAddr = (AquaSimAddress)i.ReadU16();
    m_senderAddr = (AquaSimAddress)i.ReadU16();
    m_destAddr = (AquaSimAddress)i.ReadU16();
    m_relayAddr = (AquaSimAddress)i.ReadU16();
    m_dataType = i.ReadU8();
    m_token = ((uint32_t)i.ReadU32()) / 1000.0;
    m_ts = ((uint32_t)i.ReadU32()) / 1000.0;
    m_range = ((uint64_t)i.ReadU64()) / 1000.0;

    m_ETXCost = ((double)i.ReadU32()) / 1000.0;
    m_data = ((double)i.ReadU32()) / 1000.0;

    m_position_Src.x = ((double)i.ReadU32()) / 1000000.0;
    m_position_Src.y = ((double)i.ReadU32()) / 1000000.0;
    m_position_Src.z = ((double)i.ReadU32()) / 1000000.0;

    m_position_Sender.x = ((double)i.ReadU32()) / 1000000.0;
    m_position_Sender.y = ((double)i.ReadU32()) / 1000000.0;
    m_position_Sender.z = ((double)i.ReadU32()) / 1000000.0;
    m_position_Dest.x = ((double)i.ReadU32()) / 1000000.0;
    m_position_Dest.y = ((double)i.ReadU32()) / 1000000.0;
    m_position_Dest.z = ((double)i.ReadU32()) / 1000000.0;

    m_position_Relay.x = ((double)i.ReadU32()) / 1000000.0;
    m_position_Relay.y = ((double)i.ReadU32()) / 1000000.0;
    m_position_Relay.z = ((double)i.ReadU32()) / 1000000.0;

    m_IsPosition_SrcNX = i.ReadU8();
    m_IsPosition_SrcNY = i.ReadU8();
    m_IsPosition_SrcNZ = i.ReadU8();
    m_IsPosition_SenderNX = i.ReadU8();
    m_IsPosition_SenderNY = i.ReadU8();
    m_IsPosition_SenderNZ = i.ReadU8();
    m_IsPosition_DestNX = i.ReadU8();
    m_IsPosition_DestNY = i.ReadU8();
    m_IsPosition_DestNZ = i.ReadU8();
    m_IsPosition_RelayNX = i.ReadU8();
    m_IsPosition_RelayNY = i.ReadU8();
    m_IsPosition_RelayNZ = i.ReadU8();

    if (m_IsPosition_SrcNX) {
        m_position_Src.x *= -1.0;
    }
    if (m_IsPosition_SrcNY) {
        m_position_Src.y *= -1.0;
    }
    if (m_IsPosition_SrcNZ) {
        m_position_Src.z *= -1.0;
    }

    if (m_IsPosition_SenderNX) {
        m_position_Sender.x *= -1.0;
    }
    if (m_IsPosition_SenderNY) {
        m_position_Sender.y *= -1.0;
    }
    if (m_IsPosition_SenderNZ) {
        m_position_Sender.z *= -1.0;
    }

    if (m_IsPosition_DestNX) {
        m_position_Dest.x *= -1.0;
    }
    if (m_IsPosition_DestNY) {
        m_position_Dest.y *= -1.0;
    }
    if (m_IsPosition_DestNZ) {
        m_position_Dest.z *= -1.0;
    }

    if (m_IsPosition_RelayNX) {
        m_position_Relay.x *= -1.0;
    }
    if (m_IsPosition_RelayNY) {
        m_position_Relay.y *= -1.0;
    }
    if (m_IsPosition_RelayNZ) {
        m_position_Relay.z *= -1.0;
    }

    m_error = ((double)i.ReadU32()) / 1000.0;
    m_sizeDelaunayNeighbors = i.ReadU32();

    for (uint32_t t = 0; t < m_sizeDelaunayNeighbors; t++) {
        double ETXCost = ((double)i.ReadU32()) / 1000.0;

        AquaSimAddress delaunayAddr = (AquaSimAddress)i.ReadU16();

        double m_positionX = ((double)i.ReadU32()) / 1000000.0;
        double m_positionY = ((double)i.ReadU32()) / 1000000.0;
        double m_positionZ = ((double)i.ReadU32()) / 1000000.0;

        Vector3D position;
        position.x = m_positionX;
        position.y = m_positionY;
        position.z = m_positionZ;

        double error = ((double)i.ReadU32()) / 1000.0;

        bool isXN = i.ReadU8();
        bool isYN = i.ReadU8();
        bool isZN = i.ReadU8();

        if (isXN) {
            position.x *= -1.0;
        }

        if (isYN) {
            position.y *= -1.0;
        }

        if (isZN) {
            position.z *= -1.0;
        }

        auto x = std::make_tuple(ETXCost, delaunayAddr, position, error, isXN, isYN, isZN);

        m_DelaunayNeighbors.push_back(x);
    }

    return GetSerializedSize();
}

uint32_t
UWGREHeader::GetSerializedSize(void) const
{
    //reserved bytes for header
    return (1 + 4 + 2 + 2 + 2 + 2 + 1 + 4 + 4 + 4 + 4 + 48 + 4 + 4 + 12 + 4 + 4 + (25) * m_sizeDelaunayNeighbors);
}

void UWGREHeader::Serialize(Buffer::Iterator start) const
{
    Buffer::Iterator i = start;
    i.WriteU8(m_messType);
    i.WriteU32(m_pkNum);
    i.WriteU16(m_srcAddr.GetAsInt());
    i.WriteU16(m_senderAddr.GetAsInt());
    i.WriteU16(m_destAddr.GetAsInt());
    i.WriteU16(m_relayAddr.GetAsInt());
    i.WriteU8(m_dataType);

    i.WriteU32((uint32_t)(m_token * 1000.0));
    i.WriteU32((uint32_t)(m_ts * 1000.0));
    i.WriteU64((uint64_t)(m_range * 1000.0));
    i.WriteU32((uint32_t)(m_ETXCost * 1000.0 + 0.5));
    i.WriteU32((uint32_t)(m_data * 1000.0 + 0.5));

    if (m_position_Src.x < 0.0) {
        i.WriteU32((uint32_t)(m_position_Src.x * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Src.x * 1000000.0 + 0.5));
    }
    if (m_position_Src.y < 0.0) {
        i.WriteU32((uint32_t)(m_position_Src.y * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Src.y * 1000000.0 + 0.5));
    }
    if (m_position_Src.z < 0.0) {
        i.WriteU32((uint32_t)(m_position_Src.z * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Src.z * 1000000.0 + 0.5));
    }

    if (m_position_Sender.x < 0.0) {
        i.WriteU32((uint32_t)(m_position_Sender.x * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Sender.x * 1000000.0 + 0.5));
    }
    if (m_position_Sender.y < 0.0) {
        i.WriteU32((uint32_t)(m_position_Sender.y * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Sender.y * 1000000.0 + 0.5));
    }
    if (m_position_Sender.z < 0.0) {
        i.WriteU32((uint32_t)(m_position_Sender.z * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Sender.z * 1000000.0 + 0.5));
    }

    if (m_position_Dest.x < 0.0) {
        i.WriteU32((uint32_t)(m_position_Dest.x * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Dest.x * 1000000.0 + 0.5));
    }
    if (m_position_Dest.y < 0.0) {
        i.WriteU32((uint32_t)(m_position_Dest.y * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Dest.y * 1000000.0 + 0.5));
    }
    if (m_position_Dest.z < 0.0) {
        i.WriteU32((uint32_t)(m_position_Dest.z * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Dest.z * 1000000.0 + 0.5));
    }

    if (m_position_Relay.x < 0.0) {
        i.WriteU32((uint32_t)(m_position_Relay.x * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Relay.x * 1000000.0 + 0.5));
    }
    if (m_position_Relay.y < 0.0) {
        i.WriteU32((uint32_t)(m_position_Relay.y * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Relay.y * 1000000.0 + 0.5));
    }
    if (m_position_Relay.z < 0.0) {
        i.WriteU32((uint32_t)(m_position_Relay.z * (-1.0) * 1000000.0 + 0.5));
    }
    else {
        i.WriteU32((uint32_t)(m_position_Relay.z * 1000000.0 + 0.5));
    }

    i.WriteU8((int32_t)(m_IsPosition_SrcNX));
    i.WriteU8((int32_t)(m_IsPosition_SrcNY));
    i.WriteU8((int32_t)(m_IsPosition_SrcNZ));
    i.WriteU8((int32_t)(m_IsPosition_SenderNX));
    i.WriteU8((int32_t)(m_IsPosition_SenderNY));
    i.WriteU8((int32_t)(m_IsPosition_SenderNZ));
    i.WriteU8((int32_t)(m_IsPosition_DestNX));
    i.WriteU8((int32_t)(m_IsPosition_DestNY));
    i.WriteU8((int32_t)(m_IsPosition_DestNZ));
    i.WriteU8((int32_t)(m_IsPosition_RelayNX));
    i.WriteU8((int32_t)(m_IsPosition_RelayNY));
    i.WriteU8((int32_t)(m_IsPosition_RelayNZ));

    i.WriteU32((uint32_t)(m_error * 1000.0 + 0.5));
    i.WriteU32(m_sizeDelaunayNeighbors);
    for (auto it = m_DelaunayNeighbors.begin(); it != m_DelaunayNeighbors.end(); it++) {
        i.WriteU32((uint32_t)(std::get<0>(*it) * 1000.0 + 0.5));  // ETX
        i.WriteU16((std::get<1>(*it)).GetAsInt());                // AquaSimAddress
        Vector3D position = std::get<2>(*it);

        if (std::get<4>(*it)) {
            i.WriteU32((int32_t)(position.x * (-1.0) * 1000000.0 + 0.5));
        }
        else {
            i.WriteU32((int32_t)(position.x * 1000000.0 + 0.5));
        }

        if (std::get<5>(*it)) {
            i.WriteU32((int32_t)(position.y * (-1.0) * 1000000.0 + 0.5));
        }
        else {
            i.WriteU32((int32_t)(position.y * 1000000.0 + 0.5));
        }

        if (std::get<6>(*it)) {
            i.WriteU32((int32_t)(position.z * (-1.0) * 1000000.0 + 0.5));
        }
        else {
            i.WriteU32((int32_t)(position.z * 1000000.0 + 0.5));
        }

        i.WriteU32((uint32_t)(std::get<3>(*it) * 1000.0 + 0.5));

        i.WriteU8((int32_t)std::get<4>(*it));
        i.WriteU8((int32_t)std::get<5>(*it));
        i.WriteU8((int32_t)std::get<6>(*it));
    }
}

void UWGREHeader::Print(std::ostream &os) const
{
    os << "Greedy Distance Vector Routing Header is: messType=";
    switch (m_messType) {
        case JOIN_REQ:
            os << "JOIN_REQ";
            break;
        case JOIN_REPLY:
            os << "JOIN_REPLY";
            break;
        case NB_SET_REQ:
            os << "NB_SET_REQ";
            break;
        case NB_SET_REPLY:
            os << "NB_SET_REPLY";
            break;
    }
}

TypeId
UWGREHeader::GetInstanceTypeId(void) const
{
    return GetTypeId();
}

void UWGREHeader::SetMessType(uint8_t messType)
{
    m_messType = messType;
}
void UWGREHeader::SetPkNum(uint32_t pkNum)
{
    m_pkNum = pkNum;
}

void UWGREHeader::SetSrcAddr(AquaSimAddress srcAddr)
{
    m_srcAddr = srcAddr;
}

void UWGREHeader::SetSenderAddr(AquaSimAddress senderAddr)
{
    m_senderAddr = senderAddr;
}

void UWGREHeader::SetRelayAddr(AquaSimAddress relayAddr)
{
    m_relayAddr = relayAddr;
}
void UWGREHeader::SetDestAddr(AquaSimAddress destAddr)
{
    m_destAddr = destAddr;
}

void UWGREHeader::SetDestPosition(Vector3D position_Dest)
{
    m_position_Dest = position_Dest;
}
void UWGREHeader::SetSrcPosition(Vector3D position_Src)
{
    m_position_Src = position_Src;
}
void UWGREHeader::SetSenderPosition(Vector3D position_Sender)
{
    m_position_Sender = position_Sender;
}

void UWGREHeader::SetRelayPosition(Vector3D position_Relay)
{
    m_position_Relay = position_Relay;
}

void UWGREHeader::SetDataType(uint8_t dataType)
{
    m_dataType = dataType;
}

void UWGREHeader::SetToken(uint32_t token)
{
    m_token = token;
}
void UWGREHeader::SetTs(uint32_t ts)
{
    m_ts = ts;
}
void UWGREHeader::SetRange(uint64_t range)
{
    m_range = range;
}

uint8_t
UWGREHeader::GetMessType()
{
    return m_messType;
}

uint32_t
UWGREHeader::GetPkNum()
{
    return m_pkNum;
}

AquaSimAddress
UWGREHeader::GetSrcAddr()
{
    return m_srcAddr;
}

AquaSimAddress
UWGREHeader::GetSenderAddr()
{
    return m_senderAddr;
}

AquaSimAddress
UWGREHeader::GetRelayAddr()
{
    return m_relayAddr;
}

AquaSimAddress
UWGREHeader::GetDestAddr()
{
    return m_destAddr;
}

uint8_t
UWGREHeader::GetDataType()
{
    return m_dataType;
}

uint32_t
UWGREHeader::GetToken()
{
    return m_token;
}

uint32_t
UWGREHeader::GetTs()
{
    return m_ts;
}

uint64_t
UWGREHeader::GetRange()
{
    return m_range;
}

Vector3D UWGREHeader::GetDestPosition()
{
    return m_position_Dest;
}

Vector3D UWGREHeader::GetSrcPosition()
{
    return m_position_Src;
}

Vector3D UWGREHeader::GetSenderPosition()
{
    return m_position_Sender;
}

Vector3D UWGREHeader::GetRelayPosition()
{
    return m_position_Relay;
}

void UWGREHeader::HandleNegativeSrcPosition()
{
    Vector3D myPos = GetSrcPosition();
    if (myPos.x < 0.0) {
        m_IsPosition_SrcNX = true;
    }
    else {
        m_IsPosition_SrcNX = false;
    }
    if (myPos.y < 0.0) {
        m_IsPosition_SrcNY = true;
    }
    else {
        m_IsPosition_SrcNY = false;
    }
    if (myPos.z < 0.0) {
        m_IsPosition_SrcNZ = true;
    }
    else {
        m_IsPosition_SrcNZ = false;
    }
}

void UWGREHeader::HandleNegativeDestPosition()
{
    Vector3D dest = GetDestPosition();
    if (dest.x < 0.0) {
        m_IsPosition_DestNX = true;
    }
    else {
        m_IsPosition_DestNX = false;
    }
    if (dest.y < 0.0) {
        m_IsPosition_DestNY = true;
    }
    else {
        m_IsPosition_DestNY = false;
    }
    if (dest.z < 0.0) {
        m_IsPosition_DestNZ = true;
    }
    else {
        m_IsPosition_DestNZ = false;
    }
}

void UWGREHeader::HandleNegativeSenderPosition()
{
    Vector3D myPos = GetSenderPosition();
    if (myPos.x < 0.0) {
        m_IsPosition_SenderNX = true;
    }
    else {
        m_IsPosition_SenderNX = false;
    }
    if (myPos.y < 0.0) {
        m_IsPosition_SenderNY = true;
    }
    else {
        m_IsPosition_SenderNY = false;
    }
    if (myPos.z < 0.0) {
        m_IsPosition_SenderNZ = true;
    }
    else {
        m_IsPosition_SenderNZ = false;
    }
}

void UWGREHeader::SetETXCost(double etxCost)
{
    m_ETXCost = etxCost;
}

double UWGREHeader::GetETXCost()
{
    return m_ETXCost;
}