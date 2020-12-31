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

#ifndef AQUA_SIM_HEADER_ETX_H
#define AQUA_SIM_HEADER_ETX_H

#include "ns3/header.h"
#include "ns3/vector.h"

#include "aqua-sim-address.h"
#include "uwgre-datastructure.h"

namespace ns3
{
/**
 * \brief UW-GRE
 * Protocol paper:
 */
    class ETXHeader : public Header
    {
        public:
			ETXHeader();
			virtual ~ETXHeader();
			static TypeId GetTypeId();

			//Setters
			void SetMessType(uint8_t messType);
			void SetPkNum(uint32_t pkNum);
			void SetSenderAddr(AquaSimAddress senderAddr);
			void SetSrcAddr(AquaSimAddress srcAddr);
			void SetDestAddr(AquaSimAddress destAddr);

			void SetTs(uint32_t ts);

			void SetExpectedToReceive(uint8_t);
			void SetReceived(uint8_t);

			void SetSenderPosition(Vector3D m_position_Sender);

			//Getters
			uint8_t GetMessType();
			uint32_t GetPkNum();
			AquaSimAddress GetSrcAddr();
			AquaSimAddress GetSenderAddr();
			AquaSimAddress GetDestAddr();
			uint32_t GetTs();
			uint8_t GetExpectedToReceive();
			uint8_t GetReceived();

			Vector3D GetSenderPosition();

			//inherited methods
			virtual uint32_t GetSerializedSize(void) const;
			virtual void Serialize(Buffer::Iterator start) const;
			virtual uint32_t Deserialize(Buffer::Iterator start);
			virtual void Print(std::ostream &os) const;
			virtual TypeId GetInstanceTypeId(void) const;

		protected:
			uint8_t m_messType;  //message type
			uint32_t m_pkNum;    //packet sequence num
			AquaSimAddress m_srcAddr;
			AquaSimAddress m_senderAddr;  // sender addr
			AquaSimAddress m_destAddr;    // destination addr

			Vector3D m_position_Sender;

			uint8_t m_expectedToReceive;
			uint8_t m_received;

			uint32_t m_ts;  // Timestamp when pkt is generated.

    };  // class ETXHeader

}  //namespace ns3

#endif