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

#ifndef AQUA_SIM_HEADER_UWGRE_H
#define AQUA_SIM_HEADER_UWGRE_H

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
    class UWGREHeader : public Header
    {
        public:
			UWGREHeader();

			virtual ~UWGREHeader();
			static TypeId GetTypeId();

			//Setters
			void SetMessType(uint8_t messType);
			void SetPkNum(uint32_t pkNum);
			void SetSenderAddr(AquaSimAddress senderAddr);
			void SetSrcAddr(AquaSimAddress srcAddr);
			void SetRelayAddr(AquaSimAddress relayAddr);
			void SetDestAddr(AquaSimAddress destAddr);

			void SetDataType(uint8_t dataType);
			void SetOriginalSource(Vector originalSource);
			void SetToken(uint32_t token);
			void SetTs(uint32_t ts);
			void SetRange(uint64_t range);

			void SetETXCost(double etxCost);

			void SetDestPosition(Vector3D m_position_Dest);
			void SetSrcPosition(Vector3D m_position_Src);
			void SetSenderPosition(Vector3D m_position_Sender);
			void SetRelayPosition(Vector3D m_position_Relay);

			//Getters
			uint8_t GetMessType();
			uint32_t GetPkNum();
			AquaSimAddress GetSrcAddr();
			AquaSimAddress GetSenderAddr();
			AquaSimAddress GetRelayAddr();
			AquaSimAddress GetDestAddr();
			uint8_t GetDataType();
			uint32_t GetToken();
			uint32_t GetTs();
			uint64_t GetRange();
			double GetETXCost();

			Vector3D GetDestPosition();
			Vector3D GetSrcPosition();
			Vector3D GetSenderPosition();
			Vector3D GetRelayPosition();

			void HandleNegativeSenderPosition();
			void HandleNegativeDestPosition();
			void HandleNegativeSrcPosition();

			//inherited methods
			virtual uint32_t GetSerializedSize(void) const;
			virtual void Serialize(Buffer::Iterator start) const;
			virtual uint32_t Deserialize(Buffer::Iterator start);
			virtual void Print(std::ostream &os) const;
			virtual TypeId GetInstanceTypeId(void) const;

			bool m_IsPosition_SrcNX;
			bool m_IsPosition_SrcNY;
			bool m_IsPosition_SrcNZ;
			bool m_IsPosition_SenderNX;
			bool m_IsPosition_SenderNY;
			bool m_IsPosition_SenderNZ;
			bool m_IsPosition_DestNX;
			bool m_IsPosition_DestNY;
			bool m_IsPosition_DestNZ;
			bool m_IsPosition_RelayNX;
			bool m_IsPosition_RelayNY;
			bool m_IsPosition_RelayNZ;

			uint32_t m_sizeDelaunayNeighbors;
			std::vector<std::tuple<double, AquaSimAddress, Vector3D, double, bool, bool, bool> > m_DelaunayNeighbors;

			double m_error;
			double m_data;
			uint8_t m_dataType;

		protected:
			uint8_t m_messType;  //message type
			uint32_t m_pkNum;    //packet sequence num

			uint8_t m_messType;  //message type
			uint32_t m_pkNum;    //packet sequence num
			Vector3D m_position_Sender;
			Vector3D m_position_Dest;
			Vector3D m_position_Relay;

			double m_ETXCost;

			uint32_t m_token;
			uint32_t m_ts;     // Timestamp when pkt is generated.
			uint64_t m_range;  // target range
    };  // class UWGREHeader

}  //namespace ns3

#endif