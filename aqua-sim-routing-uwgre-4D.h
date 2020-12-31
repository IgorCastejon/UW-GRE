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

#ifndef AQUA_SIM_ROUTING_UWGRE_4D_H
#define AQUA_SIM_ROUTING_UWGRE_4D_H

#include <map>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "aqua-sim-address.h"
#include "aqua-sim-channel.h"
#include "aqua-sim-routing.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"
#include "uwgre-datastructure.h"
#include "vector4D.h"

namespace ns3
{
    class AquaSimUWGRE_4D : public AquaSimRouting
    {
        public:
            AquaSimUWGRE_4D();
            static TypeId GetTypeId(void);
            int64_t AssignStreams(int64_t stream);

            virtual bool Recv(Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber);
            void sendVPOD_Initialization();

            bool isFather;

        protected:
            int m_pkCount;
            int m_counter;

            std::unordered_map<int, std::unordered_set<int> > rcvPkts;

            Ptr<UniformRandomVariable> m_rand;
            Ptr<NormalRandomVariable> m_randNormal;

            int idNetwork;
            int numNodes;
            int testNumber;

            void loadTests();
            void helperSendUWGRE(int, int);

            int toLoad;
            int isLoad;
            int testMode;

            int receivedUWGRE;
            int transmittedUWGRE;

            void Terminate();

            std::unordered_map<int, std::vector<double> > pktsSuccessReceived;
            void UWGRERouting(Ptr<Packet> pkt);
            void MDT_Greedy(Ptr<Packet> pkt);
            void SendUWGRE(AquaSimAddress destAddr, Vector4D dest, int);

            void sendSymmetricETX();
            void receiveSymmetricETX(Ptr<Packet> pkt);
            int isSymmetric;

            void AssurePath();

            void TestDump();

            void Send(Ptr<Packet> pkt);
            void Forward(Ptr<Packet> pkt);

            void SetVPODInitialization(bool init);

            AquaSimAddress Get_NextHop(Ptr<Packet> pkt);
            std::tuple<AquaSimAddress, AquaSimAddress> Get_NextHop_NoPkt(AquaSimAddress dest, AquaSimAddress relay, Vector4D destPos);

            void Adjustment();
            double calculateAveragePositionErrorNeighbors();
            void DoAdjustment();

            Vector4D findCorrectionVector(Vector4D myPos, Vector4D target, double c_C, double f, double cost);

            void Dump();

            void sendJoin_Req();
            void receiveJoin_Req(Ptr<Packet> pkt);
            void receiveJoin_Reply(Ptr<Packet> pkt);
            void VPOD_Initialize(Ptr<Packet> packet);

            void receiveNB_SET_REQ(Ptr<Packet> pkt);
            void receiveNB_SET_REQ_NOTIFICATION(Ptr<Packet> pkt);
            void receiveNB_SET_REPLY(Ptr<Packet> pkt);

            void receiveNB_SET_REPLY_MAINTENANCE(Ptr<Packet> pkt);

            void UpdateMDT_MAINTENANCE(Ptr<Packet> pkt);

            void sendMDT_Initialization();

            void UpdateNeighborPosition(Ptr<Packet> packet);
            void UpdateMyPosition();
            void UpdateMyPosition_All();
            void UpdateMDTNeigh(Ptr<Packet> packet);

            void MDT_Maintenance();

            void UpdateMDT(Ptr<Packet> pkt);

            void UpdateMyMDTStatus();

            void UpdateForwardingTable(AquaSimAddress dest, AquaSimAddress sender, double error, double cost);

            bool m_isInitializating;

            bool isInMDT;
            bool initializationMDT;

            bool noMoreNewNeighbors;

            bool is4D;

            int m_VPODInits;
            int numMaintenances;
            int numMaintenancesCurrent;

            int numPeriodsJ;
            int numPeriodsJmax;

            AquaSimAddress m_senderDFS;
            void receiveMDT_DFS_Initialization(Ptr<Packet> pkt);
            void sendMDT_DFS_Initialization();
            void receiveMDT_DFS_Comeback(Ptr<Packet> pkt);
            bool DFS;

            void Update_Neighbors(Ptr<Packet> pkt, std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > calculatedDelaunay);

            int m_waiting;
            bool m_reset;

            void Initialization();
            void Load();
            void DumpLocal();

            virtual void DoDispose();

            Vector4D myPos;

            double positionError;
            typedef struct fTableMember {
                AquaSimAddress source, pred, succ, dest;
                double cost, error;
            } fTableMember;

            std::vector<fTableMember> m_fTable;  // forwarding table uses local identifiers
            // source, pred, succ, dest, cost, error

            std::map<uint8_t, AquaSimAddress> m_addressTable;

            std::map<AquaSimAddress, uint8_t> m_packetsReceivedReq;

            std::map<AquaSimAddress, uint8_t> m_packetsReceivedReply;

            std::map<AquaSimAddress, uint8_t> m_packetsReceivedByNeigh;

            void replyPhysicalNeighbors(AquaSimAddress sender, uint8_t expectedToReceive);
            void findPhysicalNeighbors();
            void loadPhysicalNeighbors();
            void CalculateETX(AquaSimAddress sender, uint8_t expectedToReceive, Vector posNeighbor);

            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > m_PhyNeigh;     // Cost, position and Address
            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_MDTPhyNeigh;  // physical neighborhood in MDT
            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_MDTNeigh;     // multi-hop DT neighborhood // Routing Cost, position and Address

            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_partOfTheKnown;
            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_hasSentNBSET;
            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_hasReceivedNBSET;

            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > m_InitializedNeigh;

            double c_E, c_C;          // error constants for adjustment
            double adjustmentPeriod;  // adjustment period

            double timeOutMDT;

            bool hasSentVPod_TOKEN;

            std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > calculateDelaunay(std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > v, AquaSimAddress target, bool include, bool option);

            std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > > calculateDelaunaySimplexes(std::vector<std::tuple<double, AquaSimAddress, Vector4D, double> > v, AquaSimAddress target, bool include, bool option);
            bool hasSentMDT_TOKEN;
            bool hasReceivedVPod_TOKEN;
            bool hasReceivedMDT_TOKEN;

    };  //class AquaSimUWGRE_4D

}  //namespace ns3

#endif