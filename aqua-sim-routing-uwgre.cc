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

#include <algorithm>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"

#include "aqua-sim-header-etx.h"
#include "aqua-sim-header-uwgre.h"
#include "aqua-sim-header.h"
#include "aqua-sim-propagation.h"
#include "aqua-sim-pt-tag.h"
#include "aqua-sim-routing-uwgre.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimUWGRE");
NS_OBJECT_ENSURE_REGISTERED(AquaSimUWGRE);

AquaSimUWGRE::AquaSimUWGRE()
{
    Simulator::Schedule(Seconds(10), &AquaSimUWGRE::Initialization, this);
}

TypeId AquaSimUWGRE::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::AquaSimUWGRE")
                            .SetParent<AquaSimRouting>()
                            .AddConstructor<AquaSimUWGRE>()
                            .AddAttribute("NumPeriodsJ", "Number of max UWGRE J periods. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::numPeriodsJmax),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("NumMaintenances", "Number of MDT maintenances. Default is 5.",
                                          IntegerValue(5),
                                          MakeIntegerAccessor(&AquaSimUWGRE::numMaintenances),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("idNetwork", "ID of current network. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::idNetwork),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("numNodes", "Number of current nodes. Default is 1.",
                                          IntegerValue(1),
                                          MakeIntegerAccessor(&AquaSimUWGRE::numNodes),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("testMode", "If a test is going to happen. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::testMode),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("testNumber", "Number of the test. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::testNumber),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("toLoad", "Which previous network is to be loaded if so. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::toLoad),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("isLoad", "If a previous network is to be loaded. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::isLoad),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("isMDTMode", "If the network is MDT only. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::isMDTMode),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("isOptimal", "If the test are to be optimal. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::isOptimal),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("isSymmetric", "If the ETX values should be symmetric. Default is 0.",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&AquaSimUWGRE::isSymmetric),
                                          MakeIntegerChecker<int>())
                            .AddAttribute("AdjustmentPeriod", "Adjustment period of UWGRE. Default is 20.",
                                          DoubleValue(20),
                                          MakeDoubleAccessor(&AquaSimUWGRE::adjustmentPeriod),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("TimeoutPeriod", "Timeout period of MDT. Default is 240.",
                                          DoubleValue(240),
                                          MakeDoubleAccessor(&AquaSimUWGRE::timeOutMDT),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("c_C", "c_C of UWGRE. Default is 0.3.",
                                          DoubleValue(0.1),
                                          MakeDoubleAccessor(&AquaSimUWGRE::c_C),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("c_E", "c_E of UWGRE. Default is 0.25.",
                                          DoubleValue(0.25),
                                          MakeDoubleAccessor(&AquaSimUWGRE::c_E),
                                          MakeDoubleChecker<double>());
    return tid;
}

int64_t AquaSimUWGRE::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_rand->SetStream(stream);
    return 1;
}

void AquaSimUWGRE::Terminate()
{
    NS_LOG_DEBUG("AquaSimUWGRE::Terminate: Node=" << GetNetDevice()->GetAddress() << ": remaining energy=" << GetNetDevice()->EnergyModel()->GetEnergy() << ", initial energy=" << GetNetDevice()->EnergyModel()->GetInitialEnergy());
}

void AquaSimUWGRE::DoDispose()
{
    m_rand = 0;
    AquaSimRouting::DoDispose();
}

void AquaSimUWGRE::SendMDT(AquaSimAddress dest, Vector3D posDest, int idMsg)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    Ptr<Packet> pkt = Create<Packet>();

    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    ash.SetSAddr(current);
    uwgreh.SetRange((long long int)(Simulator::Now().ToDouble(Time::S) * 100.0));
    uwgreh.m_error = 0;

    uwgreh.SetMessType(MDT);
    uwgreh.SetPkNum(idMsg);

    uwgreh.SetSrcAddr(current);
    uwgreh.SetSrcPosition(GetNetDevice()->GetPosition());

    uwgreh.HandleNegativeSrcPosition();

    uwgreh.SetDestAddr(dest);
    uwgreh.SetDestPosition(posDest);
    uwgreh.SetRelayAddr((AquaSimAddress)NULL);
    uwgreh.HandleNegativeDestPosition();

    pkt->AddHeader(uwgreh);
    pkt->AddHeader(ash);
    MDTRouting(pkt);
}

void AquaSimUWGRE::helperSendMDT(int addressToSend, int idMsg)
{
    std::fstream positionInfo(std::string("nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(toLoad) + std::string("/") + std::string("positionsReal"), std::fstream::in);
    int address = 0;
    double x, y, z;
    while (positionInfo >> address >> x >> y >> z) {
        if (address == addressToSend) {
            break;
        }
    }
    positionInfo.close();
    Vector3D dest(x, y, z);
    SendMDT((AquaSimAddress)addressToSend, dest, idMsg);
}

void AquaSimUWGRE::helperSendUWGRE(int addressToSend, int idMsg)
{
    std::fstream positionInfo(std::string("nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(toLoad) + std::string("/") + std::string("positionsVirtual"), std::fstream::in);
    int address = 0;
    double x, y, z;
    while (positionInfo >> address >> x >> y >> z) {
        if (address == addressToSend) {
            break;
        }
    }
    positionInfo.close();
    Vector3D dest(x, y, z);
    SendUWGRE((AquaSimAddress)addressToSend, dest, idMsg);
}

void AquaSimUWGRE::SendUWGRE(AquaSimAddress destAddr, Vector3D dest, int idMsg)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    Ptr<Packet> pkt = Create<Packet>();

    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    ash.SetSAddr(current);
    uwgreh.SetRange((long long int)(Simulator::Now().ToDouble(Time::S) * 100.0));
    uwgreh.m_error = 0;

    uwgreh.SetMessType(UWGRE);
    uwgreh.SetPkNum(idMsg);

    uwgreh.SetSrcAddr(current);
    uwgreh.SetSrcPosition(myPos);

    uwgreh.HandleNegativeSrcPosition();

    uwgreh.SetDestPosition(dest);
    uwgreh.SetDestAddr(destAddr);
    uwgreh.SetRelayAddr((AquaSimAddress)NULL);
    uwgreh.HandleNegativeDestPosition();

    pkt->AddHeader(uwgreh);
    pkt->AddHeader(ash);
    UWGRERouting(pkt);
}

void AquaSimUWGRE::MDTRouting(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    std::fstream dataTest(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/result_test") + std::to_string(testNumber) + std::string(".txt"), std::fstream::out | std::fstream::app);

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    int idMsg = uwgreh.GetPkNum();

    int sender = uwgreh.GetSrcAddr().GetAsInt();
    int currentInt = current.GetAsInt();
    double timestampSend = uwgreh.GetRange() / 100.0;
    double timestampReceive = Simulator::Now().ToDouble(Time::S) + uwgreh.m_error;
    dataTest << idMsg << " " << sender << " " << currentInt << " " << timestampSend << " " << timestampReceive << " " << uwgreh.GetETXCost() << std::endl;

    if (current == uwgreh.GetDestAddr()) {
        pktsSuccessReceived[sender].push_back(timestampSend);

        pktsSuccessReceived[sender].push_back(timestampReceive);
    }
    else {
        Ptr<Packet> pkt2 = Create<Packet>();

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);
        transmittedMDT++;
        MDT_Greedy(pkt2);
    }

    dataTest.close();
}

void AquaSimUWGRE::UWGRERouting(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    std::string ifOptimal = std::string("");

    if (isOptimal) {
        ifOptimal = std::string("-optimal");
    }
    std::fstream dataTest(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/result_test") + std::to_string(testNumber) + ifOptimal + std::string(".txt"), std::fstream::out | std::fstream::app);

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    int idMsg = uwgreh.GetPkNum();

    int sender = uwgreh.GetSrcAddr().GetAsInt();
    int currentInt = current.GetAsInt();
    double timestampSend = uwgreh.GetRange() / 100.0;
    double timestampReceive = Simulator::Now().ToDouble(Time::S) + uwgreh.m_error;
    dataTest << idMsg << " " << sender << " " << currentInt << " " << timestampSend << " " << timestampReceive << " " << uwgreh.GetETXCost() << std::endl;

    if (myPos.x == uwgreh.GetDestPosition().x &&
        myPos.y == uwgreh.GetDestPosition().y &&
        myPos.z == uwgreh.GetDestPosition().z) {
        pktsSuccessReceived[sender].push_back(timestampSend);

        pktsSuccessReceived[sender].push_back(timestampReceive);
    }
    else if (isOptimal) {
        std::fstream optimalPath(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("optimal_msg_alt") + std::string(".txt"), std::fstream::in);
        int id_msg = 0, sender = 0;
        double cost = 0;
        int nextHop = 0;
        while (optimalPath >> id_msg >> sender >> cost) {
            if (id_msg == idMsg && sender == currentInt) {
                optimalPath >> id_msg >> sender >> cost;
                break;
            }
        }
        optimalPath.close();

        nextHop = sender;
        ash.SetNextHop((AquaSimAddress)nextHop);
        uwgreh.SetSenderAddr(current);
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        Ptr<Packet> pkt2 = Create<Packet>();
        int retries = 0;
        for (const auto &t : m_MDTPhyNeigh) {
            if (std::get<1>(t) == ash.GetNextHop()) {
                double cost = uwgreh.GetETXCost() + std::get<0>(t);
                uwgreh.SetETXCost(cost);

                double prob = m_rand->GetValue(0, 1);
                while (prob > (1.0 / std::get<0>(t))) {  // FAIL
                    transmittedUWGRE++;                  // UWGRE failed
                    retries++;
                    prob = m_rand->GetValue(0, 1);
                }
                break;
            }
        }
        ash.SetNumForwards(retries);
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                            pkt2, ash.GetNextHop(), Seconds(0));
    }
    else if (uwgreh.GetRelayAddr().GetAsInt() == NULL || uwgreh.GetRelayAddr().GetAsInt() == current.GetAsInt()) {
        double minCost = 99999999;
        std::tuple<double, AquaSimAddress, Vector3D, double> minNeigh;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == uwgreh.GetSenderAddr()) {
                continue;
            }

            double cost = std::get<0>(*it) + CalculateDistance(std::get<2>(*it), uwgreh.GetDestPosition());
            if (cost < minCost) {
                minCost = cost;
                minNeigh = (*it);
            }
        }
        double minMDTcost = 0;
        for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
            double MDTcost = 0;
            if (std::get<1>(*it) == current) {
                continue;
            }
            bool found = false;
            for (auto &v : m_MDTPhyNeigh) {
                if (std::get<1>(*it) == std::get<1>(v)) {
                    found = true;
                }
            }
            if (found) {
                continue;
            }
            MDTcost = std::get<0>(*it);

            double cost = MDTcost + CalculateDistance(std::get<2>(*it), uwgreh.GetDestPosition());
            if (cost < minCost) {
                minCost = cost;
                minNeigh = (*it);
                minMDTcost = MDTcost;
            }
        }
        if (minCost < CalculateDistance(myPos, uwgreh.GetDestPosition())) {
            bool isPhyNeigh = false;

            for (auto &v : m_MDTPhyNeigh) {
                if (std::get<1>(v) == std::get<1>(minNeigh)) {
                    isPhyNeigh = true;
                    break;
                }
            }
            if (isPhyNeigh) {
                ash.SetNextHop(std::get<1>(minNeigh));
                uwgreh.SetRelayAddr((AquaSimAddress)NULL);
            }
            else {
                uwgreh.SetRelayAddr(std::get<1>(minNeigh));
            }

            uwgreh.SetSenderAddr(current);
            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();

            Ptr<Packet> pkt2 = Create<Packet>();

            transmittedUWGRE++;
            bool fnd = false;
            if (!isPhyNeigh) {
                fTableMember f;
                for (const auto &t : m_fTable) {
                    if (t.source == current && t.dest == uwgreh.GetRelayAddr() && t.cost == minMDTcost) {
                        f = t;
                        fnd = true;
                    }
                }

                ash.SetNextHop(f.succ);
            }
            double timeTaken = 0;
            int retries = 0;
            for (const auto &t : m_MDTPhyNeigh) {
                if (std::get<1>(t) == ash.GetNextHop()) {
                    double cost = uwgreh.GetETXCost() + std::get<0>(t);
                    uwgreh.SetETXCost(cost);

                    double prob = m_rand->GetValue(0, 1);
                    while (prob > (1.0 / std::get<0>(t))) {  // FAIL
                        transmittedUWGRE++;                  // UWGRE failed
                        timeTaken += 4.8;
                        retries++;
                        prob = m_rand->GetValue(0, 1);
                    }
                    break;
                }
            }
            ash.SetNumForwards(retries);
            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                                pkt2, ash.GetNextHop(), Seconds(0));
        }
        else {
            Ptr<Packet> pkt2 = Create<Packet>();

            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);
            transmittedUWGRE++;
            MDT_Greedy(pkt2);
        }
    }
    else {
        Ptr<Packet> pkt2 = Create<Packet>();
        transmittedUWGRE++;
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);
        MDT_Greedy(pkt2);
    }
    dataTest.close();
}

void AquaSimUWGRE::MDT_Greedy(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    std::tuple<AquaSimAddress, AquaSimAddress> e = Get_NextHop_NoPkt(uwgreh.GetDestAddr(), uwgreh.GetRelayAddr(), uwgreh.GetDestPosition());

    uwgreh.SetRelayAddr(std::get<1>(e));

    if ((std::get<0>(e)).GetAsInt() == NULL) {  // node closest to dest
    }
    else {
        ash.SetNextHop(std::get<0>(e));

        double oldETX = uwgreh.GetETXCost();

        double etx = 1;

        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == ash.GetNextHop()) {
                etx = std::get<0>(*it);
            }
        }

        double etx_back = 1;
        double oldETX_back = uwgreh.m_data;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == uwgreh.GetSenderAddr()) {
                etx_back = std::get<0>(*it);
            }
        }

        uwgreh.SetETXCost(oldETX + etx);

        uwgreh.m_data = oldETX_back + etx_back;

        if (!isMDTMode && 0) {
            uwgreh.SetDestAddr(std::get<0>(e));
        }

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);
        int retries = 0;
        double prob = m_rand->GetValue(0, 1);
        while (prob > (1.0 / etx)) {  // FAIL
            if (isMDTMode) {
                transmittedMDT++;
            }
            else {
                transmittedUWGRE++;  // UWGRE failed
            }
            retries++;
            prob = m_rand->GetValue(0, 1);
        }
        uwgreh.HandleNegativeSenderPosition();
        Ptr<Packet> pkt2 = Create<Packet>();

        ash.SetNumForwards(retries);
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                            pkt2, ash.GetNextHop(), Seconds(0));
    }
}

AquaSimAddress AquaSimUWGRE::Get_NextHop(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    AquaSimAddress dest = uwgreh.GetDestAddr();
    AquaSimAddress relay = uwgreh.GetRelayAddr();

    Vector3D destPos = uwgreh.GetDestPosition();
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;

    for (it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        if (std::get<1>(*it) == dest) {
            return std::get<1>(*it);
        }
    }

    if ((relay).GetAsInt() != NULL && relay != AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
        std::vector<fTableMember>::iterator it2;

        fTableMember t;

        for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
            if ((*it2).dest == relay) {
                t = (*it2);
            }
        }

        return t.succ;
    }
    double minDist = 9999999999999;

    AquaSimAddress v, closestNeighbor;
    for (it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        AquaSimAddress neigh = std::get<1>(*it);
        Vector3D neighPos = std::get<2>(*it);

        double dist = CalculateDistance(neighPos, destPos);
        if (dist < minDist) {
            closestNeighbor = neigh;
            minDist = dist;
        }
    }

    if (CalculateDistance(myPos, destPos) > minDist) {
        uwgreh.SetRelayAddr((AquaSimAddress)NULL);
        v = closestNeighbor;
        return v;
    }

    minDist = 9999999999999;
    for (it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto &v : m_MDTPhyNeigh) {
            if (std::get<1>(*it) == std::get<1>(v)) {
                found = true;
                break;
            }
        }

        if (std::get<1>(*it) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (found) {
            continue;
        }
        AquaSimAddress neigh = std::get<1>(*it);
        Vector3D neighPos = std::get<2>(*it);

        double dist = CalculateDistance(neighPos, destPos);
        if (dist < minDist) {
            closestNeighbor = neigh;
            minDist = dist;
        }
    }

    if (CalculateDistance(myPos, destPos) > minDist) {
        v = closestNeighbor;
        std::vector<fTableMember>::iterator it2;

        fTableMember t;

        for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
            if ((*it2).dest == v) {
                t = (*it2);
            }
        }

        uwgreh.SetRelayAddr(v);

        return t.succ;
    }
    else {
        return NULL;  // NULL
    }
}

std::tuple<AquaSimAddress, AquaSimAddress> AquaSimUWGRE::Get_NextHop_NoPkt(AquaSimAddress dest, AquaSimAddress relay, Vector3D destPos)
{
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;

    for (it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        if (std::get<1>(*it) == dest) {
            return std::make_tuple(std::get<1>(*it), relay);
        }
    }

    if ((relay).GetAsInt() != NULL && relay != AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
        std::vector<fTableMember>::iterator it2;

        fTableMember t;

        for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
            if ((*it2).dest == relay) {
                t = (*it2);

                break;
            }
        }

        for (const auto &v : m_MDTPhyNeigh) {
            if (std::get<1>(v) == relay) {
                t.succ = relay;
                break;
            }
        }

        return std::make_tuple(t.succ, relay);
    }
    double minDist = 9999999999999;

    AquaSimAddress v, closestNeighbor;
    for (it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        AquaSimAddress neigh = std::get<1>(*it);
        Vector3D neighPos = std::get<2>(*it);

        double dist = CalculateDistance(neighPos, destPos);
        if (dist < minDist) {
            closestNeighbor = neigh;
            minDist = dist;
        }
    }

    if (CalculateDistance(myPos, destPos) > minDist) {
        v = closestNeighbor;
        return std::make_tuple(v, (AquaSimAddress)NULL);
    }

    minDist = 9999999999999;
    for (it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto &v : m_MDTPhyNeigh) {
            if (std::get<1>(*it) == std::get<1>(v)) {
                found = true;
                break;
            }
        }

        if (std::get<1>(*it) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (found) {
            continue;
        }

        AquaSimAddress neigh = std::get<1>(*it);
        Vector3D neighPos = std::get<2>(*it);

        double dist = CalculateDistance(neighPos, destPos);
        if (dist < minDist) {
            closestNeighbor = neigh;
            minDist = dist;
        }
    }

    if (CalculateDistance(myPos, destPos) > minDist) {
        v = closestNeighbor;
        std::vector<fTableMember>::iterator it2;

        fTableMember t;

        for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
            if ((*it2).dest == v) {
                t = (*it2);
            }
        }

        return std::make_tuple(t.succ, v);
    }
    else {
        return std::make_tuple((AquaSimAddress)NULL, relay);  // NULL
    }
}

void AquaSimUWGRE::sendSymmetricETX()
{
    for (const auto &phy : m_PhyNeigh) {
        Ptr<Packet> pkt = Create<Packet>();

        AquaSimHeader ash;
        UWGREHeader uwgreh;

        uwgreh.SetMessType(SYMMETRIC_ETX);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(phy));
        ash.SetNextHop(std::get<1>(phy));

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;
        uwgreh.m_error = positionError;
        uwgreh.m_data = std::get<0>(phy);
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        double delay = 0;

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::receiveSymmetricETX(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    AquaSimAddress src = uwgreh.GetSrcAddr();
    double etx_received = uwgreh.m_data;
    double etx_src = 0;

    for (auto &phy : m_PhyNeigh) {
        if (std::get<1>(phy) == src) {
            etx_src = std::get<0>(phy);
            if (etx_received > etx_src) {
                std::get<0>(phy) = etx_received;
            }
            break;
        }
    }
}

Vector3D AquaSimUWGRE::findCorrectionVector(Vector3D myPos, Vector3D target, double c_C, double f, double cost)
{
    double difference_x = (myPos.x - target.x);
    double difference_y = (myPos.y - target.y);
    double difference_z = (myPos.z - target.z);

    double correction_x = (difference_x) / (std::sqrt(std::pow(difference_x, 2) + std::pow(difference_y, 2) + std::pow(difference_z, 2)));
    double correction_y = (difference_y) / (std::sqrt(std::pow(difference_x, 2) + std::pow(difference_y, 2) + std::pow(difference_z, 2)));
    double correction_z = (difference_z) / (std::sqrt(std::pow(difference_x, 2) + std::pow(difference_y, 2) + std::pow(difference_z, 2)));

    double final_x = (c_C * f * (cost - CalculateDistance(myPos, target)) * correction_x);
    double final_y = (c_C * f * (cost - CalculateDistance(myPos, target)) * correction_y);
    double final_z = (c_C * f * (cost - CalculateDistance(myPos, target)) * correction_z);
    return Vector3D(final_x, final_y, final_z);
}

void AquaSimUWGRE::UpdateForwardingTable(AquaSimAddress dest, AquaSimAddress sender, double error, double cost)
{
    bool found = false;
    for (auto &v : m_fTable) {
        if (v.source == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) && v.dest == dest && v.succ == sender && v.succ != dest) {
            v.error = error;
            v.cost = cost;
            found = true;
        }
    }

    if (!found) {
        fTableMember f, j;
        f.source = AquaSimAddress::ConvertFrom(dest);
        f.pred = AquaSimAddress::ConvertFrom(sender);
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = cost;
        f.error = error;
        m_fTable.push_back(f);

        j.source = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        j.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        j.succ = AquaSimAddress::ConvertFrom(sender);
        j.dest = AquaSimAddress::ConvertFrom(dest);
        j.cost = cost;
        j.error = error;
        m_fTable.push_back(j);
    }

    for (auto &v : m_MDTNeigh) {
        if (dest == std::get<1>(v)) {
            std::get<0>(v) = cost;
            std::get<3>(v) = error;
        }
    }

    for (auto &v : m_partOfTheKnown) {
        if (dest == std::get<1>(v)) {
            std::get<0>(v) = cost;
            std::get<3>(v) = error;
        }
    }
}

void AquaSimUWGRE::Adjustment()
{
    double sumError = 0;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        double dist = CalculateDistance(myPos, std::get<2>(*it));
        if (dist > std::get<0>(*it)) {
            std::vector<fTableMember>::iterator it2;

            fTableMember t;

            AquaSimAddress v;

            v = std::get<1>(*it);

            for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
                if ((*it2).dest == v) {
                    t = (*it2);
                }
            }

            double error_v = std::get<3>(*it);

            double f = positionError / (positionError + error_v);

            myPos = myPos + findCorrectionVector(myPos, std::get<2>(*it), c_C, f, std::get<0>(*it));
            sumError += ((abs(CalculateDistance(myPos, std::get<2>(*it)) - std::get<0>(*it))) / (CalculateDistance(myPos, std::get<2>(*it))));
        }
    }
    int sizeMDTNeigh = 0;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it2;
    for (it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (it2 = m_MDTPhyNeigh.begin(); it2 != m_MDTPhyNeigh.end(); it2++) {
            if (std::get<1>(*it) == std::get<1>(*it2)) {
                found = true;
            }
        }

        if (std::get<1>(*it) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (!found) {
            sizeMDTNeigh++;
            std::vector<fTableMember>::iterator it2;

            fTableMember t;

            AquaSimAddress v;

            v = std::get<1>(*it);

            for (it2 = m_fTable.begin(); it2 != m_fTable.end(); it2++) {
                if ((*it2).dest == v) {
                    t = (*it2);
                }
            }

            double error_v = std::get<3>(*it);

            double f = positionError / (positionError + error_v);
            myPos = myPos + findCorrectionVector(myPos, std::get<2>(*it), c_C, f, std::get<0>(*it));
            sumError += ((abs(CalculateDistance(myPos, std::get<2>(*it)) - std::get<0>(*it))) / (CalculateDistance(myPos, std::get<2>(*it))));
        }
    }

    double newError = sumError / ((double)m_InitializedNeigh.size() + sizeMDTNeigh);
    positionError = positionError * (1 - c_E) + (newError * c_E);

	UpdateMyPosition_All();
}

double AquaSimUWGRE::calculateAveragePositionErrorNeighbors()
{
    double sumError = 0;
    int sizeMDTNeigh = 0;
    for (auto it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        sumError += std::get<3>(*it);
    }

    for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto it2 = m_InitializedNeigh.begin(); it2 != m_InitializedNeigh.end(); it2++) {
            if (std::get<1>(*it) == std::get<1>(*it2)) {
                found = true;
                break;
            }
        }

        if (!found) {
            sumError += std::get<3>(*it);
            sizeMDTNeigh++;
        }
    }

    return (sumError / (m_MDTPhyNeigh.size() + sizeMDTNeigh));
}

void AquaSimUWGRE::DoAdjustment()
{
    //adjustmentPeriod = duration adjustment period;
    //deltaU = adjustment interval of u;
    if (numPeriodsJ == numPeriodsJmax) {
        Simulator::Schedule(Seconds(adjustmentPeriod), &AquaSimUWGRE::MDT_Maintenance, this);
        numMaintenancesCurrent = numMaintenances - 1;
        return;
    }
    numPeriodsJ++;
    numMaintenancesCurrent = 0;
    double e = calculateAveragePositionErrorNeighbors();
    double initialDelta = 2;

    double deltaU = std::min((initialDelta / (double)e), adjustmentPeriod);

    int numberOfAdjustments = std::ceil((adjustmentPeriod / deltaU));
    for (int i = 0; i < numberOfAdjustments; i++) {
        Simulator::Schedule(Seconds((double)deltaU * i), &AquaSimUWGRE::Adjustment, this);
    }
    Simulator::Schedule(Seconds((double)deltaU * numberOfAdjustments), &AquaSimUWGRE::UpdateMyPosition_All, this);

    if (!isFather) {
        m_reset = true;
    }
    else {
        if (DFS) {
            Simulator::Schedule(Seconds(20000), &AquaSimUWGRE::sendMDT_DFS_Initialization, this);
        }
        m_reset = true;
    }
}

void AquaSimUWGRE::receiveMDT_DFS_Comeback(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > notSent;

    if (isMDTMode) {
        m_InitializedNeigh = m_PhyNeigh;
    }

    for (auto &v : m_InitializedNeigh) {
        bool found = false;
        for (auto &p : m_MDTPhyNeigh) {
            if (std::get<1>(v) == std::get<1>(p)) {
                found = true;
            }
        }

        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            found = true;
        }

        if (!found) {
            notSent.push_back(v);
        }
    }

    if (isFather && notSent.size() == 0) {
        Simulator::Schedule(Seconds(600), &AquaSimUWGRE::MDT_Maintenance, this);
        return;
    }

    if (notSent.size() != 0) {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(MDT_DFS_INITIALIZE);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(notSent.front()));
        ash.SetNextHop(std::get<1>(notSent.front()));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;

        uwgreh.m_error = positionError;

        uwgreh.m_data = (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt();
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(5), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
    else if (!isFather) {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(MDT_DFS_COMEBACK);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(m_senderDFS);
        ash.SetNextHop(m_senderDFS);

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;

        uwgreh.m_error = positionError;

        uwgreh.m_data = (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt();
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(600), &AquaSimUWGRE::MDT_Maintenance, this); 
        Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::receiveMDT_DFS_Initialization(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    m_senderDFS = uwgreh.GetSenderAddr();

    if (isMDTMode) {
        m_InitializedNeigh = m_PhyNeigh;
    }

    sendJoin_Req();
}

void AquaSimUWGRE::sendMDT_DFS_Initialization()
{
    if (isMDTMode) {
        m_InitializedNeigh = m_PhyNeigh;
    }

    if (isFather) {
        UpdateMyMDTStatus();
        initializationMDT = true;
        if (m_reset) {
            DumpLocal();
            m_reset = false;
            m_waiting = 0;
            m_MDTNeigh.clear();
            m_MDTPhyNeigh.clear();
            m_partOfTheKnown.clear();
            m_hasSentNBSET.clear();

            m_fTable.clear();

            noMoreNewNeighbors = true;
        }
    }
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > notSent;

    for (auto &v : m_InitializedNeigh) {
        bool found = false;
        for (auto &p : m_MDTPhyNeigh) {
            if (std::get<1>(v) == std::get<1>(p)) {
                found = true;
            }
        }

        if (!isFather) {
            if (m_senderDFS == std::get<1>(v)) {
                found = true;
            }
        }

        if (!found) {
            notSent.push_back(v);
        }
    }

    if (notSent.size() != 0) {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(MDT_DFS_INITIALIZE);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(notSent.front()));
        ash.SetNextHop(std::get<1>(notSent.front()));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;

        uwgreh.m_error = positionError;

        uwgreh.m_data = (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt();
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(5), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
    else {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(MDT_DFS_COMEBACK);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(m_senderDFS);
        ash.SetNextHop(m_senderDFS);

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;

        uwgreh.m_error = positionError;

        uwgreh.m_data = (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt();
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(5), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));

        Simulator::Schedule(Seconds(600), &AquaSimUWGRE::MDT_Maintenance, this);
    }
}

void AquaSimUWGRE::sendMDT_Initialization()
{
    int i = 1;
    if (isFather && numPeriodsJ == 0) {
        UpdateMyMDTStatus();
    }

    for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(MDT_INITIALIZE);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(*it));
        ash.SetNextHop(std::get<1>(*it));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;

        uwgreh.m_error = positionError;
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));

        i++;
    }
}

void AquaSimUWGRE::MDT_Maintenance()
{
    NS_LOG_FUNCTION(this);
    if (numMaintenancesCurrent == numMaintenances) {
        if (numPeriodsJ != numPeriodsJmax) {
            Simulator::Schedule(Seconds(timeOutMDT), &AquaSimUWGRE::DoAdjustment, this);
            return;
        }
        else {
            return;
        }
    }
    for (const auto &p : m_MDTNeigh) {
        for (auto &v : m_partOfTheKnown) {
            if (std::get<1>(p) == std::get<1>(v)) {
                std::get<0>(v) = std::get<0>(p);
            }
        }
    }
    numMaintenancesCurrent++;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > vazio;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > neverSentBefore;
    std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > setNeighborsDT = calculateDelaunaySimplexes(vazio, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), false, false);
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > setNeighborsAll = calculateDelaunay(vazio, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), false, false);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > V_subset;
    std::vector<unsigned int> count(1000);
    std::vector<int> V_id;

    for (std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > &v : setNeighborsDT) {
        for (std::tuple<double, AquaSimAddress, Vector3D, double> &p : v) {
            bool found = false;

            for (auto &v : m_hasSentNBSET) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    found = true;
                }
            }

            for (auto &v : neverSentBefore) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    found = true;
                }
            }

            if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                found = true;
            }

            if (!found) {
                neverSentBefore.push_back(p);
            }
        }
    }

    m_MDTNeigh.clear();
    for (auto &p : setNeighborsAll) {
        bool found = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            m_MDTNeigh.push_back(p);
        }
    }

    for (unsigned int i = 1; i < count.size(); i++) {
        if (i != (unsigned int)(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt()) {
            V_id.push_back(i);
        }
    }

    for (unsigned int i = 0; i < V_id.size(); i++) {
        for (auto &p : setNeighborsAll) {
            if (V_id[i] == std::get<1>(p)) {
                V_subset.push_back(p);
            }
        }
    }

    for (auto &v : V_subset) {
        noMoreNewNeighbors = false;

        bool found = false;

        for (auto &x : m_hasSentNBSET) {
            if (std::get<1>(v) == std::get<1>(x)) {
                found = true;
            }
        }

        if (!found) {
            m_hasSentNBSET.push_back(v);
        }

        Ptr<Packet> pkt = Create<Packet>();
        Ptr<Packet> pkt2 = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(v));

        uwgreh.SetDestAddr(std::get<1>(v));
        uwgreh.SetDestPosition(std::get<2>(v));

        uwgreh.SetMessType(NB_SET_REQ_MAINTENANCE);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcPosition(myPos);

        uwgreh.m_error = positionError;

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.SetRelayAddr((AquaSimAddress)NULL);

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        bool found2 = false;
        for (auto &v : m_fTable) {
            if (v.dest == uwgreh.GetDestAddr()) {
                ash.SetNextHop(v.succ);  // previous sender is next hop

                uwgreh.m_data = 0;
                uwgreh.m_sizeDelaunayNeighbors = 0;

                pkt2->AddHeader(uwgreh);
                pkt2->AddHeader(ash);

                Forward(pkt2);

                found2 = true;
                break;
            }
        }

        if (!found2) {
            std::tuple<AquaSimAddress, AquaSimAddress> e = Get_NextHop_NoPkt(uwgreh.GetDestAddr(), uwgreh.GetRelayAddr(), uwgreh.GetDestPosition());

            uwgreh.SetRelayAddr(std::get<1>(e));
            if ((std::get<0>(e)).GetAsInt() == NULL) {
                pkt2->AddHeader(uwgreh);
                pkt2->AddHeader(ash);

                Send(pkt2);
            }
            else {
                ash.SetNextHop(std::get<0>(e));
                double etx = 1;
                for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
                    if (std::get<1>(*it) == ash.GetNextHop()) {
                        etx = std::get<0>(*it);
                    }
                }

                uwgreh.SetETXCost(etx);

                fTableMember f, j;
                f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                f.succ = AquaSimAddress::ConvertFrom(std::get<0>(e));
                f.dest = AquaSimAddress::ConvertFrom(std::get<0>(e));
                f.cost = uwgreh.GetETXCost();
                f.error = uwgreh.m_error;
                m_fTable.push_back(f);

                j.source = AquaSimAddress::ConvertFrom(std::get<0>(e));
                j.pred = AquaSimAddress::ConvertFrom(std::get<0>(e));
                j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                j.cost = uwgreh.GetETXCost();
                j.error = uwgreh.m_error;
                m_fTable.push_back(j);

                pkt2->AddHeader(uwgreh);
                pkt2->AddHeader(ash);

                Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this, pkt2, ash.GetNextHop(), Seconds(0));
            }
        }
    }
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > neverSentMDTNeigh;

    for (auto &p : m_MDTNeigh) {
        bool found = false;

        for (auto &v : m_hasSentNBSET) {
            if (std::get<1>(p) == std::get<1>(v)) {
                found = true;
            }
        }

        if (!found) {
            neverSentMDTNeigh.push_back(p);
        }
    }

    if (1) {
        for (auto &v : neverSentMDTNeigh) {
            m_hasSentNBSET.push_back(v);

            Ptr<Packet> pkt = Create<Packet>();
            Ptr<Packet> pkt2 = Create<Packet>();

            UWGREHeader uwgreh;
            AquaSimHeader ash;

            ash.SetErrorFlag(false);
            ash.SetDirection(AquaSimHeader::DOWN);

            ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            ash.SetDAddr(std::get<1>(v));

            uwgreh.SetDestAddr(std::get<1>(v));
            uwgreh.SetDestPosition(std::get<2>(v));

            uwgreh.SetMessType(NB_SET_REQ_NOTIFICATION);

            uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSrcPosition(myPos);

            uwgreh.m_error = positionError;

            uwgreh.HandleNegativeSrcPosition();

            uwgreh.SetRelayAddr((AquaSimAddress)NULL);

            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();

            bool found = false;
            for (auto &v : m_fTable) {
                if (v.dest == uwgreh.GetDestAddr()) {
                    ash.SetNextHop(v.succ);  // previous sender is next hop

                    uwgreh.m_sizeDelaunayNeighbors = 0;

                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);

                    Forward(pkt2);
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::tuple<AquaSimAddress, AquaSimAddress> e = Get_NextHop_NoPkt(uwgreh.GetDestAddr(), uwgreh.GetRelayAddr(), uwgreh.GetDestPosition());

                uwgreh.SetRelayAddr(std::get<1>(e));
                if ((std::get<0>(e)).GetAsInt() == NULL) {
                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);

                    Send(pkt2);
                }
                else {
                    ash.SetNextHop(std::get<0>(e));
                    double etx = 1;
                    for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
                        if (std::get<1>(*it) == ash.GetNextHop()) {
                            etx = std::get<0>(*it);
                        }
                    }

                    uwgreh.SetETXCost(etx);

                    fTableMember f, j;
                    f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                    f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                    f.succ = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    f.dest = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    f.cost = uwgreh.GetETXCost();
                    f.error = uwgreh.m_error;
                    m_fTable.push_back(f);

                    j.source = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    j.pred = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                    j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                    j.cost = uwgreh.GetETXCost();
                    j.error = uwgreh.m_error;
                    m_fTable.push_back(j);

                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);

                    Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                                        pkt2, ash.GetNextHop(), Seconds(0));
                }
            }
        }
    }

    Simulator::Schedule(Seconds(timeOutMDT * 40), &AquaSimUWGRE::MDT_Maintenance, this);
}

void AquaSimUWGRE::sendVPOD_Initialization()
{
    myPos.x = 0;
    myPos.y = 0;
    myPos.z = 0;

    m_VPODInits = 1;
    m_isInitializating = false;

    UpdateMyPosition();

    for (auto it = m_PhyNeigh.begin(); it != m_PhyNeigh.end(); it++) {
        Ptr<Packet> pkt = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        uwgreh.SetMessType(VPOD_INITIALIZE);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));
        uwgreh.SetSenderPosition(myPos);
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(*it));
        ash.SetNextHop(std::get<1>(*it));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;
        uwgreh.m_error = positionError;

        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        double delay = 1;

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::Initialization()
{
    Ptr<Object> sObject = GetNetDevice()->GetNode();
    Ptr<MobilityModel> sModel = sObject->GetObject<MobilityModel>();
    myPos = sModel->GetPosition();

    positionError = 1.0;
    adjustmentPeriod = 20;  // 20 Seconds
    c_E = 0.25;
    c_C = 0.1;         // Variable
    timeOutMDT = 240;
    noMoreNewNeighbors = true;
    numMaintenances = 5;
    numMaintenancesCurrent = 0;
    numPeriodsJ = 0;
    numPeriodsJmax = 0;
    is4D = false;
    DFS = true;
    m_MDTPhyNeigh.clear();
    m_InitializedNeigh.clear();
    m_MDTNeigh.clear();
    m_PhyNeigh.clear();
    m_waiting = 0;
    m_reset = false;

    receivedUWGRE = 0;
    receivedMDT = 0;
    transmittedUWGRE = 0;
    transmittedMDT = 0;

    m_rand = CreateObject<UniformRandomVariable>();
    m_randNormal = CreateObject<NormalRandomVariable>();

    SetVPODInitialization(true);
    initializationMDT = false;
    isInMDT = false;
    isFather = false;

    m_VPODInits = 0;
    bool loadDump = isLoad;
    bool loadPhysicalNeighbor = true;

    if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) == 1) {
        isFather = true;
    }

    if (loadDump) {
        Load();

        if (testMode) {
            loadTests();
        }
        else {
            Simulator::Schedule(Seconds(10 * (std::abs(numNodes - AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()))), &AquaSimUWGRE::DoAdjustment, this);
        }
    }
    else {
        if (isFather) {
            if (!isMDTMode) {
                Simulator::Schedule(Seconds(10000), &AquaSimUWGRE::sendVPOD_Initialization, this);
            }
            if (DFS) {
                Simulator::Schedule(Seconds(25000), &AquaSimUWGRE::sendMDT_DFS_Initialization, this);
            }
            else {
                Simulator::Schedule(Seconds(6000), &AquaSimUWGRE::sendMDT_Initialization, this);
            }
        }

        if (isMDTMode) {
            numPeriodsJ = 30;
            numPeriodsJmax = 30;
        }
        if (isSymmetric) {
            Simulator::Schedule(Seconds(6000), &AquaSimUWGRE::sendSymmetricETX, this);
        }
        if (loadPhysicalNeighbor) {
            loadPhysicalNeighbors();
        }
        else {
            findPhysicalNeighbors();
        }
    }

    if (!testMode) {
        Simulator::Schedule(Seconds(95000 + AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()), &AquaSimUWGRE::Dump, this);
    }
}

void AquaSimUWGRE::loadPhysicalNeighbors()
{
    int id_sender, id_receiver;
    double posX, posY, posZ;
    double etx;

    std::fstream filePhysicalNeighbors(std::string("idNetwork") + std::to_string(idNetwork) + std::string("/physicalNeighbors.txt"), std::fstream::in);

    while (filePhysicalNeighbors >> id_sender >> id_receiver >> posX >> posY >> posZ >> etx) {
        if (id_sender == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) {
            Vector3D pos(posX, posY, posZ);

            m_PhyNeigh.push_back(std::make_tuple(etx, (AquaSimAddress)id_receiver, pos, 1.0));
        }
    }

    filePhysicalNeighbors.close();
}

void AquaSimUWGRE::findPhysicalNeighbors()
{
    int expectedToReceive = 10;

    for (int i = 0; i < expectedToReceive; i++) {
        Ptr<Packet> pkt = Create<Packet>();

        ETXHeader etxh;
        AquaSimHeader ash;

        etxh.SetMessType(NB_DISCOVER_REQ);
        etxh.SetPkNum(m_pkCount);
        m_pkCount++;
        etxh.SetTs(Simulator::Now().ToDouble(Time::S));
        etxh.SetSenderPosition(myPos);
        etxh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        etxh.SetExpectedToReceive((uint8_t)expectedToReceive);

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(AquaSimAddress::GetBroadcast());
        ash.SetNextHop(AquaSimAddress::GetBroadcast());

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        pkt->AddHeader(etxh);
        pkt->AddHeader(ash);

        double delay = 30 * (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())).GetAsInt();

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::sendJoin_Req()
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) << this);

    if (m_MDTPhyNeigh.size() >= 1) {  // If any neighbor has initialized
        AquaSimAddress v = AquaSimAddress::ConvertFrom(std::get<1>(m_MDTPhyNeigh.front()));

        Ptr<Packet> pkt = Create<Packet>();

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetDestAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetMessType(JOIN_REQ);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.m_error = positionError;
        uwgreh.SetRelayAddr((AquaSimAddress)NULL);

        uwgreh.SetETXCost(std::get<0>(m_MDTPhyNeigh.front()));
        ash.SetNextHop(AquaSimAddress::ConvertFrom(v));  // previous sender is next hop

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_data = 0;

        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        Simulator::Schedule(Seconds(3), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
    else {
        Simulator::Schedule(Seconds(5), &AquaSimUWGRE::sendJoin_Req, this);
    }
}

void AquaSimUWGRE::receiveJoin_Req(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    Ptr<Packet> pkt2 = Create<Packet>();
    Ptr<Packet> pkt3 = pkt->Copy();

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    std::tuple<AquaSimAddress, AquaSimAddress> e = Get_NextHop_NoPkt(uwgreh.GetDestAddr(), uwgreh.GetRelayAddr(), uwgreh.GetDestPosition());

    uwgreh.SetRelayAddr(std::get<1>(e));

    if ((std::get<0>(e)).GetAsInt() == NULL) {  // node closest to dest
        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(uwgreh.GetSrcAddr());

        double etx_back = 1;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == uwgreh.GetSenderAddr()) {
                etx_back = std::get<0>(*it);
            }
        }

        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        fTableMember j;
        j.source = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        j.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        j.cost = uwgreh.m_data + etx_back;
        j.error = uwgreh.m_error;
        m_fTable.push_back(j);

        uwgreh.m_IsPosition_DestNX = uwgreh.m_IsPosition_SrcNX;

        uwgreh.m_IsPosition_DestNY = uwgreh.m_IsPosition_SrcNY;

        uwgreh.m_IsPosition_DestNZ = uwgreh.m_IsPosition_SrcNZ;

        uwgreh.SetDestAddr(uwgreh.GetSrcAddr());
        uwgreh.SetDestPosition(uwgreh.GetSrcPosition());

        uwgreh.SetMessType(JOIN_REPLY);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.m_error = positionError;
        uwgreh.m_data = 0;

        uwgreh.SetRelayAddr((AquaSimAddress)NULL);

        ash.SetNextHop(uwgreh.GetSenderAddr());  // previous sender is next hop

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        double etx = 1;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == ash.GetNextHop()) {
                etx = std::get<0>(*it);
            }
        }

        uwgreh.SetETXCost(etx);

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);
        Send(pkt2);
    }
    else {
        ash.SetNextHop(std::get<0>(e));

        double oldETX = uwgreh.GetETXCost();

        double etx = 1;

        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == ash.GetNextHop()) {
                etx = std::get<0>(*it);
            }
        }

        double etx_back = 1;
        double oldETX_back = uwgreh.m_data;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == uwgreh.GetSenderAddr()) {
                etx_back = std::get<0>(*it);
            }
        }

        uwgreh.SetETXCost(oldETX + etx);

        uwgreh.m_data = oldETX_back + etx_back;

        AquaSimAddress sender = uwgreh.GetSenderAddr();

        auto it = m_fTable.begin();
        while (it != m_fTable.end()) {
            if ((((*it).source == uwgreh.GetSenderAddr()) && ((*it).pred == uwgreh.GetSenderAddr()) && (*it).dest == uwgreh.GetSrcAddr())) {
                sender = (*it).succ;
                it = m_fTable.erase(it);
            }
            else {
                it++;
            }
        }
        it = m_fTable.begin();
        while (it != m_fTable.end()) {
            if ((((*it).succ == uwgreh.GetSenderAddr()) && ((*it).dest == uwgreh.GetSenderAddr()) && (*it).source == uwgreh.GetSrcAddr())) {
                sender = (*it).pred;
                it = m_fTable.erase(it);
            }
            else {
                it++;
            }
        }

        fTableMember f, j;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(sender);
        f.succ = AquaSimAddress::ConvertFrom(std::get<0>(e));
        f.dest = AquaSimAddress::ConvertFrom(std::get<0>(e));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        j.source = AquaSimAddress::ConvertFrom(std::get<0>(e));
        j.pred = AquaSimAddress::ConvertFrom(std::get<0>(e));
        j.succ = AquaSimAddress::ConvertFrom(sender);
        j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        j.cost = uwgreh.GetETXCost();
        j.error = uwgreh.m_error;
        m_fTable.push_back(j);

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        uwgreh.SetDestAddr(uwgreh.GetSrcAddr());
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                            pkt2, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::receiveJoin_Reply(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    AquaSimAddress myAddress = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    if (myAddress == uwgreh.GetDestAddr()) {
        UpdateMyMDTStatus();

        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        bool found3 = false;
        for (auto &p : m_partOfTheKnown) {
            if (std::get<1>(p) == uwgreh.GetSrcAddr()) {
                found3 = true;
            }
        }

        if (!found3) {
            m_partOfTheKnown.push_back(std::make_tuple(uwgreh.m_data, uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));
        }

        UpdateForwardingTable(uwgreh.GetSrcAddr(), uwgreh.GetSenderAddr(), uwgreh.m_error, uwgreh.m_data);

        Ptr<Packet> pkt2 = Create<Packet>();

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(uwgreh.GetSrcAddr());

        uwgreh.m_IsPosition_DestNX = uwgreh.m_IsPosition_SrcNX;

        uwgreh.m_IsPosition_DestNY = uwgreh.m_IsPosition_SrcNY;

        uwgreh.m_IsPosition_DestNZ = uwgreh.m_IsPosition_SrcNZ;

        uwgreh.SetDestAddr(uwgreh.GetSrcAddr());
        uwgreh.SetDestPosition(uwgreh.GetSrcPosition());

        uwgreh.SetMessType(NB_SET_REQ);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcPosition(myPos);

        uwgreh.m_error = positionError;

        uwgreh.HandleNegativeSrcPosition();

        bool isNew = true;

        for (auto &v : m_MDTPhyNeigh) {
            if (std::get<1>(v) == uwgreh.GetDestAddr()) {
                isNew = false;
            }
        }

        if (isNew) {
            uwgreh.SetRelayAddr(uwgreh.GetDestAddr());
        }
        else {
            uwgreh.SetRelayAddr((AquaSimAddress)NULL);
        }

        ash.SetNextHop(uwgreh.GetSenderAddr());  // previous sender is next hop

        uwgreh.m_data = 0;
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        double etx = 1;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == ash.GetNextHop()) {
                etx = std::get<0>(*it);
            }
        }

        uwgreh.SetETXCost(etx);

        uwgreh.m_dataType = 1;

        m_hasSentNBSET.push_back(std::make_tuple(etx, uwgreh.GetDestAddr(), uwgreh.GetDestPosition(), uwgreh.m_error));

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Send(pkt2);
    }
    else {
        Ptr<Packet> pkt2 = Create<Packet>();
        AquaSimAddress sender = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        fTableMember succ;
        for (auto &it : m_fTable) {
            if (it.dest == AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr())) {
                succ = it;
                break;
            }
        }
        for (auto &it : m_fTable) {
            if (it.source == sender && it.pred == sender && it.dest == AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr())) {
                it.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            }
        }
        for (auto &it : m_fTable) {
            if (it.dest == sender && it.succ == sender && it.source == AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr())) {
                it.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            }
        }

        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(succ.succ);
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);
        m_fTable.insert(m_fTable.begin(), f);

        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        f.pred = AquaSimAddress::ConvertFrom(succ.succ);
        f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);
        m_fTable.insert(m_fTable.begin(), f);

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
}

void AquaSimUWGRE::Send(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;
    NS_LOG_FUNCTION(this);
    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);
    bool found = false;

    Ptr<Packet> pkt2 = Create<Packet>();

    for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
        if ((*it).succ == uwgreh.GetDestAddr() && (uwgreh.GetMessType() == JOIN_REPLY || (*it).succ == (*it).dest)) {
            ash.SetNextHop(uwgreh.GetDestAddr());
            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();

            double etx = 1;
            for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
                if (std::get<1>(*it) == ash.GetNextHop()) {
                    etx = std::get<0>(*it);
                }
            }
            uwgreh.SetETXCost(etx);

            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                                pkt2, ash.GetNextHop(), Seconds(0));

            found = true;
            return;
        }
    }

    if (!found) {
        fTableMember t, f;

        t.source = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        t.pred = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        t.succ = AquaSimAddress::ConvertFrom(ash.GetNextHop());
        t.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        t.cost = uwgreh.GetETXCost();
        t.error = uwgreh.m_error;
        m_fTable.push_back(t);

        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        f.pred = AquaSimAddress::ConvertFrom(ash.GetNextHop());
        f.succ = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        f.dest = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetSenderPosition(myPos);

        double et = 0;
        for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
            if (std::get<1>(*it) == ash.GetNextHop()) {
                et = std::get<0>(*it);
            }
        }
        uwgreh.SetETXCost(et);

        uwgreh.HandleNegativeSenderPosition();

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                            pkt2, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::Forward(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;
    NS_LOG_FUNCTION(this);
    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    Ptr<Packet> pkt2 = Create<Packet>();
    bool found = false;

    for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
        if ((*it).succ == uwgreh.GetDestAddr() && (uwgreh.GetMessType() == JOIN_REPLY || (*it).succ == (*it).dest)) {
            double oldETX = uwgreh.GetETXCost();

            ash.SetNextHop(uwgreh.GetDestAddr());
            double etx = 1;
            for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
                if (std::get<1>(*it) == ash.GetNextHop()) {
                    etx = std::get<0>(*it);
                }
            }
            uwgreh.SetETXCost(oldETX + etx);

            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();

            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                                pkt2, ash.GetNextHop(), Seconds(0));
            found = true;
            break;
        }
    }

    double oldETX = uwgreh.GetETXCost();
    double etx = 0;
    AquaSimAddress hop;
    bool found2 = false;
    if (!found) {
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr()) {
                if (uwgreh.GetMessType() == NB_SET_REPLY_MAINTENANCE && (*it).succ == uwgreh.GetSenderAddr()) {
                    continue;
                }
                found2 = true;

                hop = (*it).succ;
                for (auto it2 = m_InitializedNeigh.begin(); it2 != m_InitializedNeigh.end(); it2++) {
                    if (std::get<1>(*it2) == ((*it).succ)) {
                        etx = std::get<0>(*it2);
                    }
                }

                if ((uwgreh.GetMessType() == JOIN_REPLY || uwgreh.GetMessType() == NB_SET_REPLY) && uwgreh.GetSenderAddr() != hop) {
                    break;
                }
            }
        }

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        bool found3 = false;
        if (found2) {
            ash.SetNextHop(hop);
            uwgreh.SetETXCost(oldETX + etx);

            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                                pkt2, ash.GetNextHop(), Seconds(0));
        }
        else {
            for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
                if ((*it).dest == uwgreh.GetDestAddr()) {
                    hop = (*it).succ;
                    for (auto it2 = m_InitializedNeigh.begin(); it2 != m_InitializedNeigh.end(); it2++) {
                        if (std::get<1>(*it2) == ((*it).succ)) {
                            etx = std::get<0>(*it2);
                        }
                    }
                    found3 = true;
                    if (uwgreh.GetMessType() == JOIN_REPLY || uwgreh.GetMessType() == NB_SET_REPLY) {
                        break;
                    }
                }
            }

            ash.SetNextHop(hop);
            uwgreh.SetETXCost(oldETX + etx);

            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Simulator::Schedule(Seconds(0), &AquaSimRouting::SendDown, this,
                                pkt2, ash.GetNextHop(), Seconds(0));
        }
    }
}

std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > AquaSimUWGRE::calculateDelaunay(std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > v, AquaSimAddress target, bool include, bool option)
{
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Cu;
    Cu.push_back(std::make_tuple(0, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), myPos, positionError));
    for (auto it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        Cu.push_back(std::make_tuple(std::get<0>(*it), std::get<1>(*it), std::get<2>(*it), std::get<3>(*it)));
    }

    for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto &i : Cu) {
            if (std::get<1>(*it) == std::get<1>(i)) {
                found = true;
            }
        }

        if (std::get<1>(*it) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (!found) {
            Cu.push_back(std::make_tuple(std::get<0>(*it), std::get<1>(*it), std::get<2>(*it), std::get<3>(*it)));
        }
    }

    if (include) {
        for (auto &p : v) {
            bool found = false;
            for (auto &i : Cu) {
                if (std::get<1>(p) == std::get<1>(i)) {
                    found = true;
                }
            }
            if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                found = true;
            }
            if (!found) {
                Cu.push_back(p);
            }
        }
    }

    for (auto &p : m_partOfTheKnown) {
        bool found = false;
        for (auto &i : Cu) {
            if (std::get<1>(p) == std::get<1>(i)) {
                found = true;
            }
        }
        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (!found) {
            Cu.push_back(p);
        }
    }
    if (Cu.size() >= 3) {
        std::fstream data(std::string("data") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt"), std::fstream::out);
        int i = 3;
        data << i << std::endl;
        data << Cu.size() << std::endl;
        for (auto it = Cu.begin(); it != Cu.end(); it++) {
            data << std::get<2>(*it).x << " " << std::get<2>(*it).y << " " << std::get<2>(*it).z << std::endl;
        }
        data.close();
        std::string cmd = std::string("./calculateDelaunay results") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt < data") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt ");
        system(cmd.c_str());

        std::fstream data2(std::string("results") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt"), std::fstream::in);
        int n = 0;

        data2 >> n;
        std::vector<std::vector<int> > simplexes;
        for (int j = 0; j < n; j++) {
            std::vector<int> vertices;
            for (int k = 0; k < i + 1; k++) {
                int z = 0;
                data2 >> z;
                vertices.push_back(z);
            }
            simplexes.push_back(vertices);
        }
        data2.close();

        std::vector<int> neighborsTargetID;
        for (auto j = simplexes.begin(); j != simplexes.end(); j++) {
            bool foundTarget = false;
            bool foundTarget2 = false;

            for (auto k = (*j).begin(); k != (*j).end(); k++) {
                if (std::get<1>(Cu[(*k)]) == target) {
                    foundTarget = true;
                    break;
                }
            }
            if (option) {
                for (auto k = (*j).begin(); k != (*j).end(); k++) {
                    if (std::get<1>(Cu[(*k)]) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                        foundTarget2 = true;
                        break;
                    }
                }
                if (foundTarget || foundTarget2) {
                    for (auto k = (*j).begin(); k != (*j).end(); k++) {
                        if (std::find(neighborsTargetID.begin(), neighborsTargetID.end(), (*k)) == neighborsTargetID.end()) {
                            neighborsTargetID.push_back((*k));
                        }
                    }
                }
            }
            else {
                if (foundTarget) {
                    for (auto k = (*j).begin(); k != (*j).end(); k++) {
                        if (std::find(neighborsTargetID.begin(), neighborsTargetID.end(), (*k)) == neighborsTargetID.end()) {
                            neighborsTargetID.push_back((*k));
                        }
                    }
                }
            }
        }

        std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > trueNeighbors;

        for (auto &vect : neighborsTargetID) {
            trueNeighbors.push_back(Cu[vect]);
        }

        return trueNeighbors;
    }
    else {
        std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > vazio;
        return (vazio);
    }
}

std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > AquaSimUWGRE::calculateDelaunaySimplexes(std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > v, AquaSimAddress target, bool include, bool option)
{
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Cu;
    Cu.push_back(std::make_tuple(0, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), myPos, positionError));
    for (auto it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        Cu.push_back(std::make_tuple(std::get<0>(*it), std::get<1>(*it), std::get<2>(*it), std::get<3>(*it)));
    }

    for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto &i : Cu) {
            if (std::get<1>(*it) == std::get<1>(i)) {
                found = true;
            }
        }
        if (std::get<1>(*it) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            Cu.push_back(std::make_tuple(std::get<0>(*it), std::get<1>(*it), std::get<2>(*it), std::get<3>(*it)));
        }
    }

    if (include) {
        for (auto &p : v) {
            bool found = false;
            for (auto &i : Cu) {
                if (std::get<1>(p) == std::get<1>(i)) {
                    found = true;
                }
            }
            if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                found = true;
            }
            if (!found) {
                Cu.push_back(p);
            }
        }
    }

    for (auto &p : m_partOfTheKnown) {
        bool found = false;
        for (auto &i : Cu) {
            if (std::get<1>(p) == std::get<1>(i)) {
                found = true;
            }
        }
        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (!found) {
            Cu.push_back(p);
        }
    }
    if (Cu.size() >= 3) {
        std::fstream data(std::string("data") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt"), std::fstream::out);
        int i = 3;
        data << i << std::endl;
        data << Cu.size() << std::endl;
        for (auto it = Cu.begin(); it != Cu.end(); it++) {
            data << std::get<2>(*it).x << " " << std::get<2>(*it).y << " " << std::get<2>(*it).z << std::endl;
        }
        data.close();
        std::string command = std::string("./calculateDelaunay results") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt < data") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt ");
        system(command.c_str());

        std::fstream data2(std::string("results") + std::to_string(idNetwork) + std::string("-") + std::to_string(numPeriodsJ) + std::string(".txt"), std::fstream::in);
        int n = 0;

        data2 >> n;
        std::vector<std::vector<int> > simplexes;
        for (int j = 0; j < n; j++) {
            std::vector<int> vertices;
            for (int k = 0; k < i + 1; k++) {
                int z = 0;
                data2 >> z;
                vertices.push_back(z);
            }
            simplexes.push_back(vertices);
        }
        data2.close();

        std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > neighborsTarget;
        for (auto j = simplexes.begin(); j != simplexes.end(); j++) {
            bool foundTarget = false;
            bool foundTarget2 = false;

            for (auto k = (*j).begin(); k != (*j).end(); k++) {
                if (std::get<1>(Cu[(*k)]) == target) {
                    foundTarget = true;
                    break;
                }
            }
            if (option) {
                for (auto k = (*j).begin(); k != (*j).end(); k++) {
                    if (std::get<1>(Cu[(*k)]) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                        foundTarget2 = true;
                        break;
                    }
                }
                if (foundTarget && foundTarget2) {
                    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > simplex;
                    for (auto k = (*j).begin(); k != (*j).end(); k++) {
                        simplex.push_back(Cu[(*k)]);
                    }
                    neighborsTarget.push_back(simplex);
                }
            }
            else {
                if (foundTarget) {
                    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > simplex;

                    for (auto k = (*j).begin(); k != (*j).end(); k++) {
                        simplex.push_back(Cu[(*k)]);
                    }
                    neighborsTarget.push_back(simplex);
                }
            }
        }

        return neighborsTarget;
    }
    else {
        std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > vazio;
        return (vazio);
    }
}
void AquaSimUWGRE::Load()
{
    int trial = toLoad;
    std::fstream nodeInfo(std::string("nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(trial) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) + std::string("/") + std::string("info") + std::string(".txt"), std::fstream::in);
    Vector3D realPos;

    int periodsJ = 0;

    nodeInfo >> periodsJ;
    numPeriodsJ = periodsJ;
    numPeriodsJmax = periodsJ + 1;

    nodeInfo >> realPos.x;
    nodeInfo >> realPos.y;
    nodeInfo >> realPos.z;

    nodeInfo >> myPos.x;
    nodeInfo >> myPos.y;
    nodeInfo >> myPos.z;

    nodeInfo >> positionError;

    int sizePhyNeigh = 0;
    int sizeMDTPhyNeigh = 0;
    int sizeInitializedNeigh = 0;
    int sizePartOfTheKnown = 0;
    int sizeMDTNeigh = 0;
    int sizeFTable = 0;

    nodeInfo >> sizePhyNeigh;

    for (int i = 0; i < sizePhyNeigh; i++) {
        double cost = 0;
        double error = 0;
        AquaSimAddress address = 0;
        double posX = 0;
        double posY = 0;
        double posZ = 0;

        nodeInfo >> cost;
        nodeInfo >> address;

        nodeInfo >> posX;
        nodeInfo >> posY;
        nodeInfo >> posZ;

        nodeInfo >> error;

        Vector3D pos(posX, posY, posZ);

        m_PhyNeigh.push_back(std::make_tuple(cost, (AquaSimAddress)address, pos, error));
    }

    nodeInfo >> sizeInitializedNeigh;

    for (int i = 0; i < sizeInitializedNeigh; i++) {
        double cost = 0;
        double error = 0;
        AquaSimAddress address = 0;
        double posX = 0;
        double posY = 0;
        double posZ = 0;

        nodeInfo >> cost;
        nodeInfo >> address;

        nodeInfo >> posX;
        nodeInfo >> posY;
        nodeInfo >> posZ;

        nodeInfo >> error;

        Vector3D pos(posX, posY, posZ);

        m_InitializedNeigh.push_back(std::make_tuple(cost, (AquaSimAddress)address, pos, error));
    }

    nodeInfo >> sizeMDTPhyNeigh;

    for (int i = 0; i < sizeMDTPhyNeigh; i++) {
        double cost = 0;
        double error = 0;
        AquaSimAddress address = 0;
        double posX = 0;
        double posY = 0;
        double posZ = 0;

        nodeInfo >> cost;
        nodeInfo >> address;

        nodeInfo >> posX;
        nodeInfo >> posY;
        nodeInfo >> posZ;

        nodeInfo >> error;

        Vector3D pos(posX, posY, posZ);

        m_MDTPhyNeigh.push_back(std::make_tuple(cost, (AquaSimAddress)address, pos, error));
    }

    nodeInfo >> sizeMDTNeigh;

    for (int i = 0; i < sizeMDTNeigh; i++) {
        double cost = 0;
        double error = 0;
        AquaSimAddress address = 0;
        double posX = 0;
        double posY = 0;
        double posZ = 0;

        nodeInfo >> cost;
        nodeInfo >> address;

        nodeInfo >> posX;
        nodeInfo >> posY;
        nodeInfo >> posZ;

        nodeInfo >> error;

        Vector3D pos(posX, posY, posZ);

        m_MDTNeigh.push_back(std::make_tuple(cost, (AquaSimAddress)address, pos, error));
    }

    nodeInfo >> sizePartOfTheKnown;

    for (int i = 0; i < sizePartOfTheKnown; i++) {
        double cost = 0;
        double error = 0;
        AquaSimAddress address = 0;
        double posX = 0;
        double posY = 0;
        double posZ = 0;

        nodeInfo >> cost;
        nodeInfo >> address;

        nodeInfo >> posX;
        nodeInfo >> posY;
        nodeInfo >> posZ;

        nodeInfo >> error;

        Vector3D pos(posX, posY, posZ);

        m_partOfTheKnown.push_back(std::make_tuple(cost, (AquaSimAddress)address, pos, error));
    }

    nodeInfo >> sizeFTable;

    for (int i = 0; i < sizeFTable; i++) {
        AquaSimAddress source = 0;
        AquaSimAddress pred = 0;
        AquaSimAddress succ = 0;
        AquaSimAddress dest = 0;

        double error = 0;
        double cost = 0;

        fTableMember f;

        nodeInfo >> source;
        nodeInfo >> pred;
        nodeInfo >> succ;
        nodeInfo >> dest;

        nodeInfo >> error;
        nodeInfo >> cost;

        f.source = (AquaSimAddress)source;
        f.pred = (AquaSimAddress)pred;
        f.succ = (AquaSimAddress)succ;
        f.dest = (AquaSimAddress)dest;

        f.error = error;
        f.cost = cost;

        m_fTable.push_back(f);
    }

    if (myPos.x < 0.0) {
        uint32_t myPosx = ((uint32_t)(myPos.x * (-1.0) * (1000000.0) + 0.5));
        myPos.x = ((double)(-1.0) * myPosx) / 1000000.0;
    }
    else {
        uint32_t myPosx = ((uint32_t)(myPos.x * (1000000.0) + 0.5));
        myPos.x = ((double)myPosx) / 1000000.0;
    }

    if (myPos.y < 0.0) {
        uint32_t myPosy = ((uint32_t)(myPos.y * (-1.0) * (1000000.0) + 0.5));
        myPos.y = ((double)(-1.0) * myPosy) / 1000000.0;
    }
    else {
        uint32_t myPosy = ((uint32_t)(myPos.y * (1000000.0) + 0.5));
        myPos.y = ((double)myPosy) / 1000000.0;
    }

    if (myPos.z < 0.0) {
        uint32_t myPosz = ((uint32_t)(myPos.z * (-1.0) * (1000000.0) + 0.5));
        myPos.z = ((double)(-1.0) * myPosz) / 1000000.0;
    }
    else {
        uint32_t myPosz = ((uint32_t)(myPos.z * (1000000.0) + 0.5));
        myPos.z = ((double)myPosz) / 1000000.0;
    }
}

void AquaSimUWGRE::loadTests()
{
    std::fstream data(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string(".txt"), std::fstream::in);
    int senderAddress = 0;
    int receiverAddress = 0;
    double timeSend = 0;
    int id = 0;
    std::string MDTOnly = "";

    std::string ifOptimal = std::string("");

    if (isOptimal) {
        ifOptimal = std::string("-optimal");
    }
    std::fstream last_id(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("id") + MDTOnly + ifOptimal + std::string(".txt"), std::fstream::in);
    std::fstream id_sender_receiver(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("id_msg") + MDTOnly + ifOptimal + std::string(".txt"), std::fstream::out | std::fstream::app);
    last_id >> id;

    last_id.close();

    while (data >> senderAddress >> receiverAddress >> timeSend) {
        if (senderAddress == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) {
            if (isMDTMode) {
                Simulator::Schedule(Seconds(timeSend), &AquaSimUWGRE::helperSendMDT, this, receiverAddress, id);
            }
            else {
                Simulator::Schedule(Seconds(timeSend), &AquaSimUWGRE::helperSendUWGRE, this, receiverAddress, id);
            }

            id_sender_receiver << id << " " << senderAddress << " " << receiverAddress << std::endl;
            id++;
        }
    }
    id_sender_receiver.close();
    std::fstream new_id(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("id") + MDTOnly + ifOptimal + std::string(".txt"), std::fstream::out);
    new_id << id;
    new_id.close();
    data.close();

    if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() == numNodes) {
        std::string cmd = std::string("./graph " + std::to_string(idNetwork) + " " + std::to_string(toLoad) + " " + std::to_string(testNumber) + " " + std::to_string(0) + " " + std::to_string(isOptimal));
        system(cmd.c_str());
    }

    Simulator::Schedule(Seconds(95000 + AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()), &AquaSimUWGRE::TestDump, this);
}

void AquaSimUWGRE::TestDump()
{
    std::string ifOptimal = std::string("");

    if (isOptimal) {
        ifOptimal = std::string("-optimal");
    }
    std::fstream dataEnergy(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("sentReceived") + ifOptimal + std::string(".csv"), std::fstream::out | std::fstream::app);
    std::fstream dataThroughput(std::string("tests") + std::to_string(idNetwork) + std::string("/test") + std::to_string(toLoad) + std::string("/test") + std::to_string(testNumber) + std::string("throughput") + ifOptimal + std::string(".csv"), std::fstream::out | std::fstream::app);
    dataEnergy << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << "," << GetNetDevice()->EnergyModel()->GetEnergy() << std::endl;

    for (const auto &v : pktsSuccessReceived) {
        unsigned int numPkts = v.second.size();
        double pktSize = 500.0;
        double average = 0;
        for (unsigned int i = 0; i < numPkts; i += 2) {
            double difference = v.second[i + 1] - v.second[i];
            average += difference;
        }

        if (numPkts != 0) {
            average /= ((double)numPkts);
        }
        double throughput = (((double)numPkts) * pktSize) / (average);
        dataThroughput << throughput << std::endl;
    }
    dataEnergy.close();
    dataThroughput.close();
}

void AquaSimUWGRE::Dump()
{
    if (testMode) {
        return;
    }
    int toSave = numPeriodsJ;
    std::fstream data(std::string("checkDelaunay") + std::to_string(idNetwork) + std::string("-") + std::to_string(toSave) + std::string("/dataDelaunay.txt"), std::fstream::out | std::fstream::app);
    std::fstream neighbors(std::string("checkDelaunay") + std::to_string(idNetwork) + std::string("-") + std::to_string(toSave) + std::string("/neighborsFound.txt"), std::fstream::out | std::fstream::app);

    std::fstream edgesETX;
    std::fstream edgesPhy;
    std::fstream edgesMDT;

    if (!testMode) {
        edgesETX.open(std::string("checkDelaunay") + std::to_string(idNetwork) + std::string("-") + std::to_string(toSave) + std::string("/edgesETX.txt"), std::fstream::out | std::fstream::app);
        edgesPhy.open(std::string("checkDelaunay") + std::to_string(idNetwork) + std::string("-") + std::to_string(toSave) + std::string("/edgesPhy.txt"), std::fstream::out | std::fstream::app);
        edgesMDT.open(std::string("checkDelaunay") + std::to_string(idNetwork) + std::string("-") + std::to_string(toSave) + std::string("/edgesMDT.txt"), std::fstream::out | std::fstream::app);
    }

    int numNeighbors = 0;
    if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() == 1) {
        std::string cmd = std::string("[[ -d nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string(" ]] || mkdir nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ);
        system(cmd.c_str());

        data << 3 << std::endl;
        data << numNodes << std::endl;

        if (!testMode) {
            edgesETX << "i" << std::endl;
            edgesETX << numNodes + 1 << std::endl;

            edgesPhy << "i" << std::endl;
            edgesPhy << numNodes + 1 << std::endl;

            for (int i = 0; i <= numNodes; i++) {
                edgesETX << i << std::endl;
                edgesPhy << i << std::endl;
            }
        }
    }
    std::string cmd = std::string("[[ -d nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) + std::string(" ]] || mkdir nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt());
    system(cmd.c_str());
    std::fstream nodeInfo(std::string("nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) + std::string("/") + std::string("info") + std::string(".txt"), std::fstream::out);

    data << myPos.x << " " << myPos.y << " " << myPos.z << std::endl;
    data.close();

    nodeInfo << numPeriodsJ << std::endl;
    nodeInfo << (GetNetDevice()->GetPosition()).x << " " << (GetNetDevice()->GetPosition()).y << " " << (GetNetDevice()->GetPosition()).z << std::endl;
    nodeInfo << myPos.x << " " << myPos.y << " " << myPos.z << std::endl;
    nodeInfo << positionError << std::endl;

    neighbors << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << ":";

    nodeInfo << m_PhyNeigh.size() << std::endl;
    for (auto &v : m_PhyNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;

        if (!testMode) {
            edgesPhy << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << " " << std::get<1>(v).GetAsInt() << " " << CalculateDistance(GetNetDevice()->GetPosition(), std::get<2>(v)) << std::endl;
        }
    }

    nodeInfo << m_InitializedNeigh.size() << std::endl;
    for (auto &v : m_InitializedNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }

    nodeInfo << m_MDTPhyNeigh.size() << std::endl;
    numNeighbors += m_MDTPhyNeigh.size();
    for (auto &v : m_MDTPhyNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
        neighbors << (std::get<1>(v)).GetAsInt() << ",";

        if (!testMode) {
            edgesETX << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << " " << std::get<1>(v).GetAsInt() << " " << std::get<0>(v) << std::endl;
        }
    }
    neighbors << ";";

    nodeInfo << m_MDTNeigh.size() << std::endl;
    for (const auto &p : m_InitializedNeigh) {
        edgesMDT << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << " " << std::get<1>(p).GetAsInt() << " " << std::get<0>(p) << std::endl;
    }
    for (auto &v : m_MDTNeigh) {
        bool found = false;

        for (auto &p : m_MDTPhyNeigh) {
            if (std::get<1>(p) == std::get<1>(v)) {
                found = true;
            }
        }
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
        if (!found) {
            edgesMDT << AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() << " " << std::get<1>(v).GetAsInt() << " " << std::get<0>(v) << std::endl;

            neighbors << (std::get<1>(v)).GetAsInt() << ",";
            numNeighbors++;
        }
    }

    nodeInfo << m_partOfTheKnown.size() << std::endl;
    for (auto &v : m_partOfTheKnown) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }
    nodeInfo << m_fTable.size() << std::endl;
    for (auto &v : m_fTable) {
        nodeInfo << v.source << " " << v.pred << " " << v.succ << " " << v.dest << " " << v.error << " " << v.cost << std::endl;
    }
    nodeInfo << numNeighbors << std::endl;
    nodeInfo << GetNetDevice()->EnergyModel()->GetEnergy() << std::endl;
    nodeInfo << GetNetDevice()->EnergyModel()->GetInitialEnergy() << std::endl;
    nodeInfo.close();

    neighbors << std::endl;
    neighbors.close();
}

void AquaSimUWGRE::DumpLocal()
{
    int numNeighbors = 0;
    if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt() == 1) {
        std::string cmd = std::string("[[ -d nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string(" ]] || mkdir nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ);
        system(cmd.c_str());
    }
    std::string cmd = std::string("[[ -d nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) + std::string(" ]] || mkdir nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt());
    system(cmd.c_str());
    std::fstream nodeInfo(std::string("nodes") + std::to_string(idNetwork) + std::string("/nodes") + std::to_string(numPeriodsJ) + std::string("/") + std::to_string(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()).GetAsInt()) + std::string("/") + std::string("info") + std::string(".txt"), std::fstream::out);

    nodeInfo << numPeriodsJ << std::endl;
    nodeInfo << (GetNetDevice()->GetPosition()).x << " " << (GetNetDevice()->GetPosition()).y << " " << (GetNetDevice()->GetPosition()).z << std::endl;
    nodeInfo << myPos.x << " " << myPos.y << " " << myPos.z << std::endl;
    nodeInfo << positionError << std::endl;

    nodeInfo << m_PhyNeigh.size() << std::endl;
    for (auto &v : m_PhyNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }

    nodeInfo << m_InitializedNeigh.size() << std::endl;
    for (auto &v : m_InitializedNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }

    nodeInfo << m_MDTPhyNeigh.size() << std::endl;
    numNeighbors += m_MDTPhyNeigh.size();
    for (auto &v : m_MDTPhyNeigh) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }

    nodeInfo << m_MDTNeigh.size() << std::endl;
    for (auto &v : m_MDTNeigh) {
        bool found = false;
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
        for (auto &p : m_MDTPhyNeigh) {
            if (std::get<1>(p) == std::get<1>(v)) {
                found = true;
            }
        }

        if (!found) {
            numNeighbors++;
        }
    }

    nodeInfo << m_partOfTheKnown.size() << std::endl;
    for (auto &v : m_partOfTheKnown) {
        nodeInfo << std::get<0>(v) << " " << std::get<1>(v) << " " << std::get<2>(v).x << " " << std::get<2>(v).y << " " << std::get<2>(v).z << " " << std::get<3>(v) << std::endl;
    }
    nodeInfo << m_fTable.size() << std::endl;
    for (auto &v : m_fTable) {
        nodeInfo << v.source << " " << v.pred << " " << v.succ << " " << v.dest << " " << v.error << " " << v.cost << std::endl;
    }
    nodeInfo << numNeighbors << std::endl;
    nodeInfo.close();
}

void AquaSimUWGRE::receiveNB_SET_REQ(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;
    NS_LOG_FUNCTION(this);
    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    double etx_back = 0;
    for (const auto &v : m_InitializedNeigh) {
        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            etx_back = std::get<0>(v);
            break;
        }
    }
    uwgreh.m_data += etx_back;

    Ptr<Packet> pkt2 = Create<Packet>();
    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    if (uwgreh.GetRange() != (double)current.GetAsInt()) {
        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        f.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.cost = uwgreh.m_data;
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        f.source = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        f.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);
    }

    bool foundPhy = false;
    bool foundMDT = false;
    bool foundPartOfKnown = false;
    for (auto it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        if (std::get<1>(*it) == uwgreh.GetSrcAddr()) {
            foundPhy = true;
            break;
        }
    }

    for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        if (std::get<1>(*it) == uwgreh.GetSrcAddr()) {
            foundMDT = true;
            break;
        }
    }

    for (auto &v : m_partOfTheKnown) {
        if (std::get<1>(v) == uwgreh.GetSrcAddr()) {
            foundPartOfKnown = true;
            std::get<2>(v) = uwgreh.GetSrcPosition();
            std::get<3>(v) = uwgreh.m_error;

            break;
        }
    }

    if (current == uwgreh.GetDestAddr()) {
        if (!foundMDT && !foundPhy && !foundPartOfKnown) {
            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > newA;
            newA.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));
            m_partOfTheKnown.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));

            fTableMember f;
            f.source = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            f.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.GetETXCost();
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > m_MDTNeigh_almost = calculateDelaunay(newA, current, true, false);
            Ptr<Packet> pkt2 = Create<Packet>();
            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Update_Neighbors(pkt2, m_MDTNeigh_almost);
        }

        fTableMember f, j;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        m_hasReceivedNBSET.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));
        std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > newN;
        newN.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));
        int i = 0;
        while ((unsigned int)i < m_partOfTheKnown.size()) {
            bool found = false;
            for (auto &p : m_fTable) {
                if (p.source == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) && p.dest == std::get<1>(m_partOfTheKnown[i])) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                m_partOfTheKnown.erase(m_partOfTheKnown.begin() + i);
                i = 0;
                continue;
            }
            i++;
        }
        i = 0;

        auto Nw_u = calculateDelaunay(newN, uwgreh.GetSrcAddr(), true, false);  //neighbors of GetSrcPosition in DT(Cu)
        if (uwgreh.GetMessType() == NB_SET_REQ_MAINTENANCE) {
            uwgreh.SetMessType(NB_SET_REPLY_MAINTENANCE);
        }
        else {
            uwgreh.SetMessType(NB_SET_REPLY);
        }
        UpdateForwardingTable(uwgreh.GetSrcAddr(), uwgreh.GetSenderAddr(), uwgreh.m_error, uwgreh.m_data);

        if (uwgreh.m_dataType == 2) {
            uwgreh.m_dataType = 2;
        }

        uwgreh.SetDestAddr(uwgreh.GetSrcAddr());

        ash.SetNextHop(uwgreh.GetSenderAddr());

        uwgreh.SetSenderAddr(current);
        uwgreh.SetSrcAddr(current);
        uwgreh.SetRelayAddr((AquaSimAddress)NULL);

        uwgreh.m_data = 0;
        uwgreh.m_error = positionError;

        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();
        std::vector<std::tuple<double, AquaSimAddress, Vector3D, double, bool, bool, bool> > Nw_u_Updated;
        for (auto &p : Nw_u) {
            bool NX = false;
            bool NY = false;
            bool NZ = false;
            if (std::get<2>(p).x < 0.0) {
                NX = true;
            }
            if (std::get<2>(p).y < 0.0) {
                NY = true;
            }
            if (std::get<2>(p).z < 0.0) {
                NZ = true;
            }
            if (std::get<1>(p) == current) {
                std::get<0>(p) = uwgreh.m_data;
            }

            Nw_u_Updated.push_back(std::make_tuple(std::get<0>(p), std::get<1>(p), std::get<2>(p), std::get<3>(p), NX, NY, NZ));
        }

        uwgreh.m_DelaunayNeighbors = Nw_u_Updated;
        uwgreh.m_sizeDelaunayNeighbors = uwgreh.m_DelaunayNeighbors.size();

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Send(pkt2);
    }
    else if ((uwgreh.GetRelayAddr()).GetAsInt() == NULL) {
        fTableMember t, f, j, l, k, lastSent, firstBack;
        bool foundPath = false;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetDestAddr() && (*it).succ != uwgreh.GetSenderAddr() && (*it).source == uwgreh.GetSrcAddr()) {
                t = (*it);
                if (uwgreh.GetMessType() == NB_SET_REQ_MAINTENANCE) {
                    foundPath = true;
                }
                break;
            }
        }
        if (!foundPath) {
            for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
                if ((*it).dest == uwgreh.GetDestAddr() && (*it).succ != uwgreh.GetSenderAddr()) {
                    t = (*it);
                    break;
                }
            }
        }

        bool found = false;
        bool found2 = false;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr()) {
                lastSent = (*it);
                found = true;
                break;
            }
        }
        fTableMember temp;

        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetSrcAddr() && (*it).source == uwgreh.GetDestAddr()) {
                firstBack = (*it);
                found2 = true;
                break;
            }
        }
        if ((found && found2 && lastSent.succ == uwgreh.GetSenderAddr()) || (uwgreh.GetRange() == (double)current.GetAsInt())) {
            uwgreh.m_data = firstBack.cost;

            double cost_not = 0;
            double rng = 0;

            for (const auto &p : m_InitializedNeigh) {
                if (std::get<1>(p) == lastSent.succ) {
                    cost_not = std::get<0>(p);
                    break;
                }
            }
            for (const auto &j : m_fTable) {
                if ((j).source == uwgreh.GetDestAddr() && (j).pred == lastSent.succ && (j).dest == uwgreh.GetSrcAddr()) {
                    rng = (double)(j).succ.GetAsInt();
                }
            }
            std::vector<int> numRepeated;
            int last;
            int j = 0;
            auto it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if (((*it).dest == uwgreh.GetSrcAddr() && (*it).pred == uwgreh.GetSenderAddr() && (*it).source == uwgreh.GetDestAddr())) {
                    numRepeated.push_back(j);
                }

                it++;

                j++;
            }
            if (numRepeated.size() != 0) {
                last = numRepeated[((int)numRepeated.size()) - 1];
                m_fTable.erase(m_fTable.begin() + last);
            }
            std::vector<int> numRepeated2;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr() && (*it).succ == uwgreh.GetSenderAddr()) {
                    numRepeated2.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated2.size() != 0) {
                last = numRepeated2[((int)numRepeated2.size()) - 1];

                m_fTable.erase(m_fTable.begin() + last);
            }

            std::vector<int> numRepeated3;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr() && (*it).succ == lastSent.succ) {
                    numRepeated3.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated3.size() != 0) {
                last = numRepeated3[((int)numRepeated3.size()) - 1];

                m_fTable.erase(m_fTable.begin() + last);
            }

            std::vector<int> numRepeated4;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetDestAddr() && (*it).source == uwgreh.GetSrcAddr() && (*it).succ == lastSent.pred) {
                    numRepeated4.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated4.size() != 0) {
                last = numRepeated4[((int)numRepeated4.size()) - 1];

                m_fTable.erase(m_fTable.begin() + last);
            }

            uwgreh.SetETXCost(lastSent.cost - cost_not);

            if ((uwgreh.GetRange() == (double)current.GetAsInt())) {
                uwgreh.SetRange(rng);
            }
        }
        else {
            bool foundPathRan = false;

            for (const auto &v : m_fTable) {
                if (v.source == uwgreh.GetSrcAddr() && (double)(v.succ).GetAsInt() == uwgreh.GetRange() && v.dest == uwgreh.GetDestAddr()) {
                    foundPathRan = true;
                    break;
                }
            }
            if (uwgreh.GetRange() != (double)current.GetAsInt() && (double)(t.succ).GetAsInt() != uwgreh.GetRange() && foundPathRan) {
                std::vector<int> numRepeated5;
                int j = 0;
                int last = 0;
                auto it = m_fTable.begin();
                while (it != m_fTable.end()) {
                    if ((*it).source == uwgreh.GetSrcAddr() && ((double)(((*it).succ).GetAsInt())) == uwgreh.GetRange() && (*it).dest == uwgreh.GetDestAddr()) {
                        numRepeated5.push_back(j);
                    }

                    it++;
                    j++;
                }
                if (numRepeated5.size() != 0) {
                    last = numRepeated5[((int)numRepeated5.size()) - 1];
                    m_fTable.erase(m_fTable.begin() + last);
                }
            }
        }

        double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }

        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
    else if (current == uwgreh.GetRelayAddr()) {
        fTableMember t, f, j;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetDestAddr()) {
                t = (*it);
                break;
            }
        }
        double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }
        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            j.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            j.pred = AquaSimAddress::ConvertFrom(t.succ);
            j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            j.cost = uwgreh.m_data;
            j.error = uwgreh.m_error;
            m_fTable.push_back(j);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }
        uwgreh.SetMessType(NB_SET_REQ);

        uwgreh.SetRelayAddr((AquaSimAddress)NULL);
        uwgreh.SetRange((double)uwgreh.GetSenderAddr().GetAsInt());

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
    else if ((uwgreh.GetRelayAddr()).GetAsInt() != NULL) {
        fTableMember t, f, j;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetRelayAddr()) {
                t = (*it);
                break;
            }
        }

        bool isRelayNeighbor = false;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == uwgreh.GetRelayAddr()) {
                isRelayNeighbor = true;
                break;
            }
        }

        if (isRelayNeighbor) {
            t.succ = uwgreh.GetRelayAddr();
        }
        double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }
        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            j.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            j.pred = AquaSimAddress::ConvertFrom(t.succ);
            j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            j.cost = uwgreh.m_data;
            j.error = uwgreh.m_error;
            m_fTable.push_back(j);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }

        uwgreh.SetRange(200.5);
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
}

void AquaSimUWGRE::Update_Neighbors(Ptr<Packet> pkt, std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > calculatedDelaunay)
{
    UWGREHeader uwgreh1;
    AquaSimHeader ash1;

    pkt->RemoveHeader(ash1);
    pkt->RemoveHeader(uwgreh1);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_Old = m_MDTNeigh;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_new;
    m_MDTNeigh.clear();
    for (auto &p : calculatedDelaunay) {
        bool found = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            m_MDTNeigh.push_back(p);
        }
    }
    int i = 0;
    while ((unsigned int)i < m_partOfTheKnown.size()) {
        bool found = false;
        for (auto &p : m_fTable) {
            if (p.source == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) && p.dest == std::get<1>(m_partOfTheKnown[i])) {
                found = true;
                break;
            }
        }

        if (!found) {
            m_partOfTheKnown.erase(m_partOfTheKnown.begin() + i);
            i = 0;
            continue;
        }
        i++;
    }
    i = 0;

    for (auto &p : calculatedDelaunay) {
        bool found = false;

        for (auto &i : Nu_Old) {
            if (std::get<1>(i) == std::get<1>(p)) {
                found = true;
                break;
            }
        }

        for (const auto &t : m_fTable) {
            if (std::get<1>(p) == t.dest && t.source == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
                found = true;
                break;
            }
        }

        for (auto &i : m_hasSentNBSET) {
            if (std::get<1>(i) == std::get<1>(p)) {
                found = true;
                break;
            }
        }

        if (std::get<1>(p) == uwgreh1.GetSrcAddr()) {
            found = true;
        }
        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            Nu_new.push_back(p);
        }
    }

    for (const auto &p : m_MDTNeigh) {
        for (auto &v : m_partOfTheKnown) {
            if (std::get<1>(p) == std::get<1>(v)) {
                std::get<0>(v) = std::get<0>(p);
            }
        }
    }
}

void AquaSimUWGRE::receiveNB_SET_REQ_NOTIFICATION(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    double etxBack = 0;
    for (auto &v : m_MDTPhyNeigh) {
        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            etxBack = std::get<0>(v);
            break;
        }
    }
    uwgreh.m_data += etxBack;

    Ptr<Packet> pkt2 = Create<Packet>();
    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    bool foundPhy = false;
    bool foundMDT = false;
    bool foundPartOfKnown = false;
    for (auto it = m_MDTPhyNeigh.begin(); it != m_MDTPhyNeigh.end(); it++) {
        if (std::get<1>(*it) == uwgreh.GetSrcAddr()) {
            foundPhy = true;
            break;
        }
    }

    for (auto it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        if (std::get<1>(*it) == uwgreh.GetSrcAddr()) {
            foundMDT = true;
            break;
        }
    }

    for (auto &v : m_partOfTheKnown) {
        if (std::get<1>(v) == uwgreh.GetSrcAddr()) {
            foundPartOfKnown = true;
            std::get<2>(v) = uwgreh.GetSrcPosition();
            std::get<3>(v) = uwgreh.m_error;

            break;
        }
    }

    if (uwgreh.GetRange() != (double)current.GetAsInt()) {
        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
        f.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.cost = uwgreh.m_data;
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        f.source = AquaSimAddress::ConvertFrom((AquaSimAddress)NULL);
        f.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);
    }
    if (current == uwgreh.GetDestAddr()) {
        m_hasReceivedNBSET.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));

        if (!foundMDT && !foundPhy && !foundPartOfKnown) {
            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > newA;
            newA.push_back(std::make_tuple(uwgreh.m_data, uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));
            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > m_MDTNeigh_almost = calculateDelaunay(newA, current, true, false);

            m_partOfTheKnown.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));

            Ptr<Packet> pkt2 = Create<Packet>();
            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);

            Update_Neighbors(pkt2, m_MDTNeigh_almost);

            fTableMember f;
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            f.cost = uwgreh.GetETXCost();
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            fTableMember j;
            j.source = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            j.pred = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            j.cost = uwgreh.m_data;
            j.error = uwgreh.m_error;
            m_fTable.push_back(j);
        }

        UpdateForwardingTable(uwgreh.GetSrcAddr(), uwgreh.GetSenderAddr(), uwgreh.m_error, uwgreh.m_data);
    }
    else if ((uwgreh.GetRelayAddr()).GetAsInt() == NULL) {
        fTableMember t, f, j, l, k, lastSent, firstBack;
        bool foundPath = false;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetDestAddr() && (*it).succ != uwgreh.GetSenderAddr() && (*it).source == uwgreh.GetSrcAddr()) {
                t = (*it);

                if (uwgreh.GetMessType() == NB_SET_REQ_MAINTENANCE) {
                    foundPath = true;
                }

                break;
            }
        }
        if (!foundPath) {
            for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
                if ((*it).dest == uwgreh.GetDestAddr() && (*it).succ != uwgreh.GetSenderAddr()) {
                    t = (*it);

                    break;
                }
            }
        }

        bool found = false;
        bool found2 = false;

        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr()) {
                lastSent = (*it);

                found = true;
                break;
            }
        }
        fTableMember temp;

        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetSrcAddr() && (*it).source == uwgreh.GetDestAddr()) {
                firstBack = (*it);

                found2 = true;
                break;
            }
        }

        if ((found && found2 && lastSent.succ == uwgreh.GetSenderAddr()) || (uwgreh.GetRange() == (double)current.GetAsInt())) {
            uwgreh.m_data = firstBack.cost;

            double cost_not = 0;
            double rng = 0;

            for (const auto &p : m_InitializedNeigh) {
                if (std::get<1>(p) == lastSent.succ) {
                    cost_not = std::get<0>(p);
                    break;
                }
            }
            for (const auto &j : m_fTable) {
                if ((j).source == uwgreh.GetDestAddr() && (j).pred == lastSent.succ && (j).dest == uwgreh.GetSrcAddr()) {
                    rng = (double)(j).succ.GetAsInt();
                }
            }
            std::vector<int> numRepeated;
            int last;
            int j = 0;
            auto it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if (((*it).dest == uwgreh.GetSrcAddr() && (*it).pred == uwgreh.GetSenderAddr() && (*it).source == uwgreh.GetDestAddr())) {
                    numRepeated.push_back(j);
                }

                it++;

                j++;
            }

            if (numRepeated.size() != 0) {
                last = numRepeated[((int)numRepeated.size()) - 1];
                m_fTable.erase(m_fTable.begin() + last);
            }
            std::vector<int> numRepeated2;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr() && (*it).succ == uwgreh.GetSenderAddr()) {
                    numRepeated2.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated2.size() != 0) {
                last = numRepeated2[((int)numRepeated2.size()) - 1];
                m_fTable.erase(m_fTable.begin() + last);
            }

            std::vector<int> numRepeated3;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetSrcAddr() && (*it).dest == uwgreh.GetDestAddr() && (*it).succ == lastSent.succ) {
                    numRepeated3.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated3.size() != 0) {
                last = numRepeated3[((int)numRepeated3.size()) - 1];
                m_fTable.erase(m_fTable.begin() + last);
            }

            std::vector<int> numRepeated4;
            j = 0;
            it = m_fTable.begin();
            while (it != m_fTable.end()) {
                if ((*it).source == uwgreh.GetDestAddr() && (*it).source == uwgreh.GetSrcAddr() && (*it).succ == lastSent.pred) {
                    numRepeated4.push_back(j);
                }

                it++;
                j++;
            }
            if (numRepeated4.size() != 0) {
                last = numRepeated4[((int)numRepeated4.size()) - 1];
                m_fTable.erase(m_fTable.begin() + last);
            }

            uwgreh.SetETXCost(lastSent.cost - cost_not);
            if ((uwgreh.GetRange() == (double)current.GetAsInt())) {
                uwgreh.SetRange(rng);
            }
        }
        else {
            bool foundPathRan = false;

            for (const auto &v : m_fTable) {
                if (v.source == uwgreh.GetSrcAddr() && (double)(v.succ).GetAsInt() == uwgreh.GetRange() && v.dest == uwgreh.GetDestAddr()) {
                    foundPathRan = true;
                    break;
                }
            }
            if (uwgreh.GetRange() != (double)current.GetAsInt() && (double)(t.succ).GetAsInt() != uwgreh.GetRange() && foundPathRan) {
                std::vector<int> numRepeated5;
                int j = 0;
                int last = 0;
                auto it = m_fTable.begin();
                while (it != m_fTable.end()) {
                    if ((*it).source == uwgreh.GetSrcAddr() && ((double)((*it).succ).GetAsInt()) == uwgreh.GetRange() && (*it).dest == uwgreh.GetDestAddr()) {
                        numRepeated5.push_back(j);
                    }

                    it++;
                    j++;
                }
                if (numRepeated5.size() != 0) {
                    last = numRepeated5[((int)numRepeated5.size()) - 1];
                    m_fTable.erase(m_fTable.begin() + last);
                }
            }
        }
		double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }

        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
    else if (current == uwgreh.GetRelayAddr()) {
        fTableMember t, f, j;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetDestAddr()) {
                t = (*it);
                break;
            }
        }
        double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }

        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            j.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            j.pred = AquaSimAddress::ConvertFrom(t.succ);
            j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            j.cost = uwgreh.m_data;
            j.error = uwgreh.m_error;
            m_fTable.push_back(j);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }

        uwgreh.SetMessType(NB_SET_REQ_NOTIFICATION);

        uwgreh.SetRelayAddr((AquaSimAddress)NULL);
        uwgreh.SetRange((double)uwgreh.GetSenderAddr().GetAsInt());

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
    else if ((uwgreh.GetRelayAddr()).GetAsInt() != NULL) {
        fTableMember t, f, j;
        for (auto it = m_fTable.begin(); it != m_fTable.end(); it++) {
            if ((*it).dest == uwgreh.GetRelayAddr()) {
                t = (*it);
                break;
            }
        }

        bool isRelayNeighbor = false;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == uwgreh.GetRelayAddr()) {
                isRelayNeighbor = true;
                break;
            }
        }

        if (isRelayNeighbor) {
            t.succ = uwgreh.GetRelayAddr();
        }

        double etx_send = 0;

        for (const auto &v : m_InitializedNeigh) {
            if (std::get<1>(v) == t.succ) {
                etx_send = std::get<0>(v);
                break;
            }
        }
        bool isNeighbor = false;
        for (const auto &p : m_InitializedNeigh) {
            if (std::get<1>(p) == uwgreh.GetDestAddr()) {
                isNeighbor = true;
                etx_send = std::get<0>(p);
                break;
            }
        }
        if (!isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(t.succ);
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);

            j.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            j.pred = AquaSimAddress::ConvertFrom(t.succ);
            j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            j.cost = uwgreh.m_data;
            j.error = uwgreh.m_error;
            m_fTable.push_back(j);
        }
        else if (isNeighbor) {
            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.cost = uwgreh.GetETXCost() + etx_send;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
            m_fTable.insert(m_fTable.begin(), f);

            f.source = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetDestAddr());
            f.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
            f.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
            f.cost = uwgreh.m_data;
            f.error = uwgreh.m_error;
            m_fTable.push_back(f);
        }

        uwgreh.SetRange(200.5);

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Forward(pkt2);
    }
}

void AquaSimUWGRE::UpdateMDT(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh1;
    AquaSimHeader ash1;

    pkt->RemoveHeader(ash1);
    pkt->RemoveHeader(uwgreh1);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_Old = m_MDTNeigh;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > trueDelaunay;
    for (auto &p : uwgreh1.m_DelaunayNeighbors) {
        trueDelaunay.push_back(std::make_tuple(std::get<0>(p), std::get<1>(p), std::get<2>(p), std::get<3>(p)));
    }
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > m_MDTNeigh_almost = calculateDelaunay(trueDelaunay, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), true, false);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_new;

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > neighborsLost;
    for (auto &p : Nu_Old) {
        bool found = false;

        for (auto &v : m_MDTNeigh_almost) {
            if (std::get<1>(p) == std::get<1>(v)) {
                found = true;
                break;
            }
        }
        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        if (!found) {
            neighborsLost.push_back(p);
        }
    }

    m_MDTNeigh.clear();
    for (auto &p : m_MDTNeigh_almost) {
        bool found = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            m_MDTNeigh.push_back(p);
        }
    }

    for (auto &p : m_MDTNeigh_almost) {
        bool found = false;

        for (auto &i : Nu_Old) {
            if (std::get<1>(i) == std::get<1>(p)) {
                found = true;
            }
        }
        if (std::get<1>(p) == uwgreh1.GetSrcAddr()) {
            found = true;
        }

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            Nu_new.push_back(p);
        }
    }

    for (const auto &p : m_MDTNeigh) {
        for (auto &v : m_partOfTheKnown) {
            if (std::get<1>(p) == std::get<1>(v)) {
                std::get<0>(v) = std::get<0>(p);
            }
        }
    }

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > filtered_Nu_new;

    for (auto &p : Nu_new) {
        bool found = false;
        bool foundReceived = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        for (auto &v : uwgreh1.m_DelaunayNeighbors) {
            if (std::get<1>(v) == std::get<1>(p)) {
                foundReceived = true;
                break;
            }
        }

        if (!found && foundReceived) {
            filtered_Nu_new.push_back(p);
        }
    }

    for (auto &v : filtered_Nu_new) {
        Ptr<Packet> pkt2 = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;
        bool alreadySent = false;
        for (auto &p : m_hasSentNBSET) {
            if (std::get<1>(p) == std::get<1>(v)) {
                alreadySent = true;
                break;
            }
        }
        if (!alreadySent) {
            m_hasSentNBSET.push_back(std::make_tuple(std::get<0>(v), std::get<1>(v), std::get<2>(v), std::get<3>(v)));
        }

        bool alreadyReceived = false;
        for (auto &p : m_hasReceivedNBSET) {
            if (std::get<1>(p) == std::get<1>(v)) {
                alreadyReceived = true;
                break;
            }
        }

        m_waiting++;

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.m_dataType = 2;
        uwgreh.SetDestAddr(std::get<1>(v));
        uwgreh.SetDestPosition(std::get<2>(v));

        uwgreh.HandleNegativeDestPosition();

        uwgreh.SetMessType(NB_SET_REQ);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.m_error = positionError;
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        uwgreh.m_sizeDelaunayNeighbors = 0;

        if (alreadyReceived) {
            uwgreh.SetRelayAddr((AquaSimAddress)NULL);
            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);
            Forward(pkt2);
            continue;
        }

        uwgreh.SetRelayAddr(uwgreh1.GetSrcAddr());
        uwgreh.SetRelayPosition(uwgreh1.GetSrcPosition());

        uwgreh.m_IsPosition_RelayNX = uwgreh1.m_IsPosition_SrcNX;

        uwgreh.m_IsPosition_RelayNY = uwgreh1.m_IsPosition_SrcNY;

        uwgreh.m_IsPosition_RelayNZ = uwgreh1.m_IsPosition_SrcNZ;

        ash.SetNextHop(uwgreh1.GetSenderAddr());  // previous sender is next hop

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Send(pkt2);
    }

    for (auto &v : neighborsLost) {
        Ptr<Packet> pkt2 = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetDestAddr(std::get<1>(v));
        uwgreh.SetDestPosition(std::get<2>(v));

        uwgreh.HandleNegativeDestPosition();
        uwgreh.SetMessType(NB_SET_REQ);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.m_error = positionError;
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();
        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.m_sizeDelaunayNeighbors = 0;

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);
    }

    if (!initializationMDT && m_waiting == 0) {
        initializationMDT = true;
        if (DFS) {
            Simulator::Schedule(Seconds(5), &AquaSimUWGRE::sendMDT_DFS_Initialization, this);
        }
        else {
            Simulator::Schedule(Seconds(15), &AquaSimUWGRE::sendMDT_Initialization, this);
            Simulator::Schedule(Seconds(400), &AquaSimUWGRE::MDT_Maintenance, this);
        }
    }
}

void AquaSimUWGRE::UpdateMDT_MAINTENANCE(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh1;
    AquaSimHeader ash1;

    pkt->RemoveHeader(ash1);
    pkt->RemoveHeader(uwgreh1);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_Old = m_MDTNeigh;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > trueDelaunay;

    for (auto &p : uwgreh1.m_DelaunayNeighbors) {
        trueDelaunay.push_back(std::make_tuple(std::get<0>(p), std::get<1>(p), std::get<2>(p), std::get<3>(p)));
    }
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > m_MDTNeigh_almost = calculateDelaunay(trueDelaunay, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), true, false);

    std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > m_simplexes = calculateDelaunaySimplexes(trueDelaunay, AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()), true, false);
    std::vector<std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > > m_okSimplexes;

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > Nu_new;

    m_MDTNeigh.clear();
    for (auto &p : m_MDTNeigh_almost) {
        bool found = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            m_MDTNeigh.push_back(p);
        }
    }

    for (auto &p : m_MDTNeigh_almost) {
        bool found = false;

        for (auto &i : Nu_Old) {
            if (std::get<1>(i) == std::get<1>(p)) {
                found = true;
            }
        }

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }
        if (!found) {
            Nu_new.push_back(p);
        }
    }

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > filtered_Nu_new;
    for (auto &p : Nu_new) {
        bool found = false;
        bool foundReceived = false;

        if (std::get<1>(p) == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress())) {
            found = true;
        }

        for (auto &v : uwgreh1.m_DelaunayNeighbors) {
            if (std::get<1>(v) == std::get<1>(p)) {
                foundReceived = true;
                break;
            }
        }

        if (!found && foundReceived) {
            filtered_Nu_new.push_back(p);
        }
    }
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > toSend;

    for (auto &p : m_simplexes) {
        bool found = false;
        for (std::tuple<double, AquaSimAddress, Vector3D, double> &v : p) {
            for (auto &x : m_hasSentNBSET) {
                if (std::get<1>(v) == std::get<1>(x)) {
                    found = true;
                }
            }
        }

        if (!found) {
            m_okSimplexes.push_back(p);
        }
    }

    for (auto &p : m_okSimplexes) {
        for (std::tuple<double, AquaSimAddress, Vector3D, double> &v : p) {
            bool found = false;
            for (auto &x : filtered_Nu_new) {
                if (std::get<1>(v) == std::get<1>(x)) {
                    found = true;
                }
            }

            if (found) {
                toSend.push_back(v);
            }
        }
    }
    for (auto &v : toSend) {
        Ptr<Packet> pkt2 = Create<Packet>();

        UWGREHeader uwgreh;
        AquaSimHeader ash;

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetDestAddr(std::get<1>(v));
        uwgreh.SetDestPosition(std::get<2>(v));

        uwgreh.HandleNegativeDestPosition();

        m_hasSentNBSET.push_back(std::make_tuple(std::get<0>(v), std::get<1>(v), std::get<2>(v), std::get<3>(v)));

        uwgreh.SetMessType(NB_SET_REQ);

        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.m_error = positionError;

        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.SetRelayAddr(uwgreh1.GetSrcAddr());
        uwgreh.SetRelayPosition(uwgreh1.GetSrcPosition());

        uwgreh.m_IsPosition_RelayNX = uwgreh1.m_IsPosition_SrcNX;

        uwgreh.m_IsPosition_RelayNY = uwgreh1.m_IsPosition_SrcNY;

        uwgreh.m_IsPosition_RelayNZ = uwgreh1.m_IsPosition_SrcNZ;

        ash.SetNextHop(uwgreh1.GetSenderAddr());  // previous sender is next hop

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        uwgreh.m_sizeDelaunayNeighbors = 0;

        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);

        Send(pkt2);
    }

    if (toSend.size() == 0) {
        noMoreNewNeighbors = true;

        std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> > neverSentMDTNeigh;

        for (auto &p : m_MDTNeigh) {
            bool found = false;

            for (auto &v : m_hasSentNBSET) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    found = true;
                }
            }

            if (!found) {
                neverSentMDTNeigh.push_back(p);
            }
        }

        if (noMoreNewNeighbors) {
            for (auto &v : neverSentMDTNeigh) {
                m_hasSentNBSET.push_back(v);

                Ptr<Packet> pkt2 = Create<Packet>();

                UWGREHeader uwgreh;
                AquaSimHeader ash;

                bool alreadyReceived = false;
                for (auto &p : m_hasReceivedNBSET) {
                    if (std::get<1>(p) == std::get<1>(v)) {
                        alreadyReceived = true;
                        break;
                    }
                }

                ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

                uwgreh.m_dataType = 2;
                uwgreh.SetDestAddr(std::get<1>(v));
                uwgreh.SetDestPosition(std::get<2>(v));

                uwgreh.HandleNegativeDestPosition();
                uwgreh.SetMessType(NB_SET_REQ_NOTIFICATION);

                uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

                uwgreh.m_error = positionError;
                uwgreh.SetSrcPosition(myPos);

                uwgreh.HandleNegativeSrcPosition();

                uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
                uwgreh.SetSenderPosition(myPos);

                uwgreh.HandleNegativeSenderPosition();

                uwgreh.m_sizeDelaunayNeighbors = 0;
                uwgreh.SetRelayAddr((AquaSimAddress)NULL);

                if (alreadyReceived) {
                    uwgreh.SetRelayAddr((AquaSimAddress)NULL);
                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);
                    Forward(pkt2);
                    continue;
                }

                uwgreh.SetRelayAddr(uwgreh1.GetSrcAddr());
                uwgreh.SetRelayPosition(uwgreh1.GetSrcPosition());

                uwgreh.m_IsPosition_RelayNX = uwgreh1.m_IsPosition_SrcNX;

                uwgreh.m_IsPosition_RelayNY = uwgreh1.m_IsPosition_SrcNY;

                uwgreh.m_IsPosition_RelayNZ = uwgreh1.m_IsPosition_SrcNZ;

                ash.SetNextHop(uwgreh1.GetSenderAddr());  // previous sender is next hop

                pkt2->AddHeader(uwgreh);
                pkt2->AddHeader(ash);

                Send(pkt2);
                continue;

                std::tuple<AquaSimAddress, AquaSimAddress> e = Get_NextHop_NoPkt(uwgreh.GetDestAddr(), uwgreh.GetRelayAddr(), uwgreh.GetDestPosition());

                uwgreh.SetRelayAddr(std::get<1>(e));
                if ((std::get<0>(e)).GetAsInt() == NULL) {
                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);

                    Send(pkt2);
                }
                else {
                    ash.SetNextHop(std::get<0>(e));
                    double etx = 1;
                    for (auto it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
                        if (std::get<1>(*it) == ash.GetNextHop()) {
                            etx = std::get<0>(*it);
                        }
                    }

                    uwgreh.SetETXCost(etx);

                    fTableMember f, j;
                    f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                    f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                    f.succ = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    f.dest = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    f.cost = uwgreh.GetETXCost();
                    f.error = uwgreh.m_error;
                    m_fTable.push_back(f);

                    j.source = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    j.pred = AquaSimAddress::ConvertFrom(std::get<0>(e));
                    j.succ = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
                    j.dest = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
                    j.cost = uwgreh.GetETXCost();
                    j.error = uwgreh.m_error;
                    m_fTable.push_back(j);

                    pkt2->AddHeader(uwgreh);
                    pkt2->AddHeader(ash);

                    Simulator::Schedule(Seconds(1), &AquaSimRouting::SendDown, this,
                                        pkt2, ash.GetNextHop(), Seconds(0));
                }
            }
        }
    }
}

void AquaSimUWGRE::receiveNB_SET_REPLY(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    Ptr<Packet> pkt2 = pkt->Copy();
    Ptr<Packet> pkt3 = Create<Packet>();

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    double etx_back = 0;
    for (const auto &v : m_InitializedNeigh) {
        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            etx_back = std::get<0>(v);
            break;
        }
    }
    uwgreh.m_data += etx_back;
    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    if (current == uwgreh.GetDestAddr()) {
        for (auto &v : uwgreh.m_DelaunayNeighbors) {
            bool found = false;

            for (auto &p : m_partOfTheKnown) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    found = true;
                }
            }
            if (std::get<1>(v) == current) {
                found = true;
            }
            if (!found) {
                m_partOfTheKnown.push_back(std::make_tuple(std::get<0>(v), std::get<1>(v), std::get<2>(v), std::get<3>(v)));
            }
        }

        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        if (uwgreh.m_dataType == 2) {
            m_waiting--;
        }
        m_hasReceivedNBSET.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));

        UpdateMDT(pkt2);
        UpdateForwardingTable(uwgreh.GetSrcAddr(), uwgreh.GetSenderAddr(), uwgreh.m_error, uwgreh.m_data);

        int i = 0;
        while ((unsigned int)i < m_partOfTheKnown.size()) {
            bool found = false;
            for (auto &p : m_fTable) {
                if (p.source == current && p.dest == std::get<1>(m_partOfTheKnown[i])) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                m_partOfTheKnown.erase(m_partOfTheKnown.begin() + i);
                i = 0;
                continue;
            }
            i++;
        }
        i = 0;
        for (const auto &p : m_MDTNeigh) {
            for (auto &v : m_partOfTheKnown) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    std::get<0>(v) = std::get<0>(p);
                }
            }
        }
    }
    else {
        pkt3->AddHeader(uwgreh);
        pkt3->AddHeader(ash);

        Forward(pkt3);
    }
}

void AquaSimUWGRE::receiveNB_SET_REPLY_MAINTENANCE(Ptr<Packet> pkt)
{
    UWGREHeader uwgreh;
    AquaSimHeader ash;

    Ptr<Packet> pkt2 = pkt->Copy();
    Ptr<Packet> pkt3 = Create<Packet>();

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(uwgreh);

    double etx_back = 0;
    for (const auto &v : m_InitializedNeigh) {
        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            etx_back = std::get<0>(v);
            break;
        }
    }
    uwgreh.m_data += etx_back;
    AquaSimAddress current = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());

    if (current == uwgreh.GetDestAddr()) {
        for (auto &v : uwgreh.m_DelaunayNeighbors) {
            bool found = false;

            for (auto &p : m_partOfTheKnown) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    found = true;
                }
            }
            if (std::get<1>(v) == current) {
                found = true;
            }
            if (!found) {
                m_partOfTheKnown.push_back(std::make_tuple(std::get<0>(v), std::get<1>(v), std::get<2>(v), std::get<3>(v)));
            }
        }
        fTableMember f;
        f.source = AquaSimAddress::ConvertFrom(uwgreh.GetSrcAddr());
        f.pred = AquaSimAddress::ConvertFrom(uwgreh.GetSenderAddr());
        f.succ = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.dest = AquaSimAddress::ConvertFrom(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        f.cost = uwgreh.GetETXCost();
        f.error = uwgreh.m_error;
        m_fTable.push_back(f);

        m_hasReceivedNBSET.push_back(std::make_tuple(uwgreh.GetETXCost(), uwgreh.GetSrcAddr(), uwgreh.GetSrcPosition(), uwgreh.m_error));

        UpdateMDT_MAINTENANCE(pkt2);
        UpdateForwardingTable(uwgreh.GetSrcAddr(), uwgreh.GetSenderAddr(), uwgreh.m_error, uwgreh.m_data);
        int i = 0;
        while ((unsigned int)i < m_partOfTheKnown.size()) {
            bool found = false;
            for (auto &p : m_fTable) {
                if (p.source == AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) && p.dest == std::get<1>(m_partOfTheKnown[i])) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                m_partOfTheKnown.erase(m_partOfTheKnown.begin() + i);
                i = 0;
                continue;
            }
            i++;
        }
        for (const auto &p : m_MDTNeigh) {
            for (auto &v : m_partOfTheKnown) {
                if (std::get<1>(p) == std::get<1>(v)) {
                    std::get<0>(v) = std::get<0>(p);
                }
            }
        }
    }
    else {
        pkt3->AddHeader(uwgreh);
        pkt3->AddHeader(ash);

        Forward(pkt3);
    }
}

void AquaSimUWGRE::replyPhysicalNeighbors(AquaSimAddress sender, uint8_t expectedToReceive)
{
    for (int i = 0; i < (int)expectedToReceive; i++) {
        Ptr<Packet> pkt = Create<Packet>();

        AquaSimHeader ash;
        ETXHeader etxh;

        etxh.SetMessType(NB_DISCOVER_REPLY);
        etxh.SetPkNum(m_pkCount);
        m_pkCount++;
        etxh.SetTs(Simulator::Now().ToDouble(Time::S));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(AquaSimAddress::ConvertFrom((sender)));
        ash.SetNextHop(AquaSimAddress::ConvertFrom((sender)));

        etxh.SetReceived(m_packetsReceivedReq[AquaSimAddress::ConvertFrom(sender)]);
        etxh.SetExpectedToReceive(expectedToReceive);
        etxh.SetSenderPosition(GetNetDevice()->GetPosition());

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);
        ash.SetNumForwards(1);

        pkt->AddHeader(etxh);
        pkt->AddHeader(ash);

        double delay = 1;

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::CalculateETX(AquaSimAddress sender, uint8_t expectedToReceive, Vector posNeighbor)
{
    uint8_t packetsReceivedReply = m_packetsReceivedReply[(sender).GetAsInt()];
    uint8_t packetsReceivedByNeigh = m_packetsReceivedByNeigh[(sender).GetAsInt()];

    double df = ((double)packetsReceivedByNeigh) / ((double)expectedToReceive);
    double dp = ((double)packetsReceivedReply) / ((double)expectedToReceive);

    double etx = 1.0 / (df * dp);

    if (etx <= 1.0) {
        etx = 1.0;
    }

    m_PhyNeigh.push_back(std::make_tuple(etx, sender, posNeighbor, 1.0));

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
}

void AquaSimUWGRE::VPOD_Initialize(Ptr<Packet> packet)
{
    AquaSimHeader ash;
    UWGREHeader uwgreh;

    packet->RemoveHeader(ash);
    packet->RemoveHeader(uwgreh);

    bool isNew = true;
    for (auto &v : m_InitializedNeigh) {
        if (std::get<1>(v) == uwgreh.GetSenderAddr()) {
            isNew = false;
            break;
        }
    }

    if (isNew) {
        bool found = false;
        for (auto it = m_PhyNeigh.begin(); it != m_PhyNeigh.end(); it++) {
            if (std::get<1>(*it) == uwgreh.GetSrcAddr()) {
                found = true;

                m_InitializedNeigh.push_back(std::make_tuple(std::get<0>(*it), std::get<1>(*it), uwgreh.GetSenderPosition(), std::get<3>(*it)));
                fTableMember f;
                f.source = NULL;
                f.pred = NULL;
                f.succ = AquaSimAddress::ConvertFrom(std::get<1>(*it));
                f.dest = AquaSimAddress::ConvertFrom(std::get<1>(*it));
                f.cost = std::get<0>(*it);
                f.error = std::get<3>(*it);
                m_fTable.push_back(f);

                break;
            }
        }
        NS_ASSERT_MSG(found == true, "Received initialization from a not recognized physical neighbor");
    }

    if (m_VPODInits == 0) {
        if (m_InitializedNeigh.size() == 0) {
            Ptr<Packet> pkt2 = Create<Packet>();
            pkt2->AddHeader(uwgreh);
            pkt2->AddHeader(ash);
            Simulator::Schedule(Seconds(50), &AquaSimUWGRE::VPOD_Initialize, this,
                                pkt2);
            return;
        }
        m_isInitializating = false;

        for (auto &v : m_InitializedNeigh) {
            Vector3D posNeigh = std::get<2>(v);
        }

        if (m_InitializedNeigh.size() == 1) {
            double radius = std::get<0>(m_InitializedNeigh.front());  //ETX
            Vector3D posNeigh = std::get<2>(m_InitializedNeigh.front());

            double x = m_randNormal->GetValue(0, 1);
            double y = m_randNormal->GetValue(0, 1);
            double z = m_randNormal->GetValue(0, 1);

            double mag = sqrt(x * x + y * y + z * z);
            x /= mag;
            y /= mag;
            z /= mag;

            myPos.x = posNeigh.x + x * radius;
            myPos.y = posNeigh.y + y * radius;
            myPos.z = posNeigh.z + z * radius;

            if (is4D) {
            }

            UpdateMyPosition();
        }
        else {
            AquaSimAddress farestNeighbor1, farestNeighbor2;
            std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it, it2;
            Vector3D point1, point2;
            double maxDist = -1;
            for (unsigned int i = 0; i < m_InitializedNeigh.size() - 1; i++) {
                for (unsigned int k = i + 1; k < m_InitializedNeigh.size(); k++) {
                    AquaSimAddress neigh1 = std::get<1>(m_InitializedNeigh[i]);
                    AquaSimAddress neigh2 = std::get<1>(m_InitializedNeigh[k]);

                    Vector3D neigh1Pos = std::get<2>(m_InitializedNeigh[i]);
                    Vector3D neigh2Pos = std::get<2>(m_InitializedNeigh[k]);

                    double dist = CalculateDistance(neigh1Pos, neigh2Pos);

                    if (dist > maxDist) {
                        farestNeighbor1 = neigh1;
                        farestNeighbor2 = neigh2;
                        point1 = neigh1Pos;
                        point2 = neigh2Pos;
                        maxDist = dist;
                    }
                }
            }
            if (m_InitializedNeigh.size() == 2) {
                point1 = std::get<2>(m_InitializedNeigh.front());
                point2 = std::get<2>(m_InitializedNeigh[1]);
                maxDist = CalculateDistance(point1, point2);

                farestNeighbor1 = std::get<1>(m_InitializedNeigh.front());
                farestNeighbor2 = std::get<1>(m_InitializedNeigh[1]);
            }
            Vector3D midpoint((point1.x + point2.x) / 2.0, (point1.y + point2.y) / 2.0, (point1.z + point2.z) / 2.0);

            double radius = (1 / 10.0) * maxDist;

            double x = m_randNormal->GetValue(0, 1);
            double y = m_randNormal->GetValue(0, 1);
            double z = m_randNormal->GetValue(0, 1);

            double u = m_rand->GetValue(0, 1);

            double mag = sqrt(x * x + y * y + z * z);
            x /= mag;
            y /= mag;
            z /= mag;

            // cbrt is cube root
            double c = cbrt(u);

            myPos.x = midpoint.x + x * c * radius;
            myPos.y = midpoint.y + y * c * radius;
            myPos.z = midpoint.z + z * c * radius;

            UpdateMyPosition();
        }

        for (auto it = m_PhyNeigh.begin(); it != m_PhyNeigh.end(); it++) {
            Ptr<Packet> pkt = Create<Packet>();

            AquaSimHeader ash;
            UWGREHeader uwgreh;

            uwgreh.SetMessType(VPOD_INITIALIZE);
            uwgreh.SetPkNum(m_pkCount);
            m_pkCount++;
            uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

            ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            ash.SetDAddr(std::get<1>(*it));
            ash.SetNextHop(std::get<1>(*it));

            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();

            ash.SetErrorFlag(false);
            ash.SetDirection(AquaSimHeader::DOWN);

            uwgreh.m_sizeDelaunayNeighbors = 0;
            uwgreh.m_error = positionError;

            pkt->AddHeader(uwgreh);
            pkt->AddHeader(ash);

            double delay = 1;

            Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                                pkt, ash.GetNextHop(), Seconds(0));
        }
    }

    m_VPODInits += 1;
    packet->AddHeader(uwgreh);
    packet->AddHeader(ash);
}

void AquaSimUWGRE::UpdateNeighborPosition(Ptr<Packet> packet)
{
    AquaSimHeader ash;
    UWGREHeader uwgreh;

    packet->RemoveHeader(ash);
    packet->RemoveHeader(uwgreh);

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (auto &it : m_PhyNeigh) {
        if (std::get<1>(it) == uwgreh.GetSrcAddr()) {
            Vector3D senderPosition = uwgreh.GetSrcPosition();

            bool found = false;

            for (auto &tup : m_InitializedNeigh) {
                if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
                    found = true;
                    std::get<2>(tup) = uwgreh.GetSrcPosition();
                    std::get<3>(tup) = uwgreh.m_error;
                    break;
                }
            }
            if (!found) {
                m_InitializedNeigh.push_back(std::make_tuple(std::get<0>(it), std::get<1>(it), senderPosition, std::get<3>(it)));
            }
        }
    }

    for (auto &it : m_MDTNeigh) {
        if (std::get<1>(it) == uwgreh.GetSrcAddr()) {
            Vector3D senderPosition = uwgreh.GetSrcPosition();

            for (auto &tup : m_InitializedNeigh) {
                if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
                    std::get<2>(tup) = uwgreh.GetSrcPosition();
                    std::get<3>(tup) = uwgreh.m_error;
                    break;
                }
            }

            for (auto &tup : m_partOfTheKnown) {
                if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
                    std::get<2>(tup) = uwgreh.GetSrcPosition();
                    std::get<3>(tup) = uwgreh.m_error;
                    break;
                }
            }
            for (auto &tup : m_MDTPhyNeigh) {
                if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
                    std::get<2>(tup) = uwgreh.GetSrcPosition();
                    std::get<3>(tup) = uwgreh.m_error;
                    break;
                }
            }
        }
    }

    for (auto &tup : m_MDTPhyNeigh) {
        if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
            std::get<2>(tup) = uwgreh.GetSrcPosition();
            std::get<3>(tup) = uwgreh.m_error;
            break;
        }
    }

    for (auto &tup : m_partOfTheKnown) {
        if (std::get<1>(tup) == uwgreh.GetSrcAddr()) {
            std::get<2>(tup) = uwgreh.GetSrcPosition();
            std::get<3>(tup) = uwgreh.m_error;
            break;
        }
    }

    if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) != uwgreh.GetDestAddr()) {
        Ptr<Packet> pkt2 = Create<Packet>();
        pkt2->AddHeader(uwgreh);
        pkt2->AddHeader(ash);
        Forward(pkt2);
    }
}

void AquaSimUWGRE::UpdateMyPosition()
{
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (it = m_PhyNeigh.begin(); it != m_PhyNeigh.end(); it++) {
        Ptr<Packet> pkt = Create<Packet>();

        AquaSimHeader ash;
        UWGREHeader uwgreh;

        uwgreh.SetMessType(UPDATE_NEIGH_POSITION);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(*it));
        ash.SetNextHop(std::get<1>(*it));

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();
        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.SetDestAddr(std::get<1>(*it));

        uwgreh.m_sizeDelaunayNeighbors = 0;
        uwgreh.m_error = positionError;
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        double delay = 0;

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::UpdateMyMDTStatus()
{
    float i = 0;
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
        Ptr<Packet> pkt = Create<Packet>();

        AquaSimHeader ash;
        UWGREHeader uwgreh;

        uwgreh.SetMessType(MDT_NEIGH_JOIN);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(*it));
        ash.SetNextHop(std::get<1>(*it));

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.SetDestAddr(std::get<1>(*it));

        uwgreh.m_sizeDelaunayNeighbors = 0;
        uwgreh.m_error = positionError;
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        double delay = 0;

        Simulator::Schedule(Seconds(delay + i), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }
}

void AquaSimUWGRE::UpdateMyPosition_All()
{
    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
        Ptr<Packet> pkt = Create<Packet>();

        AquaSimHeader ash;
        UWGREHeader uwgreh;

        uwgreh.SetMessType(UPDATE_NEIGH_POSITION);
        uwgreh.SetPkNum(m_pkCount);
        m_pkCount++;
        uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

        ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        ash.SetDAddr(std::get<1>(*it));
        ash.SetNextHop(std::get<1>(*it));

        uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
        uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

        uwgreh.SetSenderPosition(myPos);

        uwgreh.HandleNegativeSenderPosition();
        uwgreh.SetSrcPosition(myPos);

        uwgreh.HandleNegativeSrcPosition();

        uwgreh.SetDestAddr(std::get<1>(*it));

        ash.SetErrorFlag(false);
        ash.SetDirection(AquaSimHeader::DOWN);

        uwgreh.m_sizeDelaunayNeighbors = 0;
        uwgreh.m_error = positionError;
        pkt->AddHeader(uwgreh);
        pkt->AddHeader(ash);

        double delay = 0;

        Simulator::Schedule(Seconds(delay), &AquaSimRouting::SendDown, this,
                            pkt, ash.GetNextHop(), Seconds(0));
    }

    for (it = m_MDTNeigh.begin(); it != m_MDTNeigh.end(); it++) {
        bool found = false;

        for (auto &v : m_MDTPhyNeigh) {
            if (std::get<1>(v) == std::get<1>(*it)) {
                found = true;
            }
        }
        if (AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()) == std::get<1>(*it)) {
            found = true;
        }
        if (!found) {
            Ptr<Packet> pkt = Create<Packet>();

            AquaSimHeader ash;
            UWGREHeader uwgreh;

            uwgreh.SetMessType(UPDATE_NEIGH_POSITION);
            uwgreh.SetPkNum(m_pkCount);
            m_pkCount++;
            uwgreh.SetTs(Simulator::Now().ToDouble(Time::S));

            ash.SetSAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();
            uwgreh.SetSenderAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));
            uwgreh.SetSrcAddr(AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress()));

            uwgreh.SetSenderPosition(myPos);

            uwgreh.HandleNegativeSenderPosition();
            uwgreh.SetSrcPosition(myPos);

            uwgreh.HandleNegativeSrcPosition();
            uwgreh.SetDestAddr(std::get<1>(*it));

            ash.SetErrorFlag(false);
            ash.SetDirection(AquaSimHeader::DOWN);

            uwgreh.m_sizeDelaunayNeighbors = 0;
            uwgreh.m_error = positionError;
            pkt->AddHeader(uwgreh);
            pkt->AddHeader(ash);

            Forward(pkt);
        }
    }
}

void AquaSimUWGRE::SetVPODInitialization(bool init)
{
    m_isInitializating = init;
}

void AquaSimUWGRE::UpdateMDTNeigh(Ptr<Packet> packet)
{
    AquaSimHeader ash;
    UWGREHeader uwgreh;

    packet->RemoveHeader(ash);
    packet->RemoveHeader(uwgreh);

    if (m_reset) {
        DumpLocal();
        m_reset = false;
        isInMDT = false;
        initializationMDT = false;
        m_waiting = 0;

        noMoreNewNeighbors = true;

        m_MDTNeigh.clear();
        m_MDTPhyNeigh.clear();
        m_partOfTheKnown.clear();
        m_hasSentNBSET.clear();
        m_fTable.clear();

        for (const auto &t : m_InitializedNeigh) {
            fTableMember f;
            f.source = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());
            f.pred = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());
            f.succ = AquaSimAddress::ConvertFrom(std::get<1>(t));
            f.dest = AquaSimAddress::ConvertFrom(std::get<1>(t));
            f.cost = std::get<0>(t);
            f.error = std::get<3>(t);
            m_fTable.push_back(f);
        }
    }

    if (isMDTMode) {
        m_InitializedNeigh = m_PhyNeigh;
    }

    std::vector<std::tuple<double, AquaSimAddress, Vector3D, double> >::iterator it;
    for (it = m_InitializedNeigh.begin(); it != m_InitializedNeigh.end(); it++) {
        if (std::get<1>(*it) == uwgreh.GetSenderAddr()) {
            bool found = false;

            for (auto &tup : m_MDTPhyNeigh) {
                if (std::get<1>(tup) == uwgreh.GetSenderAddr()) {
                    found = true;
                    std::get<2>(tup) = uwgreh.GetSenderPosition();
                    std::get<3>(tup) = uwgreh.m_error;
                    break;
                }
            }
            if (!found) {
                m_MDTPhyNeigh.push_back(*it);
                fTableMember f;
                f.source = NULL;
                f.pred = NULL;
                f.succ = AquaSimAddress::ConvertFrom(std::get<1>(*it));
                f.dest = AquaSimAddress::ConvertFrom(std::get<1>(*it));
                f.cost = std::get<0>(*it);
                f.error = std::get<3>(*it);
                m_fTable.push_back(f);
            }
        }
    }
    packet->AddHeader(uwgreh);
    packet->AddHeader(ash);
}

bool AquaSimUWGRE::Recv(Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this);
    AquaSimHeader ash;
    UWGREHeader uwgreh;
    ETXHeader etxh;

    uint8_t received = 0;

    if (packet->GetSize() == 59) {  // ETX
        packet->RemoveHeader(ash);
        packet->RemoveHeader(etxh);

        unsigned char msg_type = etxh.GetMessType();
        switch (msg_type) {
            case NB_DISCOVER_REQ:
                received = m_packetsReceivedReq[(ash.GetSAddr()).GetAsInt()];
                m_packetsReceivedReq[(ash.GetSAddr()).GetAsInt()] += 1;

                if (+received == 0) {
                    double delay = 10;
                    Simulator::Schedule(Seconds(delay), &AquaSimUWGRE::replyPhysicalNeighbors, this, (ash.GetSAddr()).GetAsInt(), +etxh.GetExpectedToReceive());
                }
                return true;
                break;
            case NB_DISCOVER_REPLY:

                received = m_packetsReceivedReply[(ash.GetSAddr()).GetAsInt()];
                m_packetsReceivedReply[(ash.GetSAddr()).GetAsInt()] += 1;

                if (+received == 0) {
                    double delay = 10;
                    m_packetsReceivedByNeigh[(ash.GetSAddr()).GetAsInt()] = (int)etxh.GetReceived();
                    Simulator::Schedule(Seconds(delay), &AquaSimUWGRE::CalculateETX, this, (ash.GetSAddr()).GetAsInt(), +etxh.GetExpectedToReceive(), etxh.GetSenderPosition());
                }
                return true;
                break;
        }
    }
    else {
        Ptr<Packet> pkt = packet->Copy();
        packet->RemoveHeader(ash);
        packet->RemoveHeader(uwgreh);

        int idPkt = (int)ash.GetUId();

        if (rcvPkts.find(uwgreh.GetSenderAddr().GetAsInt()) == rcvPkts.end() || rcvPkts.at(uwgreh.GetSenderAddr().GetAsInt()).find(idPkt) == rcvPkts.at(uwgreh.GetSenderAddr().GetAsInt()).end()) {
            rcvPkts[uwgreh.GetSenderAddr().GetAsInt()].insert(idPkt);
        }
        else {
            return true;
        }

        unsigned char msg_type = uwgreh.GetMessType();

        switch (msg_type) {
            case JOIN_REQ:
                receiveJoin_Req(pkt);
                break;
            case JOIN_REPLY:
                receiveJoin_Reply(pkt);
                break;
            case NB_SET_REQ_MAINTENANCE:
            case NB_SET_REQ:
                receiveNB_SET_REQ(pkt);
                break;
            case NB_SET_REPLY:
                receiveNB_SET_REPLY(pkt);
                break;
            case NB_SET_REPLY_MAINTENANCE:
                receiveNB_SET_REPLY_MAINTENANCE(pkt);
                break;
            case NB_SET_REQ_NOTIFICATION:
                receiveNB_SET_REQ_NOTIFICATION(pkt);
                break;
            case VPOD_INITIALIZE:
                if (m_isInitializating) {
                    VPOD_Initialize(pkt);
                }
                break;
            case UPDATE_NEIGH_POSITION:
                UpdateNeighborPosition(pkt);
                break;
            case MDT_NEIGH_JOIN:
                UpdateMDTNeigh(pkt);
                break;
            case MDT_INITIALIZE:
                if (!isInMDT) {
                    isInMDT = true;
                    m_MDTNeigh.clear();
                    m_partOfTheKnown.clear();
                    m_hasSentNBSET.clear();
                    sendJoin_Req();
                }
                break;
            case MDT_DFS_INITIALIZE:
                if (!isInMDT) {
                    isInMDT = true;
                    receiveMDT_DFS_Initialization(pkt);
                }
                break;
            case MDT_DFS_COMEBACK:
                receiveMDT_DFS_Comeback(pkt);
                break;
            case UWGRE:
                receivedUWGRE++;
                UWGRERouting(pkt);
                break;
            case MDT:
                receivedMDT++;
                MDTRouting(pkt);
                break;
            case SYMMETRIC_ETX:
                receiveSymmetricETX(pkt);
                break;
        }
    }
    packet = 0;
    return true;
}
