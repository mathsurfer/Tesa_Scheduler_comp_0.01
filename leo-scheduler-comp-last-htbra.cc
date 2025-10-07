
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/point-to-point-module.h"

#include <numeric>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <iomanip>
#include <map>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("SchedulerThroughputComparison");

// Structure de données pour un utilisateur
struct UE
{
    uint32_t id;
    double distance;
    double sinr;
    uint32_t mcs;
    double SE;      // spectral efficiency
    uint32_t allocRBs;
    double throughput; // kbps
    double demand;     // kbps
};

struct MCS_Info { uint32_t MCS; double SE; };

// Fonctions
// Bankruptcy inter-group allocation 
std::vector<uint32_t> Bankruptcy(std::vector<uint32_t> claims, uint32_t cp)
{
    size_t n = claims.size();
    std::vector<uint32_t> alloc(n, 0);
    uint32_t remaining = cp;
    std::vector<bool> done(n, false);

    while (remaining > 0)
    {
        // Count unsatisfied
        size_t unsatisfied = 0;
        for (size_t i = 0; i < n; ++i) if (!done[i]) unsatisfied++;
        if (unsatisfied == 0) break;

        for (size_t i = 0; i < n && remaining > 0; ++i)
        {
            if (!done[i])
            {
                alloc[i] += 1;
                remaining -= 1;
                if (alloc[i] >= claims[i]) done[i] = true;
            }
        }
    }
    return alloc;
}




// La fonction d'allocation qu'on va employer dans notre simu
std::vector<uint32_t> IntraBinAllocate2(const std::vector<uint32_t>& demands, uint32_t totalChannels)
{
    size_t n = demands.size();
    std::vector<uint32_t> alloc(n, 0);
    uint32_t remaining = totalChannels;
    for (size_t i = 0; i < n && remaining > 0; ++i)
    {
        if (demands[i] <= remaining)
        {
            alloc[i] = demands[i];
            remaining -= demands[i];
        }
        else
        {
            alloc[i] = remaining;
            remaining = 0;
        }
    }
    return alloc;
}

// SINR / MCS mapping
double ComputeSinr(double distance)
{
    double f = 2.4e9;      // Hz
    double c = 3e8;
    double P_tx = 1000.0;  // W (toy)
    double B = 180000.0;   // Hz per RB
    double k = 1.38e-23;
    double T = 290.0;

    double FSPL = pow((4.0 * M_PI * distance * f / c), 2.0);
    double P_rx = P_tx / (FSPL + 1e-30);
    double N = k * T * B;
    double sinr = P_rx / (N + 1e-30);
    return 10.0 * log10(sinr + 1e-12);
}

MCS_Info GetMcs(double sinr_db)
{
    if (sinr_db < 0) return { 0, 0.08 };
    else if (sinr_db < 5) return { 1, 0.15 };
    else if (sinr_db < 10) return { 2, 0.3 };
    else if (sinr_db < 15) return { 3, 0.6 };
    else if (sinr_db < 20) return { 4, 1.2 };
    else if (sinr_db < 25) return { 5, 2.4 };
    else if (sinr_db < 28) return { 6, 3.6 };
    else if (sinr_db < 30) return { 7, 4.8 };
    else return { 8, 6.0 };
}

// Merge helper (non utilisée mais peu être  très pratique dans les prochaines versions )
template<typename T>
std::vector<T> MergeThreeVectors(const std::vector<T>& v1,
    const std::vector<T>& v2,
    const std::vector<T>& v3)
{
    std::vector<T> merged;
    merged.reserve(v1.size() + v2.size() + v3.size());
    merged.insert(merged.end(), v1.begin(), v1.end());
    merged.insert(merged.end(), v2.begin(), v2.end());
    merged.insert(merged.end(), v3.begin(), v3.end());
    return merged;
}

// Metrics
double JainFairness_d(const std::vector<double>& values)
{
    if (values.empty()) return 1.0;
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double sumSq = 0.0;
    for (auto v : values) sumSq += v * v;
    if (sumSq == 0.0) return 1.0;
    return (sum * sum) / (values.size() * sumSq + 1e-9);
}

double Efficiency(const std::vector<double>& throughput, const std::vector<uint32_t>& demand)
{
    double sumTh = std::accumulate(throughput.begin(), throughput.end(), 0.0);
    double sumDem = std::accumulate(demand.begin(), demand.end(), 0.0);
    if (sumDem == 0.0) return 1.0;
    return sumTh / (sumDem + 1e-9);
}



// ---------------- Global sim state ----------------
// configurable via cmdline
static uint32_t g_nUes = 5;
static std::string g_scheduler = "HTBRA";
static double g_simTime = 1.0;         // seconds
static double g_leoAltitude = 1200.0;   // km
static double g_subcarrierSpacing = 120.0; // kHz
static double g_channelBandwidth = 50e6; // Hz
static double g_TTI_ms = 1.0;          // TTI length in ms
static uint16_t g_dlPort = 4000;

// derived
static uint32_t g_totalChannels = 0;
static double g_RB_BW_Hz = 0.0;
static double g_Re = 6371e3;
static double g_deltaTheta = 5.0 * M_PI / 180.0;

// Pour les valeurs de débits
static Ptr<UniformRandomVariable> g_randVar = nullptr;
static NodeContainer g_nodes;
static std::vector<Ipv4InterfaceContainer> g_interfaces;
static std::vector<UE> g_UEs;

// demand set (kbps)
static uint16_t g_values[] = { 1000,1500,2000,2500,3000,3500,4000,4500 };
static uint32_t g_nValues = sizeof(g_values) / sizeof(g_values[0]);

// TTI counters
static uint32_t g_currentTti = 0;
static uint32_t g_totalTtis = 0;

// Fonctions qui s'execute à chaque TTI
void RunPerTti()
{
    std::cout << "\n=== TTI " << g_currentTti
        << " (sim time " << Simulator::Now().GetMilliSeconds() << " ms) ===\n";



    // 1) generate les demandes des UEs
    std::vector<uint32_t> UE_demand(g_nUes, 0);
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        double raw = g_randVar->GetValue(); // in [0, nValues-1]
        uint32_t idx = static_cast<uint32_t>(std::floor(raw));
        if (idx >= g_nValues) idx = g_nValues - 1;
        UE_demand[i] = g_values[idx];
    }

    // 2) Calcul du  SINR, MCS de chaque UE, pour ensuite les inclure et les convertir en sous canaux
    g_UEs.clear();
    g_UEs.reserve(g_nUes);
    std::vector<uint32_t> demandRBs(g_nUes, 0);

    double leoAltM = g_leoAltitude * 1000.0;
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        double omega = 2.0 * M_PI / (90.0 * 60.0); // rad/s
        double theta_sat = omega * g_currentTti * (g_TTI_ms / 1000.0);
        double theta_user = i * g_deltaTheta;
        double dist = sqrt(g_Re * g_Re + (g_Re + leoAltM) * (g_Re + leoAltM)
            - 2.0 * g_Re * (g_Re + leoAltM) * cos(theta_sat - theta_user));

        double sinr = ComputeSinr(dist);
        MCS_Info mcs = GetMcs(sinr);

        UE ue;
        ue.id = i;
        ue.distance = dist;
        ue.sinr = sinr;
        ue.mcs = mcs.MCS;
        ue.SE = mcs.SE;
        ue.allocRBs = 0;
        ue.throughput = 0.0;
        ue.demand = static_cast<double>(UE_demand[i]);

        g_UEs.push_back(ue);

        double denom = ue.SE * g_RB_BW_Hz * 1e-3; // kbps per RB approx
        uint32_t rb = 0;
        if (denom > 0.0) rb = static_cast<uint32_t>(std::ceil(ue.demand / denom));
        demandRBs[i] = rb;

        std::cout << "UE " << i << " SE=" << ue.SE << " demand_kbps=" << UE_demand[i] << " demand_RBs=" << rb << "\n";
    }

    // ---------------- HTBRA scheduler (grouping + Bankruptcy + intra allocation) ----------------
    // Group UEs into low / mid / high classes using med_low / med_high (based on demandRBs)
    int maxVal = 0, minVal = 0;
    if (!demandRBs.empty())
    {
        maxVal = *std::max_element(demandRBs.begin(), demandRBs.end());
        minVal = *std::min_element(demandRBs.begin(), demandRBs.end());
    }
    uint32_t med_low = static_cast<uint32_t>(std::ceil(maxVal / 4.0));
    uint32_t med_high = static_cast<uint32_t>(std::ceil(3.0 * maxVal / 4.0));

    std::vector<uint32_t> lowDemand, midDemand, highDemand;
    std::vector<uint32_t> lowIdx, midIdx, highIdx;

    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        if (demandRBs[i] > 0 && demandRBs[i] <= med_low)
        {
            lowDemand.push_back(demandRBs[i]);
            lowIdx.push_back(i);
        }
        else if (demandRBs[i] > med_low && demandRBs[i] <= med_high)
        {
            midDemand.push_back(demandRBs[i]);
            midIdx.push_back(i);
        }
        else if (demandRBs[i] > med_high)
        {
            highDemand.push_back(demandRBs[i]);
            highIdx.push_back(i);
        }
    }

    int sum1 = std::accumulate(lowDemand.begin(), lowDemand.end(), 0);
    int sum2 = std::accumulate(midDemand.begin(), midDemand.end(), 0);
    int sum3 = std::accumulate(highDemand.begin(), highDemand.end(), 0);

    uint32_t group1_req = static_cast<uint32_t>(sum1 * 1.0);
    uint32_t group2_req = static_cast<uint32_t>(sum2 * 1.25);
    uint32_t group3_req = static_cast<uint32_t>(sum3 * 1.5);

    std::vector<uint32_t> tableDemand = { group1_req, group2_req, group3_req };

    uint32_t cp = g_totalChannels;
    std::vector<uint32_t> interAlloc = Bankruptcy(tableDemand, cp);
    uint32_t group1_alloc = interAlloc.size() > 0 ? interAlloc[0] : 0;
    uint32_t group2_alloc = interAlloc.size() > 1 ? interAlloc[1] : 0;
    uint32_t group3_alloc = interAlloc.size() > 2 ? interAlloc[2] : 0;

    std::cout << "group reqs: " << group1_req << "," << group2_req << "," << group3_req << "\n";
    std::cout << "group allocs: " << group1_alloc << "," << group2_alloc << "," << group3_alloc << "\n";

    // Intra allocations 
    std::vector<uint32_t> lowAlloc = IntraBinAllocate2(lowDemand, group1_alloc);
    std::vector<uint32_t> midAlloc = IntraBinAllocate2(midDemand, group2_alloc);
    std::vector<uint32_t> highAlloc = IntraBinAllocate2(highDemand, group3_alloc);

    // Map allocations back to original UE indices -> shares per UE
    std::vector<uint32_t> shares(g_nUes, 0);
    for (size_t j = 0; j < lowIdx.size(); ++j) if (j < lowAlloc.size()) shares[lowIdx[j]] = lowAlloc[j];
    for (size_t j = 0; j < midIdx.size(); ++j) if (j < midAlloc.size()) shares[midIdx[j]] = midAlloc[j];
    for (size_t j = 0; j < highIdx.size(); ++j) if (j < highAlloc.size()) shares[highIdx[j]] = highAlloc[j];

    // Debug print shares
    std::cout << "[INFO] Shares per UE: ";
    for (uint32_t i = 0; i < g_nUes; ++i) std::cout << shares[i] << (i + 1 < g_nUes ? "," : "\n");

    // OnOff apps pour le traffic
    // Start at Now(), stop after TTI length. Apps are short-lived per-TTI to avoid port binding conflicts.
    Time start = Simulator::Now();
    Time stop = start + MilliSeconds(static_cast<int>(g_TTI_ms));

    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        if (shares[i] == 0) continue;
        double rateMbps = 5.0 * static_cast<double>(shares[i]); // mapping used earlier
        OnOffHelper onoff("ns3::UdpSocketFactory",
            Address(InetSocketAddress(g_interfaces[i].GetAddress(1), g_dlPort + i)));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(rateMbps) + "Mbps")));
        onoff.SetAttribute("PacketSize", UintegerValue(512));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

        ApplicationContainer apps = onoff.Install(g_nodes.Get(2 * i)); // gNB transmitter is nodes[2*i]
        apps.Start(start);
        apps.Stop(stop);
    }

    // ---------------- Calcul de satisfaction + metrics ----------------
    uint32_t satisf = 0;
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        if (demandRBs[i] <= shares[i]) satisf++;
        std::cout << "UE " << i << " demandRBs=" << demandRBs[i]
            << " alloc=" << shares[i]
                << " -> " << (shares[i] >= demandRBs[i] ? "ok" : "no") << "\n";
    }
    std::cout << "Satisfaction = " << satisf << " / " << g_nUes << "\n";

    // Build throughput vector (kbps) for metrics (approx)
    std::vector<double> thruVec(g_nUes, 0.0);
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        double RB_kbps = g_UEs[i].SE * g_RB_BW_Hz * 1e-3;
        double throughput = static_cast<double>(shares[i]) * RB_kbps;
        thruVec[i] = throughput;
    }
    std::cout << "[HTBRA] JainFairness=" << JainFairness_d(thruVec)
        << " Efficiency=" << Efficiency(thruVec, demandRBs) << "\n";



    // Schedule next TTI
    g_currentTti++;
    if (g_currentTti < g_totalTtis)
    {
        Simulator::Schedule(MilliSeconds(static_cast<int>(g_TTI_ms)), &RunPerTti);
    }
    else
    {
        std::cout << "\n--- All TTIs processed (" << g_currentTti << ") ---\n";
    }
}


int main(int argc, char* argv[])
{
    // Command-line params
    CommandLine cmd;
    cmd.AddValue("nUes", "Number of UEs", g_nUes);
    cmd.AddValue("scheduler", "Scheduler type: PF, HTBRA", g_scheduler);
    cmd.AddValue("simTime", "Simulation time in seconds", g_simTime);
    cmd.AddValue("leoAltitude", "LEO altitude (km)", g_leoAltitude);
    cmd.AddValue("subcarrierSpacing", "subcarrier spacing (kHz)", g_subcarrierSpacing);
    cmd.AddValue("channelBandwidth", "Channel bandwidth (Hz)", g_channelBandwidth);
    cmd.AddValue("TTI_ms", "TTI duration in ms", g_TTI_ms);
    cmd.Parse(argc, argv);

    // Derived params
    g_totalChannels = static_cast<uint32_t>(g_channelBandwidth / (g_subcarrierSpacing * 1000.0));
    if (g_totalChannels == 0)
    {
        std::cerr << "Error: totalChannels computed as 0. Check channelBandwidth / subcarrierSpacing\n";
        return 1;
    }
    g_RB_BW_Hz = g_channelBandwidth / static_cast<double>(g_totalChannels);
    g_totalTtis = static_cast<uint32_t>(std::max(1.0, g_simTime * 1000.0 / g_TTI_ms));

    std::cout << "Total subcarriers = " << g_totalChannels << ", RB_BW_Hz=" << g_RB_BW_Hz
        << ", totalTTIs=" << g_totalTtis << "\n";

    // Node & network setup
    g_nodes.Create(2 * g_nUes); // gNB + UE per pair
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(g_nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    double speedOfLight = 3e8;
    double delaySeconds = (g_leoAltitude * 1000.0) / speedOfLight;
    std::ostringstream delayStr;
    delayStr << delaySeconds << "s";
    p2p.SetChannelAttribute("Delay", StringValue(delayStr.str()));

    InternetStackHelper stack;
    stack.Install(g_nodes);

    // IPv4 interfaces
    g_interfaces.clear();
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        NetDeviceContainer devices = p2p.Install(g_nodes.Get(2 * i), g_nodes.Get(2 * i + 1));
        std::string subnet = "10." + std::to_string((i / 256) + 1) + "." + std::to_string(i % 256) + ".0";
        Ipv4AddressHelper ipv4;
        ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(devices);
        g_interfaces.push_back(ifc);
    }

    // Install PacketSink on UE (receivers) once (prevents bind conflict)
    for (uint32_t i = 0; i < g_nUes; ++i)
    {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
            Address(InetSocketAddress(Ipv4Address::GetAny(), g_dlPort + i)));
        sink.Install(g_nodes.Get(2 * i + 1));
    }

    // RNG: single instance
    g_randVar = CreateObject<UniformRandomVariable>();
    g_randVar->SetAttribute("Min", DoubleValue(0.0));
    g_randVar->SetAttribute("Max", DoubleValue(static_cast<double>(g_nValues) - 1.0));



    // Flow monitor 
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();

    // Schedule the TTI loop: first call at time 0
    g_currentTti = 0;
    Simulator::Schedule(MilliSeconds(0), &RunPerTti);

    // Stop after all TTIs + small margin
    double stopMs = static_cast<double>(g_totalTtis) * g_TTI_ms + 50.0;
    Simulator::Stop(MilliSeconds(static_cast<int>(std::ceil(stopMs))));
    Simulator::Run();

    // Print flow monitor summary
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    std::cout << "\n=== Flow Monitor Summary ===\n";
    for (auto& kv : stats)
    {
        FlowId id = kv.first;
        const FlowMonitor::FlowStats& f = kv.second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(id);
        /*std::cout << "Flow " << id << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")"
            << " TxPkts=" << f.txPackets << " RxPkts=" << f.rxPackets
            << " Throughput=" << (f.rxBytes * 8.0 / g_simTime / 1e3) << " kbps\n";*/
    }

    Simulator::Destroy();
    std::cout << "Simulation finished cleanly.\n";
    return 0;
}
