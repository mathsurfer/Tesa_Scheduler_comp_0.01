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

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("SchedulerThroughputComparison");

// ===  Structure de données pour définir un utilisateur  ===
struct UE
{
    uint32_t id;
    double distance;
    double sinr;
    uint32_t mcs;
    double SE;      // spectral efficiency (bits/s/Hz)
    uint32_t allocRBs;
    double throughput; // kbps 
    double demand;     // kbps
};

//Infos MCS
struct MCS_Info { uint32_t MCS; double SE; };




//Mapping SINR <---> MCS
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

// SINR calculation (toy free-space model)
double ComputeSinr(double distance)
{
    double f = 2.4e9;           // Hz
    double c = 3e8;             // m/s
    double P_tx = 1000.0;       // W (toy)
    double B = 180000.0;        // Hz per RB reference
    double k = 1.38e-23;
    double T = 290;

    double FSPL = pow((4.0 * M_PI * distance * f / c), 2.0);
    double P_rx = P_tx / (FSPL + 1e-30);
    double N = k * T * B;
    double sinr = P_rx / (N + 1e-30);
    return 10.0 * log10(sinr + 1e-12);
}

// Standards de Perfs
double JainFairness_d(const std::vector<double>& values) {
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double sumSq = 0.0;
    for (auto v : values) sumSq += v * v;
    if (values.empty() || sumSq == 0.0) return 1.0;
    return (sum * sum) / (values.size() * sumSq + 1e-9);
}

double Efficiency(const std::vector<double>& throughput, const std::vector<uint32_t>& demand) {
    double sumTh = std::accumulate(throughput.begin(), throughput.end(), 0.0);
    double sumDem = std::accumulate(demand.begin(), demand.end(), 0.0);
    if (sumDem == 0.0) return 1.0;
    return sumTh / (sumDem + 1e-9);
}

double PFUtility(const std::vector<double>& throughput) {
    double sum = 0.0;
    for (auto t : throughput)
        if (t > 0) sum += std::log(t);
    return sum;
}

// Paramètres globaux
static uint32_t g_nUes = 5;
static std::string g_scheduler = "PF";
static double g_simTimeSec = 1.0;       // seconds of simulation
static double g_leoAltitudeKm = 1200.0;  // km
static double g_subcarrierSpacingKHz = 120.0;
static double g_channelBandwidthHz = 50e6;
static double g_TTI_ms = 1.0;           // TTI length in ms
static uint16_t g_dlPort = 4000;
static uint32_t g_totalChannels = 0;
static double g_RB_BW_Hz = 0.0;
static double g_Re = 6371e3;            // Earth radius (m)
static double g_deltaTheta = 5.0 * M_PI / 180.0;

static Ptr<UniformRandomVariable> g_randVar = nullptr;

// dynamic state: ajout d'un historique
static std::vector<uint32_t> g_PF_Hist; // per-UE historical throughput metric
static NodeContainer g_nodes;
static std::vector<Ipv4InterfaceContainer> g_interfaces;

// storage for UEs (réinitialisation à chaque TTI)
static std::vector<UE> g_UEs;

// valeurs de débit demandées(en kbps)
static uint16_t g_valuesArr[] = { 1000,1500,2000,2500,3000,3500,4000,4500 };
static uint32_t g_nValuesArr = sizeof(g_valuesArr) / sizeof(g_valuesArr[0]);

// TTI counters
static uint32_t g_currentTti = 0;
static uint32_t g_totalTtis = 0;

// Faire tourner des fonctions à chaque TTI (respect de la continuité temporelle de la simulation)
void RunPerTti()
{
    // Print header
    std::cout << "\n=== TTI " << g_currentTti
        << " | sim time: " << Simulator::Now().GetMilliSeconds() << " ms ===\n";

    // Check la longeur de g_PF_HIST
    if (g_PF_Hist.size() != g_nUes) g_PF_Hist.assign(g_nUes, 0);

    // 1) Generer les valeurs de débits demandés de manière aléatoire
    std::vector<uint32_t> UE_demand(g_nUes, 0);
    for (uint32_t i = 0; i < g_nUes; ++i) {
        double raw = g_randVar->GetValue(); // in [0, nValuesArr-1]
        uint32_t idx = static_cast<uint32_t>(std::floor(raw));
        if (idx >= g_nValuesArr) idx = g_nValuesArr - 1;
        UE_demand[i] = g_valuesArr[idx];
    }

    // 2) Pour chaque UE: calcul de  SINR, MCS,... avant la conversion en nombre de sous-canaux
    g_UEs.clear();
    g_UEs.reserve(g_nUes);
    std::vector<uint32_t> demandRBs(g_nUes, 0);

    for (uint32_t i = 0; i < g_nUes; ++i)
    {   //Calcul de distance UE-LEO en tenant compte des angles et déviations
        double omega = 2.0 * M_PI / (90.0 * 60.0); // rad/s (~90 min orbit)
        double theta_sat = omega * g_currentTti * (g_TTI_ms / 1000.0); // rough angle progression per TTI
        double theta_user = i * g_deltaTheta;
        double leoAltM = g_leoAltitudeKm * 1000.0;
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
        ue.demand = static_cast<double>(UE_demand[i]); // kbps

        g_UEs.push_back(ue);

        // demand in RBs = ceil( demand_kbps / (SE * RB_BW_Hz * 1e-3) )
        double denom = ue.SE * g_RB_BW_Hz * 1e-3; // kbps par sous canal
        uint32_t rb = 0;
        if (denom > 0.0) rb = static_cast<uint32_t>(std::ceil(ue.demand / denom));
        demandRBs[i] = rb;
    }

    // Debugging
    for (uint32_t i = 0; i < g_nUes; ++i) {
        std::cout << "UE " << i << " SE=" << std::fixed << std::setprecision(3) << g_UEs[i].SE
            << " MCS=" << g_UEs[i].mcs
            << " demand_kbps=" << g_UEs[i].demand
            << " demand_RBs=" << demandRBs[i] << "\n";
    }

    // --- Scheduler: PF (Proportional Fair) ---

    std::vector<double> channelQuality(g_nUes, 0.0);
    std::vector<double> avgThroughput(g_nUes, 0.0);
    std::vector<double> pfMetric(g_nUes, 0.0);
    double totalMetric = 0.0;

    for (uint32_t i = 0; i < g_nUes; ++i) {
        channelQuality[i] = g_UEs[i].SE; // use SE as channel quality
        if (g_PF_Hist[i] > 0.0) avgThroughput[i] = static_cast<double>(g_PF_Hist[i]);
        else avgThroughput[i] = g_UEs[i].SE * g_RB_BW_Hz * 1e-6; // Eviter d'utiliser des valeurs nulles
    }

    for (uint32_t i = 0; i < g_nUes; ++i) {
        pfMetric[i] = channelQuality[i] / (avgThroughput[i] + 1e-9);
        totalMetric += pfMetric[i];
    }

    // le poids prendra en compte le débit demandé ET le SE(qui est lui même fonction d'MCS)
    std::vector<double> weights(g_nUes, 1.0);
    double totalWeight = 0.0;
    for (uint32_t i = 0; i < g_nUes; ++i) {
        weights[i] = 0.5 * static_cast<double>(demandRBs[i]) + 0.5 * pfMetric[i];
        totalWeight += weights[i];
    }


    double totalCapacityMbps = static_cast<double>(g_totalChannels) * 5.0;

    // Allocation proportionnelle
    std::vector<uint32_t> allocatedRBs(g_nUes, 0);
    for (uint32_t i = 0; i < g_nUes; ++i) {
        double proportion = (totalMetric > 0.0) ? (pfMetric[i] / totalMetric) : (1.0 / g_nUes);
        double rateMbps = proportion * totalCapacityMbps;
        uint32_t alloc = static_cast<uint32_t>(std::round(rateMbps / 5.0)); // Recoversion
        // Eviter l'overflow
        alloc = std::min(alloc, demandRBs[i]);
        allocatedRBs[i] = alloc;
        g_UEs[i].allocRBs = alloc;
        // approximate throughput (kbps) = alloc * RB_kbps where RB_kbps = SE * RB_BW_Hz * 1e-3
        double RB_kbps = g_UEs[i].SE * g_RB_BW_Hz * 1e-3;
        g_UEs[i].throughput = static_cast<double>(alloc) * RB_kbps;
    }

    // Stocker les valeur à une TTI donnée
    std::vector<double> thruVec;
    for (uint32_t i = 0; i < g_nUes; ++i) thruVec.push_back(g_UEs[i].throughput);

    // Loggings
    uint32_t satisfiedCount = 0;
    for (uint32_t i = 0; i < g_nUes; ++i) {
        bool ok = (allocatedRBs[i] >= demandRBs[i]);
        if (ok) satisfiedCount++;
        std::cout << "[Alloc] UE " << i << " demandRB=" << demandRBs[i]
            << " allocRB=" << allocatedRBs[i] << " -> " << (ok ? "ok" : "no") << "\n";
    }

    double jain = JainFairness_d(thruVec);
    double eff = Efficiency(thruVec, demandRBs);
    double util = PFUtility(thruVec);

    std::cout << std::fixed << std::setprecision(4)
        << "[PF] Satisfied Users: " << satisfiedCount << " / " << g_nUes << "\n"
        << "[PF] JainFairness: " << jain << "\n"
        << "[PF] Efficiency: " << eff << "\n"
        << "[PF] Utility: " << util << "\n";

    // Update PF history 
    for (uint32_t i = 0; i < g_nUes; ++i) {
        double alpha = 0.1;
        double currentRateMbps = (g_UEs[i].throughput) * 1e-3; // kbps -> Mbps
        double prev = g_PF_Hist[i];
        double updated = (1.0 - alpha) * prev + alpha * currentRateMbps;
        g_PF_Hist[i] = static_cast<uint32_t>(updated + 0.5); // approximation utilisée
    }


    //Mise en place du traffic OnOff :
    Time start = Simulator::Now();
    Time stop = start + MilliSeconds(static_cast<int>(g_TTI_ms));

    for (uint32_t i = 0; i < g_nUes; ++i) {
        if (allocatedRBs[i] == 0) continue; // skip zero allocations

        double rateMbps = 5.0 * static_cast<double>(allocatedRBs[i]); // same Mapping as earlier
        OnOffHelper onoff("ns3::UdpSocketFactory",
            Address(InetSocketAddress(g_interfaces[i].GetAddress(1), g_dlPort + i)));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(rateMbps) + "Mbps")));
        onoff.SetAttribute("PacketSize", UintegerValue(1500));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        // Install app on transmitter node (nodes: pairs [gNb, ue])
        ApplicationContainer apps = onoff.Install(g_nodes.Get(2 * i));
        apps.Start(start);
        apps.Stop(stop);
    }

    // Increment TTI counter & schedule next TTI
    g_currentTti++;
    if (g_currentTti < g_totalTtis) {
        Simulator::Schedule(MilliSeconds(static_cast<int>(g_TTI_ms)), &RunPerTti);
    }
    else {
        // last TTI done; 
        std::cout << "\n--- Finished all TTIs (count=" << g_currentTti << ") ---\n";
    }
}


int main(int argc, char* argv[])
{
    // Default values (can be overridden via command line)
    CommandLine cmd;
    cmd.AddValue("nUes", "Number of UEs (pairs)", g_nUes);
    cmd.AddValue("scheduler", "Scheduler type (PF currently)", g_scheduler);
    cmd.AddValue("simTime", "Simulation time in seconds", g_simTimeSec);
    cmd.AddValue("leoAltitude", "LEO altitude in km", g_leoAltitudeKm);
    cmd.AddValue("subcarrierSpacing", "Subcarrier spacing in kHz", g_subcarrierSpacingKHz);
    cmd.AddValue("channelBandwidth", "Channel bandwidth in Hz", g_channelBandwidthHz);
    cmd.AddValue("TTI_ms", "TTI duration in milliseconds", g_TTI_ms);
    cmd.Parse(argc, argv);

    // Derived params
    g_totalChannels = static_cast<uint32_t>(g_channelBandwidthHz / (g_subcarrierSpacingKHz * 1000.0));
    if (g_totalChannels == 0) {
        std::cerr << "Fatal: totalChannels computed as 0. Check channelBandwidth / subcarrierSpacing.\n";
        return 1;
    }
    g_RB_BW_Hz = g_channelBandwidthHz / static_cast<double>(g_totalChannels);

    g_totalTtis = static_cast<uint32_t>(std::max(1.0, g_simTimeSec * 1000.0 / g_TTI_ms));
    g_PF_Hist.assign(g_nUes, 0);

    std::cout << "Total subcarriers = " << g_totalChannels
        << ", RB_BW_Hz=" << g_RB_BW_Hz
        << ", totalTTIs=" << g_totalTtis << "\n";

    // --- Node & network setup ---
    g_nodes.Create(2 * g_nUes);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(g_nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    std::ostringstream delayStr;
    double speedOfLight = 3e8;
    double delaySeconds = (g_leoAltitudeKm * 1000.0) / speedOfLight;
    delayStr << delaySeconds << "s";
    p2p.SetChannelAttribute("Delay", StringValue(delayStr.str()));

    InternetStackHelper stack;
    stack.Install(g_nodes);

    // IPv4: create interfaces
    g_interfaces.clear();
    for (uint32_t i = 0; i < g_nUes; ++i) {
        NetDeviceContainer devices = p2p.Install(g_nodes.Get(2 * i), g_nodes.Get(2 * i + 1));
        std::string subnet = "10." + std::to_string((i / 256) + 1) + "." + std::to_string(i % 256) + ".0";
        Ipv4AddressHelper ipv4;
        ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");
        Ipv4InterfaceContainer ifc = ipv4.Assign(devices);
        g_interfaces.push_back(ifc);
    }

    // Install PacketSink on each UE (receiver) 
    for (uint32_t i = 0; i < g_nUes; ++i) {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
            Address(InetSocketAddress(Ipv4Address::GetAny(), g_dlPort + i)));
        sink.Install(g_nodes.Get(2 * i + 1));
    }

    // Create RNG 
    g_randVar = CreateObject<UniformRandomVariable>();
    g_randVar->SetAttribute("Min", DoubleValue(0.0));
    g_randVar->SetAttribute("Max", DoubleValue(static_cast<double>(g_nValuesArr) - 1.0));

    // Initialize PF history vector (store as integers representing Mbps*1)
    g_PF_Hist.assign(g_nUes, 0);

    // Flow monitor helpful for end-of-sim stats
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();

    // Schedule first TTI at time 0
    g_currentTti = 0;
    Simulator::Schedule(MilliSeconds(0), &RunPerTti);

    // Stop time after all TTIs (add small margin)
    double simStopMs = static_cast<double>(g_totalTtis) * g_TTI_ms + 10.0;
    Simulator::Stop(MilliSeconds(static_cast<int>(std::ceil(simStopMs))));
    Simulator::Run();

    // Flow monitor printing (basic)
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    std::cout << "\n=== Flow Monitor Summary ===\n";
    for (auto& kv : stats) {
        FlowId id = kv.first;
        const FlowMonitor::FlowStats& f = kv.second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(id);
        /*std::cout << "Flow " << id << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")"
            << " TxPkts=" << f.txPackets << " RxPkts=" << f.rxPackets
            << " Throughput=" << (f.rxBytes * 8.0 / g_simTimeSec / 1e3) << " kbps\n";*/
    }

    Simulator::Destroy();
    std::cout << "\nSimulation finished cleanly.\n";
    return 0;
}
