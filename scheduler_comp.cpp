#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SchedulerThroughputComparison");

int main (int argc, char *argv[])
{
  // --- Simulation Setup ---
  uint32_t nUes = 300; //Nombre d'utilisateurs
  std::string scheduler = "PF"; //  PF,HTBRA
  double simTime = 5.0; //Temps de simulation en s

  // --- P1 LEO Parameters used for our research---
  double leoAltitude = 1200.0;            // km
  double leoInclination = 50.0;           // degrees
  double subcarrierSpacing = 120.0;        // en kHz
  bool splitOption2 = true;               // Enable CU-DU split
  double channelBandwidth = 50e6;         // en Hz
  std::string channelModel = "AWGN";     // Gaussian






  //
  CommandLine cmd;
  cmd.AddValue ("nUes", "Number of UEs", nUes);
  cmd.AddValue ("scheduler", "Scheduler type: PF, HTBRA", scheduler);
  cmd.AddValue ("simTime", "Simulation time", simTime);
  cmd.AddValue ("leoAltitude", "LEO altitude (km)", leoAltitude);
  cmd.AddValue ("leoInclination", "LEO inclination (degrees)", leoInclination);
  cmd.AddValue ("subcarrierSpacing", "Subcarrier spacing (kHz)", subcarrierSpacing);
  cmd.AddValue ("splitOption2", "Enable CU-DU split", splitOption2);
  cmd.AddValue ("channelBandwidth", "Channel bandwidth (Hz)", channelBandwidth);
  cmd.AddValue ("channelModel", "Channel model (Friis, Rician)", channelModel);
  cmd.Parse (argc, argv);

  // --- Derived Parameters ---
  double speedOfLight = 3e8;
  double delaySeconds = (leoAltitude * 1000.0) / speedOfLight;
  std::ostringstream delayStr;
  delayStr << delaySeconds << "s";
  if (splitOption2) delayStr.str("0.05s");



  //Compute the number of available channels
  uint32_t totalChannels = channelBandwidth / (subcarrierSpacing * 1000.0);
  NS_LOG_UNCOND ("Total subcarriers = " << totalChannels << ", Delay = " << delayStr.str());

  // --- Node Setup ---
  NodeContainer nodes;
  nodes.Create (2 * nUes); // gNB-UE pairs




// Mobility model for which the current position does not change once it has been set and until it is set again explicitely
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);




  // P2P connexion
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue (delayStr.str()));

  InternetStackHelper stack;
  stack.Install (nodes);

  //Ipv4AddressHelper ipv4; generate IP adresses
  std::vector<Ipv4InterfaceContainer> interfaces;

  for (uint32_t i = 0; i < nUes; ++i)
  {
      NetDeviceContainer devices = p2p.Install(nodes.Get(2 * i), nodes.Get(2 * i + 1));

      std::string subnet = "10." + std::to_string((i / 256) + 1) + "." + std::to_string(i % 256) + ".0";

      Ipv4AddressHelper ipv4; 
      ipv4.SetBase(Ipv4Address(subnet.c_str()), "255.255.255.0");

      interfaces.push_back(ipv4.Assign(devices));
  }

  

  // Create a random variable to simulate the users demands
  Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
  // Counters
  int count1 = 0, count2 = 0, count3 = 0;
  double p1 = 0.33;
  double p2 = 0.33;
  double p3 = 0.33;
  std::vector<uint32_t> demand(nUes);
  for (uint32_t i = 0; i < nUes; ++i)
  {
   

      demand[i] = (i % 3) + 1;


      
  }



  //Counter of satisfied users per class
  uint32_t sat1 = 0;
  uint32_t sat2 = 0;
  uint32_t sat3 = 0;
  




  std::cout << "Count of 1s: " << count1 << std::endl;
  std::cout << "Count of 2s: " << count2 << std::endl;
  std::cout << "Count of 3s: " << count3 << std::endl;

  uint16_t dlPort = 4000;

  // ---------------- Scheduler Logic ----------------
  

   if (scheduler == "PF")
  {
      std::vector<double> weights(nUes);
      double totalWeight = 0.0;



      //Metrics
      std::vector<double> channelQuality(nUes);
      std::vector<double> avgThroughput(nUes);
      std::vector<double> pfMetric(nUes);
      double totalMetric = 0.0;


      for (uint32_t i = 0; i < nUes; ++i)
      {
          // Random "instantaneous" channel quality (1–5)
          channelQuality[i] = 1.0 + static_cast<double>(rand() % 5);

          // Random "past average throughput" (artificial history)
          avgThroughput[i] = 0.5 + static_cast<double>(rand() % 5);
      }


      for (uint32_t i = 0; i < nUes; ++i)
      {
          // PF metric formule = cq / past throughput
          pfMetric[i] = channelQuality[i] / (avgThroughput[i]+ 1e-6);
          totalMetric += pfMetric[i];
      }



      //  Assign PF weights (here based on demand AND previous percieved throughput)
      for (uint32_t i = 0; i < nUes; ++i)
      {
          
          weights[i] = 0.5 * demand[i] + 0.5 * pfMetric[i];

          totalWeight += weights[i];
      }

      //Calculate Total available capacity
      double totalCapacityMbps = totalChannels * 5.0;

      // Install OnOff apps with globally scaled PF rate:Generate traffic to a single destination according to an OnOff pattern.
      for (uint32_t i = 0; i < nUes; ++i)
      {
          double rateMbps = (weights[i] / totalWeight) * totalCapacityMbps;

          OnOffHelper onoff("ns3::UdpSocketFactory",
              Address(InetSocketAddress(interfaces[i].GetAddress(1), dlPort + i)));

          onoff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(rateMbps) + "Mbps")));
          onoff.SetAttribute("PacketSize", UintegerValue(1500)); // Optional: fewer packets for faster transmission
          onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.4]"));
          onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.6]"));
          onoff.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
          onoff.SetAttribute("StopTime", TimeValue(Seconds(simTime)));

          onoff.Install(nodes.Get(2 * i));
      }



      

     
      // Count satisfied users
      uint32_t satisfied = 0;
      uint32_t totalWeightleft = totalWeight;
      uint32_t alloincre = 0; //Total assigned resources

      

      for (uint32_t i = 0; i < nUes; ++i)
      {
          // Convert back:subchannels ---> Débit
         

          double rateMbps = (pfMetric[i] / totalMetric) * totalCapacityMbps;

          uint32_t allocatedChannels = std::round(rateMbps / 5.0);
          allocatedChannels = std::min(allocatedChannels, (uint32_t)demand[i]);

          
           //Count the satisfied users
          if (allocatedChannels >= demand[i])
          {
              satisfied++;
              totalWeightleft = totalWeightleft -demand[i] ;
              alloincre = alloincre + allocatedChannels;



              //Count the satisfied users per class
              if (demand[i]==1) { sat1 += 1; }
              if (demand[i] == 2) { sat2 += 1; }
              if (demand[i] == 3) { sat3 += 1; }




          }

          // Optional: print per-user allocation
          std::cout << "UE " << i << " 's Demand: " << demand[i]
              << ", Allocated Channels: " << allocatedChannels
              << " -> " << (allocatedChannels >= demand[i] ? "ok" : "no") << std::endl;


          //Useful for PF historical calculation
          double alpha = 0.1; // smoothing factor
          double currentRate = rateMbps; // instantaneous rate in Mbps
          avgThroughput[i] = (1 - alpha) * avgThroughput[i] + alpha * currentRate;
      }


      std::cout << "totalChannels " << totalChannels << std::endl;
      std::cout << "totalGroupDemand " << totalWeight << std::endl;




      std::cout << "satisfied in low Qos " << (double)sat1 / (satisfied) * 100 << " % " << std::endl;
      std::cout << "satisfied in medium Qos " << (double)sat2 / (satisfied) * 100 << " % " << std::endl;
      std::cout << "satisfied in high Qos " << (double)sat3 / (satisfied) * 100 << " % " << std::endl;




     

      double satisfactionRatio = (double)satisfied / nUes * 100.0;

      std::cout << "\n[PF] Satisfied Users: " << satisfied
          << " / " << nUes
          << " (" << satisfactionRatio << "%)\n";


  }


   // Our HTBRA Approach

  else if (scheduler == "HTBRA")
  {
 
      std::vector<uint32_t> shares(nUes, 0); // Stores final allocated channels
      std::vector<uint32_t> lowDemand, midDemand, highDemand;

      // Step 1: Group UEs by demand class
      for (uint32_t i = 0; i < nUes; ++i)
      {
          if (demand[i] == 1) lowDemand.push_back(i);
          else if (demand[i] == 2) midDemand.push_back(i);
          else highDemand.push_back(i);
      }

      // Step 2: Calculate group demands
      uint32_t group1_req = lowDemand.size() * 1;
      uint32_t group2_req = midDemand.size() * 2;
      uint32_t group3_req = highDemand.size() * 3;

      uint32_t totalGroupDemand = group1_req + group2_req + group3_req;
      double budgetRatio = std::min(1.0, (double)totalChannels / totalGroupDemand);

      // Step 3: Allocation Intra: Allocate group-wise using a fair loop 
      uint32_t group1_alloc = 0, group2_alloc = 0, group3_alloc = 0;
      uint32_t cp = totalChannels;

      while (cp > 0)
      {
          if (cp >= 3 && group3_alloc < group3_req)
          {
              group3_alloc += 3;
              cp -= 3;
          }
          else if (cp >= 2 && group2_alloc < group2_req)
          {
              group2_alloc += 2;
              cp -= 2;
          }
          else if (cp >= 1 && group1_alloc < group1_req)
          {
              group1_alloc += 1;
              cp -= 1;
          }
          else break;
      }

      // Step 4: Allocation Intra :Allocate within each group and install apps
      uint32_t c1 = group1_alloc;
      for (uint32_t idx : lowDemand)
      {
          if (c1 > 0)
          {
              shares[idx] = 1;



              //Count the satisfied users per class(1)
               sat1 += 1; 
             


              c1--;
          }
      }

      uint32_t c2 = group2_alloc;
      for (uint32_t idx : midDemand)
      {
          if (c2 >= 2)
          {
              shares[idx] = 2;


              //Count the satisfied users per class(2)
              sat2 += 1; 
             


              c2 -= 2;
          }
      }

      uint32_t c3 = group3_alloc;
      for (uint32_t idx : highDemand)
      {
          if (c3 >= 3)
          {
              shares[idx] = 3;


              //Count the satisfied users per class(3)
             
              sat3 += 1; 

              c3 -= 3;
          }
      }

      // Step 5: Install OnOff apps based on share allocation
      for (uint32_t i = 0; i < nUes; ++i)
      {
          if (shares[i] > 0)
          {
              double rateMbps = 5.0 * shares[i];

              OnOffHelper onoff("ns3::UdpSocketFactory",
                  Address(InetSocketAddress(interfaces[i].GetAddress(1), dlPort + i)));

              onoff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(rateMbps) + "Mbps")));
              onoff.SetAttribute("PacketSize", UintegerValue(512));
              onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
              onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
              onoff.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
              onoff.SetAttribute("StopTime", TimeValue(Seconds(simTime)));

              onoff.Install(nodes.Get(2 * i));
          }
      }

      // Step 6: Display the shares
    
      std::cout << "group1_alloc " << group1_alloc << std::endl;
      std::cout << "group2_alloc " << group2_alloc << std::endl;
      std::cout << "group3_alloc " << group3_alloc << std::endl;
      std::cout << "totalChannels " << totalChannels << std::endl;
      std::cout << "totalGroupDemand " << totalGroupDemand << std::endl;

      uint32_t satisf = 0;

       std::cout << "\n[INFO] Shares allocated per UE:" << std::endl;
      for (uint32_t i = 0; i < nUes; ++i)
      {
          if (demand[i]<=shares[i]) { satisf++; }

          // Optional: print per-user allocation
          std::cout << "UE " << i << " 's Demand: " << demand[i]
              << ", Allocated Channels: " << shares[i]
              << " -> " << (shares[i] <= demand[i] ? "ok" : "no") << std::endl;
          //std::cout << "UE " << i << " → Demand: " << demand[i]
              //<< ", Allocated: " << shares[i] << std::endl;


      }

      std::cout << "Satisfaction= " << satisf << " / " << nUes << std::endl;
      


      std::cout << "satisfied in low Qos " << (double)sat1 / (satisf) * 100 << " % " << std::endl;
      std::cout << "satisfied in medium Qos " << (double)sat2 / (satisf) * 100 << " % " << std::endl;
      std::cout << "satisfied in high Qos " << (double)sat3 / (satisf) * 100 << " % " << std::endl;


  }


  else
  {
    NS_ABORT_MSG ("Unknown scheduler: " << scheduler);
  }

  // --- Coté Recepteur---
  for (uint32_t i = 0; i < nUes; ++i)
  {
    PacketSinkHelper sink ("ns3::UdpSocketFactory",
                           Address (InetSocketAddress (Ipv4Address::GetAny (), dlPort + i)));
    sink.Install (nodes.Get (2*i + 1));
  }

  // Flow Monitor This plays the role of the oracle
  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> monitor = flowHelper.InstallAll ();

  //Simulator::Stop (Seconds (simTime));
Simulator::Stop (Seconds (5.0));  
Simulator::Run ();

  
  Simulator::Destroy ();
  return 0;
}
