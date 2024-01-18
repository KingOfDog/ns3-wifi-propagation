#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/double.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/network-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AdhocWifiPropagationComparison");

enum PropagationModel
{
    FRIIS,
    FIXED_RSS,
    THREE_LOG_DISTANCE,
    TWO_RAY_GROUND,
    NAKAGAMI
};

inline const std::string
propagationModelToString(PropagationModel model)
{
    switch (model)
    {
    case FRIIS:
        return "Friis";
    case FIXED_RSS:
        return "FixedRSS";
    case THREE_LOG_DISTANCE:
        return "ThreeLogDistance";
    case TWO_RAY_GROUND:
        return "TwoRayGround";
    case NAKAGAMI:
        return "Nakagami";
    }
}

double averageRSS = 0;

static void
PhyTrace(Ptr<const Packet> packet,
         uint16_t channelFreqMhz,
         WifiTxVector txVector,
         MpduInfo aMpdu,
         SignalNoiseDbm signalNoise,
         uint16_t staId)

{
    NS_LOG_DEBUG("Received packet with signal: " << signalNoise.signal
                                                 << ", noise: " << signalNoise.noise);
    averageRSS = (signalNoise.signal + averageRSS) / 2.;
}

int
main(int argc, char* argv[])
{
    Time::SetResolution(Time::NS);

    // Simulation parameters
    const double distance = 10; // Distance between nodes in meters

    const double dataRate = 75e6;                              // 75 Mbps target data rate
    const uint64_t packetSize = 1450;                          // bytes
    const double interval = 1 / (dataRate / (packetSize * 8)); // delay between packets

    const double txPower = 10; // dBm
    const double txGain = 1;   // dB
    const double rxGain = 1;   // dB

    const double antennaZ = 1.5; // Antenna height in meters

    std::ofstream outputFile;
    std::string outputFileName = "output_runtime.csv";
    outputFile.open(outputFileName);
    outputFile << "runtime,rssDBm,throughputKbps\n";
    outputFile.close();

    bool connectionPossible = true;

    for (double simulationTime = 1; simulationTime <= 200; simulationTime += 1)
    {
        NS_LOG_UNCOND("Running simulation for " << simulationTime << "s");
        const uint64_t packetLimit = simulationTime / interval;

        averageRSS = 0;

        Time interPacketInterval = Seconds(interval);

        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper stack;
        stack.Install(nodes);

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211n);

        YansWifiPhyHelper wifiPhy;
        wifiPhy.Set("TxPowerStart", DoubleValue(txPower));
        wifiPhy.Set("TxPowerEnd", DoubleValue(txPower));
        wifiPhy.Set("RxGain", DoubleValue(rxGain));
        wifiPhy.Set("TxGain", DoubleValue(txGain));
        wifiPhy.Set("ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));

        YansWifiChannelHelper wifiChannel;
        wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                       "Frequency",
                                       DoubleValue(5.18e9),
                                       "SystemLoss",
                                       DoubleValue(1.0));

        wifiPhy.SetChannel(wifiChannel.Create());

        WifiMacHelper wifiMac;
        wifiMac.SetType("ns3::AdhocWifiMac");

        MobilityHelper mobility;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        positionAlloc->Add(Vector(0.0, 0.0, antennaZ));
        positionAlloc->Add(Vector(distance, 0.0, antennaZ));
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(nodes);

        NetDeviceContainer serverDevice = wifi.Install(wifiPhy, wifiMac, nodes.Get(0));
        NetDeviceContainer clientDevice = wifi.Install(wifiPhy, wifiMac, nodes.Get(1));

        Ipv4AddressHelper address;
        address.SetBase("10.1.1.0", "255.255.255.0");

        Ipv4InterfaceContainer serverInterface = address.Assign(serverDevice);
        Ipv4InterfaceContainer clientInterface = address.Assign(clientDevice);

        NS_LOG_INFO("Create UdpServer application on node 1.");
        ApplicationContainer serverApp;
        uint16_t port = 9;
        UdpServerHelper server(port);
        serverApp = server.Install(nodes.Get(0));
        serverApp.Start(Seconds(1.0));
        serverApp.Stop(Seconds(simulationTime));

        Ptr<UdpServer> serverPtr = server.GetServer();
        Address serverAddr = Address(serverInterface.GetAddress(0));

        UdpClientHelper client(serverAddr, port);
        client.SetAttribute("MaxPackets", UintegerValue(packetLimit));
        client.SetAttribute("Interval", TimeValue(interPacketInterval));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));

        ApplicationContainer clientApp = client.Install(nodes.Get(1));
        clientApp.Start(Seconds(2.0));
        clientApp.Stop(Seconds(simulationTime));

        FlowMonitorHelper flowMonitorHelper;
        Ptr<FlowMonitor> flowMonitor = flowMonitorHelper.InstallAll();

        Config::ConnectWithoutContext(
            "/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
            MakeCallback(&PhyTrace));

        Simulator::Stop(Seconds(simulationTime));
        Simulator::Run();

        flowMonitor->CheckForLostPackets();
        flowMonitor->SerializeToXmlFile("flow.xml", true, true);

        FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats();

        for (auto it = stats.begin(); it != stats.end(); ++it)
        {
            double throughput = it->second.rxBytes * 8.0 /
                                (simulationTime) /
                                1024; // Kbps

            NS_LOG_UNCOND("RSS: " << averageRSS << " dBm, Throughput: " << throughput << " Kbps");

            outputFile.open(outputFileName, std::ios_base::app);
            outputFile << simulationTime << "," << averageRSS << "," << throughput << ","
                       << std::endl;
            outputFile.close();

            if (it->second.rxBytes == 0)
            {
                connectionPossible = false;
            }
        }

        Simulator::Destroy();
    }
}
