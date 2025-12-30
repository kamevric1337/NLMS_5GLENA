// Simulate 1 UE traversing through 3 gNBs and observe handovers + throughput.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <cstdint>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandover3Gnb");

static uint32_t g_hoStartCount = 0;
static uint32_t g_hoEndOkCount = 0;
static uint32_t g_hoEndErrorCount = 0;

static void
NotifyHandoverStart(uint64_t imsi, uint16_t sourceCellId, uint16_t rnti, uint16_t targetCellId)
{
    ++g_hoStartCount;
    std::cout << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds()
              << "s HO-START imsi=" << imsi << " rnti=" << rnti << " " << sourceCellId
              << " -> " << targetCellId << std::endl;
}

static void
NotifyHandoverEndOk(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    ++g_hoEndOkCount;
    std::cout << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds()
              << "s HO-END-OK imsi=" << imsi << " rnti=" << rnti << " nowCellId=" << cellId
              << std::endl;
}

static void
NotifyHandoverEndError(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    ++g_hoEndErrorCount;
    std::cout << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds()
              << "s HO-END-ERROR imsi=" << imsi << " rnti=" << rnti << " cellId=" << cellId
              << std::endl;
}

int
main(int argc, char* argv[])
{
    // Defaults chosen so the UE traverses all 3 gNBs (and triggers 2 HOs)
    // with the default gNB spacing and UE speed.
    Time simTime = Seconds(35.0);
    Time appStartTime = Seconds(0.5);

    double gnbDistanceMeters = 200.0;
    double ueSpeedMps = 20.0;
    double ueStartX = -100.0;

    double centralFrequency = 2.8e9;
    double bandwidth = 20e6;
    uint8_t numerology = 1;

    uint32_t udpPacketSize = 1200;
    double udpRateMbps = 50.0;

    bool useIdealRrc = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.AddValue("appStartTime", "UDP app start time", appStartTime);
    cmd.AddValue("gnbDistance", "Distance between adjacent gNBs (m)", gnbDistanceMeters);
    cmd.AddValue("ueSpeed", "UE speed (m/s)", ueSpeedMps);
    cmd.AddValue("udpRateMbps", "DL offered rate (Mbps)", udpRateMbps);
    cmd.AddValue("useIdealRrc", "Use ideal RRC (recommended for demo)", useIdealRrc);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    NodeContainer gnbNodes;
    gnbNodes.Create(3);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Mobility: 3 fixed gNBs on a line; UE moves across them.
    {
        MobilityHelper gnbMob;
        Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
        gnbPos->Add(Vector(0.0, 0.0, 10.0));
        gnbPos->Add(Vector(gnbDistanceMeters, 0.0, 10.0));
        gnbPos->Add(Vector(2.0 * gnbDistanceMeters, 0.0, 10.0));
        gnbMob.SetPositionAllocator(gnbPos);
        gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        gnbMob.Install(gnbNodes);

        MobilityHelper ueMob;
        Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
        uePos->Add(Vector(ueStartX, 0.0, 1.5));
        ueMob.SetPositionAllocator(uePos);
        ueMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        ueMob.Install(ueNodes);

        Ptr<ConstantVelocityMobilityModel> cv = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        cv->SetVelocity(Vector(ueSpeedMps, 0.0, 0.0));
    }

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(useIdealRrc));

    // Automatic handover based on A3 RSRP.
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(64)));

    auto bwPair = nrHelper->CreateBandwidthParts({{centralFrequency, bandwidth, numerology}}, "UMa");

    NetDeviceContainer gnbDevs;
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        gnbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(i), bwPair.second));
    }

    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, {bwPair.second.front()});

    // Let gNB RRC accept HO requests.
    for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
    {
        Ptr<NrGnbRrc> gnbRrc = (*it)->GetObject<NrGnbNetDevice>()->GetRrc();
        gnbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(true));
    }

    // X2 interface between all gNBs is required for handover.
    nrHelper->AddX2Interface(gnbNodes);

    // Internet + remote host.
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(ueNodes);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));

    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    (void)internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Attach UE initially to the closest gNB.
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Default route on UE.
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // DL traffic: RemoteHost -> UE.
    uint16_t dlPort = 1234;

    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer sinkApps = sinkHelper.Install(ueNodes.Get(0));
    Ptr<PacketSink> dlSink = sinkApps.Get(0)->GetObject<PacketSink>();

    // Convert Mbps -> packet interval.
    double udpRateBps = udpRateMbps * 1e6;
    double pktIntervalSeconds = (udpPacketSize * 8.0) / udpRateBps;

    UdpClientHelper client(ueIpIfaces.GetAddress(0), dlPort);
    client.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    client.SetAttribute("Interval", TimeValue(Seconds(pktIntervalSeconds)));
    client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    ApplicationContainer clientApps = client.Install(remoteHost);

    sinkApps.Start(appStartTime);
    clientApps.Start(appStartTime);
    sinkApps.Stop(simTime);
    clientApps.Stop(simTime);

    // Hook UE RRC handover traces.
    {
        Ptr<NrUeNetDevice> ueNd = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
        Ptr<NrUeRrc> ueRrc = ueNd->GetRrc();
        ueRrc->TraceConnectWithoutContext("HandoverStart", MakeCallback(&NotifyHandoverStart));
        ueRrc->TraceConnectWithoutContext("HandoverEndOk", MakeCallback(&NotifyHandoverEndOk));
        ueRrc->TraceConnectWithoutContext("HandoverEndError", MakeCallback(&NotifyHandoverEndError));
    }

    Simulator::Stop(simTime);
    Simulator::Run();

    const double rxDuration = (simTime - appStartTime).GetSeconds();
    const uint64_t rxBytes = dlSink->GetTotalRx();
    const double throughputMbps = (rxBytes * 8.0) / rxDuration / 1e6;

    std::cout << "\nSummary\n";
    std::cout << "  HO starts:     " << g_hoStartCount << "\n";
    std::cout << "  HO end ok:     " << g_hoEndOkCount << "\n";
    std::cout << "  HO end error:  " << g_hoEndErrorCount << "\n";
    std::cout << "  DL rx bytes:   " << rxBytes << "\n";
    std::cout << std::fixed << std::setprecision(3) << "  DL throughput: " << throughputMbps
              << " Mbps\n";

    Simulator::Destroy();

    // Expected handovers depend on whether the UE has enough time to reach the
    // midpoints between gNB0-gNB1 and gNB1-gNB2.
    uint32_t expectedHo = 0;
    if (ueSpeedMps > 0.0)
    {
        const double tMid01 = (gnbDistanceMeters / 2.0 - ueStartX) / ueSpeedMps;
        const double tMid12 = (3.0 * gnbDistanceMeters / 2.0 - ueStartX) / ueSpeedMps;
        if (simTime.GetSeconds() > tMid01)
        {
            ++expectedHo;
        }
        if (simTime.GetSeconds() > tMid12)
        {
            ++expectedHo;
        }
    }

    if (g_hoEndOkCount < expectedHo)
    {
        std::cerr << "WARNING: Expected at least " << expectedHo
                  << " successful handovers, got " << g_hoEndOkCount << std::endl;
        return 1;
    }

    return 0;
}
