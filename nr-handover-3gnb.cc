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
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandover3Gnb");

static uint32_t g_hoStartCount = 0;
static uint32_t g_hoEndOkCount = 0;
static uint32_t g_hoEndErrorCount = 0;

static Time g_rttSumThisInterval = Seconds(0);
static uint32_t g_rttCountThisInterval = 0;
static Time g_lastRtt = Seconds(0);
static std::ofstream* g_rttCsv = nullptr;
static uint32_t g_lastCwndBytes = 0;
static std::ofstream* g_cwndCsv = nullptr;

static void
NotifyTcpLastRtt(Time /*oldValue*/, Time newValue)
{
    if (newValue.IsZero())
    {
        return;
    }
    g_lastRtt = newValue;
    g_rttSumThisInterval += newValue;
    ++g_rttCountThisInterval;
    if (g_rttCsv && g_rttCsv->is_open())
    {
        (*g_rttCsv) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ","
                   << newValue.GetSeconds() * 1000.0 << "\n";
    }
}

static void
NotifyTcpCwnd(uint32_t /*oldValue*/, uint32_t newValue)
{
    g_lastCwndBytes = newValue;
    if (g_cwndCsv && g_cwndCsv->is_open())
    {
        (*g_cwndCsv) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ","
                    << newValue << "\n";
    }
}

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

static void
PrintIntervalStats(Time interval,
                   Time startTime,
                   Time stopTime,
                   const Ptr<PacketSink>& dlSink,
                   uint64_t* lastRxBytes)
{
    const Time now = Simulator::Now();
    if (now < startTime)
    {
        Simulator::Schedule(interval,
                            &PrintIntervalStats,
                            interval,
                            startTime,
                            stopTime,
                            dlSink,
                            lastRxBytes);
        return;
    }

    const uint64_t rxBytesNow = dlSink->GetTotalRx();
    const uint64_t deltaBytes = rxBytesNow - *lastRxBytes;
    *lastRxBytes = rxBytesNow;

    const double intervalSeconds = interval.GetSeconds();
    const double thrMbps = (deltaBytes * 8.0) / intervalSeconds / 1e6;

    double rttAvgMs = -1.0;
    if (g_rttCountThisInterval > 0)
    {
        rttAvgMs = (g_rttSumThisInterval.GetSeconds() * 1000.0) / g_rttCountThisInterval;
    }

    std::cout << std::fixed << std::setprecision(3) << now.GetSeconds() << "s"
              << " interval_throughput=" << thrMbps << " Mbps";
    if (rttAvgMs >= 0.0)
    {
        std::cout << " interval_rtt_avg=" << rttAvgMs << " ms";
    }
    else
    {
        std::cout << " interval_rtt_avg=N/A";
    }
    std::cout << " (last_rtt=" << g_lastRtt.GetMilliSeconds() << " ms"
              << ", last_cwnd=" << g_lastCwndBytes << " bytes)" << std::endl;

    g_rttSumThisInterval = Seconds(0);
    g_rttCountThisInterval = 0;

    if (now + interval <= stopTime)
    {
        Simulator::Schedule(interval,
                            &PrintIntervalStats,
                            interval,
                            startTime,
                            stopTime,
                            dlSink,
                            lastRxBytes);
    }
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

    // TCP traffic parameters.
    uint32_t tcpSendSize = 1448;
    uint16_t tcpPort = 50000;

    Time statsInterval = Seconds(1.0);

    bool useIdealRrc = true;

    bool enableHarqRetx = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.AddValue("appStartTime", "UDP app start time", appStartTime);
    cmd.AddValue("gnbDistance", "Distance between adjacent gNBs (m)", gnbDistanceMeters);
    cmd.AddValue("ueSpeed", "UE speed (m/s)", ueSpeedMps);
    std::string tcpVariant = "ns3::TcpVegas";
    std::string rttCsvPath = "tcp-rtt-samples.csv";
    std::string cwndCsvPath = "tcp-cwnd-samples.csv";

    cmd.AddValue("tcpVariant",
                 "TCP congestion control TypeId name (e.g., ns3::TcpVegas, ns3::TcpCubic)",
                 tcpVariant);
    cmd.AddValue("tcpSendSize", "TCP send size (bytes)", tcpSendSize);
    cmd.AddValue("tcpPort", "TCP destination port on UE", tcpPort);
    cmd.AddValue("useIdealRrc", "Use ideal RRC (recommended for demo)", useIdealRrc);
    cmd.AddValue("statsInterval", "Interval for printing RTT/throughput stats", statsInterval);
    cmd.AddValue("enableHarqRetx",
                 "Enable NR HARQ retransmissions (may impact stability with UL traffic)",
                 enableHarqRetx);
    cmd.AddValue("rttCsv", "CSV output path for per-sample TCP RTT (time_s,rtt_ms)", rttCsvPath);
    cmd.AddValue("cwndCsv", "CSV output path for per-sample TCP cwnd (time_s,cwnd_bytes)", cwndCsvPath);
    cmd.Parse(argc, argv);

    // Use a TCP CC algorithm that is sensitive to RTT (Vegas by default).
    {
        const TypeId tcpTid = TypeId::LookupByName(tcpVariant);
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(tcpTid));
    }

    // Avoid artificial throughput limiting due to small socket buffers.
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(1 << 20));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(1 << 20));

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

    // UL traffic (e.g., Ping) can exercise HARQ; keep it off by default for stability.
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqRetx));

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
    // Remote host IP is available as internetIpIfaces.GetAddress(1) if needed.

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

    // DL traffic: RemoteHost -> UE (TCP).
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), tcpPort));
    ApplicationContainer sinkApps = sinkHelper.Install(ueNodes.Get(0));
    Ptr<PacketSink> dlSink = sinkApps.Get(0)->GetObject<PacketSink>();

    BulkSendHelper bulkHelper("ns3::TcpSocketFactory",
                              InetSocketAddress(ueIpIfaces.GetAddress(0), tcpPort));
    bulkHelper.SetAttribute("MaxBytes", UintegerValue(0));
    bulkHelper.SetAttribute("SendSize", UintegerValue(tcpSendSize));
    ApplicationContainer clientApps = bulkHelper.Install(remoteHost);

    sinkApps.Start(appStartTime);
    clientApps.Start(appStartTime);
    sinkApps.Stop(simTime);
    clientApps.Stop(simTime);

    // Periodic RTT + throughput printing.
    uint64_t lastRxBytes = 0;
    Simulator::Schedule(appStartTime,
                        &PrintIntervalStats,
                        statsInterval,
                        appStartTime,
                        simTime,
                        dlSink,
                        &lastRxBytes);

    // Hook UE RRC handover traces.
    {
        Ptr<NrUeNetDevice> ueNd = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
        Ptr<NrUeRrc> ueRrc = ueNd->GetRrc();
        ueRrc->TraceConnectWithoutContext("HandoverStart", MakeCallback(&NotifyHandoverStart));
        ueRrc->TraceConnectWithoutContext("HandoverEndOk", MakeCallback(&NotifyHandoverEndOk));
        ueRrc->TraceConnectWithoutContext("HandoverEndError", MakeCallback(&NotifyHandoverEndError));
    }

    // Open per-sample RTT CSV and hook TCP RTT trace after the BulkSend socket exists.
    std::ofstream rttCsv;
    std::ofstream cwndCsv;
    if (!rttCsvPath.empty())
    {
        rttCsv.open(rttCsvPath.c_str(), std::ofstream::out | std::ofstream::trunc);
        if (rttCsv.is_open())
        {
            rttCsv << "time_s,rtt_ms\n";
            g_rttCsv = &rttCsv;
        }
    }

    if (!cwndCsvPath.empty())
    {
        cwndCsv.open(cwndCsvPath.c_str(), std::ofstream::out | std::ofstream::trunc);
        if (cwndCsv.is_open())
        {
            cwndCsv << "time_s,cwnd_bytes\n";
            g_cwndCsv = &cwndCsv;
        }
    }

    Simulator::Schedule(appStartTime + MilliSeconds(1),
                        [app = clientApps.Get(0)]() {
                            Ptr<BulkSendApplication> bulk = app->GetObject<BulkSendApplication>();
                            if (!bulk)
                            {
                                return;
                            }
                            Ptr<Socket> s = bulk->GetSocket();
                            Ptr<TcpSocketBase> tcp = DynamicCast<TcpSocketBase>(s);
                            if (tcp)
                            {
                                tcp->TraceConnectWithoutContext("LastRTT",
                                                               MakeCallback(&NotifyTcpLastRtt));
                                tcp->TraceConnectWithoutContext("CongestionWindow",
                                                               MakeCallback(&NotifyTcpCwnd));
                            }
                        });

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

    if (rttCsv.is_open())
    {
        rttCsv.close();
    }
    if (cwndCsv.is_open())
    {
        cwndCsv.close();
    }

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
