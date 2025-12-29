// Minimal NR handover example: 1 UE moves from gNB1 to gNB2 and triggers HO.

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandover2Gnb");

static void
UeConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << "s"
              << " IMSI=" << imsi << " RNTI=" << rnti << " connected to cellId=" << cellId
              << std::endl;
}

static void
UeHandoverStart(std::string context,
                uint64_t imsi,
                uint16_t sourceCellId,
                uint16_t rnti,
                uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << "s"
              << " IMSI=" << imsi << " RNTI=" << rnti << " HO start " << sourceCellId << " -> "
              << targetCellId << std::endl;
}

static void
UeHandoverEndOk(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << "s"
              << " IMSI=" << imsi << " RNTI=" << rnti << " HO end OK, now cellId=" << cellId
              << std::endl;
}

int
main(int argc, char* argv[])
{
    double gnbDistance = 200.0; // meters
    double ueStartX = -50.0;    // meters
    double ueSpeed = 20.0;      // m/s
    double simTime = 20.0;      // seconds

    // NR PHY/Spectrum settings (kept simple)
    uint16_t numerology = 0;
    double centralFrequency = 28e9;
    double bandwidth = 100e6;

    CommandLine cmd(__FILE__);
    cmd.AddValue("gnbDistance", "Distance between the 2 gNBs [m]", gnbDistance);
    cmd.AddValue("ueStartX", "UE initial X position [m]", ueStartX);
    cmd.AddValue("ueSpeed", "UE speed along +X [m/s]", ueSpeed);
    cmd.AddValue("simTime", "Simulation time [s]", simTime);
    cmd.AddValue("numerology", "NR numerology (mu)", numerology);
    cmd.AddValue("centralFrequency", "Central frequency [Hz]", centralFrequency);
    cmd.AddValue("bandwidth", "Channel bandwidth [Hz]", bandwidth);
    cmd.Parse(argc, argv);

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Mobility: two fixed gNBs, one UE moving from gNB1 to gNB2.
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);

    Ptr<MobilityModel> gnb1Mob = gnbNodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> gnb2Mob = gnbNodes.Get(1)->GetObject<MobilityModel>();
    gnb1Mob->SetPosition(Vector(0.0, 0.0, 10.0));
    gnb2Mob->SetPosition(Vector(gnbDistance, 0.0, 10.0));

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    Ptr<ConstantVelocityMobilityModel> ueMob =
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    ueMob->SetPosition(Vector(ueStartX, 0.0, 1.5));
    ueMob->SetVelocity(Vector(ueSpeed, 0.0, 0.0));

    // Helpers
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Configure channel model (fast updates, no shadowing for deterministic HO)
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Use A3 RSRP HO algorithm with aggressive settings so HO is easy to see
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(0)));

    // Create one band with one CC and one BWP
    const uint8_t numCcPerBand = 1;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Configure numerology on both gNB PHYs (BWP 0)
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerology));
    NrHelper::GetGnbPhy(gnbDevs.Get(1), 0)->SetAttribute("Numerology", UintegerValue(numerology));

    // X2 interface is required for handover between gNBs
    nrHelper->AddX2Interface(gnbNodes);

    // Internet + EPC remote host
    InternetStackHelper internet;
    internet.Install(ueNodes);

    auto [remoteHost, remoteHostAddr] =
        nrEpcHelper->SetupRemoteHost("10Gb/s", 1500, Seconds(0));
    internet.Install(remoteHost);

    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(ueDevs);

    // Default route for UE (AssignUeIpv4Address already does this for NoBackhaul EPC,
    // but keeping it explicit makes the scenario robust if EPC helper changes.)
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    // Attach UE to best (closest) gNB initially.
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Simple DL traffic: RemoteHost -> UE (keeps bearer active across HO)
    const uint16_t dlPort = 1234;
    UdpServerHelper dlServer(dlPort);
    ApplicationContainer serverApps = dlServer.Install(ueNodes.Get(0));

    UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dlClient.SetAttribute("PacketSize", UintegerValue(200));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0)); // unlimited
    ApplicationContainer clientApps = dlClient.Install(remoteHost);

    serverApps.Start(Seconds(0.1));
    clientApps.Start(Seconds(0.2));

    // Hook RRC traces to print connection + handover events
    bool ok = true;
    ok = ok && Config::ConnectFailSafe("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
                                      MakeCallback(&UeConnectionEstablished));
    ok = ok && Config::ConnectFailSafe("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                                      MakeCallback(&UeHandoverStart));
    ok = ok && Config::ConnectFailSafe("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                                      MakeCallback(&UeHandoverEndOk));
    NS_ABORT_MSG_IF(!ok, "Failed to connect one or more NR UE RRC trace sources");

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
