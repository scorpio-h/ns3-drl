#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/traffic-control-module.h"
#include "ns3/gnuplot.h"
#include "ns3/flow-id-tag.h"
#include "ns3/ipv4-flow-probe.h"
#include "ns3/netanim-module.h"
#include "ns3/opengym-module.h"

#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <utility>
#include <set>
#include <string>
#include <map>

// The CDF in TrafficGenerator
extern "C"
{
#include "cdf.h"
}

#define LINK_CAPACITY_BASE    1000000000          // 1Gbps
#define BUFFER_SIZE 600                           // 250 packets

#define RED_QUEUE_MARKING 65 		        	  // 65 Packets (available only in DcTcp)


// The flow port range, each flow will be assigned a random port number within this range
#define PORT_START 10000
#define PORT_END 50000

// Adopted from the simulation from WANG PENG
// Acknowledged to https://williamcityu@bitbucket.org/williamcityu/2016-socc-simulation.git
#define PACKET_SIZE 1400

#define PRESTO_RATIO 10

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WCMP");

class MyFlow
{
   public:
      int src;   // srcID
      int dest;  // destID
      uint16_t srcport;
      uint16_t destport;
      int pro;
      uint32_t size;      
      double  start;  //startTime (ns)     
      double  last;  //last packet recvtime (ns)
      bool active;
      uint32_t recvbytes;
      int appnum;
};

double th1 = 0.0;
double th2 = 0.0;
std::vector<MyFlow> activelist;
std::vector<MyFlow> finishedlist;
std::vector<MyFlow> myflows;
std::map<Ipv4Address,int> hostid;

/*
Define action space
*/
Ptr<OpenGymSpace> MyGetActionSpace(void)
{
  uint32_t parameterNum = 128;
  float low = 0.0;
  float high = 1.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<float> ();
  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_UNCOND ("MyGetActionSpace: " << box);
  return box;
}

/*
Define observation space
*/
Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
  uint32_t parameterNum = 750;
  uint32_t low = 0;
  uint32_t high = 1000000000;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint32_t> ();
  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_UNCOND ("MyGetObservationSpace: " << box);
  return box;
}

/*
Define game over condition
*/
bool MyGetGameOver(void)
{
  bool isGameOver = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 10000) {
      isGameOver = true;
  }
  NS_LOG_UNCOND ("MyGetGameOver: " << isGameOver);
  return isGameOver;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer> MyGetObservation(void)
{
  uint32_t parameterNum = 750;
  std::vector<uint32_t> shape = {parameterNum,};
  Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);
  for(int i=0;i<10;i++){
      box->AddValue(activelist[i].src);
      box->AddValue(activelist[i].dest);
      box->AddValue(activelist[i].srcport);
      box->AddValue(activelist[i].destport);
      box->AddValue(activelist[i].pro);
     // NS_LOG_UNCOND("activelist"<<activelist[i].src<<" "<<activelist[i].dest<<" "<<activelist[i].recvbytes<<" "<<activelist[i].size<<" "<<activelist[i].last<<" "<<activelist[i].start);
  }

  for(int i=0;i<100;i++){
      box->AddValue(finishedlist[i].src);
      box->AddValue(finishedlist[i].dest);
      box->AddValue(finishedlist[i].srcport);
      box->AddValue(finishedlist[i].destport);
      box->AddValue(finishedlist[i].pro);
      box->AddValue(finishedlist[i].size);
      box->AddValue((finishedlist[i].last-finishedlist[i].start)*1000000000);
      //NS_LOG_UNCOND("finished"<<finishedlist[i].src<<" "<<finishedlist[i].dest<<" "<<finishedlist[i].recvbytes<<" "<<finishedlist[i].size<<" "<<finishedlist[i].last<<" "<<finishedlist[i].start);
  }
  NS_LOG_UNCOND("MyGetObservation: " << box);
  return box;
}

/*
Define reward function
*/
float MyGetReward(void)
{
  float reward = 0.0;
  if(th1 != 0.0){
      reward = th2/th1;
  }
  return reward;
}

/*
Define extra info. Optional
*/
std::string MyGetExtraInfo(void)
{
  std::string myInfo = "testInfo";
  NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
  return myInfo;
}
/*
Execute received actions
*/
bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
  Ptr<OpenGymBoxContainer<float> > box = DynamicCast<OpenGymBoxContainer<float> >(action);
//  NS_LOG_UNCOND ("MyExecuteActions: " << action);
  return true;
}

bool sort_by_start(const MyFlow& obj1,const MyFlow& obj2) 
{
    return obj1.start< obj2.start;
}
 
bool sort_by_end(const MyFlow& obj1,const MyFlow& obj2)
{
    return obj1.last > obj2.last;
}
void ScheduleNextStateRead(double envStepTime, FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon,Ptr<OpenGymInterface> openGym)
{
    //find srcport
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
    {	
        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
        std::map<Ipv4Address, int>::iterator iter;                                             
        iter = hostid.find(fiveTuple.sourceAddress);  
        int sid = iter->second;
        iter = hostid.find(fiveTuple.destinationAddress); 
        int did = iter->second;
        int sport = fiveTuple.sourcePort;
        int dport = fiveTuple.destinationPort;
        for(size_t i=0;i<myflows.size();i++){          
            if(myflows[i].recvbytes>0 && myflows[i].srcport==0 && myflows[i].src==sid && myflows[i].dest==did && myflows[i].destport==dport){
                myflows[i].srcport = sport;
             }
        }       
    }

    activelist.clear();
    finishedlist.clear();
    double th = 0.0;
    for(size_t i=0;i<myflows.size();i++){
        if(myflows[i].active == false){
            finishedlist.push_back(myflows[i]);
            double fct = myflows[i].last-myflows[i].start;
            th += myflows[i].size/fct;
        }
        else{
            if(myflows[i].recvbytes>0){
                 activelist.push_back(myflows[i]);
            }
        }
    }
    sort(activelist.begin(),activelist.end(),sort_by_start);
    sort(finishedlist.begin(),finishedlist.end(),sort_by_end);
    activelist.resize(10);
    finishedlist.resize(100);
    th1 = th2;
    th2 = th;
    Simulator::Schedule (Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, fmhelper, flowMon,openGym);
    openGym->NotifyCurrentState();
}


// Port from Traffic Generator
// Acknowledged to https://github.com/HKUST-SING/TrafficGenerator/blob/master/src/common/common.c
double poission_gen_interval(double avg_rate)
{
    if (avg_rate > 0)
       return -logf(1.0 - (double)rand() / RAND_MAX) / avg_rate;
    else
       return 0;
}

template<typename T>
T rand_range (T min, T max)
{
    return min + ((double)max - min) * rand () / RAND_MAX;
}

void SinkRx (std::string path, Ptr<const Packet> p, const Address &ad)
{
        int dest,appnum;
        char *p1 = (char*)path.c_str();
        sscanf(p1, "/NodeList/%d/ApplicationList/%d/$ns3::PacketSink/Rx", &dest, &appnum);
        dest = dest-16;
        int size = p->GetSize();
        for(size_t i =0;i<myflows.size();i++){
            if(dest == myflows[i].dest && appnum ==myflows[i].appnum && myflows[i].active == true){
                myflows[i].recvbytes += size;
                myflows[i].last = Simulator::Now().GetSeconds();
                if(myflows[i].recvbytes == myflows[i].size){
                    myflows[i].active = false;
                    //NS_LOG_UNCOND(myflows[i].src<<" "<<dest<<" "<<myflows[i].recvbytes<<" "<<myflows[i].size<<" "<<myflows[i].last-myflows[i].start);
                }
                break;
            }
        }
}
int nodeapp[32]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

void install_applications (int fromLeafId, NodeContainer servers, double requestRate, struct cdf_table *cdfTable,
        long &flowCount, long &totalFlowSize, int SERVER_COUNT, int LEAF_COUNT, double START_TIME, double END_TIME, double FLOW_LAUNCH_END_TIME, uint32_t applicationPauseThresh, uint32_t applicationPauseTime)
{

    NS_LOG_INFO ("Install applications:");
    for (int i = 0; i < SERVER_COUNT; i++)
    {
        int fromServerIndex = fromLeafId * SERVER_COUNT + i;

        double startTime = START_TIME + poission_gen_interval (requestRate);
        while (startTime < FLOW_LAUNCH_END_TIME)
        {
            flowCount ++;
            uint16_t port = rand_range (PORT_START, PORT_END);

            int destServerIndex = fromServerIndex;
      	    while (destServerIndex >= fromLeafId * SERVER_COUNT && destServerIndex < fromLeafId * SERVER_COUNT + SERVER_COUNT)
            {
	     	    destServerIndex = rand_range (0, SERVER_COUNT * LEAF_COUNT);
            }

	        Ptr<Node> destServer = servers.Get (destServerIndex);
	        Ptr<Ipv4> ipv4 = destServer->GetObject<Ipv4> ();
	        Ipv4InterfaceAddress destInterface = ipv4->GetAddress (1,0);
	        Ipv4Address destAddress = destInterface.GetLocal ();

            BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (destAddress, port));
            uint32_t flowSize = gen_random_cdf (cdfTable);
            if(flowSize==0){
                flowSize=1;
            }
            totalFlowSize += flowSize;
 	    source.SetAttribute ("SendSize", UintegerValue (PACKET_SIZE));
            source.SetAttribute ("MaxBytes", UintegerValue(flowSize));
            source.SetAttribute ("DelayThresh", UintegerValue (applicationPauseThresh));
            source.SetAttribute ("DelayTime", TimeValue (MicroSeconds (applicationPauseTime)));

            // Install apps
            ApplicationContainer sourceApp = source.Install (servers.Get (fromServerIndex));
            sourceApp.Start (Seconds (startTime));
            sourceApp.Stop (Seconds (END_TIME));
            nodeapp[fromServerIndex] += 1; 

            // Install packet sinks
            PacketSinkHelper sink ("ns3::TcpSocketFactory",
            InetSocketAddress (Ipv4Address::GetAny (), port));
            ApplicationContainer sinkApp = sink.Install (servers. Get (destServerIndex));
            sinkApp.Start (Seconds (START_TIME));
            nodeapp[destServerIndex] += 1;

            //Flow install
            MyFlow ft;
            ft.src = fromServerIndex;
            ft.dest = destServerIndex;
            ft.srcport = 0;
            ft.destport = port;
            ft.pro = 6;
            ft.size = flowSize;
            ft.start = startTime;
            ft.last = 0;
            ft.active = true;
            ft.recvbytes = 0;
            ft.appnum = nodeapp[destServerIndex];
            startTime += poission_gen_interval (requestRate);
            myflows.push_back(ft);
        }
    }
}

int main (int argc, char *argv[])
{
#if 1
    LogComponentEnable ("WCMP", LOG_LEVEL_INFO);
#endif

    hostid.insert(std::make_pair(Ipv4Address("10.1.1.2"), 0));
    hostid.insert(std::make_pair(Ipv4Address("10.1.1.4"), 1));
    hostid.insert(std::make_pair(Ipv4Address("10.1.1.6"), 2));
    hostid.insert(std::make_pair(Ipv4Address("10.1.1.8"), 3));
    hostid.insert(std::make_pair(Ipv4Address("10.1.2.2"), 4));
    hostid.insert(std::make_pair(Ipv4Address("10.1.2.4"), 5));
    hostid.insert(std::make_pair(Ipv4Address("10.1.2.6"), 6));
    hostid.insert(std::make_pair(Ipv4Address("10.1.2.8"), 7));
    hostid.insert(std::make_pair(Ipv4Address("10.1.3.2"), 8));
    hostid.insert(std::make_pair(Ipv4Address("10.1.3.4"), 9));
    hostid.insert(std::make_pair(Ipv4Address("10.1.3.6"), 10));
    hostid.insert(std::make_pair(Ipv4Address("10.1.3.8"), 11));
    hostid.insert(std::make_pair(Ipv4Address("10.1.4.2"), 12));
    hostid.insert(std::make_pair(Ipv4Address("10.1.4.4"), 13));
    hostid.insert(std::make_pair(Ipv4Address("10.1.4.6"), 14));
    hostid.insert(std::make_pair(Ipv4Address("10.1.4.8"), 15));
    hostid.insert(std::make_pair(Ipv4Address("10.1.5.2"), 16));
    hostid.insert(std::make_pair(Ipv4Address("10.1.5.4"), 17));
    hostid.insert(std::make_pair(Ipv4Address("10.1.5.6"), 18));
    hostid.insert(std::make_pair(Ipv4Address("10.1.5.8"), 19));
    hostid.insert(std::make_pair(Ipv4Address("10.1.6.2"), 20));
    hostid.insert(std::make_pair(Ipv4Address("10.1.6.4"), 21));
    hostid.insert(std::make_pair(Ipv4Address("10.1.6.6"), 22));
    hostid.insert(std::make_pair(Ipv4Address("10.1.6.8"), 23));
    hostid.insert(std::make_pair(Ipv4Address("10.1.7.2"), 24));
    hostid.insert(std::make_pair(Ipv4Address("10.1.7.4"), 25));
    hostid.insert(std::make_pair(Ipv4Address("10.1.7.6"), 26));
    hostid.insert(std::make_pair(Ipv4Address("10.1.7.8"), 27));
    hostid.insert(std::make_pair(Ipv4Address("10.1.8.2"), 28));
    hostid.insert(std::make_pair(Ipv4Address("10.1.8.4"), 29));
    hostid.insert(std::make_pair(Ipv4Address("10.1.8.6"), 30));
    hostid.insert(std::make_pair(Ipv4Address("10.1.8.8"), 31));
    // Parameters of the scenario
    uint32_t simSeed = 1;
    double envStepTime = 0.001; //seconds, ns3gym env step time interval 1ms
    uint32_t openGymPort = 5555;
    // Command line parameters parsing
    std::string id = "4";
    unsigned randomSeed = 0;
    std::string cdfFileName = "cdf/VL2_CDF.txt";   //""
    double load = 0.9;   //0.0
    std::string transportProt = "Tcp";
    std::string weightFileName = "scratch/rl-vl2/Weight.txt";

    // The simulation starting and ending time
    double START_TIME = 0.0;
    double END_TIME = 10.0001;
    uint32_t linkLatency = 10;

    bool asymCapacity = true;

    uint32_t asymCapacityPoss = 20;  // 40 %

    int SERVER_COUNT = 4;
    int SPINE_COUNT = 8;
    int LEAF_COUNT = 8;
    int LINK_COUNT = 1;

    uint64_t spineLeafCapacity = 10;
    uint64_t leafServerCapacity = 10;

    uint32_t applicationPauseThresh = 0;
    uint32_t applicationPauseTime = 1000;

    bool enableLargeDupAck = false;

    bool enableRandomDrop = false;
    double randomDropRate = 0.005; // 0.5%

    uint32_t blackHoleMode = 0; // When the black hole is enabled, the
    std::string blackHoleSrcAddrStr = "10.1.1.1";
    std::string blackHoleSrcMaskStr = "255.255.255.0";
    std::string blackHoleDestAddrStr = "10.1.2.0";
    std::string blackHoleDestMaskStr = "255.255.255.0";

    bool asymCapacity2 = false;

    bool enableLargeSynRetries = false;

    bool enableLargeDataRetries = false;

    bool enableFastReConnection = false;

    CommandLine cmd;
    cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
    cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
    cmd.AddValue ("ID", "Running ID", id);
    cmd.AddValue ("StartTime", "Start time of the simulation", START_TIME);
    cmd.AddValue ("EndTime", "End time of the simulation", END_TIME);
    cmd.AddValue ("StepTime", "Step time", envStepTime);
//    cmd.AddValue ("FlowLaunchEndTime", "End time of the flow launch period", FLOW_LAUNCH_END_TIME);
    cmd.AddValue ("randomSeed", "Random seed, 0 for random generated", randomSeed);
    cmd.AddValue ("cdfFileName", "File name for flow distribution", cdfFileName);
    cmd.AddValue ("weightFileName", "File name for flow distribution", weightFileName);
    cmd.AddValue ("load", "Load of the network, 0.0 - 1.0", load);
    cmd.AddValue ("transportProt", "Transport protocol to use: Tcp, DcTcp", transportProt);
    cmd.AddValue ("linkLatency", "Link latency, should be in MicroSeconds", linkLatency);

    cmd.AddValue ("asymCapacity", "Whether the capacity is asym, which means some link will have only 1/10 the capacity of others", asymCapacity);
    cmd.AddValue ("asymCapacityPoss", "The possibility that a path will have only 1/10 capacity", asymCapacityPoss);

    cmd.AddValue ("serverCount", "The Server count", SERVER_COUNT);
    cmd.AddValue ("spineCount", "The Spine count", SPINE_COUNT);
    cmd.AddValue ("leafCount", "The Leaf count", LEAF_COUNT);
    cmd.AddValue ("linkCount", "The Link count", LINK_COUNT);

    cmd.AddValue ("spineLeafCapacity", "Spine <-> Leaf capacity in Gbps", spineLeafCapacity);
    cmd.AddValue ("leafServerCapacity", "Leaf <-> Server capacity in Gbps", leafServerCapacity);

    cmd.AddValue ("applicationPauseThresh", "How many packets can pass before we have delay, 0 for disable", applicationPauseThresh);
    cmd.AddValue ("applicationPauseTime", "The time for a delay, in MicroSeconds", applicationPauseTime);

    cmd.AddValue ("enableLargeDupAck", "Whether to set the ReTxThreshold to a very large value to mask reordering", enableLargeDupAck);

    cmd.AddValue ("enableRandomDrop", "Whether the Spine-0 to other leaves has the random drop problem", enableRandomDrop);
    cmd.AddValue ("randomDropRate", "The random drop rate when the random drop is enabled", randomDropRate);

    cmd.AddValue ("blackHoleMode", "The packet black hole mode, 0 to disable, 1 src, 2 dest, 3 src/dest pair", blackHoleMode);
    cmd.AddValue ("blackHoleSrcAddr", "The packet black hole source address", blackHoleSrcAddrStr);
    cmd.AddValue ("blackHoleSrcMask", "The packet black hole source mask", blackHoleSrcMaskStr);
    cmd.AddValue ("blackHoleDestAddr", "The packet black hole destination address", blackHoleDestAddrStr);
    cmd.AddValue ("blackHoleDestMask", "The packet black hole destination mask", blackHoleDestMaskStr);

    cmd.AddValue ("asymCapacity2", "Whether the Spine0-Leaf0's capacity is asymmetric", asymCapacity2);

    cmd.AddValue ("enableLargeSynRetries", "Whether the SYN packet would retry thousands of times", enableLargeSynRetries);
    cmd.AddValue ("enableFastReConnection", "Whether the SYN gap will be very small when reconnecting", enableFastReConnection);
    cmd.AddValue ("enableLargeDataRetries", "Whether the data retransmission will be more than 6 times", enableLargeDataRetries);

    cmd.Parse (argc, argv);
    double FLOW_LAUNCH_END_TIME = END_TIME;
    RngSeedManager::SetSeed (1);
    RngSeedManager::SetRun (simSeed);

    uint64_t SPINE_LEAF_CAPACITY = spineLeafCapacity * LINK_CAPACITY_BASE;
    uint64_t LEAF_SERVER_CAPACITY = leafServerCapacity * LINK_CAPACITY_BASE;
    Time LINK_LATENCY = MicroSeconds (linkLatency);
/*
    Ipv4Address blackHoleSrcAddr = Ipv4Address (blackHoleSrcAddrStr.c_str ());
    Ipv4Mask blackHoleSrcMask = Ipv4Mask (blackHoleSrcMaskStr.c_str ());
    Ipv4Address blackHoleDestAddr = Ipv4Address (blackHoleDestAddrStr.c_str ());
    Ipv4Mask blackHoleDestMask = Ipv4Mask (blackHoleDestMaskStr.c_str ());
*/

    if (load < 0.0 || load >= 1.0)
    {
        NS_LOG_ERROR ("The network load should within 0.0 and 1.0");
        return 0;
    }

    NS_LOG_INFO ("Config parameters");
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue(PACKET_SIZE));
    Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (0));
    if (enableFastReConnection)
    {
        Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MicroSeconds (40)));
    }
    else
    {
        Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MilliSeconds (5)));
    }
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));
    Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (MilliSeconds (5)));
    Config::SetDefault ("ns3::TcpSocketBase::ClockGranularity", TimeValue (MicroSeconds (100)));
    Config::SetDefault ("ns3::RttEstimator::InitialEstimation", TimeValue (MicroSeconds (80)));
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (160000000));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (160000000));

    if (enableLargeDupAck)
    {
        Config::SetDefault ("ns3::TcpSocketBase::ReTxThreshold", UintegerValue (1000));
    }

    if (enableLargeSynRetries)
    {
        Config::SetDefault ("ns3::TcpSocket::ConnCount", UintegerValue (10000));
    }

    if (enableLargeDataRetries)
    {
        Config::SetDefault ("ns3::TcpSocket::DataRetries", UintegerValue (10000));
    }

    NodeContainer spines;
    spines.Create (SPINE_COUNT);
    NodeContainer leaves;
    leaves.Create (LEAF_COUNT);
    NodeContainer servers;
    servers.Create (SERVER_COUNT * LEAF_COUNT);

    NS_LOG_INFO ("Install Internet stacks");
    InternetStackHelper internet;
    Ipv4StaticRoutingHelper staticRoutingHelper;
    Ipv4GlobalRoutingHelper globalRoutingHelper;
    Ipv4ListRoutingHelper listRoutingHelper;

    srand((unsigned)time(NULL));//使得每次启动程序的随机数不同
    NS_LOG_INFO("Enabling Per Flow WCMP");
    internet.SetRoutingHelper (globalRoutingHelper);
    Config::SetDefault ("ns3::Ipv4GlobalRouting::PerflowWcmpRouting", BooleanValue(true));
    Config::SetDefault ("ns3::Ipv4GlobalRouting::WeightFile", StringValue(weightFileName));
    internet.Install (servers);
    internet.Install (spines);
    internet.Install (leaves);   

    NS_LOG_INFO ("Install channels and assign addresses");

    PointToPointHelper p2p;
    Ipv4AddressHelper ipv4;

    TrafficControlHelper tc;

    NS_LOG_INFO ("Configuring servers");
    // Setting servers
    p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (LEAF_SERVER_CAPACITY)));
    p2p.SetChannelAttribute ("Delay", TimeValue(LINK_LATENCY));
    if (transportProt.compare ("Tcp") == 0)
    {
     	p2p.SetQueue ("ns3::DropTailQueue", "MaxPackets", UintegerValue (BUFFER_SIZE));
    }
    else
    {
	p2p.SetQueue ("ns3::DropTailQueue", "MaxPackets", UintegerValue (10));
    }

    ipv4.SetBase ("10.1.0.0", "255.255.255.0");

    std::vector<Ipv4Address> leafNetworks (LEAF_COUNT);

    std::vector<Ipv4Address> serverAddresses (SERVER_COUNT * LEAF_COUNT);

    std::map<std::pair<int, int>, uint32_t> leafToSpinePath;
    std::map<std::pair<int, int>, uint32_t> spineToLeafPath;

    for (int i = 0; i < LEAF_COUNT; i++)
    {
	Ipv4Address network = ipv4.NewNetwork ();
        leafNetworks[i] = network;

        for (int j = 0; j < SERVER_COUNT; j++)
        {
            int serverIndex = i * SERVER_COUNT + j;
            NodeContainer nodeContainer = NodeContainer (leaves.Get (i), servers.Get (serverIndex));
            NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);

            Ipv4InterfaceContainer interfaceContainer = ipv4.Assign (netDeviceContainer);
/*
            NS_LOG_INFO ("Leaf - " << i << " is connected to Server - " << j << " with address "
                    << interfaceContainer.GetAddress(0) << " <-> " << interfaceContainer.GetAddress (1)
                    << " with port " << netDeviceContainer.Get (0)->GetIfIndex () << " <-> " << netDeviceContainer.Get (1)->GetIfIndex ());
*/
            //std::cout<<serverIndex<<" "<<interfaceContainer.GetAddress(1)<<std::endl;
            serverAddresses [serverIndex] = interfaceContainer.GetAddress (1);
            if (transportProt.compare ("Tcp") == 0)
            {
                tc.Uninstall (netDeviceContainer);
            }
        }
    }

    NS_LOG_INFO ("Configuring switches");
    // Setting up switches
    p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (SPINE_LEAF_CAPACITY)));
    std::set<std::pair<uint32_t, uint32_t> > asymLink; // set< (A, B) > Leaf A -> Spine B is asymmetric

    int aleaf[13] =  {0,1,2,2,2,3,3,4,4,5,6,6,7};
    int aspine[13] = {5,1,2,6,7,0,1,0,3,3,0,6,1};   

    for (int i = 0; i < LEAF_COUNT; i++)
    {
        for (int j = 0; j < SPINE_COUNT; j++)
        {

            for (int l = 0; l < LINK_COUNT; l++)
            {
                bool isAsymCapacity = false;

                if (asymCapacity && static_cast<uint32_t> (rand () % 100) < asymCapacityPoss)
                {
             	   //isAsymCapacity = true;
            	}
                if(asymCapacity){
                    for(int k=0;k<13;k++){
                        if(aleaf[k]==i && aspine[k]==j){
                            isAsymCapacity = true;
                            break;
                         }
                     }
                }	
                
                // TODO
            	uint64_t spineLeafCapacity = SPINE_LEAF_CAPACITY;

            	if (isAsymCapacity)
            	{
              	    spineLeafCapacity = SPINE_LEAF_CAPACITY / 5;
               	    asymLink.insert (std::make_pair (i, j));
               	    asymLink.insert (std::make_pair (j, i));
                    std::cout<<i<<" "<<j<<std::endl;
             	 }

            	p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (spineLeafCapacity)));
            	ipv4.NewNetwork ();

            	NodeContainer nodeContainer = NodeContainer (leaves.Get (i), spines.Get (j));
                NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);
/*	        if (transportProt.compare ("DcTcp") == 0)
                {
			NS_LOG_INFO ("Install RED Queue for leaf: " << i << " and spine: " << j);
                	if (enableRandomDrop)
                	{
                   	    if (j == 0)
                    	    {
                            	Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (0.0));
                            	tc.Install (netDeviceContainer.Get (0)); // Leaf to Spine Queue
                            	Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (randomDropRate));
                            	tc.Install (netDeviceContainer.Get (1)); // Spine to Leaf Queue
                             }
                             else
                             {
                                 Config::SetDefault ("ns3::RedQueueDisc::DropRate", DoubleValue (0.0));
	                         tc.Install (netDeviceContainer);
                             }
                         }
                         else if (blackHoleMode != 0)
                         {
                             if (j == 0)
                             {
                                 Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (0));
                        	 tc.Install (netDeviceContainer.Get (0)); // Leaf to Spine Queue
                        	 Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (blackHoleMode));
                        	 tc.Install (netDeviceContainer.Get (1)); // Spine to Leaf Queue
                        	 Ptr<TrafficControlLayer> tc = netDeviceContainer.Get (1)->GetNode ()->GetObject<TrafficControlLayer> ();
                       	 	 Ptr<QueueDisc> queueDisc = tc->GetRootQueueDiscOnDevice (netDeviceContainer.Get (1));
                       		 Ptr<RedQueueDisc> redQueueDisc = DynamicCast<RedQueueDisc> (queueDisc);
                       		 redQueueDisc->SetBlackHoleSrc (blackHoleSrcAddr, blackHoleSrcMask);
                       		 redQueueDisc->SetBlackHoleDest (blackHoleDestAddr, blackHoleDestMask);
                    	      }
                              else
                              {
                                 Config::SetDefault ("ns3::RedQueueDisc::BlackHole", UintegerValue (0));
                        	 tc.Install (netDeviceContainer);
                               }
                          }
                          else
                          {
	                  	 tc.Install (netDeviceContainer);
                	  }
                 }*/
            	 Ipv4InterfaceContainer ipv4InterfaceContainer = ipv4.Assign (netDeviceContainer);
/*
            	 NS_LOG_INFO ("Leaf - " << i << " is connected to Spine - " << j << " with address "
                    << ipv4InterfaceContainer.GetAddress(0) << " <-> " << ipv4InterfaceContainer.GetAddress (1)
                    << " with port " << netDeviceContainer.Get (0)->GetIfIndex () << " <-> " << netDeviceContainer.Get (1)->GetIfIndex ()
                    << " with data rate " << spineLeafCapacity);
*/

                 if (transportProt.compare ("Tcp") == 0)
                 {
                     tc.Uninstall (netDeviceContainer);
                 }
            }
        }
    }


    NS_LOG_INFO ("Populate global routing tables");
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    double oversubRatio = static_cast<double>(SERVER_COUNT * LEAF_SERVER_CAPACITY) / (SPINE_LEAF_CAPACITY * SPINE_COUNT * LINK_COUNT);
    NS_LOG_INFO ("Over-subscription ratio: " << oversubRatio);

    NS_LOG_INFO ("Initialize CDF table");
    struct cdf_table* cdfTable = new cdf_table ();
    init_cdf (cdfTable);
    load_cdf (cdfTable, cdfFileName.c_str ());

    NS_LOG_INFO ("Calculating request rate");
    double requestRate = load * LEAF_SERVER_CAPACITY * SERVER_COUNT / oversubRatio / (8 * avg_cdf (cdfTable)) / SERVER_COUNT;
    NS_LOG_INFO ("Average request rate: " << requestRate << " per second");

    NS_LOG_INFO ("Initialize random seed: " << randomSeed);
    if (randomSeed == 0)
    {
        srand ((unsigned)time (NULL));
    }
    else
    {
        srand (randomSeed);
    }

    NS_LOG_INFO ("Create applications");
 
    long flowCount = 0;
    long totalFlowSize = 0;

    for (int fromLeafId = 0; fromLeafId < LEAF_COUNT; fromLeafId ++)
    {
        install_applications(fromLeafId, servers, requestRate, cdfTable, flowCount, totalFlowSize, SERVER_COUNT, LEAF_COUNT, START_TIME, END_TIME, FLOW_LAUNCH_END_TIME, applicationPauseThresh, applicationPauseTime);
    }

    NS_LOG_INFO ("Total flow: " << flowCount);

    NS_LOG_INFO ("Actual average flow size: " << static_cast<double> (totalFlowSize) / flowCount);

    NS_LOG_INFO ("Enabling flow monitor");

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

    flowMonitor->CheckForLostPackets ();

    std::stringstream flowMonitorFilename;

    flowMonitorFilename << id << "-1-large-load-" << LEAF_COUNT << "X" << SPINE_COUNT << "-" << load << "-"  << transportProt <<"-";

    flowMonitorFilename << "wcmp-simulation-";

    flowMonitorFilename << randomSeed << "-";

    if (asymCapacity)
    {
        flowMonitorFilename << "capacity-asym-";
    }

    if (asymCapacity2)
    {
        flowMonitorFilename << "capacity-asym2-";
    }

    if (applicationPauseThresh > 0)
    {
        flowMonitorFilename << "p" << applicationPauseThresh << "-" << applicationPauseTime << "-";
    }

    if (enableRandomDrop)
    {
        flowMonitorFilename << "random-drop-" << randomDropRate << "-";
    }

    if (blackHoleMode != 0)
    {
        flowMonitorFilename << "black-hole-" << blackHoleMode << "-";
    }

    flowMonitorFilename << "b" << BUFFER_SIZE << ".xml";
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&SinkRx));

    // OpenGym Env
    Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (openGymPort);
    openGym->SetGetActionSpaceCb( MakeCallback (&MyGetActionSpace) );
    openGym->SetGetObservationSpaceCb( MakeCallback (&MyGetObservationSpace) );
    openGym->SetGetGameOverCb( MakeCallback (&MyGetGameOver) );
    openGym->SetGetObservationCb( MakeCallback (&MyGetObservation) );
    openGym->SetGetRewardCb( MakeCallback (&MyGetReward) );
    openGym->SetGetExtraInfoCb( MakeCallback (&MyGetExtraInfo) );
    openGym->SetExecuteActionsCb( MakeCallback (&MyExecuteActions) );
    Simulator::Schedule (Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, &flowHelper,flowMonitor,openGym);

    NS_LOG_INFO ("Start simulation");
    Simulator::Stop (Seconds (END_TIME));

    Simulator::Run ();

    flowMonitor->SerializeToXmlFile(flowMonitorFilename.str (), true, true);
    openGym->NotifySimulationEnd();
    Simulator::Destroy ();
    free_cdf (cdfTable);
    NS_LOG_INFO ("Stop simulation");
}
