#include "ns3/basic-energy-source-helper.h"
#include "ns3/basic-energy-source.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/zigbee-module.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace ns3;
using namespace ns3::energy;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

namespace
{

NS_LOG_COMPONENT_DEFINE ("PolykarposSimulation");

struct SimulationConfig
{
  uint32_t routers = 3;
  uint32_t endDevices = 6;
  uint32_t payloadSize = 64;
  double duration = 1000.0;
  std::string trafficModel = "poisson";
  std::string outputPath = ".";
  uint64_t seed = static_cast<uint64_t> (time (nullptr));
  int64_t run = 1;
  bool enableRouteDiscovery = false;
  double startTrafficAfterSeconds = 35.0;
  double jitterSeconds = 7.0;
  double statsIntervalSeconds = 0.0;
};

struct FlowKey
{
  uint32_t src;
  uint32_t dst;
  bool operator< (const FlowKey &other) const
  {
    return std::tie (src, dst) < std::tie (other.src, other.dst);
  }
};

class DeviceEnergyController
{
public:
  explicit DeviceEnergyController (Ptr<SimpleDeviceEnergyModel> model)
      : m_model (model)
  {
    if (m_model)
      {
        m_model->SetCurrentA (m_idleCurrent);
      }
    m_state = State::Idle;
  }

  void EnterTx ()
  {
    TransitionIfNeeded (State::Transmit);
  }

  void EnterRx ()
  {
    TransitionIfNeeded (State::Receive);
  }

  void EnterIdle ()
  {
    TransitionIfNeeded (State::Idle);
  }

  void EnterSleep ()
  {
    TransitionIfNeeded (State::Sleep);
  }

private:
  enum class State
  {
    Idle,
    Transmit,
    Receive,
    Sleep
  };

  void TransitionIfNeeded (State next)
  {
    if (m_state == next)
      {
        return;
      }

    switch (next)
      {
      case State::Idle:
        m_model->SetCurrentA (m_idleCurrent);
        break;
      case State::Transmit:
        m_model->SetCurrentA (m_txCurrent);
        break;
      case State::Receive:
        m_model->SetCurrentA (m_rxCurrent);
        break;
      case State::Sleep:
        m_model->SetCurrentA (m_sleepCurrent);
        break;
      }

    m_state = next;
  }

  Ptr<SimpleDeviceEnergyModel> m_model;
  State m_state {State::Idle};

  static constexpr double m_txCurrent = 0.0174;      // 17.4 mA
  static constexpr double m_rxCurrent = 0.0188;      // 18.8 mA
  static constexpr double m_idleCurrent = 0.000426;  // 0.426 mA
  static constexpr double m_sleepCurrent = 0.00002;  // 0.02 mA
};

class StatisticsCollector
{
public:
  void RegisterSend (uint32_t node)
  {
    ++m_totalSent;
    ++m_nodeSent[node];
  }

  void RegisterReceive (uint32_t node)
  {
    ++m_totalReceived;
    ++m_nodeReceived[node];
  }

  void RegisterDrop ()
  {
    ++m_totalDropped;
  }

  void DumpFinalStats (uint32_t totalNodes, const std::string &filePath) const
  {
    std::ofstream out (filePath);
    out << "node_id,sent_packets,received_packets" << std::endl;
    for (uint32_t i = 0; i < totalNodes; ++i)
      {
        auto sent = GetOrZero (m_nodeSent, i);
        auto recv = GetOrZero (m_nodeReceived, i);
        out << i << "," << sent << "," << recv << std::endl;
      }

    out.flush ();

    std::cout << "\n=== PACKET STATISTICS ===" << std::endl;
    std::cout << "Total packets sent: " << m_totalSent << std::endl;
    std::cout << "Total packets received: " << m_totalReceived << std::endl;
    std::cout << "Total packets dropped: " << m_totalDropped << std::endl;
    double deliveryRatio = m_totalSent > 0 ? (100.0 * m_totalReceived / m_totalSent) : 0.0;
    std::cout << "Packet delivery ratio: " << std::fixed << std::setprecision (2) << deliveryRatio << "%" << std::endl;
  }

  void WriteSnapshot (std::ostream &out, double nowSeconds) const
  {
    out << std::setprecision (std::numeric_limits<double>::max_digits10)
        << nowSeconds << "," << m_totalSent << "," << m_totalReceived << "," << m_totalDropped << std::endl;
  }

  uint64_t GetTotalSent () const { return m_totalSent; }
  uint64_t GetTotalReceived () const { return m_totalReceived; }
  uint64_t GetTotalDropped () const { return m_totalDropped; }

private:
  template <typename Map>
  static uint64_t GetOrZero (const Map &m, uint32_t key)
  {
    auto it = m.find (key);
    return it == m.end () ? 0 : it->second;
  }

  uint64_t m_totalSent {0};
  uint64_t m_totalReceived {0};
  uint64_t m_totalDropped {0};
  std::map<uint32_t, uint64_t> m_nodeSent;
  std::map<uint32_t, uint64_t> m_nodeReceived;
};

class TrafficManager
{
public:
  TrafficManager (const SimulationConfig &config,
                  ZigbeeStackContainer *container,
                  StatisticsCollector *stats)
      : m_config (config), m_container (container), m_stats (stats)
  {
  }

  void Initialize ()
  {
    BuildFlowTable ();
  }

  void OnNodeJoined (uint32_t nodeId)
  {
    auto range = m_flows.equal_range (nodeId);
    for (auto it = range.first; it != range.second; ++it)
      {
        const auto &flow = it->second;
        Time baseDelay = Seconds (m_config.startTrafficAfterSeconds);
        if (m_config.trafficModel == "event")
          {
            Ptr<UniformRandomVariable> joinRv = CreateObject<UniformRandomVariable> ();
            joinRv->SetAttribute ("Min", DoubleValue (2.0));
            joinRv->SetAttribute ("Max", DoubleValue (10.0));
            joinRv->SetStream (AllocateStream (nodeId, nodeId + 0x8000));
            baseDelay = Seconds (joinRv->GetValue ());
          }
        Time initialDelay = baseDelay + flow.intervalGenerator ();
        ScheduleNextTransmission (flow, initialDelay);
      }
  }

  void SendPacket (FlowDefinition flow)
  {
    auto srcStack = GetStack (flow.srcNode);
    uint32_t dstNode = flow.dstSelector ? flow.dstSelector () : flow.dstNode;
    auto dstStack = GetStack (dstNode);
    if (srcStack == nullptr || dstStack == nullptr)
      {
        return;
      }

    uint32_t payloadSize = std::max (1u, std::min (static_cast<uint32_t> (m_config.payloadSize), 127u));
    Ptr<Packet> packet = Create<Packet> (payloadSize);

    NldeDataRequestParams params;
    params.m_dstAddrMode = UCST_BCST;
    params.m_dstAddr = dstStack->GetNwk ()->GetNetworkAddress ();
    params.m_discoverRoute = m_config.enableRouteDiscovery ? ENABLE_ROUTE_DISCOVERY : DISABLE_ROUTE_DISCOVERY;

    uint32_t handle = m_nextHandle.fetch_add (1, std::memory_order_relaxed);
    params.m_nsduHandle = handle;

    m_inflight[handle] = {flow.srcNode, dstNode, payloadSize, Simulator::Now ()};

    m_stats->RegisterSend (flow.srcNode);
    ++m_txCount[flow.srcNode];
    UpdateActivityPenalty (flow.srcNode);

    NS_LOG_INFO ("Node " << flow.srcNode << " sending to " << dstNode);

    Simulator::ScheduleNow (&ZigbeeNwk::NldeDataRequest, srcStack->GetNwk (), params, packet);

    ScheduleNextTransmission (flow, flow.intervalGenerator ());
  }

  struct FlowDefinition
  {
    uint32_t srcNode;
    uint32_t dstNode;
    std::function<uint32_t ()> dstSelector;
    Ptr<RandomVariableStream> rv;
    std::function<Time ()> intervalGenerator;
  };

  struct HandleInfo
  {
    uint32_t src {0};
    uint32_t dst {0};
    uint32_t size {0};
    Time sentAt {Seconds (0)};
  };

  HandleInfo PopHandleInfo (uint32_t handle)
  {
    auto it = m_inflight.find (handle);
    if (it == m_inflight.end ())
      {
        return {};
      }
    HandleInfo info = it->second;
    m_inflight.erase (it);
    return info;
  }

private:
  void BuildFlowTable ()
  {
    uint32_t totalNodes = m_config.routers + m_config.endDevices + 1;

    auto addFlow = [&](uint32_t src,
                       uint32_t dst,
                       Ptr<RandomVariableStream> rv,
                       std::function<Time ()> intervalFn) {
      FlowDefinition flow {src, dst, nullptr, rv, std::move (intervalFn)};
      m_flows.emplace (src, flow);
    };

    auto addFlowWithSelector = [&](uint32_t src,
                                   Ptr<RandomVariableStream> rv,
                                   std::function<uint32_t ()> selector,
                                   std::function<Time ()> intervalFn) {
      FlowDefinition flow {src, 0, selector, rv, std::move (intervalFn)};
      m_flows.emplace (src, flow);
    };

    auto registerPoisson = [&](uint32_t src, const std::vector<uint32_t> &dests, double mean) {
      for (uint32_t dst : dests)
        {
          Ptr<ExponentialRandomVariable> rv = CreateObject<ExponentialRandomVariable> ();
          rv->SetAttribute ("Mean", DoubleValue (mean));
          rv->SetStream (AllocateStream (src, dst));
          addFlow (src, dst, rv, [rv]() { return Seconds (rv->GetValue ()); });
        }
    };

    auto registerAdaptive = [&](uint32_t src, const std::vector<uint32_t> &dests, double baseInterval) {
      for (uint32_t dst : dests)
        {
          Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
          rv->SetAttribute ("Min", DoubleValue (-baseInterval * 0.3));
          rv->SetAttribute ("Max", DoubleValue (baseInterval * 0.3));
          rv->SetStream (AllocateStream (src, dst));
          addFlow (src, dst, rv, [this, rv, baseInterval, src]() {
            double jitter = rv->GetValue ();
            double interval = std::max (1.0, baseInterval + jitter + m_activityPenalty[src]);
            return Seconds (interval);
          });
        }
    };

    auto registerLoadBased = [&](uint32_t node) {
      Ptr<NormalRandomVariable> intervalRv = CreateObject<NormalRandomVariable> ();
      intervalRv->SetAttribute ("Mean", DoubleValue (20.0));
      intervalRv->SetAttribute ("Variance", DoubleValue (8.0));
      intervalRv->SetStream (AllocateStream (node, 0));

      Ptr<UniformRandomVariable> destRv = CreateObject<UniformRandomVariable> ();
      destRv->SetAttribute ("Min", DoubleValue (0.0));
      destRv->SetAttribute ("Max", DoubleValue (1.0));
      destRv->SetStream (AllocateStream (node, totalNodes + 1));

      auto selector = [this, destRv, node]() {
        double roll = destRv->GetValue ();
        if (IsCoordinator (node))
          {
            uint32_t router = 1 + static_cast<uint32_t> (roll * std::max (1u, m_config.routers));
            router = std::min (router, m_config.routers);
            return router;
          }
        if (IsRouter (node))
          {
            if (roll < 0.2)
              {
                uint32_t router = 1 + static_cast<uint32_t> (roll * m_config.routers);
                if (router == node)
                  {
                    return 0u;
                  }
                return std::min (router, m_config.routers);
              }
            return 0u;
          }

        // End device
        uint32_t parent = 1 + ((node - m_config.routers - 1) % m_config.routers);
        return roll < 0.4 ? parent : 0u;
      };

      addFlowWithSelector (node, intervalRv, selector, [this, intervalRv, node]() {
        double draw = std::max (2.0, std::min (intervalRv->GetValue (), 60.0));
        return Seconds (draw * m_activityPenalty[node]);
      });
    };

    for (uint32_t node = 0; node < totalNodes; ++node)
      {
        m_activityPenalty[node] = 1.0;
      }

    auto configurePoisson = [&]() {
      for (uint32_t src = 0; src < totalNodes; ++src)
        {
          std::vector<uint32_t> dests;
          double mean = 20.0;
          if (IsCoordinator (src))
            {
              mean = 60.0;
              for (uint32_t i = 1; i <= m_config.routers; ++i)
                {
                  dests.push_back (i);
                }
            }
          else if (IsRouter (src))
            {
              mean = 45.0;
              dests.push_back (0);
            }
          else
            {
              mean = 20.0;
              dests.push_back (0);
              uint32_t parent = 1 + ((src - m_config.routers - 1) % m_config.routers);
              dests.push_back (parent);
            }
          registerPoisson (src, dests, mean);
        }
    };

    if (m_config.trafficModel == "poisson")
      {
        configurePoisson ();
      }
    else if (m_config.trafficModel == "adaptive")
      {
        for (uint32_t src = 0; src < totalNodes; ++src)
          {
            double base = IsCoordinator (src) ? 30.0 : (IsRouter (src) ? 25.0 : 15.0);
            std::vector<uint32_t> dests;
            if (IsCoordinator (src))
              {
                for (uint32_t i = 1; i <= m_config.routers; ++i)
                  {
                    dests.push_back (i);
                  }
              }
            else if (IsRouter (src))
              {
                dests.push_back (0);
              }
            else
              {
                dests.push_back (0);
                uint32_t parent = 1 + ((src - m_config.routers - 1) % m_config.routers);
                dests.push_back (parent);
              }
            registerAdaptive (src, dests, base);
          }
      }
    else if (m_config.trafficModel == "loadbased")
      {
        for (uint32_t node = 0; node < totalNodes; ++node)
          {
            registerLoadBased (node);
          }
      }
    else if (m_config.trafficModel == "original")
      {
        for (uint32_t src = 0; src < totalNodes; ++src)
          {
            for (uint32_t dst = 0; dst < totalNodes; ++dst)
              {
                if (src == dst)
                  {
                    continue;
                  }
                Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
                rv->SetAttribute ("Min", DoubleValue (m_config.jitterSeconds * -0.5));
                rv->SetAttribute ("Max", DoubleValue (m_config.jitterSeconds * 0.5));
                rv->SetStream (AllocateStream (src, dst));
                double baseInterval = 15.0;
                addFlow (src, dst, rv, [rv, baseInterval]() {
                  double interval = baseInterval + rv->GetValue ();
                  return Seconds (std::max (1.0, interval));
                });
              }
          }
      }
    else if (m_config.trafficModel == "event")
      {
        // Flows are attached upon join via the same logic as adaptive devices.
        for (uint32_t src = 0; src < totalNodes; ++src)
          {
            double base = IsCoordinator (src) ? 15.0 : (IsRouter (src) ? 20.0 : 30.0);
            std::vector<uint32_t> dests;
            if (IsCoordinator (src))
              {
                for (uint32_t i = 1; i < totalNodes; ++i)
                  {
                    dests.push_back (i);
                  }
              }
            else
              {
                dests.push_back (0);
              }
            registerAdaptive (src, dests, base);
          }
      }
    else
      {
        NS_LOG_WARN ("Unknown traffic model " << m_config.trafficModel << ", falling back to poisson");
        configurePoisson ();
      }
  }

  void ScheduleNextTransmission (const FlowDefinition &flow, Time interval)
  {
    if (interval.IsZero ())
      {
        interval = Seconds (1.0);
      }
    Simulator::Schedule (interval, &TrafficManager::SendPacket, this, flow);
  }

  void UpdateActivityPenalty (uint32_t node)
  {
    uint32_t count = m_txCount[node];
    double penalty = 1.0;
    if (count > 80)
      {
        penalty = IsRouter (node) || IsCoordinator (node) ? 3.0 : 2.5;
      }
    else if (count > 40)
      {
        penalty = IsRouter (node) || IsCoordinator (node) ? 2.0 : 1.8;
      }
    else if (count > 20)
      {
        penalty = 1.5;
      }
    m_activityPenalty[node] = penalty;
  }

  Ptr<ZigbeeStack> GetStack (uint32_t nodeId) const
  {
    if (!m_container)
      {
        return nullptr;
      }

    auto begin = m_container->Begin ();
    if (nodeId >= m_container->GetN ())
      {
        return nullptr;
      }
    auto it = begin;
    std::advance (it, nodeId);
    return *it;
  }

  uint32_t AllocateStream (uint32_t src, uint32_t dst)
  {
    FlowKey key {src, dst};
    auto it = m_flowStreams.find (key);
    if (it != m_flowStreams.end ())
      {
        return it->second;
      }
    uint32_t stream = m_nextStream++;
    m_flowStreams[key] = stream;
    return stream;
  }

  bool IsCoordinator (uint32_t node) const
  {
    return node == 0;
  }

  bool IsRouter (uint32_t node) const
  {
    return node > 0 && node <= m_config.routers;
  }

  const SimulationConfig &m_config;
  ZigbeeStackContainer *m_container;
  StatisticsCollector *m_stats;

  std::multimap<uint32_t, FlowDefinition> m_flows;
  std::map<FlowKey, uint32_t> m_flowStreams;
  std::map<uint32_t, double> m_activityPenalty;
  std::map<uint32_t, HandleInfo> m_inflight;
  std::map<uint32_t, uint32_t> m_txCount;
  std::atomic<uint32_t> m_nextHandle {1};
  uint32_t m_nextStream {1};
};

class PolykarposSimulation
{
public:
  explicit PolykarposSimulation (SimulationConfig config)
      : m_config (std::move (config)), m_traffic (m_config, &m_zigbeeStacks, &m_statistics)
  {
  }

  void Run ()
  {
    ConfigureSeed ();
    PrepareOutputDirectory ();
    CreateNodes ();
    InstallMobility ();
    InstallDevices ();
    InstallEnergyModels ();
    InstallZigbeeStacks ();
    m_traffic.Initialize ();

    SetupCallbacks ();
    ScheduleNetworkProcedures ();
    ScheduleStatisticsSampling ();

    Simulator::Stop (Seconds (m_config.duration));
    Simulator::Run ();
    Simulator::Destroy ();

    m_statistics.DumpFinalStats (m_totalNodes, m_config.outputPath + "/packet_statistics.csv");
    if (m_energyCsv.is_open ())
      {
        m_energyCsv.close ();
      }
    if (m_statsTimeseries.is_open ())
      {
        m_statsTimeseries.close ();
      }
  }

private:
  void ConfigureSeed () const
  {
    RngSeedManager::SetSeed (m_config.seed);
    RngSeedManager::SetRun (m_config.run);
  }

  void PrepareOutputDirectory ()
  {
    std::filesystem::create_directories (m_config.outputPath);

    m_energyCsv.open (m_config.outputPath + "/energy_ns3_log.csv", std::ios::out | std::ios::trunc);
    if (!m_energyCsv)
      {
        throw std::runtime_error ("Failed to open energy log file in output directory");
      }
    m_energyCsv << "time_s,node,remaining_j" << std::endl;

    if (m_config.statsIntervalSeconds > 0.0)
      {
        m_statsTimeseries.open (m_config.outputPath + "/traffic_timeseries.csv", std::ios::out | std::ios::trunc);
        if (!m_statsTimeseries)
          {
            throw std::runtime_error ("Failed to open traffic timeseries log file");
          }
        m_statsTimeseries << "time_s,total_sent,total_received,total_dropped" << std::endl;
        m_statistics.WriteSnapshot (m_statsTimeseries, 0.0);
        m_statsTimeseries.flush ();
      }

    WriteMetadataFile ();
  }

  void CreateNodes ()
  {
    m_totalNodes = m_config.routers + m_config.endDevices + 1;
    m_nodes.Create (m_totalNodes);
  }

  void WriteMetadataFile () const
  {
    std::ofstream meta (m_config.outputPath + "/simulation_metadata.json", std::ios::out | std::ios::trunc);
    if (!meta)
      {
        throw std::runtime_error ("Failed to open simulation metadata file");
      }

    meta << "{\n";
    meta << "  \"routers\": " << m_config.routers << ",\n";
    meta << "  \"endDevices\": " << m_config.endDevices << ",\n";
    meta << "  \"payloadSize\": " << m_config.payloadSize << ",\n";
    meta << "  \"duration\": " << m_config.duration << ",\n";
    meta << "  \"trafficModel\": \"" << m_config.trafficModel << "\",\n";
    meta << std::setprecision (std::numeric_limits<double>::max_digits10);
    meta << "  \"startTrafficAfterSeconds\": " << m_config.startTrafficAfterSeconds << ",\n";
    meta << "  \"jitterSeconds\": " << m_config.jitterSeconds << ",\n";
    meta << "  \"statsIntervalSeconds\": " << m_config.statsIntervalSeconds << ",\n";
    meta << "  \"seed\": " << m_config.seed << ",\n";
    meta << "  \"run\": " << m_config.run << "\n";
    meta << "}" << std::endl;
  }

  void InstallMobility ()
  {
    double centerX = 0.0;
    double centerY = 0.0;
    double rRouter = 20.0;
    double rEndAway = 6.0;
    double startAngleDeg = 0.0;

    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    allocator->Add (Vector (centerX, centerY, 0.0));

    for (uint32_t ri = 0; ri < m_config.routers; ++ri)
      {
        double angle = (2.0 * M_PI * ri / m_config.routers) + (startAngleDeg * M_PI / 180.0);
        double x = centerX + rRouter * std::cos (angle);
        double y = centerY + rRouter * std::sin (angle);
        allocator->Add (Vector (x, y, 0.0));
      }

    for (uint32_t ei = 0; ei < m_config.endDevices; ++ei)
      {
        uint32_t routerIdx = 1 + (ei % m_config.routers);
        double routerAngle = (2.0 * M_PI * (routerIdx - 1) / m_config.routers) + (startAngleDeg * M_PI / 180.0);
        double offsetAngle = (static_cast<double> (ei / m_config.routers) * 0.6) - 0.3;
        double angle = routerAngle + offsetAngle;

        double rx = centerX + rRouter * std::cos (routerAngle);
        double ry = centerY + rRouter * std::sin (routerAngle);
        double ex = rx + rEndAway * std::cos (angle);
        double ey = ry + rEndAway * std::sin (angle);
        allocator->Add (Vector (ex, ey, 0.0));
      }

    MobilityHelper mobility;
    mobility.SetPositionAllocator (allocator);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (m_nodes);
  }

  void InstallDevices ()
  {
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    Ptr<ConstantSpeedPropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
    channel->AddPropagationLossModel (loss);
    channel->SetPropagationDelayModel (delay);

    LrWpanHelper helper;
    helper.SetChannel (channel);
    m_lrwpanDevices = helper.Install (m_nodes);
    helper.SetExtendedAddresses (m_lrwpanDevices);
  }

  void InstallEnergyModels ()
  {
    BasicEnergySourceHelper sourceHelper;
    for (uint32_t i = 0; i < m_totalNodes; ++i)
      {
        double initialEnergy = 10800.0;
        if (i == 0)
          {
            initialEnergy = 43200.0;
          }
        else if (i <= m_config.routers)
          {
            initialEnergy = 21600.0;
          }
        sourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (initialEnergy));
        m_energySources.Add (sourceHelper.Install (NodeContainer (m_nodes.Get (i))));
      }

    for (uint32_t i = 0; i < m_totalNodes; ++i)
      {
        Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource> (m_energySources.Get (i));
        Ptr<SimpleDeviceEnergyModel> deviceModel = CreateObject<SimpleDeviceEnergyModel> ();
        deviceModel->SetNode (m_nodes.Get (i));
        deviceModel->SetEnergySource (source);
        source->AppendDeviceEnergyModel (deviceModel);
        m_energyModels.push_back (deviceModel);
        m_energyControllers.emplace_back (deviceModel);

        source->TraceConnect ("RemainingEnergy", std::to_string (i), MakeCallback (&PolykarposSimulation::OnRemainingEnergy, this));
      }
  }

  void InstallZigbeeStacks ()
  {
    ZigbeeHelper helper;
    m_zigbeeStacks = helper.Install (m_lrwpanDevices);
  }

  void SetupCallbacks ()
  {
    for (uint32_t i = 0; i < m_totalNodes; ++i)
      {
        Ptr<LrWpanNetDevice> device = m_lrwpanDevices.Get (i)->GetObject<LrWpanNetDevice> ();
        Ptr<LrWpanPhy> phy = device->GetPhy ();

        phy->TraceConnectWithoutContext ("PhyTxBegin", MakeBoundCallback (&PolykarposSimulation::OnPhyTxBegin, this, i));
        phy->TraceConnectWithoutContext ("PhyTxEnd", MakeBoundCallback (&PolykarposSimulation::OnPhyStateIdle, this, i));
        phy->TraceConnectWithoutContext ("PhyRxBegin", MakeBoundCallback (&PolykarposSimulation::OnPhyRxBegin, this, i));
        phy->TraceConnectWithoutContext ("PhyRxEnd", MakeBoundCallback (&PolykarposSimulation::OnPhyStateIdle, this, i));
        phy->TraceConnectWithoutContext ("PhyTxDrop", MakeBoundCallback (&PolykarposSimulation::OnPhyDrop, this, i));
        phy->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&PolykarposSimulation::OnPhyDrop, this, i));
      }

    for (auto it = m_zigbeeStacks.Begin (); it != m_zigbeeStacks.End (); ++it)
      {
        Ptr<ZigbeeStack> stack = *it;
        uint32_t index = std::distance (m_zigbeeStacks.Begin (), it);

        stack->GetNwk ()->SetNldeDataIndicationCallback (MakeBoundCallback (&PolykarposSimulation::OnDataIndication, this, stack));
        stack->GetNwk ()->SetNldeDataConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnDataConfirm, this, stack));
        stack->GetNwk ()->SetNlmeJoinConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnJoinConfirm, this, stack));
        stack->GetNwk ()->SetNlmeRouteDiscoveryConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnRouteDiscoveryConfirm, this, stack));

        if (index == 0)
          {
            stack->GetNwk ()->SetNlmeNetworkFormationConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnNetworkFormationConfirm, this, stack));
          }
        else if (index <= m_config.routers)
          {
            stack->GetNwk ()->SetNlmeNetworkDiscoveryConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnRouterDiscoveryConfirm, this, stack));
          }
        else
          {
            stack->GetNwk ()->SetNlmeNetworkDiscoveryConfirmCallback (MakeBoundCallback (&PolykarposSimulation::OnEndDeviceDiscoveryConfirm, this, stack));
          }
      }
  }

  void ScheduleNetworkProcedures ()
  {
    for (auto it = m_zigbeeStacks.Begin (); it != m_zigbeeStacks.End (); ++it)
      {
        Ptr<ZigbeeStack> stack = *it;
        uint32_t index = std::distance (m_zigbeeStacks.Begin (), it);

        if (index == 0)
          {
            NlmeNetworkFormationRequestParams params;
            params.m_scanChannelList.channelPageCount = 1;
            params.m_scanChannelList.channelsField[0] = 0x00007800;
            params.m_scanDuration = 2;
            params.m_superFrameOrder = 15;
            params.m_beaconOrder = 15;

            Simulator::ScheduleWithContext (stack->GetNode ()->GetId (), MilliSeconds (index * 500),
                                            &ZigbeeNwk::NlmeNetworkFormationRequest, stack->GetNwk (), params);
          }
        else if (index <= m_config.routers)
          {
            NlmeNetworkDiscoveryRequestParams params;
            params.m_scanChannelList.channelPageCount = 1;
            params.m_scanChannelList.channelsField[0] = 0x00007800;
            params.m_scanDuration = 2;

            Simulator::ScheduleWithContext (stack->GetNode ()->GetId (), Seconds (index * 2),
                                            &ZigbeeNwk::NlmeNetworkDiscoveryRequest, stack->GetNwk (), params);
          }
        else
          {
            NlmeNetworkDiscoveryRequestParams params;
            params.m_scanChannelList.channelPageCount = 1;
            params.m_scanChannelList.channelsField[0] = 0x00007800;
            params.m_scanDuration = 2;

            Simulator::ScheduleWithContext (stack->GetNode ()->GetId (), Seconds (10 + index * 2),
                                            &ZigbeeNwk::NlmeNetworkDiscoveryRequest, stack->GetNwk (), params);
          }
      }
  }

  void OnRemainingEnergy (std::string context, double oldValue, double remaining)
  {
    (void) oldValue;
    m_energyCsv << Simulator::Now ().GetSeconds () << "," << context << ","
                << std::setprecision (std::numeric_limits<double>::max_digits10) << remaining << std::endl;
    m_energyCsv.flush ();
  }

  void ScheduleStatisticsSampling ()
  {
    if (m_config.statsIntervalSeconds <= 0.0)
      {
        return;
      }

    Simulator::Schedule (Seconds (m_config.statsIntervalSeconds), &PolykarposSimulation::WriteStatisticsSnapshot, this);
  }

  void WriteStatisticsSnapshot ()
  {
    if (!m_statsTimeseries.is_open ())
      {
        return;
      }

    m_statistics.WriteSnapshot (m_statsTimeseries, Simulator::Now ().GetSeconds ());
    m_statsTimeseries.flush ();
    ScheduleStatisticsSampling ();
  }

  void OnPhyTxBegin (uint32_t nodeId, Ptr<const Packet>)
  {
    m_energyControllers[nodeId].EnterTx ();
  }

  void OnPhyRxBegin (uint32_t nodeId, Ptr<const Packet>)
  {
    m_energyControllers[nodeId].EnterRx ();
  }

  void OnPhyStateIdle (uint32_t nodeId, Ptr<const Packet>)
  {
    m_energyControllers[nodeId].EnterIdle ();
  }

  void OnPhyDrop (uint32_t nodeId, Ptr<const Packet>)
  {
    m_statistics.RegisterDrop ();
    m_energyControllers[nodeId].EnterIdle ();
  }

  void OnDataIndication (Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> packet)
  {
    uint32_t nodeId = stack->GetNode ()->GetId ();
    m_statistics.RegisterReceive (nodeId);
    NS_LOG_INFO (Simulator::Now ().GetSeconds () << "s Node " << nodeId
                                                 << " received packet size=" << packet->GetSize ());
  }

  void OnDataConfirm (Ptr<ZigbeeStack>, NldeDataConfirmParams params)
  {
    auto info = m_traffic.PopHandleInfo (params.m_nsduHandle);
    if (info.size == 0 && info.sentAt.IsZero ())
      {
        std::cout << Simulator::Now ().GetSeconds () << "s CONFIRM handle=" << params.m_nsduHandle
                  << " status=" << params.m_status << " (unknown handle)" << std::endl;
        return;
      }

    std::cout << Simulator::Now ().GetSeconds () << "s CONFIRM handle=" << params.m_nsduHandle
              << " status=" << params.m_status
              << " src=" << info.src
              << " dst=" << info.dst
              << " size=" << info.size
              << " sentAt=" << info.sentAt.GetSeconds () << "s" << std::endl;
  }

  void OnJoinConfirm (Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params)
  {
    if (params.m_status != NwkStatus::SUCCESS)
      {
        std::cout << "Node " << stack->GetNode ()->GetId () << " failed to join status=" << params.m_status << std::endl;
        return;
      }

    uint32_t nodeId = stack->GetNode ()->GetId ();
    std::cout << Simulator::Now ().GetSeconds () << "s Node " << nodeId << " joined with addr "
              << std::hex << params.m_networkAddress << std::dec << std::endl;

    m_traffic.OnNodeJoined (nodeId);

    if (nodeId != 0)
      {
        NlmeStartRouterRequestParams paramsStart;
        Simulator::ScheduleNow (&ZigbeeNwk::NlmeStartRouterRequest, stack->GetNwk (), paramsStart);
      }
  }

  void OnRouteDiscoveryConfirm (Ptr<ZigbeeStack>, NlmeRouteDiscoveryConfirmParams params)
  {
    std::cout << "Route discovery status=" << params.m_status << std::endl;
  }

  void OnNetworkFormationConfirm (Ptr<ZigbeeStack>, NlmeNetworkFormationConfirmParams)
  {
    std::cout << "Coordinator formed network at " << Simulator::Now ().GetSeconds () << "s" << std::endl;
    m_traffic.OnNodeJoined (0);
  }

  void OnRouterDiscoveryConfirm (Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params)
  {
    if (params.m_status == NwkStatus::SUCCESS)
      {
        NlmeJoinRequestParams join;
        CapabilityInformation info;
        info.SetDeviceType (ROUTER);
        info.SetAllocateAddrOn (true);
        join.m_capabilityInfo = info.GetCapability ();
        join.m_extendedPanId = params.m_netDescList[0].m_extPanId;
        join.m_rejoinNetwork = JoiningMethod::ASSOCIATION;
        Simulator::ScheduleNow (&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk (), join);
      }
  }

  void OnEndDeviceDiscoveryConfirm (Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params)
  {
    if (params.m_status == NwkStatus::SUCCESS)
      {
        NlmeJoinRequestParams join;
        CapabilityInformation info;
        info.SetDeviceType (ENDDEVICE);
        info.SetAllocateAddrOn (true);
        join.m_capabilityInfo = info.GetCapability ();
        join.m_extendedPanId = params.m_netDescList[0].m_extPanId;
        join.m_rejoinNetwork = JoiningMethod::ASSOCIATION;
        Simulator::ScheduleNow (&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk (), join);
      }
    else
      {
        std::cout << "End device " << stack->GetNode ()->GetId () << " failed to discover network status=" << params.m_status << std::endl;
      }
  }

  bool IsRouter (uint32_t node) const
  {
    return node > 0 && node <= m_config.routers;
  }

  bool IsCoordinator (uint32_t node) const
  {
    return node == 0;
  }

  SimulationConfig m_config;
  NodeContainer m_nodes;
  NetDeviceContainer m_lrwpanDevices;
  EnergySourceContainer m_energySources;
  std::vector<Ptr<SimpleDeviceEnergyModel>> m_energyModels;
  std::vector<DeviceEnergyController> m_energyControllers;
  ZigbeeStackContainer m_zigbeeStacks;
  StatisticsCollector m_statistics;
  TrafficManager m_traffic;
  uint32_t m_totalNodes {0};
  mutable std::ofstream m_energyCsv;
  mutable std::ofstream m_statsTimeseries;
};

SimulationConfig ParseArguments (int argc, char *argv[])
{
  SimulationConfig config;

  CommandLine cmd;
  cmd.AddValue ("router_number", "Number of routers", config.routers);
  cmd.AddValue ("device_number", "Number of end devices", config.endDevices);
  cmd.AddValue ("payload_size", "MAC payload size (1-127 bytes)", config.payloadSize);
  cmd.AddValue ("sim_duration", "Simulation duration", config.duration);
  cmd.AddValue ("traffic_model", "Traffic model", config.trafficModel);
  cmd.AddValue ("output_path", "Output directory", config.outputPath);
  cmd.AddValue ("seed", "RNG seed", config.seed);
  cmd.AddValue ("run", "RNG run index", config.run);
  cmd.AddValue ("enable_route_discovery", "Enable automatic route discovery", config.enableRouteDiscovery);
  cmd.AddValue ("start_traffic_after", "Seconds before first traffic", config.startTrafficAfterSeconds);
  cmd.AddValue ("jitter_seconds", "Traffic jitter window", config.jitterSeconds);
  cmd.AddValue ("stats_interval", "Sampling interval for aggregate traffic stats", config.statsIntervalSeconds);
  cmd.Parse (argc, argv);

  config.payloadSize = std::max (1u, std::min (config.payloadSize, 127u));
  config.statsIntervalSeconds = std::max (0.0, config.statsIntervalSeconds);
  config.startTrafficAfterSeconds = std::max (0.0, config.startTrafficAfterSeconds);
  config.jitterSeconds = std::max (0.0, config.jitterSeconds);

  return config;
}

} // namespace

int
main (int argc, char *argv[])
{
  auto config = ParseArguments (argc, argv);
  PolykarposSimulation simulation (config);
  simulation.Run ();
  return 0;
}
