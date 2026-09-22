// Manus Integrated SDK world-space hand skeleton collector (meters).
//
// stdout protocol for manus_bridge.manus_hand_input (one record per line):
//   HAND <gloveId hex8> <Left|Right> <nodeCount>
//   NODE <gloveId> <arrayIndex> <nodeId> <parentId> <chainType> <side> <fingerJointType>
//   EDGE <gloveId hex8> <childNodeId> <parentNodeId> <chainType>
//   POSE <gloveId> <seq> <source_monotonic_ns> <sdk_publish_time>
//        <x0> <y0> <z0> <qw0> <qx0> <qy0> <qz0> ...
//        (每手套独立 seq；source_monotonic_ns 在 SDK 回调入口采集)
//
// 编译：使用本包 build.sh（系统工具链，链接 vendor/manus_sdk）。
//   ./build.sh           # 产出 <package>/rawviz，SDK 位于 <workspace>/vendor/manus_sdk

#include <cstdio>
#include <cstring>
#include <csignal>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <deque>
#include <condition_variable>
#include <limits.h>

#include "ManusSDK.h"

// 节点数上限:正常 25 节点/手套;恶意/损坏 dongle 上报超大计数时拒绝分配
static const uint32_t kMaxNodesPerGlove = 64;
// 校准文件大小上限(正常 ~几 KB;防被替换为超大文件触发大分配)
static const long kMaxCalibrationBytes = 4 * 1024 * 1024;
// 每手套帧队列容量:100Hz 下约 160ms 缓冲,吸收下游瞬时阻塞而不丢帧;
// 持续过载时丢最旧帧保实时性,内存与延迟有界
static const size_t kMaxQueuedFrames = 16;

// The launcher explicitly selects a user and that user's profiles/<user>/manus directory.
static std::string g_CalibrationUser;
static std::string g_CalibrationDirectory;

// Preserve the per-user filenames inside the explicitly selected directory.
static void BuildCalibrationPath(const char* t_Base, char* t_Out, size_t t_OutSize)
{
	snprintf(t_Out, t_OutSize, "%s/%s%s", g_CalibrationDirectory.c_str(),
	         g_CalibrationUser.c_str(), t_Base);
}

// ---------------------------------------------------------------------------
// 帧队列缓存（SDK 回调线程入队，主循环出队）——按手套独立队列,
// 每手套自己的 seq(左右手解耦:一只遮挡不影响另一只)。
// 队列有界(kMaxQueuedFrames):吸收下游瞬时阻塞而不丢帧;
// 持续过载时丢最旧帧保实时性,内存与延迟有界。
// ---------------------------------------------------------------------------
struct NodePose
{
	uint32_t id;
	float x, y, z;
	float qw, qx, qy, qz;
};
struct GloveStream
{
	uint64_t seq = 0;
	uint64_t sourceMonotonicNs = 0;
	uint64_t sdkPublishTime = 0;
	std::vector<NodePose> nodes;
};
struct GloveQueue
{
	std::deque<GloveStream> frames;   // 待输出帧(先进先出)
	uint64_t lastSeq = 0;             // 已入队最大 seq(队列清空后继续自增)
};

static std::mutex g_FrameMutex;
static std::condition_variable g_FrameCV;
static std::map<uint32_t, GloveQueue> g_FrameQueues;   // gloveId -> 帧队列
static std::atomic<bool> g_Running{ true };

// Only the main output thread owns topology. IDs, never SDK array positions,
// associate each frame with the once-emitted NODE array order.
struct GloveTopology
{
	std::vector<NodeInfo> nodes;
	std::map<uint32_t, uint32_t> indexById;
};
static std::map<uint32_t, GloveTopology> g_Topology;

static bool OrderFrameNodes(const GloveTopology& p_Topology, const GloveStream& p_Stream,
                           std::array<const NodePose*, kMaxNodesPerGlove>& p_Ordered)
{
	if (p_Stream.nodes.size() != p_Topology.nodes.size()) return false;
	p_Ordered.fill(nullptr);
	for (const NodePose& t_Node : p_Stream.nodes)
	{
		const auto t_Index = p_Topology.indexById.find(t_Node.id);
		if (t_Index == p_Topology.indexById.end() || p_Ordered[t_Index->second])
			return false;
		p_Ordered[t_Index->second] = &t_Node;
	}
	// Unique IDs and equal counts guarantee every metadata slot is present.
	return true;
}

// ---------------------------------------------------------------------------
// 校准文件加载（仿 wuji-hand-teleop：CoreSdk_SetGloveCalibration）
// 校准显著改善骨架质量；个人文件位于工作区 profiles/<user>/manus。
// ---------------------------------------------------------------------------
static void LoadCalibration(uint32_t p_GloveId, uint32_t p_Side)
{
	const char* t_Base = (p_Side == Side_Left) ? "LeftMetaglovePro.mcal"
	                                           : "RightMetaglovePro.mcal";
	// Both the user and directory are required before starting the SDK.
	char t_Path[PATH_MAX * 2];
	BuildCalibrationPath(t_Base, t_Path, sizeof(t_Path));

	FILE* t_File = fopen(t_Path, "rb");
	if (!t_File)
	{
		fprintf(stderr, "[rawviz] 校准文件不存在: %s(跳过)\n", t_Path);
		return;
	}
	fseek(t_File, 0, SEEK_END);
	long t_Length = ftell(t_File);
	fseek(t_File, 0, SEEK_SET);
	if (t_Length <= 0)
	{
		fprintf(stderr, "[rawviz] 校准文件为空: %s\n", t_Path);
		fclose(t_File);
		return;
	}
	if (t_Length > kMaxCalibrationBytes)
	{
		fprintf(stderr, "[rawviz] 校准文件过大(%ld B,上限 %ld B),跳过: %s\n",
		        t_Length, kMaxCalibrationBytes, t_Path);
		fclose(t_File);
		return;
	}
	std::vector<unsigned char> t_Data(t_Length);
	if (fread(t_Data.data(), 1, t_Length, t_File) != (size_t)t_Length)
	{
		fprintf(stderr, "[rawviz] 校准文件读取失败: %s\n", t_Path);
		fclose(t_File);
		return;
	}
	fclose(t_File);

	SetGloveCalibrationReturnCode t_Result;
	// API 返回 SDKReturnCode(调用是否成功),实际结果在 t_Result 中
	if (CoreSdk_SetGloveCalibration(p_GloveId, t_Data.data(), (int)t_Length, &t_Result)
	    == SDKReturnCode_Success && t_Result == SetGloveCalibrationReturnCode_Success)
	{
		fprintf(stderr, "[rawviz] 校准已加载: %s (glove %08X)\n", t_Path, p_GloveId);
	}
	else
	{
		fprintf(stderr, "[rawviz] 校准加载失败 %s: %d\n", t_Path, (int)t_Result);
	}
}

// ---------------------------------------------------------------------------
// 信号处理：干净退出（先关 SDK 再退出，避免 dongle 停在异常状态）
// ---------------------------------------------------------------------------
static void HandleSignal(int)
{
	g_Running = false;
}

// ---------------------------------------------------------------------------
// Raw skeleton 流回调：把最新帧拷进缓存
// ---------------------------------------------------------------------------
static void OnRawSkeletonStream(const SkeletonStreamInfo* const p_Info)
{
	if (!p_Info) return;
	const uint64_t t_SourceMonotonicNs = (uint64_t)std::chrono::duration_cast<
		std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();

	uint32_t t_Dropped = 0;               // 本次回调因队列满丢弃的帧数
	{
		std::lock_guard<std::mutex> t_Lock(g_FrameMutex);
		for (uint32_t i = 0; i < p_Info->skeletonsCount; i++)
		{
			RawSkeletonInfo t_SkelInfo{};
			if (CoreSdk_GetRawSkeletonInfo(i, &t_SkelInfo) != SDKReturnCode_Success)
				continue;

			GloveStream t_Stream;
			t_Stream.seq = ++g_FrameQueues[t_SkelInfo.gloveId].lastSeq;
			t_Stream.sourceMonotonicNs = t_SourceMonotonicNs;
			t_Stream.sdkPublishTime = t_SkelInfo.publishTime.time;

			if (t_SkelInfo.nodesCount == 0 || t_SkelInfo.nodesCount > kMaxNodesPerGlove)
			{
				fprintf(stderr, "[rawviz] invalid node count %u (glove %08X)\n",
				        (unsigned)t_SkelInfo.nodesCount, (unsigned)t_SkelInfo.gloveId);
				continue;
			}
			std::array<SkeletonNode, kMaxNodesPerGlove> t_Nodes;
			if (CoreSdk_GetRawSkeletonData(i, t_Nodes.data(), t_SkelInfo.nodesCount) != SDKReturnCode_Success)
				continue;
			t_Stream.nodes.reserve(t_SkelInfo.nodesCount);
			for (uint32_t n = 0; n < t_SkelInfo.nodesCount; n++)
			{
				const ManusTransform& t_Transform = t_Nodes[n].transform;
				t_Stream.nodes.push_back({
					t_Nodes[n].id,
					t_Transform.position.x, t_Transform.position.y, t_Transform.position.z,
					t_Transform.rotation.w, t_Transform.rotation.x,
					t_Transform.rotation.y, t_Transform.rotation.z
				});
			}

			GloveQueue& t_Queue = g_FrameQueues[t_SkelInfo.gloveId];
			if (t_Queue.frames.size() >= kMaxQueuedFrames)
			{
				t_Queue.frames.pop_front();   // 丢最旧:保最新帧,过载时输出 seq 跳号
				t_Dropped++;
			}
			t_Queue.frames.push_back(std::move(t_Stream));
		}
	}

	if (t_Dropped > 0)
	{
		// 限频警告:每 1s 最多一条,避免刷屏
		static uint64_t t_LastWarnNs = 0;
		const uint64_t t_NowNs = (uint64_t)std::chrono::duration_cast<
			std::chrono::nanoseconds>(
				std::chrono::steady_clock::now().time_since_epoch()).count();
		if (t_NowNs - t_LastWarnNs >= 1000000000ull)
		{
			fprintf(stderr, "[rawviz] 队列满,丢最旧 %u 帧(消费跟不上,seq 将跳号)\n",
			        t_Dropped);
			t_LastWarnNs = t_NowNs;
		}
	}
	g_FrameCV.notify_all();               // 唤醒主循环(锁外,避免唤醒即抢锁)
}

// ---------------------------------------------------------------------------
// SDK 日志改道到 stderr（stdout 只留给协议数据）
// ---------------------------------------------------------------------------
static void OnSdkLog(LogSeverity, const char* const p_Log, uint32_t p_Length)
{
	fprintf(stderr, "%.*s\n", (int)p_Length, p_Log ? p_Log : "");
	fflush(stderr);
}

// ---------------------------------------------------------------------------
// 首次见到某只手套时，获取节点层级（父子关系）并输出
// ---------------------------------------------------------------------------
static const GloveTopology* EnsureTopology(uint32_t p_GloveId)
{
	auto t_Existing = g_Topology.find(p_GloveId);
	if (t_Existing != g_Topology.end())
		return &t_Existing->second;

	uint32_t t_NodeCount = 0;
	if (CoreSdk_GetRawSkeletonNodeCount(p_GloveId, t_NodeCount) != SDKReturnCode_Success
	    || t_NodeCount == 0 || t_NodeCount > kMaxNodesPerGlove)
		return nullptr;

	GloveTopology t_Topology;
	t_Topology.nodes.resize(t_NodeCount);
	if (CoreSdk_GetRawSkeletonNodeInfoArray(p_GloveId, t_Topology.nodes.data(), t_NodeCount) != SDKReturnCode_Success)
		return nullptr;

	uint32_t t_Side = Side_Invalid;
	for (uint32_t i = 0; i < t_NodeCount; i++)
	{
		const NodeInfo& t_Node = t_Topology.nodes[i];
		if (!t_Topology.indexById.emplace(t_Node.nodeId, i).second)
			return nullptr;
		if (t_Node.side != Side_Left && t_Node.side != Side_Right)
			return nullptr;
		if (t_Side != Side_Invalid && t_Side != (uint32_t)t_Node.side)
			return nullptr;
		t_Side = t_Node.side;
	}

	// Emit all metadata before any POSE. The parser validates joint semantics.
	printf("HAND %08X %s %u\n", p_GloveId,
	       t_Side == Side_Left ? "Left" : "Right", t_NodeCount);
	for (uint32_t i = 0; i < t_NodeCount; i++)
	{
		const NodeInfo& t_Node = t_Topology.nodes[i];
		printf("NODE %08X %u %u %u %u %u %u\n", p_GloveId, i,
		       t_Node.nodeId, t_Node.parentId, (uint32_t)t_Node.chainType,
		       (uint32_t)t_Node.side, (uint32_t)t_Node.fingerJointType);
		if (t_Node.parentId != 0 && t_Node.parentId != t_Node.nodeId)
			printf("EDGE %08X %u %u %u\n", p_GloveId, t_Node.nodeId,
			       t_Node.parentId, (uint32_t)t_Node.chainType);
	}
	fflush(stdout);
	LoadCalibration(p_GloveId, t_Side);
	return &g_Topology.emplace(p_GloveId, std::move(t_Topology)).first->second;
}

// ---------------------------------------------------------------------------
// 连接流程（Integrated 模式）
// ---------------------------------------------------------------------------
static bool ConnectIntegrated()
{
	// 等待设备就绪：先初始化 SDK
	SDKReturnCode t_Result = CoreSdk_InitializeIntegrated();
	if (t_Result != SDKReturnCode_Success)
	{
		fprintf(stderr, "[rawviz] InitializeIntegrated failed: %d\n", (int)t_Result);
		return false;
	}

	// 坐标系：z-up、右手系、x 朝观察者、单位米、世界坐标（与官方示例一致）
	CoordinateSystemVUH t_VUH;
	CoordinateSystemVUH_Init(&t_VUH);
	t_VUH.handedness = Side_Right;
	t_VUH.up = AxisPolarity_PositiveZ;
	t_VUH.view = AxisView_XFromViewer;
	t_VUH.unitScale = 1.0f;

	if (CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true) != SDKReturnCode_Success)
	{
		fprintf(stderr, "[rawviz] Coordinate system init failed\n");
		return false;
	}

	// 注册回调
	if (CoreSdk_RegisterCallbackForOnLog(OnSdkLog) != SDKReturnCode_Success)
	{
		fprintf(stderr, "[rawviz] Register log callback failed\n");
		return false;
	}
	if (CoreSdk_RegisterCallbackForRawSkeletonStream(OnRawSkeletonStream) != SDKReturnCode_Success)
	{
		fprintf(stderr, "[rawviz] Register callback failed\n");
		return false;
	}

	// 找主机并连接（Integrated 模式下 LookForHosts 找本地 Core）
	for (int t_Try = 0; t_Try < 10 && g_Running; t_Try++)
	{
		CoreSdk_LookForHosts(1, true);

		uint32_t t_NumHosts = 0;
		if (CoreSdk_GetNumberOfAvailableHostsFound(&t_NumHosts) == SDKReturnCode_Success && t_NumHosts > 0)
		{
			std::vector<ManusHost> t_Hosts(t_NumHosts);
			if (CoreSdk_GetAvailableHostsFound(t_Hosts.data(), t_NumHosts) == SDKReturnCode_Success)
			{
				SDKReturnCode t_Conn = CoreSdk_ConnectToHost(t_Hosts[0]);
				if (t_Conn == SDKReturnCode_Success)
				{
					// 原始骨架模式(不做手势估计),与 wuji-hand-teleop 一致
					CoreSdk_SetRawSkeletonHandMotion(HandMotion_None);
					return true;
				}
				fprintf(stderr, "[rawviz] ConnectToHost failed: %d\n", (int)t_Conn);
			}
		}
		fprintf(stderr, "[rawviz] waiting for dongle...\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	return false;
}

// ---------------------------------------------------------------------------
// 启动前校验:所选用户的左右手校准文件必须存在,缺失报错退出
// ---------------------------------------------------------------------------
static bool CheckCalibrationFilesExist()
{
	bool t_Ok = true;
	for (const char* t_Name : {"LeftMetaglovePro.mcal", "RightMetaglovePro.mcal"})
	{
		char t_Path[PATH_MAX * 2];
		BuildCalibrationPath(t_Name, t_Path, sizeof(t_Path));
		FILE* t_F = fopen(t_Path, "rb");
		if (!t_F)
		{
			fprintf(stderr, "[rawviz] 错误: 用户 '%s' 的校准文件不存在: %s\n",
			        g_CalibrationUser.c_str(), t_Path);
			t_Ok = false;
		}
		else
		{
			fclose(t_F);
		}
	}
	return t_Ok;
}

// ---------------------------------------------------------------------------
// 主循环：等待新帧并输出（SDK 回调可达 120Hz,输出不得再压到 30fps）
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			printf("Usage: %s --user NAME --calibration-dir PATH\n"
			       "Stream Manus Integrated SDK HAND/NODE/EDGE/POSE records to stdout.\n"
			       "Both options are required; PATH selects profiles/NAME/manus.\n"
			       "--user NAME selects NAMELeft/RightMetaglovePro.mcal in PATH.\n", argv[0]);
			return 0;
		}
		else if (strcmp(argv[i], "--calibration-dir") == 0 && i + 1 < argc)
		{
			g_CalibrationDirectory = argv[++i];
		}
		else if (strcmp(argv[i], "--user") == 0 && i + 1 < argc)
		{
			g_CalibrationUser = argv[++i];
		}
		else
		{
			fprintf(stderr, "[rawviz] 未知参数: %s\n", argv[i]);
			fprintf(stderr, "Usage: %s --user NAME --calibration-dir PATH\n", argv[0]);
			return 2;
		}
	}
	if (g_CalibrationUser.empty() || g_CalibrationDirectory.empty())
	{
		fprintf(stderr, "[rawviz] --user and --calibration-dir are required.\n");
		fprintf(stderr, "Usage: %s --user NAME --calibration-dir PATH\n", argv[0]);
		return 2;
	}
	if (!CheckCalibrationFilesExist())
	{
		fprintf(stderr, "[rawviz] 校准文件缺失,退出。请先在 '%s' 为用户 '%s' 准备 "
		        "%sLeftMetaglovePro.mcal 与 %sRightMetaglovePro.mcal\n",
		        g_CalibrationDirectory.c_str(), g_CalibrationUser.c_str(),
		        g_CalibrationUser.c_str(), g_CalibrationUser.c_str());
		return 2;
	}

	signal(SIGINT, HandleSignal);
	signal(SIGTERM, HandleSignal);

	if (!ConnectIntegrated())
	{
		fprintf(stderr, "[rawviz] failed to connect to dongle\n");
		return 1;
	}
	fprintf(stderr, "[rawviz] connected, streaming raw skeleton...\n");

	while (g_Running)
	{
		// 条件变量等待新帧:帧到达即唤醒,无需轮询,检出延迟 ~μs 级。
		// 2ms 超时兜底:信号处理器不能安全 notify,靠超时醒来检查 g_Running。
		// 队列容量内零丢帧;过载时回调侧丢最旧,输出侧始终跟得上。
		std::vector<std::pair<uint32_t, std::deque<GloveStream>>> t_Batch;
		{
			std::unique_lock<std::mutex> t_Lock(g_FrameMutex);
			g_FrameCV.wait_for(t_Lock, std::chrono::milliseconds(2), [] {
				if (!g_Running) return true;
				for (const auto& t_Entry : g_FrameQueues)
					if (!t_Entry.second.frames.empty()) return true;
				return false;
			});
			// 锁内批量搬移,printf 全部放锁外(printf 可能因管道背压阻塞)
			for (auto& t_Entry : g_FrameQueues)
				if (!t_Entry.second.frames.empty())
					t_Batch.emplace_back(t_Entry.first,
					                     std::move(t_Entry.second.frames));
		}

		for (const auto& t_B : t_Batch)
		{
			const uint32_t t_GloveId = t_B.first;
			const std::deque<GloveStream>& t_Frames = t_B.second;
			const GloveTopology* t_Topology = EnsureTopology(t_GloveId);
			if (!t_Topology) continue;
			for (const GloveStream& t_Stream : t_Frames)
			{
				std::array<const NodePose*, kMaxNodesPerGlove> t_Ordered;
				if (!OrderFrameNodes(*t_Topology, t_Stream, t_Ordered)) continue;
				printf("POSE %08X %llu %llu %llu", t_GloveId,
				       (unsigned long long)t_Stream.seq,
				       (unsigned long long)t_Stream.sourceMonotonicNs,
				       (unsigned long long)t_Stream.sdkPublishTime);
				for (size_t i = 0; i < t_Stream.nodes.size(); i++)
				{
					const NodePose& t_N = *t_Ordered[i];
					printf(" %.6f %.6f %.6f %.7f %.7f %.7f %.7f",
					       t_N.x, t_N.y, t_N.z, t_N.qw, t_N.qx, t_N.qy, t_N.qz);
				}
				printf("\n");
			}
		}
		fflush(stdout);
	}

	CoreSdk_ShutDown();
	return 0;
}
