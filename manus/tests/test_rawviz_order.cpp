// Link with the same SDK/dependencies as rawviz; no SDK initialization is called.
#define main manus_collector_main
#include "../rawviz.cpp"
#undef main
#include <cassert>

int main()
{
	GloveTopology topology;
	topology.nodes.resize(3);
	topology.indexById = {{901, 0}, {17, 1}, {420, 2}};
	GloveStream frame;
	frame.nodes = {{420, 4, 0, 0, 1, 0, 0, 0},
	               {901, 9, 0, 0, 1, 0, 0, 0},
	               {17, 1, 0, 0, 1, 0, 0, 0}};
	std::array<const NodePose*, kMaxNodesPerGlove> ordered{};
	assert(OrderFrameNodes(topology, frame, ordered));
	assert(ordered[0]->x == 9 && ordered[1]->x == 1 && ordered[2]->x == 4);

	// The callback is permitted to change SkeletonNode order on every frame.
	std::swap(frame.nodes[0], frame.nodes[1]);
	assert(OrderFrameNodes(topology, frame, ordered));
	assert(ordered[0]->x == 9 && ordered[1]->x == 1 && ordered[2]->x == 4);
	frame.nodes[0].id = 420; // Duplicate, with 901 now missing.
	assert(!OrderFrameNodes(topology, frame, ordered));
	frame.nodes[0].id = 999; // Unknown ID must not be assigned an array slot.
	assert(!OrderFrameNodes(topology, frame, ordered));
	frame.nodes.pop_back();
	assert(!OrderFrameNodes(topology, frame, ordered));
}
