#pragma once

#include <unordered_set>
#include "Anatomy.h"
#include "GeometryMath.h"
#include "fluid/Solver.h"
#include "fluid/Clusters.h"
#include "fluid/Surface.h"

namespace OStrands
{
	class FluidManager
	{
	public:
		enum class Liquid
		{
			kSemen,
			kMilk
		};

		static constexpr std::int32_t kNoThread = -1;

		static FluidManager& Get();

		void SetOCum(bool a_installed);
		void Climax(std::int32_t a_thread, RE::ActorHandle a_actor, std::vector<Pair> a_pairs, std::vector<RE::ActorHandle> a_actors);
		void Load(RE::ActorHandle a_actor, float a_fraction);
		void EndThread(std::int32_t a_thread);
		void Scene(std::int32_t a_thread, std::vector<RE::ActorHandle> a_actors);
		enum MilkKind : int
		{
			kCycle = 0,    // the regular flow of full breasts (shaped by the spray mode)
			kSqueeze = 1,  // one SGO milking squeeze
			kClimax = 2,   // a climax during sex
			kDribble = 3,  // small drip between squeezes
			kCrotch = 4    // milk that ran down the belly dripping off the vulva (top leak stage)
		};

		// Milk from both nipples.
		// a_power scales only how hard the milk comes out (1 = the configured strength).
		void Lactate(RE::ActorHandle a_actor, float a_amount, int a_kind, std::int32_t a_thread = kNoThread,
			std::vector<RE::ActorHandle> a_hosts = {}, float a_power = 1.0f);

		void Reset();
		void Update(float a_delta);
		void HideShapes();
		void WaitForBodyEditor(bool a_wait);
		bool SemenEnabled() const;
		bool OCumInstalled() const { return _ocum; }
		bool InScene(RE::ActorHandle a_actor) const;

		static constexpr std::size_t kRopeBones = 16;

	private:
		struct Event
		{
			enum class Type
			{
				kClimax,
				kLoad,
				kEnd,
				kScene,
				kLactate
			};

			Type              type{ Type::kClimax };
			std::int32_t      thread{ 0 };
			RE::ActorHandle   actor;
			std::vector<Pair>            pairs;
			float                        value{ 0.0f };
			std::vector<RE::ActorHandle> actors;
			bool                         squirt{ false };
			int                          kind{ 0 };
			float                        power{ 1.0f };  // milk only: share of the spray strength (fill ramp)
		};

		struct Pulse
		{
			float at{ 0.0f };
			float speed{ 0.0f };
			float spacing{ 0.0f };
			int   particles{ 0 };
			float spread{ 0.0f };
			float angle{ 0.0f };
			float wave{ 0.0f };
			float frequency{ 0.0f };
			int   streams{ 1 };      // milk: number of streams fanned out from the nipple
			float cone{ 0.0f };      // milk: fan angle of those streams (radians)
			bool  chain{ false };    // milk: continues the previous pulse's streams without a break
			bool  taper{ true };
		};

		struct Emitter
		{
			std::uint32_t       id{ 0 };
			std::int32_t        thread{ 0 };
			RE::ActorHandle     actor;
			std::optional<Pair> pair;
			std::vector<RE::ActorHandle> hosts;
			float               wait{ 0.0f };
			float               amount{ 1.0f };
			bool                loaded{ false };
			std::vector<Pulse>  pulses;
			std::size_t         pulse{ 0 };
			int                 emitted{ 0 };
			float               slot{ 0.0f };
			float               clock{ 0.0f };
			float               hang{ 0.0f };
			int                 count{ 0 };
			bool                oozing{ false };
			std::uint32_t       run{ 0 };
			std::uint32_t       domain{ 0 };
			RE::NiPoint3        lastNozzle;
			RE::NiPoint3        carried;
			bool                hasNozzle{ false };
			bool                inside{ false };
			Liquid              liquid{ Liquid::kSemen };
			int                 side{ 0 };
			bool                squirt{ true };
			int                 kind{ 0 };
			float               power{ 1.0f };
			int                 streams{ 1 };
		};

		struct Blob
		{
			RE::NiPointer<RE::NiNode>          root;
			std::vector<RE::NiAVObject*>       bones;
			std::vector<Fluid::SurfacePoint>   surface;
			std::size_t                        seed{ 0 };
			bool                               live{ false };
			bool                               used{ false };
			bool                               puddle{ false };     // currently showing a floor puddle cell
			std::int64_t                       puddleKey{ 0 };
		};

		struct Rope
		{
			RE::NiPointer<RE::NiNode>                     root;
			std::array<RE::NiAVObject*, kRopeBones>       bones{};
			std::array<GeometryMath::Frame, kRopeBones>   frames{};
			bool                                          used{ false };
		};

		struct Meshes
		{
			RE::NiPointer<RE::NiNode> blob;
			RE::NiPointer<RE::NiNode> rope;
			bool                      failed{ false };
		};

		struct Reservoir
		{
			std::uint32_t                id{ 0 };
			RE::ActorHandle              actor;
			Anatomy::Opening             opening{ Anatomy::Opening::kVagina };
			RE::ActorHandle              plug;
			std::vector<RE::ActorHandle> hosts;
			float                        volume{ 0.0f };
			float                        gush{ 0.0f };
			float                        release{ 0.0f };
			float                        age{ 0.0f };
			float                        dressed{ 0.0f };
			bool                         plugged{ true };
			std::uint32_t                domain{ 0 };
			std::uint32_t                run{ 0 };
			int                          glob{ 0 };
			float                        globSpeed{ 1.0f };
			float                        phase{ 0.0f };
			float                        period{ 2.0f };
			RE::NiPoint3                 lastExit;
			bool                         hasExit{ false };
		};

		struct Puddle
		{
			Fluid::Vec3 sum;
			float       count{ 0.0f };
			float       life{ 0.0f };
			Fluid::Vec3 flow;            // summed sliding velocity of the milk that ran into this cell
			float       flowWeight{ 0.0f };
			bool        probed{ false };  // real ground under it measured
			float       surface{ 0.0f };  // highest ground under the puddle (local m)
			Fluid::Vec3 normal{ 0.0f, 0.0f, 1.0f };
			int         blob{ -1 };       // blob showing it, kept from frame to frame
			float       shownRadius{ -1.0f };
			Fluid::Vec3 shownFlow;
			bool        shownProbed{ false };
			Fluid::Vec3 shift;               // pushed away from a wall standing in it
		};

		struct Domain
		{
			std::uint32_t                  id{ 0 };
			RE::NiPoint3                   origin;
			RE::TESObjectCELL*             cell{ nullptr };
			bool                           broken{ false };
			std::vector<RE::ActorHandle>   hosts;
			std::vector<Fluid::Particle>   particles;
			std::vector<Fluid::Anchor>     anchors;
			std::vector<std::pair<RE::ActorHandle, Anatomy::Opening>> openings;
			std::unique_ptr<Fluid::Solver> solver;
			float                          accumulator{ 0.0f };
			RE::NiPointer<RE::NiNode>      root;
			std::vector<Blob>              blobs;
			std::vector<Rope>              ropes;
			float                          opacity{ -1.0f };
			float                          shine{ -1.0f };
			float                          gloss{ -1.0f };
			float                          age{ 0.0f };
			Fluid::Ground                  ground{ 0.08f };
			std::vector<Fluid::Capsule>    capsules;
			std::vector<Fluid::Hole>       holes;
			Fluid::Skin                    skin;
			std::size_t                    pooled{ 0 };
			std::size_t                    shownBlobs{ 0 };
			std::size_t                    shownRopes{ 0 };
			int                            spinning{ 0 };
			std::string                    spinSample;
			bool                           diagnostic{ false };
			bool                           incompleteReported{ false };
			Liquid                         liquid{ Liquid::kSemen };
			float                          glow{ -1.0f };
			std::unordered_map<std::int64_t, Puddle> puddles;
			float                                    idle{ 0.0f };
			// Other people and creatures standing near the milk: it hits and runs down them like the hosts' skin.
			std::vector<RE::ActorHandle>             nearby;
			float                                    nearbyAge{ 1e9f };
			std::uint32_t                            groundTick{ 0 };
			// Walls, pillars and other world geometry the milk met, found by short ray casts and kept (it does not move).
			// They join the body skin in the solver, so milk runs down them the way it runs down skin.
			std::vector<Fluid::Patch>                 world;
			std::unordered_map<std::int64_t, int>     worldIndex;
			struct Owner
			{
				RE::FormID   id{ 0 };  // 0 = fixed geometry
				RE::NiPoint3 at;       // where the object stood when the wall was found
			};
			std::vector<Owner>                        worldOwner;      // per wall: the door or movable object it belongs to
			std::size_t                               worldCursor{ 0 };
			// Flight-path checks: space known to be empty or to hold a surface (4 cm cells, kept a few seconds),
			// the surface each particle meets this frame, and raised tops hit by fast drops (measured first).
			struct Voxel
			{
				float       until{};
				bool        blocked{};
				Fluid::Vec3 point;
				Fluid::Vec3 normal;
			};
			std::unordered_map<std::int64_t, Voxel>   voxels;
			std::vector<Fluid::Plane>                 guards;
			std::vector<Fluid::Vec3>                  groundHints;
			std::size_t                               guardCursor{ 0 };
			std::size_t                               worldFrom{ 0 };  // contact index where world patches begin this frame
			std::vector<int>                          worldOrder;      // which wall each of those contact slots is  // seconds with nothing in it  // settled floor milk, no longer simulated
		};

		void        Apply(Event& a_event);
		void        StartClimax(const Event& a_event);
		void        Plan(Emitter& a_emitter);
		void        StartMilk(const Event& a_event);
		void        PlanMilk(Emitter& a_emitter, float a_from);
		void        Regroup(Domain& a_domain);
		bool        Step(Emitter& a_emitter, float a_delta);
		void        Deposit(const Emitter& a_emitter, float a_particles);
		bool        Leak(Reservoir& a_reservoir, float a_delta);
		Domain*     FindDomain(std::uint32_t a_id);
		static bool Holds(const Domain& a_domain, RE::Actor* a_actor, const RE::NiPoint3& a_point);
		Domain*     DomainFor(RE::Actor* a_actor, const RE::NiPoint3& a_point, const std::vector<RE::ActorHandle>& a_hosts, Liquid a_liquid);
		bool        Attach(Domain& a_domain);
		void        Simulate(Domain& a_domain, float a_delta);
		void        SampleGround(Domain& a_domain);
		void        Expire(Domain& a_domain, float a_delta);
		void        Draw(Domain& a_domain);
		void        Watch(Domain& a_domain, const Fluid::Run& a_run);
		Meshes&     MeshesFor();
		RE::NiNode* CollisionMesh();
		Blob*       AcquireBlob(Domain& a_domain);
		bool        CloneBlob(Blob& a_blob);
		bool        CloneRope(Rope& a_rope, RE::NiNode* a_mesh);
		Rope*       AcquireRope(Domain& a_domain);
		void        PoseBlob(Domain& a_domain, Blob& a_blob);
		void        PoseDrop(Domain& a_domain, Blob& a_blob, Fluid::Vec3 a_center, float a_radius, Fluid::Vec3 a_velocity = {},
					std::optional<Fluid::Vec3> a_contact = std::nullopt);
		void        PosePuddle(Domain& a_domain, Blob& a_blob, Fluid::Vec3 a_center, float a_radius, Fluid::Vec3 a_flow, std::uint64_t a_seed, Fluid::Vec3 a_up);
		void        ProbePuddle(Domain& a_domain, Puddle& a_cell, Fluid::Vec3 a_center);
		void        ReleaseShapes(Domain& a_domain);
		void        ProbeWorld(Domain& a_domain);
		void        GuardPaths(Domain& a_domain, float a_seconds);
		void        AddWorldPatch(Domain& a_domain, const RE::NiPoint3& a_point, const RE::NiPoint3& a_normal, bool a_floorToo, RE::FormID a_owner = 0,
			const RE::NiPoint3& a_ownerAt = {});
		void        PoseRope(Domain& a_domain, Rope& a_rope, std::size_t a_begin, std::size_t a_end, const Fluid::Anchor* a_anchor);
		void        PoseCapsule(Rope& a_rope, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, float a_radius);
		void        Bend(Rope& a_rope, const std::array<RE::NiPoint3, kRopeBones>& a_positions, const std::array<float, kRopeBones>& a_radii);
		void        Reshade(Domain& a_domain);
		void        ShowShapes(float a_delta);
		void        Shade(Domain& a_domain, RE::NiAVObject* a_root);
		void        Release(Domain& a_domain);
		Fluid::Vec3 ToLocal(const Domain& a_domain, const RE::NiPoint3& a_point) const;
		RE::NiPoint3 ToWorld(const Domain& a_domain, Fluid::Vec3 a_point) const;

		std::mutex         _lock;
		std::vector<Event> _pending;
		std::atomic<bool>  _ocum{ false };

		std::vector<Emitter>                                  _emitters;
		std::vector<Reservoir>                                _reservoirs;
		std::vector<std::unique_ptr<Domain>>                  _domains;
		std::unique_ptr<Domain>                               _shapes;
		std::unordered_map<std::int32_t, std::vector<RE::ActorHandle>> _scenes;
		std::unordered_map<std::uint32_t, std::pair<float, float>> _loads;
		Meshes                                                _meshes;
		// Emissive colours we allocated ourselves, per shader property (see Shade: never write through a shared one).
		std::unordered_map<const void*, RE::NiColor*>         _ownEmissive;
		std::vector<Blob>                                     _spareBlobs;
		std::vector<Rope>                                     _spareRopes;
		int                                                   _cloneBudget{ 0 };
		bool                                                  _warm{ false };
		std::unordered_set<std::int64_t>                      _nearCells;  // milk was used this session: keep spare meshes ready
		void                                                  Prewarm();
		RE::NiPointer<RE::NiNode>                             _collisionMesh;
		bool                                                   _collisionMeshFailed{ false };
		bool                                                   _debugAwaitingEditor{ false };
		bool                                                   _debugReady{ false };
		float                                                  _debugStableTime{}, _debugProbeTime{};
		std::vector<std::uint64_t>                             _debugMeshSignature;
		std::vector<Fluid::Vec3>                              _directions;
		std::vector<Fluid::SurfacePoint>                      _scratch;
		std::uint32_t                                         _nextDomain{ 0 };
		std::uint32_t                                         _nextRun{ 0 };
		std::uint32_t                                         _nextEmitter{ 0 };
		std::mt19937                                          _rng{ std::random_device{}() };

		float  _statTime{ 0.0f };
		double _statSimulate{ 0.0 };
		double _statDraw{ 0.0 };
		double _statWorst{ 0.0 };
		int    _statClones{ 0 };
		int    _statFrames{ 0 };
	};
}
