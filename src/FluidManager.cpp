#include "FluidManager.h"

#include "Body.h"
#include "Lactation.h"
#include "Revealing.h"
#include "Settings.h"
#include "fluid/Clusters.h"

#include <format>
#include <numeric>
#include <numbers>
#include <unordered_set>

namespace OStrands
{
	using namespace Anatomy;

	namespace
	{
		constexpr float       kUnitsPerMeter = 70.0f;
		constexpr float       kSpacing = 0.006f;
		constexpr float       kVolume = kSpacing * kSpacing * kSpacing;
		constexpr float       kKernel = 2.0f * kSpacing;
		constexpr float       kThreshold = 1.75f;
		constexpr float       kBulkEnter = 3.0f;
		constexpr float       kBulkLeave = 2.5f;
		constexpr float       kCovered = 0.3f;
		constexpr float       kPieceRadius = 4.0f * kSpacing;
		constexpr float       kSlotSpacing = 0.9f;
		constexpr float       kDribbleSpacing = 0.6f;
		constexpr std::size_t kBlobMinimum = 6;
		constexpr float       kSmoothing = 0.05f;
		constexpr float       kStep = 1.0f / 120.0f;
		constexpr int         kMaxSteps = 6;
		constexpr std::size_t kMaxDomains = 6;
		constexpr std::size_t kMaxBlobs = 256;
		constexpr std::size_t kMaxRopes = 192;
		constexpr int         kClonesPerFrame = 8;
		// Spare meshes built ahead of time, a few per frame, so a new pool does not clone dozens in one frame
		// (that was the worst-frame spike in the logs). Same meshes, same look; only when they are made changes.
		constexpr std::size_t kSpareBlobTarget = 96;
		constexpr float       kNearCell = 40.0f;
		constexpr float       kPoolLinger = 10.0f;  // s an empty milk pool stays open for the next cycle
		constexpr std::size_t kMaxEmptyPools = 2;   // more empty pools than this (walking around): the oldest close
		constexpr int         kPuddleProbes = 6;
		constexpr int         kWorldRays = 16;          // ray casts per pool per frame looking for walls ahead of the milk
		constexpr std::size_t kMaxWorldPatches = 1500;
		constexpr float       kWorldPatchCell = 0.02f;  // m: one wall sample per 2 cm
		constexpr float       kWallNormalZ = 0.8f;      // flatter than this is floor (the ground grid already has it)    // puddle cells whose real ground is measured per frame
		constexpr float       kPuddleProbe = 0.05f; // m: spacing of the ground samples around a puddle
		constexpr float       kPuddleLift = 0.005f; // m above the highest ground under it  // units: skin within about a cell of the milk is posed every frame
		constexpr std::size_t kSpareRopeTarget = 48;
		constexpr std::size_t kMaxDebugShapes = 256;
		constexpr float       kDomainReach = 300.0f;
		constexpr float       kDomainStep = 80.0f;
		constexpr float       kGroundAbove = 80.0f;
		constexpr float       kGroundBelow = 300.0f;
		constexpr float       kGroundStart = 0.14f;  // m above the milk a floor probe starts
		constexpr float       kRaised = 0.08f;       // m above the pool floor: check that a flat surface is wide enough
		constexpr float       kLedge = 2.5f;         // units to the sides a raised surface must continue (about 7 cm across)
		constexpr int         kGroundBudget = 32;      // floor ray casts per pool per frame
		constexpr float       kNearbyEvery = 0.3f;     // s between looks for other actors near the milk
		constexpr float       kNearbyReach = 90.0f;    // units around the milk an actor's position may be
		constexpr float       kNearbyBelow = 160.0f;   // an actor's position is its feet: allow for its height
		constexpr std::uint64_t kBystanderRefresh = 12; // frames between re-posing a bystander's skin away from the milk
		constexpr std::size_t kNearbyMax = 4;          // nearest other actors given collision per pool
		constexpr float       kVoxel = 0.04f;          // m: cells of space known empty / holding a surface
		constexpr float       kFreeLife = 2.0f;        // s an empty cell is trusted (doors close, things move)
		constexpr float       kBlockLife = 6.0f;       // s a surface on fixed geometry is trusted
		constexpr float       kMovableLife = 0.5f;     // s a surface on a door or movable object is trusted
		constexpr std::size_t kMaxVoxels = 60000;
		constexpr int         kGuardRays = 128;        // flight-path ray casts per pool per frame (most paths are cached)
		constexpr float       kGuardSpeed = 0.4f;      // m/s: slower milk is handled by the surface samples alone
		constexpr float       kGuardBack = 0.05f;      // m behind a drop its path check starts
		constexpr float       kGuardAhead = 0.04f;     // m past where it ends up
		constexpr int         kFineBudget = 24;        // fine (2 cm) floor ray casts at edges per pool per frame
		constexpr float       kStreak = 1.5f;
		constexpr float       kStreakStart = 0.3f;
		constexpr float       kMaxStretch = 3.0f;
		constexpr float       kSquash = 0.45f;
		constexpr float       kGlobDisc = 0.8f;
		constexpr float       kGroundSearch = 400.0f;
		constexpr float       kLoadMemory = 6.0f;
		constexpr float       kNozzleOffset = 0.4f;
		constexpr float       kMilkNozzleOffset = 0.1f;
		constexpr float       kMilkStep = 1.0f / 75.0f;
		constexpr int         kMilkMaxSteps = 2;
		constexpr float       kMilkLinkReach = 4.5f;
		constexpr std::size_t kMaxPuddleDraws = 120;   // puddle shapes drawn per pool (the rest merge visually anyway)
		bool Finite(const RE::NiPoint3& a_point) { return std::isfinite(a_point.x) && std::isfinite(a_point.y) && std::isfinite(a_point.z); }
		bool Finite(const Fluid::Vec3& a_point) { return std::isfinite(a_point.x) && std::isfinite(a_point.y) && std::isfinite(a_point.z); }
		constexpr float       kPuddleCell = 0.035f;     // m: grid of settled floor milk
		constexpr std::size_t kMaxPuddleCells = 260;
		constexpr float       kPuddleThickness = 0.0035f;  // m  // milk strands bridge gaps up to this many spacings (fewer particles per stream)
		constexpr float       kMilkSpacing = 1.15f;
		constexpr float       kMilkHole = 2.5f;
		constexpr float       kMilkSquash = 0.22f;
		constexpr float       kMilkSpread = 1.4f;
		constexpr float       kMilkBulkEnter = 2.0f;
		constexpr float       kMilkBulkLeave = 1.6f;
		constexpr float       kMilkPieceRadius = 10.0f * 0.006f;
		constexpr float       kMaxNozzleSpeed = 2.0f;
		constexpr float       kDryLoad = 0.01f;
		constexpr float       kTypicalLoad = 0.2f;
		constexpr float       kAfterScene = 1.0f;
		constexpr float       kHangTime = 4.0f;
		constexpr float       kCarry = 0.6f;
		constexpr float       kCarrySmoothing = 0.08f;
		constexpr float       kTailStart = 0.55f;
		constexpr float       kTailSlowdown = 0.85f;
		constexpr float       kOozeSpeed = 0.15f;
		constexpr float       kHoleRadius = 2.0f;
		constexpr float       kNozzleHole = 1.0f;
		constexpr float       kSkinCell = 0.07f;
		constexpr float       kSkinMargin = 0.3f;
		constexpr std::size_t kWrapDots = 90;
		constexpr float       kSpinning = 4.0f * std::numbers::pi_v<float>;
		constexpr float       kExitDepth = 0.6f;
		constexpr float       kGushShare = 0.35f;
		constexpr float       kGushRate = 40.0f;
		constexpr float       kSeep = 0.03f;
		constexpr float       kLeak = 0.12f;
		constexpr float       kLeakSpeed = 0.1f;
		constexpr float       kReservoirLife = 90.0f;
		constexpr float       kDressedGrace = 2.5f;
		constexpr int         kGlobMin = 12;
		constexpr int         kGlobMax = 40;
		constexpr float       kMarkerRadius = 0.35f;
		constexpr float       kPointerLength = 3.0f;

		float Taper(float a_progress)
		{
			const float t = std::clamp((a_progress - kTailStart) / (1.0f - kTailStart), 0.0f, 1.0f);
			return 1.0f - kTailSlowdown * t * t * (3.0f - 2.0f * t);
		}

		constexpr const char* kBlobPath = "OStrands\\semen_blob.nif";
		constexpr const char* kRopePath = "OStrands\\semen_rope.nif";
		constexpr const char* kCollisionPath = "OStrands\\collision.nif";

		using Vec3 = Fluid::Vec3;

		struct MeshNames
		{
			RE::BSFixedString blobShape{ "BlobShape" };
			RE::BSFixedString ropeShape{ "RopeShape" };

			static const MeshNames& Get()
			{
				static const MeshNames names;
				return names;
			}
		};

		RE::NiPoint3 Normalized(RE::NiPoint3 v, const RE::NiPoint3& fallback)
		{
			return v.Unitize() > 1e-5f ? v : fallback;
		}

		Vec3 ToVec(const RE::NiPoint3& a_point) { return { a_point.x, a_point.y, a_point.z }; }

		Fluid::Parameters SemenParameters()
		{
			const auto& settings = Settings::Get();
			return { 4, settings.semenCohesion, settings.semenViscosity, 9.81f, settings.semenStretch, 30.0f, settings.semenStickiness };
		}

		Fluid::Parameters MilkParameters()
		{
			const auto& settings = Settings::Get();
			// Milk: soft pressure and no bounce off bodies, so it pours instead of spraying.
			return { 2, settings.milkCohesion, settings.milkViscosity, 9.81f, settings.milkStretch, 12.0f, settings.milkStickiness, 0.35f, 0.0f, 25.0f, 0.1f, 0.6f, 0.5f, 0.8f, false,
				settings.milkStickinessWorld,
				// Strength up to 20 never comes near the 6 m/s limit (unchanged); stronger jets need room above it.
				std::max(6.0f, settings.milkSpeedMax + 2.5f) };
		}

		Fluid::Parameters ParametersFor(FluidManager::Liquid a_liquid)
		{
			return a_liquid == FluidManager::Liquid::kMilk ? MilkParameters() : SemenParameters();
		}

		float OpacityOf(FluidManager::Liquid a_liquid)
		{
			const auto& settings = Settings::Get();
			return a_liquid == FluidManager::Liquid::kMilk ? settings.milkOpacity : settings.semenOpacity;
		}

		const char* LiquidName(FluidManager::Liquid a_liquid)
		{
			return a_liquid == FluidManager::Liquid::kMilk ? "milk" : "semen";
		}

		const char* OpeningName(Opening a_opening)
		{
			return a_opening == Opening::kVagina ? "vagina" : a_opening == Opening::kAnus ? "anus" : "mouth";
		}

		bool TipInside(RE::Actor* a_giver, const RE::NiPoint3& a_center, const RE::NiPoint3& a_inward)
		{
			const auto tip = Body::TipOf(a_giver);
			return tip && (tip->point - a_center).Dot(a_inward) > 0.0f;
		}

		RE::NiPoint3 ToPoint(Vec3 a_vec) { return { a_vec.x, a_vec.y, a_vec.z }; }

		RE::NiMatrix3 Rotation(const GeometryMath::Frame& a_frame)
		{
			const auto& [x, y, z] = a_frame;
			RE::NiMatrix3 m;
			m.entry[0][0] = x.x; m.entry[1][0] = x.y; m.entry[2][0] = x.z;
			m.entry[0][1] = y.x; m.entry[1][1] = y.y; m.entry[2][1] = y.z;
			m.entry[0][2] = z.x; m.entry[1][2] = z.y; m.entry[2][2] = z.z;
			return m;
		}

		RE::NiMatrix3 ArcRotation(Vec3 a_from, Vec3 a_to)
		{
			RE::NiMatrix3 m;
			const float   c = Fluid::Dot(a_from, a_to);
			if (c < -0.999f) {
				Vec3 axis = std::abs(a_from.x) < 0.9f ? Vec3{ 1.0f, 0.0f, 0.0f } : Vec3{ 0.0f, 1.0f, 0.0f };
				axis = Fluid::Normalize(axis - a_from * Fluid::Dot(axis, a_from), { 0.0f, 0.0f, 1.0f });
				const float a[3]{ axis.x, axis.y, axis.z };
				for (int r = 0; r < 3; ++r) {
					for (int col = 0; col < 3; ++col) {
						m.entry[r][col] = 2.0f * a[r] * a[col] - (r == col ? 1.0f : 0.0f);
					}
				}
				return m;
			}
			const Vec3  v = Fluid::Cross(a_from, a_to);
			const float k = 1.0f / (1.0f + c);
			m.entry[0][0] = 1.0f - k * (v.y * v.y + v.z * v.z);
			m.entry[0][1] = -v.z + k * v.x * v.y;
			m.entry[0][2] = v.y + k * v.x * v.z;
			m.entry[1][0] = v.z + k * v.x * v.y;
			m.entry[1][1] = 1.0f - k * (v.x * v.x + v.z * v.z);
			m.entry[1][2] = -v.x + k * v.y * v.z;
			m.entry[2][0] = -v.y + k * v.x * v.z;
			m.entry[2][1] = v.x + k * v.y * v.z;
			m.entry[2][2] = 1.0f - k * (v.x * v.x + v.y * v.y);
			return m;
		}

		// Everything with collision stops milk; only doors are special: an open door lets it through (CastRay checks the
		// door state), and a door is never taken as a floor. Trigger volumes are not objects at all.
		bool Passable(RE::TESObjectREFR* a_ref, RE::COL_LAYER a_layer)
		{
			if (a_layer == RE::COL_LAYER::kTrigger) {
				return true;
			}
			auto* base = a_ref ? a_ref->GetBaseObject() : nullptr;
			return base && base->Is(RE::FormType::Door);
		}

		// Things that can be moved or knocked over: what milk met on them is forgotten once they move.
		bool Movable(const RE::TESObjectREFR* a_ref, RE::COL_LAYER a_layer)
		{
			if (!a_ref) {
				return false;
			}
			switch (a_layer) {
			case RE::COL_LAYER::kClutter:
			case RE::COL_LAYER::kDebrisLarge:
			case RE::COL_LAYER::kDebrisSmall:
			case RE::COL_LAYER::kProps:
			case RE::COL_LAYER::kWeapon:
			case RE::COL_LAYER::kProjectile:
			case RE::COL_LAYER::kTrap:
				return true;
			default:
				break;
			}
			const auto* base = a_ref->GetBaseObject();
			return base && (base->Is(RE::FormType::Door) || base->Is(RE::FormType::MovableStatic));
		}

		bool IsDoor(const RE::TESObjectREFR* a_ref)
		{
			const auto* base = a_ref ? a_ref->GetBaseObject() : nullptr;
			return base && base->Is(RE::FormType::Door);
		}

		// A door blocks milk only while it is shut; open, opening or swinging shut it lets the milk through.
		bool DoorShut(const RE::TESObjectREFR* a_ref)
		{
			const auto state = RE::BGSOpenCloseForm::GetOpenState(a_ref);
			return state == RE::BGSOpenCloseForm::OPEN_STATE::kClosed || state == RE::BGSOpenCloseForm::OPEN_STATE::kNone;
		}

		std::optional<float> CastDown(RE::bhkWorld* a_world, RE::NiPoint3 a_from, const RE::NiPoint3& a_to, RE::NiPoint3* a_normal = nullptr)
		{
			RE::bhkWorld* world = a_world;
			if (!world) {
				return std::nullopt;
			}
			const float          scale = RE::bhkWorld::GetWorldScale();
			const RE::NiPoint3   to = a_to;
			RE::NiPoint3         from = a_from;
			RE::BSReadLockGuard  guard(world->worldLock);
			for (int attempt = 0; attempt < 32 && from.z > to.z; ++attempt) {
				RE::bhkPickData pick{};
				pick.rayInput.from = RE::hkVector4(from * scale);
				pick.rayInput.to = RE::hkVector4(to * scale);
				pick.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kCameraPick);
				pick.rayInput.enableShapeCollectionFilter = true;
				if (!world->PickObject(pick) || !pick.rayOutput.HasHit() || !pick.rayOutput.rootCollidable) {
					return std::nullopt;
				}
				const float fraction = pick.rayOutput.hitFraction;
				if (!std::isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) {
					return std::nullopt;
				}
				const RE::NiPoint3 hit = from + (to - from) * fraction;
				if ((hit - from).Length() < 0.5f) {
					return std::nullopt;  // started inside a pillar, wall or prop: not a floor under it
				}
				const auto&        collidable = *pick.rayOutput.rootCollidable;
				const auto         layer = collidable.GetCollisionLayer();
				auto*              ref = RE::TESHavokUtilities::FindCollidableRef(collidable);
				if ((ref && ref->As<RE::Actor>()) || layer == RE::COL_LAYER::kBiped || layer == RE::COL_LAYER::kCharController ||
					layer == RE::COL_LAYER::kBipedNoCC || layer == RE::COL_LAYER::kDeadBip || Passable(ref, layer)) {
					from = hit - RE::NiPoint3{ 0.0f, 0.0f, 2.0f };
					continue;
				}
				if (a_normal) {
					alignas(16) float n[4];
					_mm_store_ps(n, pick.rayOutput.normal.quad);
					RE::NiPoint3 normal{ n[0], n[1], n[2] };
					if (normal.z < 0.0f) {
						normal = -normal;
					}
					*a_normal = normal.Unitize() > 1e-4f && std::isfinite(normal.x) ? normal : RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
				}
				return hit.z;
			}
			return std::nullopt;
		}

		struct RayHit
		{
			RE::NiPoint3 point;
			RE::NiPoint3 normal;
			RE::FormID   owner{ 0 };  // hit a door or a movable object: the wall it makes lasts only while it stays shut / in place
			RE::NiPoint3 ownerAt;
		};

		// Any direction; stops at the first static/world hit. Actors are ignored (the body has its own skin).
		std::optional<RayHit> CastRay(RE::bhkWorld* a_world, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to)
		{
			if (!a_world) {
				return std::nullopt;
			}
			const float         scale = RE::bhkWorld::GetWorldScale();
			RE::BSReadLockGuard guard(a_world->worldLock);
			RE::bhkPickData     pick{};
			pick.rayInput.from = RE::hkVector4(a_from * scale);
			pick.rayInput.to = RE::hkVector4(a_to * scale);
			pick.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kCameraPick);
			pick.rayInput.enableShapeCollectionFilter = true;
			if (!a_world->PickObject(pick) || !pick.rayOutput.HasHit() || !pick.rayOutput.rootCollidable) {
				return std::nullopt;
			}
			const float fraction = pick.rayOutput.hitFraction;
			if (!std::isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) {
				return std::nullopt;
			}
			const auto& collidable = *pick.rayOutput.rootCollidable;
			const auto  layer = collidable.GetCollisionLayer();
			auto*       ref = RE::TESHavokUtilities::FindCollidableRef(collidable);
			const bool  shutDoor = IsDoor(ref) && DoorShut(ref);
			if ((ref && ref->As<RE::Actor>()) || layer == RE::COL_LAYER::kBiped || layer == RE::COL_LAYER::kCharController ||
				layer == RE::COL_LAYER::kBipedNoCC || layer == RE::COL_LAYER::kDeadBip || (!shutDoor && Passable(ref, layer))) {
				return std::nullopt;  // bodies have their own skin; open doors and loose things move, so they are not remembered
			}
			alignas(16) float n[4];
			_mm_store_ps(n, pick.rayOutput.normal.quad);
			RE::NiPoint3 normal{ n[0], n[1], n[2] };
			const RE::NiPoint3 dir = a_to - a_from;
			if (normal.Dot(dir) > 0.0f) {
				normal = -normal;
			}
			if (normal.Unitize() < 1e-4f || !std::isfinite(normal.x)) {
				return std::nullopt;
			}
			const bool tagged = shutDoor || Movable(ref, layer);
			return RayHit{ a_from + dir * fraction, normal, tagged ? ref->GetFormID() : 0, tagged ? ref->GetPosition() : RE::NiPoint3{} };
		}

		std::optional<float> GroundBelow(RE::TESObjectCELL* a_cell, const RE::NiPoint3& a_point)
		{
			return CastDown(a_cell ? a_cell->GetbhkWorld() : nullptr, a_point, a_point - RE::NiPoint3{ 0.0f, 0.0f, kGroundSearch });
		}

		Vec3 Stretch(Vec3 a_value, Vec3 a_axis, float a_factor)
		{
			const float side = 1.0f / std::sqrt(a_factor);
			return a_value * side + a_axis * (Fluid::Dot(a_value, a_axis) * (a_factor - side));
		}
	}

	FluidManager& FluidManager::Get()
	{
		static FluidManager manager;
		return manager;
	}

	void FluidManager::SetOCum(bool a_installed)
	{
		_ocum = a_installed;
	}

	void FluidManager::Climax(std::int32_t a_thread, RE::ActorHandle a_actor, std::vector<Pair> a_pairs, std::vector<RE::ActorHandle> a_actors)
	{
		std::scoped_lock guard(_lock);
		_pending.push_back({ Event::Type::kClimax, a_thread, a_actor, std::move(a_pairs), 0.0f, std::move(a_actors) });
	}

	void FluidManager::Load(RE::ActorHandle a_actor, float a_fraction)
	{
		std::scoped_lock guard(_lock);
		_pending.push_back({ Event::Type::kLoad, 0, a_actor, {}, a_fraction });
	}

	void FluidManager::EndThread(std::int32_t a_thread)
	{
		std::scoped_lock guard(_lock);
		_pending.push_back({ Event::Type::kEnd, a_thread });
	}

	void FluidManager::Scene(std::int32_t a_thread, std::vector<RE::ActorHandle> a_actors)
	{
		std::scoped_lock guard(_lock);
		_pending.push_back({ Event::Type::kScene, a_thread, {}, {}, 0.0f, std::move(a_actors) });
	}

	void FluidManager::Lactate(RE::ActorHandle a_actor, float a_amount, int a_kind, std::int32_t a_thread, std::vector<RE::ActorHandle> a_hosts, float a_power)
	{
		std::scoped_lock guard(_lock);
		const bool strong = a_kind == kSqueeze || a_kind == kClimax;
		const float power = std::isfinite(a_power) ? std::clamp(a_power, 0.0f, 1.0f) : 1.0f;
		_pending.push_back({ Event::Type::kLactate, a_thread, a_actor, {}, a_amount, std::move(a_hosts), strong, a_kind, power });
	}

	void FluidManager::Reset()
	{
		{
			std::scoped_lock guard(_lock);
			_pending.clear();
		}
		_emitters.clear();
		_reservoirs.clear();
		for (auto& domain : _domains) {
			Release(*domain);
		}
		_domains.clear();
		_spareBlobs.clear();
		_spareRopes.clear();
		_loads.clear();
		_scenes.clear();
		HideShapes();
		_debugAwaitingEditor = false;
	}

	bool FluidManager::InScene(RE::ActorHandle a_actor) const
	{
		return std::ranges::any_of(_scenes, [&](const auto& a_scene) { return std::ranges::find(a_scene.second, a_actor) != a_scene.second.end(); });
	}

	bool FluidManager::SemenEnabled() const
	{
		const int mode = Settings::Get().semen;
		return mode > 0 || (mode < 0 && _ocum);
	}

	void FluidManager::Apply(Event& a_event)
	{
		const auto& settings = Settings::Get();
		switch (a_event.type) {
		case Event::Type::kClimax:
			if (settings.milk && settings.milkOnClimax) {
				const auto actor = a_event.actor.get();
				if (actor && Lactation::ClimaxAllowed(actor.get())) {
					Event milk = a_event;
					const int level = Lactation::MilkPercent(actor.get());
					milk.value = settings.milkAmount * (level >= 0 ? std::clamp(0.5f + level / 100.0f, 0.5f, 1.5f) : 1.0f);
					milk.squirt = true;
					milk.kind = kClimax;
					StartMilk(milk);
				}
			}
			StartClimax(a_event);
			break;
		case Event::Type::kLactate:
			StartMilk(a_event);
			break;
		case Event::Type::kScene:
			_scenes[a_event.thread] = a_event.actors;
			break;
		case Event::Type::kEnd:
			_scenes.erase(a_event.thread);
			std::erase_if(_emitters, [&](Emitter& a_emitter) {
				if (a_emitter.thread != a_event.thread) {
					return false;
				}
				if (a_emitter.pulses.empty()) {
					return true;
				}
				const float cutoff = a_emitter.clock + kAfterScene;
				const auto  first = std::min(a_emitter.pulse + 1, a_emitter.pulses.size());
				const auto  late = std::find_if(a_emitter.pulses.begin() + first, a_emitter.pulses.end(), [&](const Pulse& a_pulse) { return a_pulse.at > cutoff; });
				a_emitter.pulses.erase(late, a_emitter.pulses.end());
				return false;
			});
			break;
		case Event::Type::kLoad:
			{
				const float fraction = std::isfinite(a_event.value) ? std::max(a_event.value, 0.0f) : kTypicalLoad;
				_loads[a_event.actor.native_handle()] = { fraction, 0.0f };
				const float amount = fraction < kDryLoad ? 0.0f : std::clamp(fraction / kTypicalLoad, 0.2f, 2.0f) * settings.semenAmount;
				for (auto& emitter : _emitters) {
					if (emitter.actor != a_event.actor || emitter.loaded) {
						continue;
					}
					emitter.loaded = true;
					emitter.wait = 0.0f;
					if (emitter.pulses.empty()) {
						emitter.amount = amount;
					} else if (amount <= 0.0f) {
						emitter.pulses.resize(emitter.pulse + (emitter.emitted > 0 ? 1 : 0));
					} else {
						const float scale = amount / std::max(emitter.amount, 0.01f);
						for (std::size_t p = emitter.pulse + 1; p < emitter.pulses.size(); ++p) {
							emitter.pulses[p].particles = std::max(1, static_cast<int>(std::lround(emitter.pulses[p].particles * scale)));
						}
						emitter.amount = amount;
					}
				}
				if (settings.debugLog) {
					const auto actor = a_event.actor.get();
					logger::info("OCum load for {}: {:.0f}% of storage, amount x{:.2f}{}", actor ? actor->GetDisplayFullName() : "?",
						fraction * 100.0f, amount, amount <= 0.0f ? " (dry)" : "");
				}
			}
			break;
		}
	}

	void FluidManager::StartClimax(const Event& a_event)
	{
		const auto& settings = Settings::Get();
		if (!SemenEnabled()) {
			if (settings.debugLog) {
				logger::info("semen is off; set [Semen] Enabled = true, or install OCum while it is auto");
			}
			return;
		}
		const auto actor = a_event.actor.get();
		auto*      root = actor ? actor->Get3D(false) : nullptr;
		if (!root) {
			if (settings.debugLog) {
				logger::info("creature semen diagnostics: climax ignored because actor or loaded 3D is missing (actor handle {})", a_event.actor.native_handle());
			}
			return;
		}
		const auto shaft = GetShaftState(actor.get());
		if (!shaft.schlong) {
			if (settings.debugLog) {
				logger::info("{} climaxed without a schlong; no semen", actor->GetDisplayFullName());
			}
			return;
		}
		if (shaft.strapOn) {
			if (settings.debugLog) {
				logger::info("{} climaxed wearing a strap-on; no semen", actor->GetDisplayFullName());
			}
			return;
		}
		if (std::ranges::any_of(_emitters, [&](const Emitter& e) { return e.actor == a_event.actor && e.liquid == Liquid::kSemen; })) {
			return;
		}
		Emitter emitter;
		emitter.id = ++_nextEmitter;
		emitter.thread = a_event.thread;
		emitter.actor = a_event.actor;
		emitter.hosts = a_event.actors;
		emitter.amount = settings.semenAmount;
		for (const auto& pair : a_event.pairs) {
			if (pair.giver == a_event.actor && IsPenis(pair.act)) {
				emitter.pair = pair;
				break;
			}
		}
		if (auto it = _loads.find(a_event.actor.native_handle()); it != _loads.end()) {
			const float fraction = it->second.first;
			if (fraction < kDryLoad) {
				if (settings.debugLog) {
					logger::info("{} climaxed dry (OCum)", actor->GetDisplayFullName());
				}
				return;
			}
			emitter.amount = std::clamp(fraction / kTypicalLoad, 0.2f, 2.0f) * settings.semenAmount;
			emitter.loaded = true;
		} else if (_ocum && a_event.thread == 0) {
			emitter.wait = settings.ocumWait;
		}
		if (settings.debugLog) {
			logger::info("{} climaxes in thread {}: {}, amount x{:.2f}{}", actor->GetDisplayFullName(), a_event.thread,
				emitter.pair ? std::format("{} partner", ActName(emitter.pair->act)) : std::string("no penetration"), emitter.amount,
				emitter.wait > 0.0f ? std::format(", waiting up to {}s for OCum", emitter.wait) : std::string{});
		}
		_emitters.push_back(std::move(emitter));
	}

	void FluidManager::Plan(Emitter& a_emitter)
	{
		const auto&                           settings = Settings::Get();
		std::uniform_real_distribution<float> unit(0.0f, 1.0f);
		auto&                                 pulses = a_emitter.pulses;
		pulses.clear();
		const float amount = a_emitter.amount;
		const int count = settings.semenPulses;
		float     at = 0.0f;
		for (int k = 0; k < count; ++k) {
			const float progress = count > 1 ? static_cast<float>(k) / static_cast<float>(count - 1) : 0.0f;
			const float strength = std::max(0.15f, 1.0f - 0.85f * progress) * (0.85f + 0.3f * unit(_rng));
			const bool wavy = unit(_rng) < 0.6f;
			pulses.push_back({ at, (0.2f + 2.0f * strength) * settings.semenStrength, kSlotSpacing * kSpacing,
				std::max(1, static_cast<int>(std::lround(40.0f * strength * amount))), 0.03f + 0.06f * unit(_rng),
				unit(_rng) * 2.0f * std::numbers::pi_v<float>, wavy ? 0.03f + 0.09f * unit(_rng) : 0.0f, 6.0f + 8.0f * unit(_rng) });
			at += 0.5f + 0.35f * progress + 0.15f * unit(_rng);
		}
		pulses.push_back({ at, 0.08f, kDribbleSpacing * kSpacing, std::max(1, static_cast<int>(std::lround(8.0f * amount))), 0.1f, 0.0f });
	}

	void FluidManager::StartMilk(const Event& a_event)
	{
		const auto& settings = Settings::Get();
		const float amount = std::isfinite(a_event.value) ? a_event.value : 0.0f;
		if (!settings.milk || amount <= 0.0f) {
			return;
		}
		const auto actor = a_event.actor.get();
		if (!actor || !actor->Get3D(false)) {
			return;
		}
		int started = 0;
		const int firstSide = a_event.kind == kCrotch ? 2 : 0;
		const int lastSide = a_event.kind == kCrotch ? 3 : 2;
		for (int side = firstSide; side < lastSide; ++side) {
			if (!Lactation::NippleOf(actor.get(), side)) {
				continue;
			}
			auto existing = std::ranges::find_if(_emitters, [&](const Emitter& e) {
				return e.liquid == Liquid::kMilk && e.actor == a_event.actor && e.side == side;
			});
			if (existing != _emitters.end()) {
				const bool busy = existing->pulse < existing->pulses.size();
				if (busy && (a_event.kind == kDribble || a_event.kind == kCrotch || (a_event.kind == kCycle && existing->squirt))) {
					continue;  // already flowing; a drip or a regular cycle never cuts into it
				}
				if (busy && existing->pulse + 1 < existing->pulses.size()) {
					// A new squeeze replaces whatever was still queued, so jets never pile up into a backlog.
					existing->pulses.erase(existing->pulses.begin() + static_cast<std::ptrdiff_t>(existing->pulse + 1), existing->pulses.end());
				}
				const float from = existing->pulses.empty() ? existing->clock :
				                                              std::max(existing->pulses.back().at + 0.05f, existing->clock + 0.02f);
				existing->squirt = a_event.squirt;
				existing->kind = a_event.kind;
				existing->power = a_event.power;
				existing->amount = amount;
				existing->hang = 0.0f;
				if (a_event.thread != kNoThread) {
					existing->thread = a_event.thread;
				}
				PlanMilk(*existing, from);
				++started;
				continue;
			}
			Emitter emitter;
			emitter.id = ++_nextEmitter;
			emitter.thread = a_event.thread;
			emitter.actor = a_event.actor;
			emitter.hosts = a_event.actors;
			emitter.liquid = Liquid::kMilk;
			emitter.side = side;
			emitter.squirt = a_event.squirt;
			emitter.kind = a_event.kind;
			emitter.power = a_event.power;
			emitter.amount = amount;
			emitter.loaded = true;
			PlanMilk(emitter, side == 1 ? 0.07f : 0.0f);
			_emitters.push_back(std::move(emitter));
			++started;
		}
		if (settings.debugLog && started > 0) {
			logger::info("milk: {} {} from {}, amount x{:.2f}", actor->GetDisplayFullName(), a_event.squirt ? "squirts" : "leaks",
				a_event.kind == kCrotch ? std::string("the vulva") : std::format("{} nipple(s)", started), amount);
		}
	}

	void FluidManager::PlanMilk(Emitter& a_emitter, float a_from)
	{
		const auto&                           settings = Settings::Get();
		std::uniform_real_distribution<float> unit(0.0f, 1.0f);
		auto&                                 pulses = a_emitter.pulses;
		const float                           amount = std::max(a_emitter.amount, 0.05f);
		const float                           pause = std::max(settings.milkInterval, 0.0f);
		const float                           flow = std::clamp(settings.milkFlow, 0.2f, 30.0f);
		// Fill ramp: only how hard the milk comes out is scaled here; spacing, physics and pooling are untouched.
		const float                           power = std::clamp(a_emitter.power, 0.0f, 1.0f);
		const float                           low = std::max(settings.milkSpeedMin * power, 0.02f);
		const float                           high = std::max(settings.milkSpeedMax * power, low);
		const int                             fewest = std::clamp(settings.milkStreamsMin, 1, 12);
		const int                             most = std::clamp(settings.milkStreamsMax, 1, 12);
		const float                           cone = settings.milkCone;
		const float                           angle = unit(_rng) * 2.0f * std::numbers::pi_v<float>;
		constexpr float                       kSegment = 0.5f;
		float                                 at = a_from;

		// Streams follow the strength: the minimum count at the minimum strength, the maximum count at the maximum.
		const auto streamsAt = [&](float a_speed) {
			const float t = high - low > 1e-4f ? std::clamp((a_speed - low) / (high - low), 0.0f, 1.0f) : 1.0f;
			return std::clamp(static_cast<int>(std::lround(fewest + (most - fewest) * t)), 1, 12);
		};
		// One continuous piece of flow. Pieces chain into each other so the streams never break between them.
		const auto add = [&](float a_duration, float a_speed, int a_streams, float a_share) {
			// Particles stay under 2 spacings apart so each stream remains one linked strand; the amount only
			// changes how densely they are packed (more milk, more particles, more cost).
			// Fast jets are sparse (the strand mesh bridges the gaps), slow trickles a bit denser. The amount mostly
			// changes how big the milk looks, not how many particles it costs.
			const float base = a_speed > 0.4f ? 3.2f : 1.8f;
			const float spacing = std::clamp(base * kSpacing / std::pow(std::max(amount * a_share, 0.05f), 0.25f), 1.3f * kSpacing, 4.0f * kSpacing);
			const int   perStream = std::max(2, static_cast<int>(std::lround(a_duration * a_speed / spacing)));
			const bool  chain = !pulses.empty() && pulses.back().streams == a_streams && pulses.back().chain;
			Pulse       pulse{ at, a_speed, spacing, perStream * a_streams, a_streams > 1 ? 0.01f : 0.015f, angle, 0.0f, 0.0f };
			pulse.streams = a_streams;
			pulse.cone = a_streams > 1 ? cone : 0.0f;
			pulse.chain = true;
			pulse.taper = false;
			if (!chain && !pulses.empty()) {
				pulses.back().chain = false;
			}
			pulses.push_back(pulse);
			at += a_duration;
		};
		const auto steady = [&](float a_duration, float a_speed, int a_streams, float a_share) {
			for (float done = 0.0f; done < a_duration - 1e-3f; done += kSegment) {
				add(std::min(kSegment, a_duration - done), a_speed, a_streams, a_share);
			}
		};
		// Strength (and with it the stream count) easing from a_from to a_to over a_duration.
		const auto ramp = [&](float a_duration, float a_from, float a_to, float a_share) {
			const int steps = std::max(1, static_cast<int>(std::ceil(a_duration / kSegment)));
			for (int k = 0; k < steps; ++k) {
				const float t = steps > 1 ? static_cast<float>(k) / static_cast<float>(steps - 1) : 0.0f;
				const float speed = a_from + (a_to - a_from) * t;
				add(a_duration / steps, speed, streamsAt(speed), a_share);
			}
		};
		const auto finish = [&]() {
			// A short drip at the end so the last of the milk runs off the nipple.
			Pulse tail{ at, std::min(low, 0.08f), kDribbleSpacing * kSpacing, std::max(3, static_cast<int>(std::lround(8.0f * amount))), 0.05f, angle };
			pulses.push_back(tail);
		};

		const int mode = settings.milkMode;
		switch (a_emitter.kind) {
		case kSqueeze:
		case kClimax:
			{
				// Sex and SGO milking: full strength in every mode except "dribbling".
				const float duration = a_emitter.kind == kSqueeze ? std::min(flow, 1.0f) : std::min(flow, 2.5f);
				if (mode == 0) {
					steady(duration, low, streamsAt(low), 1.0f);
				} else {
					steady(duration, high, most, 1.0f);
				}
				finish();
			}
			return;
		case kDribble:
			steady(std::min(1.0f, flow), low, fewest, 0.5f);
			return;
		case kCrotch:
			{
				// A few slow drops off the vulva: each a short hanging strand that lets go.
				const int drops = 2 + static_cast<int>(unit(_rng) * 3.0f);
				for (int k = 0; k < drops; ++k) {
					Pulse drop{ at, 0.03f + 0.03f * unit(_rng), kDribbleSpacing * kSpacing,
						std::max(3, static_cast<int>(std::lround((5.0f + 4.0f * unit(_rng)) * std::min(amount, 3.0f)))), 0.04f, angle };
					pulses.push_back(drop);
					at += 0.5f + 0.9f * unit(_rng);
				}
			}
			return;
		default:
			break;
		}
		switch (mode) {
		case 2:  // generous cow: full strength easing down to the minimum
			ramp(flow, high, low, 1.0f);
			finish();
			break;
		case 3:  // overflowing cow: as above, then it keeps seeping through the pause
			ramp(flow, high, low, 1.0f);
			steady(std::min(pause, 60.0f), low, fewest, 0.35f);
			finish();
			break;
		case 4:  // cow in heat: full strength throughout, streams thinning out, held until the next cycle
			{
				const int steps = std::max(1, static_cast<int>(std::ceil(flow / kSegment)));
				for (int k = 0; k < steps; ++k) {
					const float t = steps > 1 ? static_cast<float>(k) / static_cast<float>(steps - 1) : 0.0f;
					add(flow / steps, high, std::clamp(static_cast<int>(std::lround(most + (fewest - most) * t)), 1, 12), 1.0f);
				}
				steady(std::min(pause, 60.0f), high, fewest, 1.0f);
			}
			break;
		default:  // dribbling / conditional cow: the minimum strength while it flows
			steady(flow, low, streamsAt(low), 1.0f);
			finish();
			break;
		}
	}

	void FluidManager::Regroup(Domain& a_domain)
	{
		// Several streams emit into one pool at once, so their particles interleave. The solver and the renderer
		// link each particle to the next one in the array, so put every stream's particles back together.
		auto& particles = a_domain.particles;
		if (particles.size() < 2) {
			return;
		}
		bool                              needed = false;
		std::unordered_set<std::uint32_t> seen;
		std::uint32_t                     current = particles.front().run;
		seen.insert(current);
		for (const auto& particle : particles) {
			if (particle.run != current) {
				current = particle.run;
				if (!seen.insert(current).second) {
					needed = true;
					break;
				}
			}
		}
		if (!needed) {
			return;
		}
		std::vector<std::size_t> order(particles.size());
		std::iota(order.begin(), order.end(), std::size_t{ 0 });
		std::ranges::stable_sort(order, [&](std::size_t a_left, std::size_t a_right) { return particles[a_left].run < particles[a_right].run; });
		std::vector<std::size_t>     remap(particles.size());
		std::vector<Fluid::Particle> sorted;
		sorted.reserve(particles.size());
		for (std::size_t k = 0; k < order.size(); ++k) {
			remap[order[k]] = k;
			sorted.push_back(particles[order[k]]);
		}
		// A run's last particle must not point at the next run's first.
		for (std::size_t k = 0; k + 1 < sorted.size(); ++k) {
			if (sorted[k].run != sorted[k + 1].run) {
				sorted[k].link = 0.0f;
			}
		}
		if (!sorted.empty()) {
			sorted.back().link = 0.0f;
		}
		particles = std::move(sorted);
		for (auto& anchor : a_domain.anchors) {
			if (anchor.particle < remap.size()) {
				anchor.particle = remap[anchor.particle];
			}
		}
		for (auto& blob : a_domain.blobs) {
			if (blob.live && blob.seed < remap.size()) {
				blob.seed = remap[blob.seed];
			}
		}
	}

	bool FluidManager::Step(Emitter& a_emitter, float a_delta)
	{
		const auto& settings = Settings::Get();
		if (a_emitter.wait > 0.0f) {
			a_emitter.wait -= a_delta;
			if (a_emitter.wait > 0.0f) {
				return true;
			}
		}
		if (a_emitter.pulses.empty()) {
			if (a_emitter.amount <= 0.0f) {
				return false;
			}
			Plan(a_emitter);
		}
		const auto actor = a_emitter.actor.get();

		std::vector<Point> rim;
		RE::NiPoint3       center;
		RE::NiPoint3       inward;
		RE::NiPoint3       axis;
		RE::NiPoint3       nozzle;
		bool               inside = false;
		std::vector<RE::ActorHandle> hosts = a_emitter.hosts;
		if (std::ranges::find(hosts, a_emitter.actor) == hosts.end()) {
			hosts.push_back(a_emitter.actor);
		}
		const bool milk = a_emitter.liquid == Liquid::kMilk;
		const auto tip = milk ? Lactation::NippleOf(actor.get(), a_emitter.side) : Body::TipOf(actor.get());
		if (!tip || !actor) {
			if (settings.debugLog) {
				logger::info("{} emitter {} stopped; no {} available for {}", LiquidName(a_emitter.liquid), a_emitter.id,
					milk ? (a_emitter.side == 2 ? "vulva" : "nipple") : "genital tip", actor ? actor->GetDisplayFullName() : "unloaded actor");
			}
			return false;
		}
		axis = tip->axis;
		const RE::NiPoint3 end = tip->point;
		nozzle = end + axis * (milk ? kMilkNozzleOffset : kNozzleOffset);
		if (a_emitter.pair) {
			if (std::ranges::find(hosts, a_emitter.pair->receiver) == hosts.end()) {
				hosts.push_back(a_emitter.pair->receiver);
			}
			const auto receiver = a_emitter.pair->receiver.get();
			auto*      receiver3D = receiver ? receiver->Get3D(false) : nullptr;
			if (receiver3D && !FindOpening(receiver3D, OpeningOf(a_emitter.pair->act), rim, center, inward)) {
				inside = (end - center).Dot(inward) > 0.0f;
			}
		}
		if (a_emitter.hasNozzle && a_delta > 0.0f) {
			RE::NiPoint3 measured = (nozzle - a_emitter.lastNozzle) / (a_delta * kUnitsPerMeter);
			if (const float speed = measured.Length(); speed > kMaxNozzleSpeed) {
				measured = measured * (kMaxNozzleSpeed / speed);
			}
			const float blend = 1.0f - std::exp(-a_delta / kCarrySmoothing);
			a_emitter.carried = a_emitter.carried + (measured - a_emitter.carried) * blend;
		}
		const RE::NiPoint3 carried = a_emitter.carried * kCarry;
		a_emitter.lastNozzle = nozzle;
		a_emitter.hasNozzle = true;
		if (inside != a_emitter.inside && settings.debugLog) {
			logger::info("{} is now {} the opening", actor->GetDisplayFullName(), inside ? "inside" : "outside");
		}
		a_emitter.inside = inside;
		a_emitter.clock += a_delta;

		Domain* domain = FindDomain(a_emitter.domain);
		if (domain && !Holds(*domain, actor.get(), nozzle)) {
			domain = nullptr;
		}
		if (!domain) {
			domain = DomainFor(actor.get(), nozzle, hosts, a_emitter.liquid);
			if (!domain) {
				return false;
			}
			a_emitter.domain = domain->id;
		}
		if (a_emitter.pair) {
			const std::pair opening{ a_emitter.pair->receiver, OpeningOf(a_emitter.pair->act) };
			if (std::ranges::find(domain->openings, opening) == domain->openings.end()) {
				domain->openings.push_back(opening);
			}
		}

		std::uniform_real_distribution<float> unit(0.0f, 1.0f);
		const RE::NiPoint3                    side = Normalized(axis.Cross({ 0.0f, 0.0f, 1.0f }), { 1.0f, 0.0f, 0.0f });
		const RE::NiPoint3                    lift = side.Cross(axis);
		auto&                                 particles = domain->particles;
		const std::size_t                     capacity = static_cast<std::size_t>(settings.fluidMaxParticles);
		const auto                            source = [&]() {
			if (!a_emitter.oozing || rim.empty()) {
				return std::pair{ nozzle, axis };
			}
			return std::pair{ center - inward * kExitDepth, -inward };
		};

		const float                now = a_emitter.clock - domain->accumulator;
		std::optional<std::size_t> newest;
		while (a_emitter.pulse < a_emitter.pulses.size()) {
			const Pulse& pulse = a_emitter.pulses[a_emitter.pulse];
			if (now < pulse.at) {
				break;
			}
			if (a_emitter.emitted == 0 && a_emitter.slot <= 0.0f) {
				const int streams = std::max(pulse.streams, 1);
				const bool keep = milk && a_emitter.pulse > 0 && a_emitter.pulses[a_emitter.pulse - 1].chain && a_emitter.streams == streams && a_emitter.run != 0;
				if (!keep) {
					a_emitter.run = _nextRun + 1;
					_nextRun += static_cast<std::uint32_t>(streams);
				}
				a_emitter.streams = streams;
				a_emitter.oozing = inside && !rim.empty();
				a_emitter.count = a_emitter.oozing ? std::max(1, static_cast<int>(std::lround(pulse.particles * settings.semenOverflow))) :
				                                     pulse.particles;
				if (a_emitter.oozing) {
					Deposit(a_emitter, static_cast<float>(pulse.particles - a_emitter.count));
				}
			}
			const auto [origin, outlet] = source();
			const bool         oozing = a_emitter.oozing && !rim.empty();
			const float        spacing = oozing ? kDribbleSpacing * kSpacing : pulse.spacing;
			const float        elapsed = now - pulse.at;
			const float        tilt = oozing ? 0.0f : std::tan(pulse.spread);
			const RE::NiPoint3 aim = Normalized(outlet + (side * std::cos(pulse.angle) + lift * std::sin(pulse.angle)) * tilt, outlet);
			const int streams = std::max(pulse.streams, 1);
			while (a_emitter.emitted < a_emitter.count && a_emitter.slot <= elapsed) {
				const float taper = pulse.taper ? Taper(static_cast<float>(a_emitter.emitted) / static_cast<float>(a_emitter.count)) : 1.0f;
				const float speed = oozing ? kOozeSpeed : pulse.speed * taper;
				const float age = elapsed - a_emitter.slot;
				const int   stream = a_emitter.emitted % streams;
				a_emitter.slot += spacing / std::max(speed, 0.02f) / static_cast<float>(streams);
				++a_emitter.emitted;
				if (particles.size() >= capacity && milk && capacity > 0) {
					// Full pool: start fading the oldest milk so the stream keeps flowing.
					// Pool full: let the oldest milk vanish quickly so new milk is never held back for seconds.
					const std::size_t oldest = std::min<std::size_t>(particles.size(), std::max<std::size_t>(16, capacity / 25));
					for (std::size_t k = 0; k < oldest; ++k) {
						particles[k].life = std::min(particles[k].life, 0.25f);
					}
				}
				// Milk may overrun the cap a little while the oldest fades, so the stream never stalls.
				if (particles.size() >= (milk ? capacity + capacity / 5 + 32 : capacity)) {
					continue;
				}
				const float        sway = oozing ? 0.0f : pulse.wave * std::sin(2.0f * std::numbers::pi_v<float> * pulse.frequency * (a_emitter.slot - age));
				const RE::NiPoint3 bend = (lift * std::cos(pulse.angle) - side * std::sin(pulse.angle)) * sway;
				const RE::NiPoint3 wobble = (side * (unit(_rng) - 0.5f) + lift * (unit(_rng) - 0.5f)) * (oozing ? 0.3f : milk ? 0.006f : 0.03f);
				RE::NiPoint3 streamAim = aim;
				RE::NiPoint3 ringOffset;
				if (streams > 1) {
					// Shower head: every stream leaves the same point, fanned out evenly around the nipple axis.
					const float ring = pulse.angle + 2.0f * std::numbers::pi_v<float> * static_cast<float>(stream) / static_cast<float>(streams);
					streamAim = Normalized(outlet + (side * std::cos(ring) + lift * std::sin(ring)) * std::tan(pulse.cone), outlet);
					ringOffset = (side * std::cos(ring) + lift * std::sin(ring)) * 0.25f;
				}
				// Milk leaves the nipple angled slightly down, so even the hardest jet bends into an arc.
				const RE::NiPoint3 droop = milk && a_emitter.side != 2 ? RE::NiPoint3{ 0.0f, 0.0f, -0.18f } : RE::NiPoint3{};
				const RE::NiPoint3 direction = Normalized(streamAim + droop + bend + wobble, streamAim);
				const RE::NiPoint3 velocity = direction * speed + carried * (oozing ? 0.5f : 1.0f);
				const float        turn = unit(_rng) * 2.0f * std::numbers::pi_v<float>;
				const RE::NiPoint3 disc = oozing ? (side * std::cos(turn) + lift * std::sin(turn)) * (kGlobDisc * kSpacing * kUnitsPerMeter * std::sqrt(unit(_rng))) :
				                                   RE::NiPoint3{};
				Fluid::Particle    particle;
				particle.life = (milk ? settings.milkLifetime : settings.fluidLifetime) * (0.85f + 0.3f * unit(_rng));
				RE::NiPoint3 offset = disc;
				if (milk && streams > 1) {
					offset += ringOffset;  // a few millimetres apart: still one nipple, no pile-up
				}
				particle.position = ToLocal(*domain, origin + offset + velocity * (age * kUnitsPerMeter));
				particle.velocity = ToVec(velocity);
				particle.run = a_emitter.run + static_cast<std::uint32_t>(stream);
				{
					// Link to the previous particle of the same stream (Regroup makes them neighbours afterwards).
					std::size_t previous = Fluid::kNone;
					for (std::size_t k = particles.size(); k > 0; --k) {
						if (particles[k - 1].run == particle.run) {
							previous = k - 1;
							break;
						}
					}
					if (previous != Fluid::kNone) {
						const float gap = Fluid::Length(particle.position - particles[previous].position);
						if (gap < (milk ? kMilkLinkReach : 2.0f) * kSpacing) {
							particles[previous].link = std::max(spacing, gap);
						}
					}
				}
				newest = particles.size();
				particles.push_back(particle);
			}
			if (a_emitter.emitted < a_emitter.count) {
				break;
			}
			++a_emitter.pulse;
			a_emitter.emitted = 0;
			a_emitter.slot = 0.0f;
		}
		const Fluid::Vec3 hold = ToLocal(*domain, source().first);
		if (milk && a_emitter.streams > 1) {
			newest.reset();
			std::erase_if(domain->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_emitter.id; });
		}
		const auto        held = std::ranges::find_if(domain->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_emitter.id; });
		if (newest) {
			const float rest = std::max(Fluid::Length(particles[*newest].position - hold), 0.5f * kSpacing);
			if (held != domain->anchors.end()) {
				*held = { *newest, hold, rest, a_emitter.id };
			} else {
				domain->anchors.push_back({ *newest, hold, rest, a_emitter.id });
			}
		} else if (held != domain->anchors.end()) {
			held->point = hold;
		}
		if (a_emitter.pulse < a_emitter.pulses.size()) {
			return true;
		}
		a_emitter.hang += a_delta;
		const bool holding = std::ranges::any_of(domain->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_emitter.id; });
		if (holding && a_emitter.hang < kHangTime) {
			return true;
		}
		std::erase_if(domain->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_emitter.id; });
		return false;
	}

	void FluidManager::Deposit(const Emitter& a_emitter, float a_particles)
	{
		if (!a_emitter.pair || a_particles <= 0.0f) {
			return;
		}
		const Opening opening = OpeningOf(a_emitter.pair->act);
		auto          it = std::ranges::find_if(_reservoirs, [&](const Reservoir& r) {
			return r.actor == a_emitter.pair->receiver && r.opening == opening;
		});
		if (it == _reservoirs.end()) {
			Reservoir reservoir;
			std::uniform_real_distribution<float> unit(0.0f, 1.0f);
			reservoir.id = ++_nextEmitter;
			reservoir.actor = a_emitter.pair->receiver;
			reservoir.phase = unit(_rng) * 2.0f * std::numbers::pi_v<float>;
			reservoir.period = 1.4f + unit(_rng);
			reservoir.opening = opening;
			_reservoirs.push_back(std::move(reservoir));
			it = std::prev(_reservoirs.end());
		}
		it->plug = a_emitter.actor;
		it->hosts = a_emitter.hosts;
		it->volume += a_particles;
		it->plugged = true;
		it->age = 0.0f;
	}

	bool FluidManager::Leak(Reservoir& a_reservoir, float a_delta)
	{
		const auto& settings = Settings::Get();
		a_reservoir.age += a_delta;
		const auto actor = a_reservoir.actor.get();
		auto*      root = actor ? actor->Get3D(false) : nullptr;
		const bool holding = std::ranges::any_of(_domains, [&](const auto& d) {
			return std::ranges::any_of(d->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_reservoir.id; });
		});
		const bool empty = a_reservoir.volume < 0.5f && a_reservoir.gush < 0.5f;
		const bool dressed = root && !Revealing::Exposed(actor.get());
		if (dressed) {
			if (a_reservoir.dressed <= 0.0f && settings.debugLog) {
				logger::info("{} is dressed; their {} spills for {}s more", actor->GetDisplayFullName(), OpeningName(a_reservoir.opening), kDressedGrace);
			}
			a_reservoir.dressed += a_delta;
		}
		if (!root || a_reservoir.dressed > kDressedGrace || a_reservoir.age > kReservoirLife || (empty && !holding)) {
			if (settings.debugLog && actor) {
				logger::info("{}'s {} stops leaking with {:.0f} left", actor->GetDisplayFullName(), OpeningName(a_reservoir.opening), a_reservoir.volume);
			}
			return false;
		}
		std::vector<Point> rim;
		RE::NiPoint3       center;
		RE::NiPoint3       inward;
		if (FindOpening(root, a_reservoir.opening, rim, center, inward)) {
			return false;
		}
		const RE::NiPoint3 outward = -inward;
		const RE::NiPoint3 exit = center + outward * kExitDepth;
		const auto         plug = a_reservoir.plug.get();
		const bool         plugged = !dressed && TipInside(plug.get(), center, inward);
		if (a_reservoir.plugged && !plugged && a_reservoir.volume >= 1.0f) {
			const float gush = a_reservoir.volume * kGushShare;
			a_reservoir.gush += gush;
			a_reservoir.volume -= gush;
			if (settings.debugLog) {
				logger::info("{} pulls out of {}'s {}: {:.0f} gush out, {:.0f} still inside", plug ? plug->GetDisplayFullName() : "?",
					actor->GetDisplayFullName(), OpeningName(a_reservoir.opening), gush, a_reservoir.volume);
			}
		}
		a_reservoir.plugged = plugged;

		std::vector<RE::ActorHandle> hosts = a_reservoir.hosts;
		if (std::ranges::find(hosts, a_reservoir.actor) == hosts.end()) {
			hosts.push_back(a_reservoir.actor);
		}
		Domain* domain = FindDomain(a_reservoir.domain);
		if (domain && !Holds(*domain, actor.get(), exit)) {
			domain = nullptr;
		}
		if (!domain) {
			domain = DomainFor(actor.get(), exit, hosts, Liquid::kSemen);
			if (!domain) {
				return false;
			}
			a_reservoir.domain = domain->id;
		}
		const std::pair opening{ a_reservoir.actor, a_reservoir.opening };
		if (std::ranges::find(domain->openings, opening) == domain->openings.end()) {
			domain->openings.push_back(opening);
		}

		const float facing = std::clamp(0.5f - outward.z, 0.05f, 1.0f);
		const float wave = std::max(0.0f, std::sin(2.0f * std::numbers::pi_v<float> * a_reservoir.age / a_reservoir.period + a_reservoir.phase));
		const float pulse = 0.25f + 1.5f * wave * wave;
		float       rate = (plugged ? a_reservoir.volume * kSeep : a_reservoir.volume * kLeak * facing) * pulse;
		if (!plugged && a_reservoir.gush > 0.0f) {
			rate += kGushRate * std::max(facing, 0.3f);
		}
		RE::NiPoint3 carried;
		if (a_reservoir.hasExit && a_delta > 0.0f) {
			carried = (exit - a_reservoir.lastExit) / (a_delta * kUnitsPerMeter);
			if (const float speed = carried.Length(); speed > kMaxNozzleSpeed) {
				carried = carried * (kMaxNozzleSpeed / speed);
			}
		}
		a_reservoir.lastExit = exit;
		a_reservoir.hasExit = true;
		a_reservoir.release += rate * a_delta;

		std::uniform_real_distribution<float> unit(0.0f, 1.0f);
		const RE::NiPoint3                    side = Normalized(outward.Cross({ 0.0f, 0.0f, 1.0f }), { 1.0f, 0.0f, 0.0f });
		const RE::NiPoint3                    lift = side.Cross(outward);
		auto&                                 particles = domain->particles;
		const std::size_t                     capacity = static_cast<std::size_t>(settings.fluidMaxParticles);
		const Fluid::Vec3                     hold = ToLocal(*domain, exit);
		std::optional<std::size_t>            newest;
		while (a_reservoir.release >= 1.0f && (a_reservoir.volume >= 1.0f || a_reservoir.gush >= 1.0f)) {
			a_reservoir.release -= 1.0f;
			if (a_reservoir.gush >= 1.0f) {
				a_reservoir.gush -= 1.0f;
			} else {
				a_reservoir.volume -= 1.0f;
			}
			if (particles.size() >= capacity) {
				continue;
			}
			if (a_reservoir.glob <= 0) {
				a_reservoir.run = ++_nextRun;
				a_reservoir.glob = std::uniform_int_distribution<int>(kGlobMin, kGlobMax)(_rng);
				a_reservoir.globSpeed = 0.5f + 0.8f * unit(_rng);
			}
			--a_reservoir.glob;
			const RE::NiPoint3 wobble = (side * (unit(_rng) - 0.5f) + lift * (unit(_rng) - 0.5f)) * 0.4f;
			const RE::NiPoint3 velocity = Normalized(outward + wobble, outward) * (kLeakSpeed * a_reservoir.globSpeed) + carried * 0.5f;
			const float        turn = unit(_rng) * 2.0f * std::numbers::pi_v<float>;
			const RE::NiPoint3 disc = (side * std::cos(turn) + lift * std::sin(turn)) * (kGlobDisc * kSpacing * kUnitsPerMeter * std::sqrt(unit(_rng)));
			Fluid::Particle    particle;
			particle.life = settings.fluidLifetime * (0.85f + 0.3f * unit(_rng));
			particle.position = ToLocal(*domain, exit + disc + velocity * (unit(_rng) * a_delta * kUnitsPerMeter));
			particle.velocity = ToVec(velocity);
			particle.run = a_reservoir.run;
			if (!particles.empty() && particles.back().run == a_reservoir.run) {
				const float gap = Fluid::Length(particle.position - particles.back().position);
				if (gap < 2.0f * kSpacing) {
					particles.back().link = std::max(kDribbleSpacing * kSpacing, gap);
				}
			}
			newest = particles.size();
			particles.push_back(particle);
		}
		if (a_reservoir.volume < 1.0f && a_reservoir.gush < 1.0f) {
			a_reservoir.release = 0.0f;
		}
		const auto held = std::ranges::find_if(domain->anchors, [&](const Fluid::Anchor& a) { return a.owner == a_reservoir.id; });
		if (newest) {
			const float rest = std::max(Fluid::Length(particles[*newest].position - hold), 0.5f * kSpacing);
			if (held != domain->anchors.end()) {
				*held = { *newest, hold, rest, a_reservoir.id };
			} else {
				domain->anchors.push_back({ *newest, hold, rest, a_reservoir.id });
			}
		} else if (held != domain->anchors.end()) {
			held->point = hold;
		}
		return true;
	}

	FluidManager::Domain* FluidManager::FindDomain(std::uint32_t a_id)
	{
		if (a_id == 0) {
			return nullptr;
		}
		auto it = std::ranges::find_if(_domains, [&](const auto& d) { return d->id == a_id; });
		return it != _domains.end() ? it->get() : nullptr;
	}

	bool FluidManager::Holds(const Domain& a_domain, RE::Actor* a_actor, const RE::NiPoint3& a_point)
	{
		const RE::NiPoint3 offset = a_point - a_domain.origin;
		return !a_domain.broken && a_domain.cell && a_domain.cell == a_actor->GetParentCell() && std::hypot(offset.x, offset.y) < kDomainReach &&
		       std::abs(offset.z) < kDomainReach;
	}

	FluidManager::Domain* FluidManager::DomainFor(RE::Actor* a_actor, const RE::NiPoint3& a_point, const std::vector<RE::ActorHandle>& a_hosts, Liquid a_liquid)
	{
		const auto& settings = Settings::Get();
		auto*       cell = a_actor->GetParentCell();
		if (!cell) {
			return nullptr;
		}
		float ground = a_actor->GetPosition().z;
		for (const auto& handle : a_hosts) {
			if (const auto host = handle.get()) {
				ground = std::min(ground, host->GetPosition().z);
			}
		}
		if (const auto hit = GroundBelow(cell, a_point)) {
			ground = *hit;
		}
		for (auto& domain : _domains) {
			const RE::NiPoint3 offset = a_point - domain->origin;
			if (!domain->broken && domain->liquid == a_liquid && domain->cell == cell && std::abs(domain->origin.z - ground) < kDomainStep &&
				std::hypot(offset.x, offset.y) < kDomainReach) {
				for (const auto& handle : a_hosts) {
					if (std::ranges::find(domain->hosts, handle) == domain->hosts.end()) {
						domain->hosts.push_back(handle);
					}
				}
				return domain.get();
			}
		}
		if (_domains.size() >= kMaxDomains) {
			Release(*_domains.front());
			_domains.erase(_domains.begin());
		}
		auto domain = std::make_unique<Domain>();
		domain->id = ++_nextDomain;
		domain->origin = { a_point.x, a_point.y, ground };
		domain->cell = cell;
		domain->hosts = a_hosts;
		_warm = true;
		domain->blobs.reserve(kMaxBlobs);
		domain->ropes.reserve(kMaxRopes);
		domain->liquid = a_liquid;
		domain->solver = std::make_unique<Fluid::Solver>(kSpacing, ParametersFor(a_liquid));
		if (settings.debugLog) {
			logger::info("{} pool {} opened, floor {:.1f} units below the emitter", LiquidName(a_liquid), domain->id,
				a_point.z - ground);
		}
		_domains.push_back(std::move(domain));
		return _domains.back().get();
	}

	bool FluidManager::Attach(Domain& a_domain)
	{
		RE::NiAVObject* host = nullptr;
		RE::NiAVObject* player = nullptr;
		for (const auto& handle : a_domain.hosts) {
			const auto actor = handle.get();
			auto*      model = actor ? actor->Get3D(false) : nullptr;
			if (!model || !model->parent) {
				continue;
			}
			if (actor->IsPlayerRef()) {
				player = player ? player : model;
			} else if (!host) {
				host = model;
			}
		}
		if (!host) {
			host = player;
		}
		if (!host) {
			return false;
		}
		RE::NiNode* parent = host->parent;
		if (!a_domain.root) {
			a_domain.root.reset(RE::NiNode::Create(0));
			if (!a_domain.root) {
				return false;
			}
			a_domain.root->name = "OStrandsFluid";
			a_domain.root->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
		}
		if (a_domain.root->parent != parent) {
			if (a_domain.root->parent) {
				a_domain.root->parent->DetachChild2(a_domain.root.get());
			}
			parent->AttachChild(a_domain.root.get(), true);
		}
		a_domain.root->local = parent->world.Invert();
		a_domain.root->SetAppCulled(!a_domain.diagnostic && host->GetAppCulled());
		return true;
	}

	void FluidManager::Simulate(Domain& a_domain, float a_delta)
	{
		a_domain.age += a_delta;
		if (a_domain.particles.empty()) {
			a_domain.accumulator = 0.0f;
			return;
		}
		// Milk runs at a coarser physics rate with fewer catch-up steps: it is mostly free flight and pooling,
		// and this is where most of the frame time went with thousands of particles.
		const bool  milkPool = a_domain.liquid == Liquid::kMilk;
		const float step = milkPool ? kMilkStep : kStep;
		const int   maxSteps = milkPool ? kMilkMaxSteps : kMaxSteps;
		a_domain.accumulator = std::min(a_domain.accumulator + a_delta, maxSteps * step);
		auto& capsules = a_domain.capsules;
		auto& holes = a_domain.holes;
		auto& patches = a_domain.skin.Patches();
		capsules.clear();
		holes.clear();
		patches.clear();
		const auto add = [&](std::span<const Body::Capsule> a_shapes) {
			for (const auto& capsule : a_shapes) {
				capsules.push_back({ ToLocal(a_domain, capsule.a), ToLocal(a_domain, capsule.b), capsule.radius / kUnitsPerMeter,
					ToVec(capsule.velocityA / kUnitsPerMeter), ToVec(capsule.velocityB / kUnitsPerMeter) });
			}
		};
		Vec3 low = a_domain.particles.front().position;
		Vec3 high = low;
		for (const auto& particle : a_domain.particles) {
			low = { std::min(low.x, particle.position.x), std::min(low.y, particle.position.y), std::min(low.z, particle.position.z) };
			high = { std::max(high.x, particle.position.x), std::max(high.y, particle.position.y), std::max(high.z, particle.position.z) };
		}
		low = low - Vec3{ kSkinMargin, kSkinMargin, kSkinMargin };
		high = high + Vec3{ kSkinMargin, kSkinMargin, kSkinMargin };
		// Only the skin near the milk has to be exact this frame (cells of kNearCell units, one cell of margin).
		{
			std::unordered_set<std::int64_t> occupied;
			for (const auto& particle : a_domain.particles) {
				occupied.insert(Body::CellKey(ToWorld(a_domain, particle.position), kNearCell));
			}
			_nearCells.clear();
			for (const auto key : occupied) {
				const std::int64_t ix = (key >> 42) & 0x1FFFFF, iy = (key >> 21) & 0x1FFFFF, iz = key & 0x1FFFFF;
				for (int dx = -1; dx <= 1; ++dx) {
					for (int dy = -1; dy <= 1; ++dy) {
						for (int dz = -1; dz <= 1; ++dz) {
							_nearCells.insert((((ix + dx) & 0x1FFFFF) << 42) | (((iy + dy) & 0x1FFFFF) << 21) | ((iz + dz) & 0x1FFFFF));
						}
					}
				}
			}
		}
		// Everyone near the milk (NPCs, enemies, animals), not only the actors in the scene, re-checked a few times a second.
		if ((a_domain.nearbyAge += a_delta) > kNearbyEvery) {
			a_domain.nearbyAge = 0.0f;
			const RE::NiPoint3 lo = ToWorld(a_domain, low) - RE::NiPoint3{ kNearbyReach, kNearbyReach, kNearbyBelow };
			const RE::NiPoint3 hi = ToWorld(a_domain, high) + RE::NiPoint3{ kNearbyReach, kNearbyReach, kNearbyReach };
			std::vector<std::pair<float, RE::ActorHandle>> found;
			const RE::NiPoint3 middle = (lo + hi) * 0.5f;
			const auto consider = [&](RE::Actor* a_actor) {
				if (!a_actor || !a_actor->Is3DLoaded() || a_actor->IsDisabled() || a_actor->IsDeleted()) {
					return;
				}
				const RE::ActorHandle handle = a_actor->GetHandle();
				if (std::ranges::find(a_domain.hosts, handle) != a_domain.hosts.end()) {
					return;
				}
				const RE::NiPoint3 at = a_actor->GetPosition();
				if (at.x < lo.x || at.y < lo.y || at.z < lo.z || at.x > hi.x || at.y > hi.y || at.z > hi.z) {
					return;
				}
				found.emplace_back(at.GetSquaredDistance(middle), handle);
			};
			consider(RE::PlayerCharacter::GetSingleton());
			if (auto* lists = RE::ProcessLists::GetSingleton()) {
				for (const auto& handle : lists->highActorHandles) {
					consider(handle.get().get());
				}
			}
			std::ranges::sort(found, {}, &std::pair<float, RE::ActorHandle>::first);
			// Reading a newcomer's body mesh the first time is the costly part: take at most one new actor per look,
			// so several people walking up at once do not land in the same frame.
			const std::vector<RE::ActorHandle> before = std::move(a_domain.nearby);
			a_domain.nearby.clear();
			bool added = false;
			for (std::size_t k = 0; k < found.size() && a_domain.nearby.size() < kNearbyMax; ++k) {
				const bool known = std::ranges::find(before, found[k].second) != before.end();
				if (!known && added) {
					continue;
				}
				added = added || !known;
				a_domain.nearby.push_back(found[k].second);
			}
		}
		std::vector<RE::ActorHandle> bodies = a_domain.hosts;
		bodies.insert(bodies.end(), a_domain.nearby.begin(), a_domain.nearby.end());
		for (const auto& handle : bodies) {
			const auto actor = handle.get();
			if (!actor || !actor->Is3DLoaded()) {
				continue;
			}
			const bool bystander = std::ranges::find(a_domain.hosts, handle) == a_domain.hosts.end();
			const auto surface = Body::SurfaceOf(actor.get(), _nearCells, kNearCell, bystander ? kBystanderRefresh : 3);
			if (surface.patches.empty()) {
				add(Body::Of(actor.get()));
				add(Body::ShaftOf(actor.get()));
				continue;
			}
			for (const auto& patch : surface.patches) {
				const Vec3 position = ToLocal(a_domain, patch.position);
				if (position.x < low.x || position.y < low.y || position.z < low.z || position.x > high.x || position.y > high.y ||
					position.z > high.z) {
					continue;
				}
				patches.push_back({ position, ToVec(patch.normal), ToVec(patch.velocity / kUnitsPerMeter) });
			}
			add(surface.extras);
		}
		// Walls and pillars: same treatment as skin (collision, running down, stickiness), after the body patches.
		ProbeWorld(a_domain);
		GuardPaths(a_domain, a_domain.accumulator + step);
		a_domain.worldFrom = capsules.size() + patches.size();
		a_domain.worldOrder.clear();
		std::unordered_map<RE::FormID, std::pair<bool, RE::NiPoint3>> present;  // door shut / object still there, once per frame
		for (std::size_t w = 0; w < a_domain.world.size(); ++w) {
			const auto& wall = a_domain.world[w];
			if (w < a_domain.worldOwner.size() && a_domain.worldOwner[w].id != 0) {
				const auto& owner = a_domain.worldOwner[w];
				auto        it = present.find(owner.id);
				if (it == present.end()) {
					const auto* ref = RE::TESForm::LookupByID<RE::TESObjectREFR>(owner.id);
					const bool  there = ref && !ref->IsDisabled() && !ref->IsDeleted() && ref->Is3DLoaded() && (!IsDoor(ref) || DoorShut(ref));
					it = present.emplace(owner.id, std::make_pair(there, there ? ref->GetPosition() : RE::NiPoint3{})).first;
				}
				if (!it->second.first || it->second.second.GetDistance(owner.at) > 2.0f) {
					continue;  // the door was opened, or the object was moved or taken: nothing there now
				}
			}
			if (wall.position.x < low.x || wall.position.y < low.y || wall.position.z < low.z || wall.position.x > high.x ||
				wall.position.y > high.y || wall.position.z > high.z) {
				continue;  // outside the milk's box this frame
			}
			a_domain.worldOrder.push_back(static_cast<int>(w));
			patches.push_back(wall);
		}
		a_domain.skin.Build(kSkinCell);

		for (const auto& emitter : _emitters) {
			if (emitter.domain == a_domain.id && emitter.hasNozzle) {
				holes.push_back({ ToLocal(a_domain, emitter.lastNozzle), (emitter.liquid == Liquid::kMilk ? kMilkHole : kNozzleHole) / kUnitsPerMeter });
			}
		}
		std::vector<Point> rim;
		RE::NiPoint3       center;
		RE::NiPoint3       inward;
		for (const auto& [handle, opening] : a_domain.openings) {
			const auto actor = handle.get();
			auto*      root = actor ? actor->Get3D(false) : nullptr;
			if (root && !FindOpening(root, opening, rim, center, inward)) {
				holes.push_back({ ToLocal(a_domain, center), kHoleRadius * root->world.scale / kUnitsPerMeter });
			}
		}
		SampleGround(a_domain);
		a_domain.solver->Configure(ParametersFor(a_domain.liquid));
		const Fluid::Colliders colliders{ capsules, holes, &a_domain.skin, !a_domain.worldOrder.empty(),
			a_domain.guards.size() == a_domain.particles.size() ? std::span<const Fluid::Plane>(a_domain.guards) : std::span<const Fluid::Plane>() };
		int                    steps = 0;
		while (a_domain.accumulator >= step && steps < maxSteps) {
			a_domain.solver->Step(a_domain.particles, a_domain.anchors, colliders, a_domain.ground, step);
			a_domain.accumulator -= step;
			++steps;
		}
		if (steps == maxSteps) {
			a_domain.accumulator = 0.0f;
		}
		std::erase_if(a_domain.anchors, [](const Fluid::Anchor& a_anchor) { return a_anchor.rest <= 0.0f; });
	}

	void FluidManager::SampleGround(Domain& a_domain)
	{
		auto* world = a_domain.cell ? a_domain.cell->GetbhkWorld() : nullptr;
		if (!world) {
			return;
		}
		auto&                             ground = a_domain.ground;
		std::unordered_set<std::uint64_t> seen;
		std::unordered_set<std::uint64_t> seenFine;
		int                               budget = kGroundBudget;
		int                               fineBudget = kFineBudget;
		const auto keyOf = [](int a_x, int a_y) {
			return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a_x)) << 32) | static_cast<std::uint32_t>(a_y);
		};
		std::vector<float> layers;
		const std::uint32_t tick = ++a_domain.groundTick;
		std::uint32_t       index = 0;
		// Table tops, seats and lids a fast drop is about to land on are measured first.
		std::vector<Fluid::Particle> hinted;
		hinted.reserve(a_domain.groundHints.size());
		for (const auto& hint : a_domain.groundHints) {
			Fluid::Particle probe{};
			probe.position = hint + Vec3{ 0.0f, 0.0f, 0.03f };
			hinted.push_back(probe);
		}
		a_domain.groundHints.clear();
		std::vector<const Fluid::Particle*> order;
		order.reserve(hinted.size() + a_domain.particles.size());
		for (const auto& probe : hinted) {
			order.push_back(&probe);
		}
		for (const auto& particle : a_domain.particles) {
			order.push_back(&particle);
		}
		for (const auto* pointer : order) {
			const auto& particle = *pointer;
			if (budget <= 0 && fineBudget <= 0) {
				break;
			}
			if (particle.asleep && ((index++ + tick) & 3u) != 0) {
				continue;  // resting milk: its edge check every 4th frame is plenty
			}
			const float z = particle.position.z;
			const int   x = ground.Node(particle.position.x);
			const int   y = ground.Node(particle.position.y);
			// resting milk was measured before it settled; it only needs the edge detail below (it is woken if that drops it)
			for (int dx = 0; dx <= 1 && budget > 0 && !particle.asleep; ++dx) {
				for (int dy = 0; dy <= 1 && budget > 0; ++dy) {
					if (ground.Known(x + dx, y + dy, z) || !seen.insert(keyOf(x + dx, y + dy)).second) {
						continue;
					}
					// From just above the milk straight down to the floor, keeping every surface on the way:
					// a wide flat one (floor, table top, seat, bed) is somewhere milk can lie; a slanted or narrow one
					// (beam, chair back, rail) is part of an object's surface that milk runs down like skin.
					const float        start = z + kGroundStart;
					const RE::NiPoint3 node = ToWorld(a_domain, { static_cast<float>(x + dx) * ground.Cell(), static_cast<float>(y + dy) * ground.Cell(), start });
					const RE::NiPoint3 bottom = node - RE::NiPoint3{ 0.0f, 0.0f, kGroundBelow + start * kUnitsPerMeter };
					RE::NiPoint3       from = node;
					layers.clear();
					bool floorFound = false;
					for (int cast = 0; cast < 6 && layers.size() < 4 && budget > 0; ++cast) {
						--budget;
						RE::NiPoint3 normal{ 0.0f, 0.0f, 1.0f };
						const auto   hit = CastDown(world, from, bottom, &normal);
						if (!hit) {
							break;
						}
						const RE::NiPoint3 at{ from.x, from.y, *hit };
						const float        local = (*hit - a_domain.origin.z) / kUnitsPerMeter;
						bool               structure = normal.z < kWallNormalZ;  // slanted beam, chair back: milk runs down it
						if (!structure && local > kRaised) {
							// Raised and flat: somewhere to lie only if it is wide; a rail, rung or edge is structure.
							budget -= 2;
							for (const auto& side : { RE::NiPoint3{ kLedge, 0.0f, 0.0f }, RE::NiPoint3{ -kLedge, 0.0f, 0.0f }, RE::NiPoint3{ 0.0f, kLedge, 0.0f },
									 RE::NiPoint3{ 0.0f, -kLedge, 0.0f } }) {
								const auto around = CastDown(world, from + side, bottom + side);
								if (!around || std::abs(*around - *hit) > 1.0f) {
									structure = true;
									break;
								}
							}
						}
						if (structure) {
							AddWorldPatch(a_domain, at, normal, true);  // part of the object's surface, like skin
						} else {
							layers.push_back(local);
							if (local <= kRaised) {
								floorFound = true;
								break;  // the floor the milk stands on: nothing below matters
							}
						}
						from = at - RE::NiPoint3{ 0.0f, 0.0f, 1.0f };  // and keep looking below it
					}
					if (!floorFound && (layers.empty() || layers.back() > 0.01f)) {
						layers.push_back(0.0f);  // nothing more below: the pool's own floor
					}
					ground.Set(x + dx, y + dy, start, layers);
				}
			}
			// At an edge (table, seat, step) the coarse grid cannot say where the surface ends: measure every 2 cm there,
			// so milk lies on the surface right up to its edge and falls off it, instead of stopping in the air beside it.
			if (fineBudget <= 0 || !ground.Step(particle.position.x, particle.position.y, z) ||
				z - ground.Below(particle.position.x, particle.position.y, z) > 0.15f) {
				continue;
			}
			const int fx = ground.FineNode(particle.position.x);
			const int fy = ground.FineNode(particle.position.y);
			for (int dx = 0; dx <= 1 && fineBudget > 0; ++dx) {
				for (int dy = 0; dy <= 1 && fineBudget > 0; ++dy) {
					if (ground.FineKnown(fx + dx, fy + dy, z) || !seenFine.insert(keyOf(fx + dx, fy + dy)).second) {
						continue;
					}
					const float        start = z + kGroundStart;
					const float        px = static_cast<float>(fx + dx) * ground.FineCell();
					const float        py = static_cast<float>(fy + dy) * ground.FineCell();
					const RE::NiPoint3 node = ToWorld(a_domain, { px, py, start });
					const RE::NiPoint3 bottom = node - RE::NiPoint3{ 0.0f, 0.0f, kGroundBelow + start * kUnitsPerMeter };
					// One ray: the top surface here; what lies under it the coarse grid already knows.
					--fineBudget;
					layers.clear();
					RE::NiPoint3 normal{ 0.0f, 0.0f, 1.0f };
					const auto   hit = CastDown(world, node, bottom, &normal);
					float        under = start;
					if (hit) {
						const float local = (*hit - a_domain.origin.z) / kUnitsPerMeter;
						under = local - 0.03f;
						// only surfaces the coarse grid accepted as wide and flat (so a rail between two points stays a rail)
						if (normal.z >= kWallNormalZ && (local <= kRaised || ground.NearLayer(px, py, local, 0.015f))) {
							layers.push_back(local);
						}
					}
					if (layers.empty() || layers.back() > kRaised) {
						if (const auto lower = ground.Under(px, py, under)) {
							layers.push_back(*lower);
						} else if (layers.empty() || layers.back() > 0.01f) {
							layers.push_back(0.0f);
						}
					}
					ground.SetFine(fx + dx, fy + dy, start, layers);
				}
			}
		}
	}

	void FluidManager::Expire(Domain& a_domain, float a_delta)
	{
		auto& particles = a_domain.particles;
		bool  expired = false;
		for (auto& particle : particles) {
			particle.life -= a_delta;
			expired = expired || particle.life <= 0.0f;
		}
		if (!expired) {
			return;
		}
		std::vector<std::size_t> remap(particles.size(), Fluid::kNone);
		std::size_t              write = 0;
		bool                     previousKept = false;
		for (std::size_t read = 0; read < particles.size(); ++read) {
			if (particles[read].life <= 0.0f) {
				if (previousKept) {
					particles[write - 1].link = 0.0f;
				}
				previousKept = false;
				continue;
			}
			remap[read] = write;
			particles[write++] = particles[read];
			previousKept = true;
		}
		particles.resize(write);
		for (auto& blob : a_domain.blobs) {
			if (blob.live) {
				blob.seed = blob.seed < remap.size() ? remap[blob.seed] : Fluid::kNone;
				blob.live = blob.seed != Fluid::kNone;
			}
		}
		std::erase_if(a_domain.anchors, [&](Fluid::Anchor& a_anchor) {
			a_anchor.particle = a_anchor.particle < remap.size() ? remap[a_anchor.particle] : Fluid::kNone;
			return a_anchor.particle == Fluid::kNone;
		});
	}

	FluidManager::Meshes& FluidManager::MeshesFor()
	{
		auto& meshes = _meshes;
		if (meshes.failed || (meshes.blob && meshes.rope)) {
			return meshes;
		}
		RE::BSModelDB::DBTraits::ArgsType args{};
		for (auto [path, target] : { std::pair{ kBlobPath, &meshes.blob }, std::pair{ kRopePath, &meshes.rope } }) {
			const auto error = RE::BSModelDB::Demand(path, *target, args);
			if (error != RE::BSResource::ErrorCode::kNone || !*target) {
				logger::error("could not load meshes\\{} (error {}); semen stays invisible", path, std::to_underlying(error));
				meshes.failed = true;
				return meshes;
			}
		}
		if (_directions.empty()) {
			for (std::size_t i = 0;; ++i) {
				auto* bone = meshes.blob->GetObjectByName(std::format("Blob{:02}", i));
				if (!bone) {
					break;
				}
				_directions.push_back(Fluid::Normalize(ToVec(bone->local.translate), { 0.0f, 0.0f, 1.0f }));
			}
			_scratch.resize(_directions.size());
		}
		if (_directions.size() < 12 || !meshes.rope->GetObjectByName(std::format("Rope{:02}", kRopeBones - 1))) {
			logger::error("semen meshes are from another version; reinstall OStrands");
			meshes.failed = true;
		}
		return meshes;
	}

	RE::NiNode* FluidManager::CollisionMesh()
	{
		if (!_collisionMesh && !_collisionMeshFailed) {
			RE::BSModelDB::DBTraits::ArgsType args{};
			const auto error = RE::BSModelDB::Demand(kCollisionPath, _collisionMesh, args);
			if (error != RE::BSResource::ErrorCode::kNone || !_collisionMesh ||
				!_collisionMesh->GetObjectByName(MeshNames::Get().ropeShape)) {
				_collisionMeshFailed = true;
			}
			for (std::size_t b = 0; !_collisionMeshFailed && b < kRopeBones; ++b) {
				_collisionMeshFailed = !_collisionMesh->GetObjectByName(std::format("Rope{:02}", b));
			}
			if (_collisionMeshFailed) {
				logger::error("collision display: missing or incompatible meshes\\{}; reinstall the debug assets", kCollisionPath);
				_collisionMesh.reset();
			}
		}
		return _collisionMesh.get();
	}

	void FluidManager::Shade(Domain& a_domain, RE::NiAVObject* a_root)
	{
		if (a_domain.diagnostic) {
			return;
		}
		const auto& settings = Settings::Get();
		const auto& names = MeshNames::Get();
		const bool  milk = a_domain.liquid == Liquid::kMilk;
		for (const auto& name : { names.blobShape, names.ropeShape }) {
			auto* shape = a_root->GetObjectByName(name);
			auto* geometry = shape ? shape->AsGeometry() : nullptr;
			auto* shader = geometry ? geometry->GetGeometryRuntimeData().shaderProperty.get() : nullptr;
			if (!shader) {
				continue;
			}
			const bool lighting = shader->GetMaterialType() == RE::BSShaderMaterial::Type::kLighting;
			if (milk && lighting) {
				// Give milk its own material so it never shares colour, alpha or shine with OStrands' semen,
				// which is cloned from the same cached mesh.
				auto*       template3D = name == names.blobShape ? _meshes.blob.get() : _meshes.rope.get();
				auto*       source = template3D ? template3D->GetObjectByName(name) : nullptr;
				auto*       sourceGeometry = source ? source->AsGeometry() : nullptr;
				auto*       sourceShader = sourceGeometry ? sourceGeometry->GetGeometryRuntimeData().shaderProperty.get() : nullptr;
				auto*       base = static_cast<RE::BSLightingShaderMaterialBase*>(shader->GetBaseMaterial());
				if (base && (!sourceShader || sourceShader->GetBaseMaterial() == base)) {
					if (auto* copy = static_cast<RE::BSLightingShaderMaterialBase*>(base->Create())) {
						copy->CopyMembers(base);
						// SetMaterial(..., true) stores its own unique copy. Our temporary is deliberately not freed:
						// a few hundred bytes per mesh is a far safer price than a use-after-free if a game version
						// kept the pointer instead.
						shader->SetMaterial(copy, true);
						shader->SetupGeometry(geometry);
						shader->FinishSetupGeometry(geometry);
					}
				}
			}
			shader->SetMaterialAlpha(OpacityOf(a_domain.liquid));
			if (!lighting) {
				continue;
			}
			auto* material = static_cast<RE::BSLightingShaderMaterialBase*>(shader->GetBaseMaterial());
			if (!milk) {
				if (material) {
					material->specularColorScale = settings.shine;
					material->specularPower = settings.gloss;
				}
				continue;
			}
			if (material) {
				// A soft white sheen; strong coloured highlights are what tinted milk brown under torchlight.
				material->specularColor = RE::NiColor(1.0f, 1.0f, 1.0f);
				material->specularColorScale = settings.milkShine;
				material->specularPower = 60.0f;
				material->materialAlpha = OpacityOf(a_domain.liquid);
			}
			// Self-lit white keeps milk white in dark or warm light.
			// The emissive colour is a small separate allocation the property only points to, and clones of the cached
			// mesh can share it with other clones (OStrands' own semen included). Writing through it after another
			// clone freed it corrupted the game's heap (crash in tbbmalloc). So every milk shape gets a colour of its
			// own, allocated from the game's heap, and only that one is ever written.
			auto* lit = static_cast<RE::BSLightingShaderProperty*>(shader);
			RE::NiColor*& own = _ownEmissive[lit];
			if (!own || lit->emissiveColor != own) {
				own = RE::malloc<RE::NiColor>(sizeof(RE::NiColor));
				if (own) {
					*own = RE::NiColor(1.0f, 1.0f, 1.0f);
					lit->emissiveColor = own;
				}
			}
			if (own && lit->emissiveColor == own) {
				*own = RE::NiColor(1.0f, 1.0f, 1.0f);
			}
			lit->emissiveMult = settings.milkGlow;
			shader->SetFlags(RE::BSShaderProperty::EShaderPropertyFlag8::kOwnEmit, settings.milkGlow > 0.0f);
		}
	}

	bool FluidManager::CloneBlob(Blob& a_blob)
	{
		auto& meshes = MeshesFor();
		auto* clone = meshes.failed ? nullptr : meshes.blob->Clone();
		auto* node = clone ? clone->AsNode() : nullptr;
		if (!node) {
			return false;
		}
		a_blob.root.reset(node);
		a_blob.bones.clear();
		for (std::size_t i = 0; i < _directions.size(); ++i) {
			auto* bone = node->GetObjectByName(std::format("Blob{:02}", i));
			if (!bone) {
				return false;
			}
			a_blob.bones.push_back(bone);
		}
		node->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
		if (auto* shape = node->GetObjectByName(MeshNames::Get().blobShape)) {
			shape->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
		}
		++_statClones;
		return true;
	}

	void FluidManager::Prewarm()
	{
		if (!_warm) {
			return;
		}
		auto& meshes = MeshesFor();
		if (meshes.failed) {
			return;
		}
		// quiet frames build a few, busy frames at most one
		int budget = _emitters.empty() ? 3 : 1;
		while (budget > 0 && _spareBlobs.size() < kSpareBlobTarget) {
			Blob blob;
			--budget;
			if (!CloneBlob(blob)) {
				return;
			}
			blob.root->SetAppCulled(true);
			_spareBlobs.push_back(std::move(blob));
		}
		while (budget > 0 && _spareRopes.size() < kSpareRopeTarget) {
			Rope rope;
			--budget;
			if (!CloneRope(rope, _meshes.rope.get())) {
				return;
			}
			rope.root->SetAppCulled(true);
			_spareRopes.push_back(std::move(rope));
		}
	}

	bool FluidManager::CloneRope(Rope& a_rope, RE::NiNode* a_mesh)
	{
		auto* clone = a_mesh ? a_mesh->Clone() : nullptr;
		auto* node = clone ? clone->AsNode() : nullptr;
		if (!node) {
			return false;
		}
		a_rope.root.reset(node);
		for (std::size_t i = 0; i < kRopeBones; ++i) {
			a_rope.bones[i] = node->GetObjectByName(std::format("Rope{:02}", i));
			if (!a_rope.bones[i]) {
				return false;
			}
		}
		node->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
		if (auto* shape = node->GetObjectByName(MeshNames::Get().ropeShape)) {
			shape->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
		}
		++_statClones;
		return true;
	}

	FluidManager::Blob* FluidManager::AcquireBlob(Domain& a_domain)
	{
		for (auto& blob : a_domain.blobs) {
			if (!blob.used) {
				blob.used = true;
				blob.puddle = false;
				return &blob;
			}
		}
		if (a_domain.blobs.size() >= kMaxBlobs) {
			return nullptr;
		}
		Blob blob;
		if (!_spareBlobs.empty()) {
			blob = std::move(_spareBlobs.back());
			_spareBlobs.pop_back();
		} else if (_cloneBudget <= 0 || (--_cloneBudget, !CloneBlob(blob))) {
			return nullptr;
		}
		blob.root->local = RE::NiTransform{};
		a_domain.root->AttachChild(blob.root.get(), true);
		Shade(a_domain, blob.root.get());
		blob.used = true;
		blob.puddle = false;
		a_domain.blobs.push_back(std::move(blob));
		return &a_domain.blobs.back();
	}

	FluidManager::Rope* FluidManager::AcquireRope(Domain& a_domain)
	{
		for (auto& rope : a_domain.ropes) {
			if (!rope.used) {
				rope.used = true;
				return &rope;
			}
		}
		const auto limit = a_domain.diagnostic ? kMaxDebugShapes : kMaxRopes;
		if (a_domain.ropes.size() >= limit) {
			return nullptr;
		}
		Rope rope;
		if (a_domain.diagnostic) {
			if (!CloneRope(rope, CollisionMesh())) {
				return nullptr;
			}
		} else if (!_spareRopes.empty()) {
			rope = std::move(_spareRopes.back());
			_spareRopes.pop_back();
		} else if (_cloneBudget <= 0 || (--_cloneBudget, !CloneRope(rope, MeshesFor().failed ? nullptr : _meshes.rope.get()))) {
			return nullptr;
		}
		rope.root->local = RE::NiTransform{};
		a_domain.root->AttachChild(rope.root.get(), true);
		Shade(a_domain, rope.root.get());
		rope.used = true;
		a_domain.ropes.push_back(std::move(rope));
		return &a_domain.ropes.back();
	}

	void FluidManager::PoseBlob(Domain& a_domain, Blob& a_blob)
	{
		for (std::size_t i = 0; i < a_blob.bones.size(); ++i) {
			RE::NiTransform world;
			world.translate = ToWorld(a_domain, a_blob.surface[i].position);
			world.rotate = ArcRotation(_directions[i], a_blob.surface[i].normal);
			a_blob.bones[i]->local = world;
		}
		a_blob.root->SetAppCulled(false);
	}

	void FluidManager::PoseDrop(Domain& a_domain, Blob& a_blob, Vec3 a_center, float a_radius, Vec3 a_velocity, std::optional<Vec3> a_contact)
	{
		const float flying = a_contact ? 0.0f : std::max(Fluid::Length(a_velocity) - kStreakStart, 0.0f);
		float       stretch = 1.0f + std::min(flying * kStreak, kMaxStretch - 1.0f);
		Vec3        along = Fluid::Normalize(a_velocity, { 0.0f, 0.0f, 1.0f });
		const bool  milk = a_domain.liquid == Liquid::kMilk;
		float       squash = a_contact ? (milk ? kMilkSquash : kSquash) : 1.0f;
		const bool  onSkin = milk && a_contact && a_contact->z < 0.9f;
		if (onSkin) {
			// Milk running over skin: a thin film drawn out along its path (or downhill when it barely moves),
			// instead of a round bead.
			const Vec3  normal = *a_contact;
			const Vec3  sliding = a_velocity - normal * Fluid::Dot(a_velocity, normal);
			const Vec3  downhill = Vec3{ 0.0f, 0.0f, -1.0f } - normal * (-normal.z);
			const float speed = Fluid::Length(sliding);
			along = Fluid::Normalize(speed > 0.02f ? sliding : downhill, Fluid::Normalize(downhill, { 0.0f, 0.0f, -1.0f }));
			stretch = std::min(1.8f + speed * 8.0f, 4.0f);
			squash = 0.3f;
		}
		if (onSkin) {
			a_radius *= 1.05f;
		} else if (a_contact && milk) {
			a_radius *= 1.1f + 0.8f * Settings::Get().milkMerge;
		} else if (milk) {
			a_radius *= Settings::Get().milkThickness * 1.3f;  // drops in flight are part of a stream; puddles keep their size
		}
		const Vec3  normal = a_contact ? *a_contact : Vec3{ 0.0f, 0.0f, 1.0f };
		const Vec3  center = a_contact ? a_center - normal * (a_radius * (1.0f - squash) * 0.7f) : a_center;
		for (std::size_t i = 0; i < a_blob.bones.size(); ++i) {
			const Vec3      direction = _directions[i];
			const Vec3      offset = Stretch(Stretch(direction, along, stretch), normal, squash);
			const Vec3      facing = Fluid::Normalize(Stretch(Stretch(direction, along, 1.0f / stretch), normal, 1.0f / squash), direction);
			RE::NiTransform world;
			world.translate = ToWorld(a_domain, center + offset * a_radius);
			world.rotate = ArcRotation(direction, facing);
			a_blob.bones[i]->local = world;
		}
		a_blob.live = false;
		a_blob.root->SetAppCulled(false);
	}

	void FluidManager::PosePuddle(Domain& a_domain, Blob& a_blob, Vec3 a_center, float a_radius, Vec3 a_flow, std::uint64_t a_seed, Vec3 a_up)
	{
		// Milk that hit the floor while moving is a splat drawn out along its run: long in the direction it travelled,
		// narrow across, heavier at the front where it piled up, with a ragged edge. Still milk (no flow) stays roundish.
		// It lies in the plane of the ground under it (a_up), so it follows slopes instead of cutting into them.
		const Vec3  up = Fluid::Normalize(a_up, { 0.0f, 0.0f, 1.0f });
		const float speed = std::sqrt(a_flow.x * a_flow.x + a_flow.y * a_flow.y);
		// horizontal reference axes (as before), then laid onto the ground plane
		const Vec3  alongFlat = speed > 1e-4f ? Vec3{ a_flow.x / speed, a_flow.y / speed, 0.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
		const Vec3  acrossFlat{ -alongFlat.y, alongFlat.x, 0.0f };
		const Vec3  along = Fluid::Normalize(alongFlat - up * Fluid::Dot(alongFlat, up), alongFlat);
		const Vec3  across = Fluid::Normalize(Fluid::Cross(up, along), acrossFlat);
		const float stretch = std::clamp(1.0f + speed * 3.5f, 1.0f, 3.2f);
		const float narrow = 1.0f / std::sqrt(stretch);
		const float flat = std::clamp(kPuddleThickness / std::max(a_radius, 1e-4f), 0.08f, 0.6f);
		const float phase1 = static_cast<float>(a_seed % 628u) * 0.01f;
		const float phase2 = static_cast<float>((a_seed / 628u) % 628u) * 0.01f;
		const bool  moving = speed > 0.05f;
		const Vec3  base = a_center + up * (0.5f * kPuddleThickness);
		for (std::size_t i = 0; i < a_blob.bones.size(); ++i) {
			const Vec3  direction = _directions[i];
			const float pa = Fluid::Dot(direction, alongFlat);
			const float pb = Fluid::Dot(direction, acrossFlat);
			const float pz = direction.z;
			const float angle = std::atan2(pb, pa);
			// Ragged rim: a few soft lobes, different for every cell.
			const float rim = 1.0f + 0.16f * std::sin(3.0f * angle + phase1) + 0.09f * std::sin(5.0f * angle + phase2);
			float       front = 1.0f;
			if (moving) {
				front = pa > 0.0f ? 1.0f + 0.3f * pa : 1.0f + 0.15f * pa;  // heavier head, tapering tail
			}
			const float sa = stretch * front * rim;
			const float sb = narrow * rim;
			const Vec3  offset = (along * (pa * sa) + across * (pb * sb)) * a_radius + up * (pz * a_radius * flat);
			const Vec3  facing = Fluid::Normalize(along * (pa / sa) * flat + across * (pb / sb) * flat + up * pz, direction);
			const auto  translate = ToWorld(a_domain, base + offset);
			if (!Finite(translate)) {
				a_blob.root->SetAppCulled(true);
				return;
			}
			RE::NiTransform world;
			world.translate = translate;
			world.rotate = ArcRotation(direction, facing);
			a_blob.bones[i]->local = world;
		}
		a_blob.live = false;
		a_blob.root->SetAppCulled(false);
	}

	void FluidManager::AddWorldPatch(Domain& a_domain, const RE::NiPoint3& a_point, const RE::NiPoint3& a_normal, bool a_floorToo, RE::FormID a_owner,
		const RE::NiPoint3& a_ownerAt)
	{
		if (!a_floorToo && a_normal.z > kWallNormalZ) {
			return;  // floor: the ground grid handles it
		}
		const Vec3         local = ToLocal(a_domain, a_point);
		const auto         q = [](float a_v) { return static_cast<std::int64_t>(std::floor(a_v / kWorldPatchCell)) & 0x1FFFFF; };
		const std::int64_t key = (q(local.x) << 42) | (q(local.y) << 21) | q(local.z);
		if (a_domain.worldIndex.contains(key) || a_domain.world.size() >= kMaxWorldPatches) {
			return;
		}
		a_domain.worldIndex.emplace(key, static_cast<int>(a_domain.world.size()));
		a_domain.world.push_back({ local, ToVec(a_normal), {}, true });
		a_domain.worldOwner.resize(a_domain.world.size());
		a_domain.worldOwner.back() = { a_owner, a_ownerAt };
	}

	void FluidManager::GuardPaths(Domain& a_domain, float a_seconds)
	{
		auto& particles = a_domain.particles;
		auto& guards = a_domain.guards;
		guards.assign(particles.size(), {});
		auto* world = a_domain.cell ? a_domain.cell->GetbhkWorld() : nullptr;
		if (!world || particles.empty()) {
			return;
		}
		auto&       voxels = a_domain.voxels;
		const float now = a_domain.age;
		if (voxels.size() > kMaxVoxels) {
			std::erase_if(voxels, [&](const auto& a_entry) { return a_entry.second.until <= now; });
			if (voxels.size() > kMaxVoxels) {
				voxels.clear();
			}
		}
		const auto key = [](Vec3 a_point) {
			const auto q = [](float a_v) { return static_cast<std::int64_t>(std::floor(a_v / kVoxel)) & 0x1FFFFF; };
			return (q(a_point.x) << 42) | (q(a_point.y) << 21) | q(a_point.z);
		};
		const auto markFree = [&](Vec3 a_from, Vec3 a_to) {
			const Vec3  span = a_to - a_from;
			const int   samples = std::max(1, static_cast<int>(std::ceil(Fluid::Length(span) / (kVoxel * 0.75f))));
			for (int s = 0; s <= samples; ++s) {
				auto& voxel = voxels[key(a_from + span * (static_cast<float>(s) / static_cast<float>(samples)))];
				if (!voxel.blocked || voxel.until <= now) {
					voxel = { now + kFreeLife, false, {}, {} };
				}
			}
		};
		const float seconds = std::clamp(a_seconds, 1.0f / 240.0f, 0.1f);
		const std::size_t count = particles.size();
		int               budget = kGuardRays;
		for (std::size_t visited = 0; visited < count; ++visited) {
			const std::size_t i = (a_domain.guardCursor + visited) % count;
			const auto&       particle = particles[i];
			if (particle.asleep || particle.touching) {
				continue;  // resting, or already on a surface (its samples handle it)
			}
			const float speed = Fluid::Length(particle.velocity);
			if (speed < kGuardSpeed) {
				continue;
			}
			// Where it flies this frame: from a little behind it (so one that just slipped into a face is caught too)
			// to a little past where it ends up.
			const Vec3 direction = particle.velocity * (1.0f / speed);
			const Vec3 from = particle.position - direction * kGuardBack;
			const Vec3 to = particle.position + particle.velocity * seconds + Vec3{ 0.0f, 0.0f, -4.905f * seconds * seconds } + direction * kGuardAhead;
			const Vec3 span = to - from;
			const int  samples = std::max(1, static_cast<int>(std::ceil(Fluid::Length(span) / (kVoxel * 0.75f))));
			bool       unknown = false;
			const Domain::Voxel* surface = nullptr;
			for (int s = 0; s <= samples; ++s) {
				const auto it = voxels.find(key(from + span * (static_cast<float>(s) / static_cast<float>(samples))));
				if (it == voxels.end() || it->second.until <= now) {
					unknown = true;
					break;
				}
				if (it->second.blocked) {
					surface = &it->second;
					break;
				}
			}
			if (!unknown && !surface) {
				continue;  // the whole path is known to be empty
			}
			// A surface already known on the path: measure it again on this exact path while the budget allows
			// (exact contact point), else use the known one.
			if (budget <= 0 || (surface && budget <= kGuardRays / 3)) {
				if (surface) {
					guards[i] = { surface->point, surface->normal, true };
				}
				continue;
			}
			--budget;
			const auto hit = CastRay(world, ToWorld(a_domain, from), ToWorld(a_domain, to));
			if (!hit) {
				markFree(from, to);
				continue;
			}
			const Vec3  point = ToLocal(a_domain, hit->point);
			const Vec3  normal = ToVec(hit->normal);
			const auto  hitKey = key(point);
			const bool  fresh = !voxels.contains(hitKey) || !voxels[hitKey].blocked || voxels[hitKey].until <= now;
			const float back = Fluid::Length(point - from);
			if (back > kVoxel) {
				markFree(from, from + direction * (back - kVoxel));
			}
			voxels[hitKey] = { now + (hit->owner ? kMovableLife : kBlockLife), true, point, normal };
			const bool floorLevel = normal.z >= kWallNormalZ && point.z <= kRaised;
			if (!floorLevel) {
				guards[i] = { point, normal, true };  // the floor itself stays with the ground grid (puddles as before)
			}
			if (!fresh) {
				continue;
			}
			if (normal.z < kWallNormalZ) {
				// a wall, pillar or the side of an object: remember it so the milk runs down it like skin
				AddWorldPatch(a_domain, hit->point, hit->normal, false, hit->owner, hit->ownerAt);
				const RE::NiPoint3 n = hit->normal;
				const RE::NiPoint3 side = Normalized(n.Cross({ 0.0f, 0.0f, 1.0f }), { 1.0f, 0.0f, 0.0f });
				const RE::NiPoint3 upish = side.Cross(n);
				for (const auto& offset : { side * 1.6f, side * -1.6f, upish * -1.6f, upish * 1.6f }) {
					if (budget <= 0) {
						break;
					}
					--budget;
					const RE::NiPoint3 around = hit->point + offset;
					if (const auto more = CastRay(world, around + n * 2.0f, around - n * 4.0f)) {
						AddWorldPatch(a_domain, more->point, more->normal, false, more->owner, more->ownerAt);
					}
				}
			} else if (!floorLevel) {
				a_domain.groundHints.push_back(point);  // a table top, seat or lid: measure the ground there right away
			}
		}
		a_domain.guardCursor = (a_domain.guardCursor + 1) % std::max<std::size_t>(count, 1);
	}

	void FluidManager::ProbeWorld(Domain& a_domain)
	{
		auto* world = a_domain.cell ? a_domain.cell->GetbhkWorld() : nullptr;
		auto& particles = a_domain.particles;
		if (!world || particles.empty() || a_domain.world.size() >= kMaxWorldPatches) {
			return;
		}
		const auto  add = [&](const RayHit& a_hit) { AddWorldPatch(a_domain, a_hit.point, a_hit.normal, false, a_hit.owner, a_hit.ownerAt); };
		const std::size_t count = particles.size();
		const float       dt = std::max(RE::GetSecondsSinceLastFrame(), 1.0f / 240.0f);
		// a particle is visited about every count/kWorldRays frames, so its ray covers the path until its next turn
		const float       horizon = std::clamp(static_cast<float>(count) / static_cast<float>(kWorldRays) * dt, 0.05f, 0.8f);
		int               budget = kWorldRays;
		// 1) milk already running on a wall: look just ahead of it so the wall below is known before it gets there
		for (std::size_t k = 0; k < count && budget > kWorldRays / 2; ++k) {
			const auto& particle = particles[k];
			if (!particle.touching || particle.surface < 0) {
				continue;
			}
			const auto index = static_cast<std::size_t>(particle.surface);
			if (index < a_domain.worldFrom || index - a_domain.worldFrom >= a_domain.worldOrder.size()) {
				continue;  // on the body, not on a wall (contact slots are from the last step)
			}
			const Vec3 normal = a_domain.world[static_cast<std::size_t>(a_domain.worldOrder[index - a_domain.worldFrom])].normal;
			const Vec3 ahead = particle.position + Fluid::Normalize(particle.velocity, { 0.0f, 0.0f, -1.0f }) * 0.03f;
			if (const auto hit = CastRay(world, ToWorld(a_domain, ahead + normal * 0.03f), ToWorld(a_domain, ahead - normal * 0.06f))) {
				add(*hit);
			}
			--budget;
		}
		// 2) moving milk, in turn: a ray along where it is heading
		for (std::size_t visited = 0; visited < count && budget > 0; ++visited) {
			const std::size_t k = (a_domain.worldCursor + visited) % count;
			const auto&       particle = particles[k];
			if (particle.asleep || (particle.touching && particle.surface >= 0 && static_cast<std::size_t>(particle.surface) < a_domain.worldFrom)) {
				continue;  // resting, or on the body
			}
			const float speed = Fluid::Length(particle.velocity);
			if (speed < 0.2f) {
				continue;
			}
			// the arc it will fly until it is looked at again, plus a little: one chord, or two for a long arc
			const auto along = [&](float a_t) {
				return particle.position + particle.velocity * a_t + Vec3{ 0.0f, 0.0f, -0.5f * 9.81f * a_t * a_t };
			};
			const Vec3 to = along(horizon) + particle.velocity * (0.03f / speed);
			std::optional<RayHit> found;
			if (horizon > 0.25f && budget > 1) {
				const Vec3 middle = along(horizon * 0.5f);
				found = CastRay(world, ToWorld(a_domain, particle.position), ToWorld(a_domain, middle));
				--budget;
				if (!found) {
					found = CastRay(world, ToWorld(a_domain, middle), ToWorld(a_domain, to));
				}
			} else {
				found = CastRay(world, ToWorld(a_domain, particle.position), ToWorld(a_domain, to));
			}
			if (const auto& hit = found) {
				add(*hit);
				// fill in a little of the wall around the hit, so the splash lands on a surface
				const RE::NiPoint3 n = hit->normal;
				const RE::NiPoint3 side = Normalized(n.Cross({ 0.0f, 0.0f, 1.0f }), { 1.0f, 0.0f, 0.0f });
				const RE::NiPoint3 upish = side.Cross(n);
				for (const auto& offset : { side * 2.5f, side * -2.5f, upish * -2.5f }) {
					if (budget <= 1) {
						break;
					}
					const RE::NiPoint3 around = hit->point + offset;
					if (const auto more = CastRay(world, around + n * 2.0f, around - n * 4.0f)) {
						add(*more);
					}
					--budget;
				}
			}
			--budget;
			a_domain.worldCursor = k + 1;
		}
	}

	void FluidManager::ProbePuddle(Domain& a_domain, Puddle& a_cell, Vec3 a_center)
	{
		// The ground grid is coarse; on bumpy or sloped terrain a puddle placed on it sinks into the real surface.
		// Measure the real ground at the centre and around it once: the puddle sits on the highest point under it,
		// tilted to the local slope.
		a_cell.probed = true;
		auto* world = a_domain.cell ? a_domain.cell->GetbhkWorld() : nullptr;
		if (!world) {
			a_cell.surface = a_center.z;
			return;
		}
		const auto sample = [&](float a_dx, float a_dy) -> std::optional<float> {
			const RE::NiPoint3 point = ToWorld(a_domain, { a_center.x + a_dx, a_center.y + a_dy, a_center.z });
			const auto         hit = CastDown(world, point + RE::NiPoint3{ 0.0f, 0.0f, 20.0f }, point - RE::NiPoint3{ 0.0f, 0.0f, 40.0f });
			if (!hit) {
				return std::nullopt;
			}
			return (*hit - a_domain.origin.z) / kUnitsPerMeter;
		};
		const auto c = sample(0.0f, 0.0f);
		const auto xp = sample(kPuddleProbe, 0.0f);
		const auto xn = sample(-kPuddleProbe, 0.0f);
		const auto yp = sample(0.0f, kPuddleProbe);
		const auto yn = sample(0.0f, -kPuddleProbe);
		const float centre = c ? *c : a_center.z;
		// Samples that jump (a rug edge, a prop, a step) are not the ground the puddle lies on: leave them out.
		const auto  usable = [&](const std::optional<float>& a_h) { return a_h && std::abs(*a_h - centre) < 0.03f; };
		float       top = centre;
		for (const auto& h : { xp, xn, yp, yn }) {
			if (usable(h)) {
				top = std::max(top, *h);
			}
		}
		// Tilt only where the ground really slopes: both sides agree, else one side, else flat.
		const auto slope = [&](const std::optional<float>& a_plus, const std::optional<float>& a_minus) {
			const bool p = usable(a_plus), m = usable(a_minus);
			if (p && m) {
				const float a = *a_plus - centre, b = centre - *a_minus;
				return (a * b > 0.0f || std::abs(a - b) < 0.006f) ? (a + b) / (2.0f * kPuddleProbe) : 0.0f;
			}
			if (p) {
				return (*a_plus - centre) / kPuddleProbe;
			}
			if (m) {
				return (centre - *a_minus) / kPuddleProbe;
			}
			return 0.0f;
		};
		const float gx = std::clamp(slope(xp, xn), -0.58f, 0.58f);  // at most about 30 degrees
		const float gy = std::clamp(slope(yp, yn), -0.58f, 0.58f);
		Vec3 normal = Fluid::Normalize(Vec3{ -gx, -gy, 1.0f }, { 0.0f, 0.0f, 1.0f });
		if (!Finite(normal) || normal.z < 0.85f) {
			normal = Fluid::Normalize(Vec3{ normal.x, normal.y, std::max(normal.z, 0.85f) }, { 0.0f, 0.0f, 1.0f });
		}
		a_cell.normal = normal;
		// never below where the milk itself settled
		a_cell.surface = std::max(top, a_center.z - 0.5f * kPuddleThickness) + kPuddleLift;
		// A wall or pillar standing in the puddle: move it out so it lies against the wall instead of inside it.
		const float reach = 2.2f * kPuddleCell;
		Vec3        shift{};
		const bool  wallNear = std::ranges::any_of(a_domain.world, [&](const Fluid::Patch& a_wall) {
			const Vec3 d = a_wall.position - a_center;
			return d.x * d.x + d.y * d.y < (reach + 0.05f) * (reach + 0.05f) && std::abs(d.z) < 0.3f;
		});
		for (const auto& dir : { Vec3{ 1.0f, 0.0f, 0.0f }, Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec3{ 0.0f, -1.0f, 0.0f } }) {
			if (!wallNear) {
				break;  // no wall known around here: skip the side rays
			}
			const Vec3 from{ a_center.x, a_center.y, a_cell.surface + 0.01f };
			if (const auto hit = CastRay(world, ToWorld(a_domain, from), ToWorld(a_domain, from + dir * reach))) {
				if (hit->normal.z < 0.5f) {
					const float distance = Fluid::Length(ToLocal(a_domain, hit->point) - from);
					shift = shift - dir * (reach - distance) * 0.8f;
				}
			}
		}
		a_cell.shift = { shift.x, shift.y, 0.0f };
	}

	void FluidManager::ReleaseShapes(Domain& a_domain)
	{
		if (a_domain.diagnostic || !a_domain.root) {
			return;
		}
		for (auto& blob : a_domain.blobs) {
			if (!blob.root) {
				continue;
			}
			a_domain.root->DetachChild2(blob.root.get());
			if (_spareBlobs.size() >= kMaxBlobs) {
				continue;
			}
			blob.root->SetAppCulled(true);
			blob.used = blob.live = blob.puddle = false;
			blob.surface.clear();
			_spareBlobs.push_back(std::move(blob));
		}
		for (auto& rope : a_domain.ropes) {
			if (!rope.root) {
				continue;
			}
			a_domain.root->DetachChild2(rope.root.get());
			if (_spareRopes.size() >= kMaxRopes) {
				continue;
			}
			rope.root->SetAppCulled(true);
			rope.used = false;
			_spareRopes.push_back(std::move(rope));
		}
		a_domain.blobs.clear();
		a_domain.ropes.clear();
		for (auto& [key, cell] : a_domain.puddles) {
			cell.blob = -1;
		}
	}

	void FluidManager::PoseRope(Domain& a_domain, Rope& a_rope, std::size_t a_begin, std::size_t a_end, const Fluid::Anchor* a_anchor)
	{
		const float                  fade = std::max(Settings::Get().fluidFade, 1e-3f);
		std::vector<Fluid::Particle> points(a_domain.particles.begin() + a_begin, a_domain.particles.begin() + a_end);
		if (a_anchor) {
			Fluid::Particle tip = points.back();
			tip.position = a_anchor->point;
			points.push_back(tip);
		}
		const std::size_t  count = points.size();
		std::vector<float> along(count, 0.0f);
		for (std::size_t k = 1; k < count; ++k) {
			along[k] = along[k - 1] + Fluid::Length(points[k].position - points[k - 1].position);
		}
		const float nominal = std::sqrt(kVolume / (std::numbers::pi_v<float> * kSlotSpacing * kSpacing));
		const float total = along.back();
		const float cap = nominal;
		const auto  gapAt = [&](std::size_t k) {
			const std::size_t lo = k > 0 ? k - 1 : 0;
			const std::size_t hi = std::min(k + 2, count - 1);
			return (along[hi] - along[lo]) / static_cast<float>(std::max<std::size_t>(hi - lo, 1));
		};

		std::array<RE::NiPoint3, kRopeBones> positions{};
		std::array<float, kRopeBones>        radii{};
		std::size_t                          segment = 0;
		for (std::size_t b = 0; b < kRopeBones; ++b) {
			const float t = static_cast<float>(b) / static_cast<float>(kRopeBones - 1);
			const float s = -cap + (total + 2.0f * cap) * t;
			const float clamped = std::clamp(s, 0.0f, total);
			while (segment + 2 < count && along[segment + 1] < clamped) {
				++segment;
			}
			const auto& a = points[segment];
			const auto& c = points[std::min(segment + 1, count - 1)];
			const float length = along[std::min(segment + 1, count - 1)] - along[segment];
			const float u = length > 1e-6f ? std::clamp((clamped - along[segment]) / length, 0.0f, 1.0f) : 0.0f;
			Vec3        point = a.position + (c.position - a.position) * u;
			const Vec3  tangent = Fluid::Normalize(c.position - a.position, { 0.0f, 0.0f, 1.0f });
			if (s < 0.0f) {
				point = point + tangent * s;
			} else if (s > total) {
				point = point + tangent * (s - total);
			}
			const float gap = std::max(gapAt(segment), 0.3f * kSpacing);
			float       radius = std::clamp(std::sqrt(kVolume / (std::numbers::pi_v<float> * gap)), 0.2f * nominal, 1.6f * nominal);
			if (a_domain.liquid == Liquid::kMilk) {
				// A water-gun jet keeps an even width; only the very end of a strand narrows.
				radius = nominal * (0.85f + 0.15f * std::clamp(radius / nominal, 0.0f, 1.0f));
			}
			const float weight = std::clamp((a.life + (c.life - a.life) * u) / fade, 0.0f, 1.0f);
			radius *= std::sqrt(weight);
			if (b == 0 || (b + 1 == kRopeBones && !a_anchor)) {
				radius *= 0.55f;
			} else if (b == 1 || (b + 2 == kRopeBones && !a_anchor)) {
				radius *= 0.85f;
			}
			positions[b] = ToWorld(a_domain, point);
			radii[b] = radius;
		}
		const float thickness = a_domain.liquid == Liquid::kMilk ? Settings::Get().milkThickness : 1.0f;
		for (auto& radius : radii) {
			radius *= kUnitsPerMeter * thickness;
		}
		Bend(a_rope, positions, radii);
	}

	void FluidManager::PoseCapsule(Rope& a_rope, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, float a_radius)
	{
		static_assert(kRopeBones >= 4 && kRopeBones % 2 == 0);
		constexpr std::size_t half = kRopeBones / 2;
		const auto axis = Normalized(a_to - a_from, { 0.0f, 1.0f, 0.0f });
		std::array<RE::NiPoint3, kRopeBones> positions{};
		std::array<float, kRopeBones>        radii{};
		for (std::size_t b = 0; b < kRopeBones; ++b) {
			const bool first = b < half;
			const float t = static_cast<float>(b % half) / static_cast<float>(half - 1);
			const float angle = (t - (first ? 1.0f : 0.0f)) * std::numbers::pi_v<float> * 0.5f;
			positions[b] = (first ? a_from : a_to) + axis * (std::sin(angle) * a_radius);
			radii[b] = std::max(std::cos(angle), 0.0f) * a_radius;
		}
		Bend(a_rope, positions, radii);
	}

	void FluidManager::Bend(Rope& a_rope, const std::array<RE::NiPoint3, kRopeBones>& a_positions, const std::array<float, kRopeBones>& a_radii)
	{
		GeometryMath::UpdateFrames(a_rope.frames, a_positions, -1);
		for (std::size_t b = 0; b < kRopeBones; ++b) {
			RE::NiTransform world;
			world.rotate = Rotation(a_rope.frames[b]);
			world.translate = a_positions[b];
			world.scale = std::max(a_radii[b], 0.0005f);
			a_rope.bones[b]->local = world;
		}
		a_rope.root->SetAppCulled(false);
	}

	void FluidManager::Draw(Domain& a_domain)
	{
		const auto& settings = Settings::Get();
		auto&       meshes = MeshesFor();
		if (meshes.failed) {
			return;
		}
		auto& particles = a_domain.particles;
		if (particles.empty() && a_domain.puddles.empty() && a_domain.blobs.empty() && a_domain.ropes.empty()) {
			return;  // an empty pool waiting for the next cycle: nothing to draw or update
		}
		const Fluid::DensityField field(particles, kKernel, settings.fluidFade);
		std::vector<std::size_t>  pooled;
		for (std::size_t i = 0; i < particles.size(); ++i) {
			if (!particles[i].asleep) {
				const float count = field.Count(particles[i].position);
				const bool milky = a_domain.liquid == Liquid::kMilk;
				float enter = milky ? 3.0f - 2.0f * settings.milkMerge : kBulkEnter;
				const bool airborne = milky && !particles[i].touching &&
				                      particles[i].position.z - a_domain.ground.Below(particles[i].position.x, particles[i].position.y, particles[i].position.z) > 3.0f * kSpacing;
				if (milky && particles[i].touching) {
					enter *= 2.5f;  // on skin it should run in streaks, only real pools become blobs
				}
				if (airborne) {
					enter = 1e9f;  // in the air milk is drawn as strands and drops, never merged into blobs
				}
				const float leave = milky ? enter * 0.8f : kBulkLeave;
				particles[i].bulk = particles[i].bulk ? count >= leave : count >= enter;
			}
			if (particles[i].bulk) {
				pooled.push_back(i);
			}
		}
		const auto clusters = Fluid::FindClusters(particles, pooled, 2.0f * kSpacing);
		std::vector<std::size_t> owner(particles.size(), Fluid::kNone);
		for (std::size_t c = 0; c < clusters.size(); ++c) {
			for (const auto member : clusters[c].members) {
				owner[member] = c;
			}
		}
		for (auto& blob : a_domain.blobs) {
			blob.used = false;
		}
		// settled puddles keep their blob from frame to frame (they are only re-posed when they change)
		for (const auto& [key, cell] : a_domain.puddles) {
			if (cell.blob >= 0 && static_cast<std::size_t>(cell.blob) < a_domain.blobs.size()) {
				auto& blob = a_domain.blobs[static_cast<std::size_t>(cell.blob)];
				if (blob.puddle && blob.puddleKey == key) {
					blob.used = true;
				}
			}
		}
		for (auto& rope : a_domain.ropes) {
			rope.used = false;
		}
		std::vector<std::vector<std::size_t>> hints(clusters.size());
		for (std::size_t b = 0; b < a_domain.blobs.size(); ++b) {
			auto& blob = a_domain.blobs[b];
			if (blob.live && blob.seed < particles.size() && owner[blob.seed] != Fluid::kNone) {
				hints[owner[blob.seed]].push_back(b);
				blob.used = true;
			} else {
				blob.live = false;
			}
		}

		const float blend = 1.0f - std::exp(-RE::GetSecondsSinceLastFrame() / kSmoothing);
		std::vector<std::size_t> claimed;
		struct Droplet
		{
			Vec3                center;
			float               radius;
			Vec3                velocity;
			std::optional<Vec3> contact;
		};
		std::vector<Droplet> droplets;
		const auto           contactOf = [&](std::span<const std::size_t> a_members, Vec3 a_center) -> std::optional<Vec3> {
			for (const auto m : a_members) {
				const auto& particle = particles[m];
				if (!particle.touching || particle.surface < 0) {
					continue;
				}
				const auto index = static_cast<std::size_t>(particle.surface);
				if (index < a_domain.capsules.size()) {
					const auto& capsule = a_domain.capsules[index];
					return Fluid::Normalize(particle.position - (capsule.a + (capsule.b - capsule.a) * particle.along), { 0.0f, 0.0f, 1.0f });
				}
				const auto& skin = a_domain.skin.Patches();
				if (index - a_domain.capsules.size() < skin.size()) {
					return skin[index - a_domain.capsules.size()].normal;
				}
			}
			if (a_center.z - a_domain.ground.Below(a_center.x, a_center.y, a_center.z) < kSpacing) {
				return Vec3{ 0.0f, 0.0f, 1.0f };
			}
			return std::nullopt;
		};
		const auto velocityOf = [&](std::span<const std::size_t> a_members) {
			Vec3 sum{};
			for (const auto m : a_members) {
				sum += particles[m].velocity;
			}
			return a_members.empty() ? sum : sum * (1.0f / static_cast<float>(a_members.size()));
		};
		const auto           weightOf = [&](std::span<const std::size_t> a_members) {
			float sum = 0.0f;
			for (const auto m : a_members) {
				sum += field.Weight(particles[m]);
			}
			return a_members.empty() ? 0.0f : sum / static_cast<float>(a_members.size());
		};
		for (std::size_t c = 0; c < clusters.size(); ++c) {
			const auto& cluster = clusters[c];
			if (cluster.members.size() < kBlobMinimum) {
				droplets.push_back({ cluster.center, Fluid::VolumeRadius(cluster.members.size(), kVolume) * std::cbrt(weightOf(cluster.members)),
					velocityOf(cluster.members), contactOf(cluster.members, cluster.center) });
				continue;
			}
			std::vector<std::size_t> seeds;
			for (const auto b : hints[c]) {
				seeds.push_back(a_domain.blobs[b].seed);
			}
			const bool settled = std::ranges::all_of(cluster.members, [&](std::size_t a_member) {
				return particles[a_member].asleep && particles[a_member].life > settings.fluidFade;
			});
			const float pieceRadius = a_domain.liquid == Liquid::kMilk ? (5.0f + 15.0f * settings.milkMerge) * kSpacing : kPieceRadius;
			for (const auto& piece : Fluid::SplitCluster(particles, cluster, pieceRadius, seeds)) {
				Blob* blob = piece.hint != Fluid::kNone ? &a_domain.blobs[hints[c][piece.hint]] : AcquireBlob(a_domain);
				if (!blob) {
					continue;
				}
				if (piece.hint != Fluid::kNone) {
					claimed.push_back(hints[c][piece.hint]);
				}
				const bool warm = blob->live && blob->surface.size() == _directions.size();
				if (settled && warm && piece.hint != Fluid::kNone) {
					blob->seed = piece.seed;
					continue;
				}
				const auto previous = warm ? std::span<const Fluid::SurfacePoint>(blob->surface) : std::span<const Fluid::SurfacePoint>{};
				// Lower surface threshold for milk = fatter surfaces that flow into each other.
				const float threshold = a_domain.liquid == Liquid::kMilk ? 2.2f - 1.4f * settings.milkMerge : kThreshold;
				if (!Fluid::WrapSurface(field, particles[piece.seed].position, piece.extent + kKernel, threshold, _directions, previous, _scratch)) {
					PoseDrop(a_domain, *blob, piece.center, Fluid::VolumeRadius(piece.count, kVolume));
					continue;
				}
				if (warm) {
					for (std::size_t i = 0; i < _scratch.size(); ++i) {
						auto&       current = blob->surface[i];
						const auto& target = _scratch[i];
						current.position = current.position + (target.position - current.position) * blend;
						current.normal = Fluid::Normalize(current.normal + (target.normal - current.normal) * blend, target.normal);
						current.distance = target.distance;
					}
				} else {
					blob->surface = _scratch;
				}
				blob->seed = piece.seed;
				blob->live = true;
				PoseBlob(a_domain, *blob);
			}
		}
		for (auto& hinted : hints) {
			for (const auto b : hinted) {
				if (std::ranges::find(claimed, b) == claimed.end()) {
					a_domain.blobs[b].used = false;
					a_domain.blobs[b].live = false;
				}
			}
		}

		std::size_t                                   ropes = 0;
		for (const auto& run : Fluid::FindRuns(particles)) {
			const auto held = std::ranges::find_if(a_domain.anchors, [&](const Fluid::Anchor& a) { return a.particle + 1 == run.end; });
			const auto* anchor = held != a_domain.anchors.end() ? &*held : nullptr;
			if (settings.debugLog && !anchor && run.Size() >= 2 && run.Size() <= 4) {
				Watch(a_domain, run);
			}
			if (run.Size() >= 2 || anchor) {
				if (Rope* rope = AcquireRope(a_domain)) {
					PoseRope(a_domain, *rope, run.begin, run.end, anchor);
					++ropes;
				}
				continue;
			}
			const auto& single = particles[run.begin];
			if (field.BulkCount(single.position) > kCovered) {
				continue;
			}
			const std::size_t index = run.begin;
			droplets.push_back({ single.position, Fluid::VolumeRadius(1, kVolume) * std::cbrt(field.Weight(single)), single.velocity,
				contactOf({ &index, 1 }, single.position) });
		}
		for (const auto& anchor : a_domain.anchors) {
			if (anchor.particle < particles.size() && particles[anchor.particle].bulk) {
				if (Rope* rope = AcquireRope(a_domain)) {
					PoseRope(a_domain, *rope, anchor.particle, anchor.particle + 1, &anchor);
					++ropes;
				}
			}
		}
		for (const auto& droplet : droplets) {
			if (droplet.radius <= 0.0f) {
				continue;
			}
			if (Blob* blob = AcquireBlob(a_domain)) {
				PoseDrop(a_domain, *blob, droplet.center, droplet.radius, droplet.velocity, droplet.contact);
			}
		}

		if (!a_domain.puddles.empty()) {
			const float fade = std::max(settings.fluidFade, 1e-3f);
			const float spread = 0.8f + 0.6f * settings.milkMerge;
			std::size_t drawn = 0;
			int         probes = kPuddleProbes;
			for (auto& [key, cell] : a_domain.puddles) {
				if (cell.count < 1.0f) {
					continue;  // only milk that slid through so far
				}
				if (drawn >= kMaxPuddleDraws) {
					break;
				}
				const Vec3 settled = cell.sum * (1.0f / cell.count);
				if (!Finite(settled)) {
					continue;
				}
				if (!cell.probed && probes > 0) {
					--probes;
					ProbePuddle(a_domain, cell, settled);
				}
				Blob* blob = nullptr;
				bool  fresh = false;
				if (cell.blob >= 0 && static_cast<std::size_t>(cell.blob) < a_domain.blobs.size()) {
					auto& kept = a_domain.blobs[static_cast<std::size_t>(cell.blob)];
					if (kept.puddle && kept.puddleKey == key) {
						blob = &kept;
					}
				}
				if (!blob) {
					blob = AcquireBlob(a_domain);
					if (!blob) {
						break;
					}
					blob->puddle = true;
					blob->puddleKey = key;
					cell.blob = static_cast<int>(blob - a_domain.blobs.data());
					fresh = true;
				}
				blob->used = true;
				++drawn;
				// Flat film: area from the milk volume; neighbouring cells overlap into one continuous puddle.
				float radius = std::sqrt(cell.count * kVolume * 2.5f / (std::numbers::pi_v<float> * kPuddleThickness)) * spread;
				radius = std::clamp(radius, 0.6f * kPuddleCell, 2.2f * kPuddleCell) * std::sqrt(std::clamp(cell.life / fade, 0.0f, 1.0f));
				const Vec3 flow = cell.flowWeight > 0.0f ? cell.flow * (1.0f / cell.flowWeight) : Vec3{};
				const bool changed = fresh || cell.probed != cell.shownProbed || std::abs(radius - cell.shownRadius) > 0.01f * radius + 1e-5f ||
				                     Fluid::Length(flow - cell.shownFlow) > 0.02f;
				if (changed) {
					const Vec3 center = cell.probed ? Vec3{ settled.x + cell.shift.x, settled.y + cell.shift.y, cell.surface } : settled;
					PosePuddle(a_domain, *blob, center, radius, flow, static_cast<std::uint64_t>(key) * 2654435761ull, cell.probed ? cell.normal : Vec3{ 0.0f, 0.0f, 1.0f });
					cell.shownRadius = radius;
					cell.shownFlow = flow;
					cell.shownProbed = cell.probed;
				} else {
					blob->root->SetAppCulled(false);
				}
			}
		}
		// Last line of defence: a shape with a broken transform must never reach the renderer.
		const auto broken = [](RE::NiAVObject* a_bone) {
			return a_bone && (!Finite(a_bone->local.translate) || !std::isfinite(a_bone->local.rotate.entry[0][0]) ||
								 !std::isfinite(a_bone->local.rotate.entry[1][1]) || !std::isfinite(a_bone->local.rotate.entry[2][2]));
		};
		for (auto& blob : a_domain.blobs) {
			if (blob.used && std::ranges::any_of(blob.bones, [&](const auto& a_bone) { return broken(a_bone); })) {
				blob.used = false;
				blob.live = false;
			}
		}
		for (auto& rope : a_domain.ropes) {
			if (rope.used && std::ranges::any_of(rope.bones, [&](const auto& a_bone) { return broken(a_bone); })) {
				rope.used = false;
			}
		}
		std::size_t blobs = 0;
		for (auto& blob : a_domain.blobs) {
			if (blob.used) {
				++blobs;
			} else {
				blob.live = false;
				blob.root->SetAppCulled(true);
			}
		}
		for (auto& rope : a_domain.ropes) {
			if (!rope.used) {
				rope.root->SetAppCulled(true);
			}
		}
		Reshade(a_domain);
		RE::NiUpdateData update{};
		a_domain.root->Update(update);
		a_domain.shownBlobs = blobs;
		a_domain.shownRopes = ropes;
		a_domain.pooled = pooled.size();
	}

	void FluidManager::Watch(Domain& a_domain, const Fluid::Run& a_run)
	{
		const auto& particles = a_domain.particles;
		const auto& first = particles[a_run.begin];
		const auto& last = particles[a_run.end - 1];
		const Vec3  span = last.position - first.position;
		const float length = Fluid::Length(span);
		if (length < 1e-4f) {
			return;
		}
		const Vec3 axis = span * (1.0f / length);
		const Vec3 relative = last.velocity - first.velocity;
		const bool resting = std::all_of(particles.begin() + static_cast<std::ptrdiff_t>(a_run.begin), particles.begin() + static_cast<std::ptrdiff_t>(a_run.end),
			[&](const Fluid::Particle& a_particle) {
				return a_particle.touching || a_particle.position.z - a_domain.ground.Below(a_particle.position.x, a_particle.position.y, a_particle.position.z) < kSpacing;
			});
		if (!resting) {
			return;
		}
		const float turn = Fluid::Length(relative - axis * Fluid::Dot(relative, axis)) / length;
		if (turn < kSpinning) {
			return;
		}
		++a_domain.spinning;
		if (!a_domain.spinSample.empty()) {
			return;
		}
		a_domain.spinSample = std::format("{} linked turning {:.0f} deg/s:", a_run.Size(), turn * 180.0f / std::numbers::pi_v<float>);
		for (std::size_t i = a_run.begin; i < a_run.end; ++i) {
			const auto&       particle = particles[i];
			const auto&       ground = a_domain.ground;
			const bool        known = ground.Known(ground.Node(particle.position.x), ground.Node(particle.position.y), particle.position.z);
			const std::size_t surface = particle.surface < 0 ? 0 : static_cast<std::size_t>(particle.surface);
			const char*       on = !particle.touching ? "free" : surface < a_domain.capsules.size() ? "capsule" : "skin";
			a_domain.spinSample += std::format(" [{}{}, {:.1f} cm above {} floor, speed {:.2f} m/s, link {:.1f}]", on, particle.asleep ? " asleep" : "",
				(particle.position.z - ground.Below(particle.position.x, particle.position.y, particle.position.z)) * 100.0f, known ? "measured" : "unmeasured",
				Fluid::Length(particle.velocity), particle.link / kSpacing);
		}
	}

	void FluidManager::Reshade(Domain& a_domain)
	{
		const auto& settings = Settings::Get();
		const float opacity = OpacityOf(a_domain.liquid);
		if (a_domain.opacity == opacity && a_domain.shine == settings.shine && a_domain.gloss == settings.gloss && a_domain.glow == settings.milkGlow) {
			return;
		}
		for (auto& blob : a_domain.blobs) {
			Shade(a_domain, blob.root.get());
		}
		for (auto& rope : a_domain.ropes) {
			Shade(a_domain, rope.root.get());
		}
		a_domain.opacity = opacity;
		a_domain.shine = settings.shine;
		a_domain.gloss = settings.gloss;
		a_domain.glow = settings.milkGlow;
	}

	void FluidManager::ShowShapes(float a_delta)
	{
		if (_debugAwaitingEditor) return;
		std::vector<RE::ActorHandle> actors;
		if (Settings::Get().showShapes) {
			const auto selected = RE::Console::GetSelectedRef();
			auto* actor = selected ? selected->As<RE::Actor>() : nullptr;
			if (actor && actor->Get3D(false)) {
				actors.push_back(actor->GetHandle());
			} else {
				for (const auto& [thread, members] : _scenes) {
					for (const auto& handle : members) {
						const auto member = handle.get();
						if (member && member->Get3D(false) && std::ranges::find(actors, handle) == actors.end()) {
							actors.push_back(handle);
						}
					}
				}
				if (actors.empty()) {
					if (auto* player = RE::PlayerCharacter::GetSingleton(); player && player->Get3D(false)) {
						actors.push_back(player->GetHandle());
					}
				}
			}
		}
		if (actors.empty()) {
			HideShapes();
			return;
		}
		if (_shapes && _shapes->hosts != actors) HideShapes();
		if (!_debugReady) {
			_debugStableTime += a_delta;
			_debugProbeTime -= a_delta;
			if (_debugProbeTime > 0.0f) return;
			_debugProbeTime = 0.25f;
			std::vector<std::uint64_t> signature;
			for (const auto& handle : actors) {
				const auto actor = handle.get();
				signature.push_back(Body::DebugMeshSignature(actor.get()));
			}
			const bool available = std::ranges::none_of(signature, [](auto value) { return value == 0; });
			if (!available || signature != _debugMeshSignature) _debugStableTime = 0.0f;
			_debugMeshSignature = std::move(signature);
			if (!available || _debugStableTime < 2.0f) return;
			_debugReady = true;
		}
		if (!CollisionMesh()) {
			HideShapes();
			return;
		}
		if (!_shapes) {
			_shapes = std::make_unique<Domain>();
			_shapes->diagnostic = true;
			_shapes->ropes.reserve(kMaxDebugShapes);
		}
		auto& domain = *_shapes;
		domain.hosts = actors;
		if (!Attach(domain)) {
			HideShapes();
			return;
		}
		domain.root->name = "OStrandsCollisionDebug";
		for (auto& rope : domain.ropes) {
			rope.used = false;
		}
		std::size_t missing = 0;
		const auto tube = [&](const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, float a_radius) {
			if (Rope* rope = AcquireRope(domain)) {
				PoseCapsule(*rope, a_from, a_to, a_radius);
			} else {
				++missing;
			}
		};
		const auto sphere = [&](const RE::NiPoint3& a_center, float a_radius) { tube(a_center, a_center, a_radius); };
		std::vector<Point> rim;
		RE::NiPoint3       center;
		RE::NiPoint3       inward;
		for (const auto& handle : actors) {
			const auto actor = handle.get();
			auto*      root = actor ? actor->Get3D(false) : nullptr;
			if (!root) {
				continue;
			}
			if (const auto surface = Body::SurfaceOf(actor.get()); !surface.patches.empty()) {
				const std::size_t stride = std::max<std::size_t>(1, surface.patches.size() / kWrapDots);
				for (std::size_t k = 0; k < surface.patches.size(); k += stride) {
					sphere(surface.patches[k].position, kMarkerRadius);
				}
				for (const auto& capsule : surface.extras) {
					tube(capsule.a, capsule.b, capsule.radius);
				}
			} else {
				for (const auto& capsule : Body::Of(actor.get())) {
					tube(capsule.a, capsule.b, capsule.radius);
				}
				for (const auto& capsule : Body::ShaftOf(actor.get())) {
					tube(capsule.a, capsule.b, capsule.radius);
				}
			}
			if (HasSchlong(actor.get())) {
				if (const auto tip = Body::TipOf(actor.get())) {
					sphere(tip->point, kMarkerRadius);
					tube(tip->point, tip->point + tip->axis * kPointerLength, kMarkerRadius * 0.3f);
				}
			}
			for (const auto opening : { Opening::kVagina, Opening::kAnus, Opening::kMouth }) {
				if (!FindOpening(root, opening, rim, center, inward)) {
					sphere(center - inward * kExitDepth, kMarkerRadius);
				}
			}
		}
		for (auto& rope : domain.ropes) {
			if (!rope.used) {
				rope.root->SetAppCulled(true);
			}
		}
		if (missing && !domain.incompleteReported) {
			logger::warn("collision display incomplete: {} shapes could not be drawn (limit {}); select one actor in the console", missing, kMaxDebugShapes);
		}
		domain.incompleteReported = missing != 0;
		RE::NiUpdateData update{};
		domain.root->Update(update);
	}

	void FluidManager::HideShapes()
	{
		_debugReady = false;
		_debugStableTime = _debugProbeTime = 0.0f;
		_debugMeshSignature.clear();
		if (_shapes) {
			Release(*_shapes);
			_shapes.reset();
		}
	}

	void FluidManager::WaitForBodyEditor(bool a_wait)
	{
		HideShapes();
		_debugAwaitingEditor = a_wait;
	}

	void FluidManager::Release(Domain& a_domain)
	{
		if (a_domain.root && a_domain.root->parent) {
			a_domain.root->parent->DetachChild2(a_domain.root.get());
		}
		if (!a_domain.diagnostic && a_domain.root) {
			for (auto& blob : a_domain.blobs) {
				if (!blob.root || _spareBlobs.size() >= kMaxBlobs) {
					continue;
				}
				a_domain.root->DetachChild2(blob.root.get());
				blob.root->SetAppCulled(true);
				blob.used = false;
				blob.live = false;
				blob.surface.clear();
				_spareBlobs.push_back(std::move(blob));
			}
			for (auto& rope : a_domain.ropes) {
				if (!rope.root || _spareRopes.size() >= kMaxRopes) {
					continue;
				}
				a_domain.root->DetachChild2(rope.root.get());
				rope.root->SetAppCulled(true);
				rope.used = false;
				_spareRopes.push_back(std::move(rope));
			}
		}
		a_domain.blobs.clear();
		a_domain.ropes.clear();
		a_domain.root.reset();
		a_domain.particles.clear();
		a_domain.puddles.clear();
	}

	Vec3 FluidManager::ToLocal(const Domain& a_domain, const RE::NiPoint3& a_point) const
	{
		return ToVec((a_point - a_domain.origin) / kUnitsPerMeter);
	}

	RE::NiPoint3 FluidManager::ToWorld(const Domain& a_domain, Vec3 a_point) const
	{
		return a_domain.origin + ToPoint(a_point) * kUnitsPerMeter;
	}

	void FluidManager::Update(float a_delta)
	{
		auto        settingsGuard = Settings::Lock();
		const auto& settings = Settings::Get();
		std::vector<Event> events;
		{
			std::scoped_lock guard(_lock);
			events.swap(_pending);
		}
		if (!settings.enabled) {
			if (!_domains.empty() || !_emitters.empty()) {
				Reset();
			}
			HideShapes();
			return;
		}
		for (auto& event : events) {
			Apply(event);
		}
		if (a_delta <= 0.0f) {
			return;
		}
		a_delta = std::min(a_delta, 0.05f);
		ShowShapes(a_delta);
		for (auto it = _loads.begin(); it != _loads.end();) {
			it->second.second += a_delta;
			it = it->second.second > kLoadMemory ? _loads.erase(it) : std::next(it);
		}
		Prewarm();
		if (_emitters.empty() && _domains.empty()) {
			return;
		}
		_cloneBudget = kClonesPerFrame;
		std::erase_if(_domains, [&](const std::unique_ptr<Domain>& a_domain) {
			if (a_domain->cell && a_domain->cell->IsAttached()) {
				return false;
			}
			if (settings.debugLog) {
				logger::info("milk pool {} closed: its cell unloaded", a_domain->id);
			}
			Release(*a_domain);
			return true;
		});
		const auto start = std::chrono::steady_clock::now();
		for (auto& domain : _domains) {
			try {
				Simulate(*domain, a_delta);
				if (domain->liquid == Liquid::kMilk) {
					// Milk that reaches the floor pools instead of lying there as strings.
					auto& particles = domain->particles;
					for (std::size_t k = 0; k < particles.size(); ++k) {
						const auto& particle = particles[k];
						const bool  floor = particle.position.z - domain->ground.Below(particle.position.x, particle.position.y, particle.position.z) < 1.5f * kSpacing;
						if (!Finite(particle.position) || !Finite(particle.velocity)) {
							// Never let a broken particle reach the renderer: drop it.
							particles[k].position = {};
							particles[k].velocity = {};
							particles[k].life = 0.0f;
							particles[k].link = 0.0f;
							if (k > 0) {
								particles[k - 1].link = 0.0f;
							}
							continue;
						}
						// on the body it is not a puddle yet; against a wall or pillar it is
						const bool onBody = particle.touching && particle.surface >= 0 && static_cast<std::size_t>(particle.surface) < domain->worldFrom;
						const float speed2 = particle.velocity.x * particle.velocity.x + particle.velocity.y * particle.velocity.y + particle.velocity.z * particle.velocity.z;
						if (floor && !onBody && speed2 >= 0.12f * 0.12f) {
							// Still sliding: remember which way the milk runs here so the splat is drawn along it.
							const auto mx = static_cast<std::int32_t>(std::floor(particle.position.x / kPuddleCell));
							const auto my = static_cast<std::int32_t>(std::floor(particle.position.y / kPuddleCell));
							const auto mkey = (static_cast<std::int64_t>(mx) << 32) ^ static_cast<std::uint32_t>(my);
							auto       moving = domain->puddles.find(mkey);
							if (moving == domain->puddles.end() && domain->puddles.size() < kMaxPuddleCells) {
								moving = domain->puddles.emplace(mkey, Puddle{}).first;
								moving->second.life = particle.life;
							}
							if (moving != domain->puddles.end()) {
								moving->second.flow += Vec3{ particle.velocity.x, particle.velocity.y, 0.0f };
								moving->second.flowWeight += 1.0f;
							}
						}
						// Settled = asleep, or simply slow on the floor (pressure jitter can keep a pile from ever sleeping).
						const bool flatHere = domain->ground.Flat(particle.position.x, particle.position.y, particle.position.z);
						if (floor && flatHere && !onBody && (particle.asleep || speed2 < 0.12f * 0.12f) && particle.link <= 0.0f &&
							(k == 0 || particles[k - 1].link <= 0.0f)) {
							// Settled on the floor: hand it to the static puddle and drop it from the simulation.
							// Most milk ends up here, so this is what keeps thousands of particles cheap.
							const auto cx = static_cast<std::int32_t>(std::floor(particle.position.x / kPuddleCell));
							const auto cy = static_cast<std::int32_t>(std::floor(particle.position.y / kPuddleCell));
							const auto key = (static_cast<std::int64_t>(cx) << 32) ^ static_cast<std::uint32_t>(cy);
							auto       cell = domain->puddles.find(key);
							if (cell == domain->puddles.end() && domain->puddles.size() < kMaxPuddleCells) {
								cell = domain->puddles.emplace(key, Puddle{}).first;
							}
							if (cell != domain->puddles.end()) {
								cell->second.sum += particle.position;
								cell->second.count += 1.0f;
								cell->second.life = std::max(cell->second.life, particle.life);
								particles[k].life = 0.0f;  // removed by Expire next frame
							}
						}
						if (floor || particle.asleep) {
							particles[k].link = 0.0f;
							if (k > 0) {
								particles[k - 1].link = 0.0f;
							}
						}
						// on a wall: a strand stretched far apart (a drop held back, another slid away) is let go
						const bool onWall = particle.touching && particle.surface >= 0 && static_cast<std::size_t>(particle.surface) >= domain->worldFrom;
						if (onWall && k + 1 < particles.size() && particles[k].link > 0.0f &&
							Fluid::Length(particles[k + 1].position - particle.position) > 3.0f * kSpacing) {
							particles[k].link = 0.0f;
						}
					}
				}
				Expire(*domain, a_delta);
				std::erase_if(domain->puddles, [&](auto& a_cell) { return (a_cell.second.life -= a_delta) <= 0.0f; });
			} catch (const std::exception& error) {
				logger::error("milk pool {} closed: {}", domain->id, error.what());
				domain->particles.clear();
				domain->anchors.clear();
				domain->broken = true;
			}
		}
		std::erase_if(_emitters, [&](Emitter& a_emitter) { return !Step(a_emitter, a_delta); });
		for (auto& domain : _domains) {
			if (domain->liquid == Liquid::kMilk) {
				Regroup(*domain);
			}
		}
		std::erase_if(_reservoirs, [&](Reservoir& a_reservoir) { return !Leak(a_reservoir, a_delta); });
		for (auto& domain : _domains) {
			std::erase_if(domain->anchors, [&](const Fluid::Anchor& a_anchor) {
				return std::ranges::none_of(_emitters, [&](const Emitter& e) { return e.id == a_anchor.owner; }) &&
				       std::ranges::none_of(_reservoirs, [&](const Reservoir& r) { return r.id == a_anchor.owner; });
			});
		}
		const auto middle = std::chrono::steady_clock::now();
		// Idle time from which an empty pool counts as surplus (only the newest kMaxEmptyPools may wait).
		float surplusIdle = std::numeric_limits<float>::max();
		{
			std::vector<float> idles;
			for (const auto& domain : _domains) {
				if (domain->particles.empty() && domain->puddles.empty() && domain->idle > 0.0f) {
					idles.push_back(domain->idle);
				}
			}
			if (idles.size() > kMaxEmptyPools) {
				std::ranges::sort(idles);
				surplusIdle = idles[kMaxEmptyPools];
			}
		}
		std::erase_if(_domains, [&](const std::unique_ptr<Domain>& a_domain) {
			const bool fed = std::ranges::any_of(_emitters, [&](const Emitter& e) { return e.domain == a_domain->id; }) ||
			                 std::ranges::any_of(_reservoirs, [&](const Reservoir& r) { return r.domain == a_domain->id; });
			// An empty milk pool stays open for a while: the next leak cycle (usually seconds later) reuses its meshes and
			// floor samples instead of closing and rebuilding everything every cycle.
			const bool empty = a_domain->particles.empty() && a_domain->puddles.empty() && !fed;
			a_domain->idle = empty ? a_domain->idle + a_delta : 0.0f;
			const bool surplus = empty && a_domain->idle >= surplusIdle;  // too many empty pools: oldest go first
			const bool done = empty && (a_domain->liquid != Liquid::kMilk || a_domain->idle > kPoolLinger || surplus);
			if (empty && !done && (!a_domain->blobs.empty() || !a_domain->ropes.empty())) {
				ReleaseShapes(*a_domain);  // its meshes go back to the spares at once, ready for the next pool
			}
			if (a_domain->broken || done || !Attach(*a_domain)) {
				if (settings.debugLog) {
					logger::info("milk pool {} closed after {:.0f}s", a_domain->id, a_domain->age);
				}
				Release(*a_domain);
				return true;
			}
			return false;
		});
		for (auto& domain : _domains) {
			Draw(*domain);
		}

		if (settings.debugLog) {
			const auto end = std::chrono::steady_clock::now();
			_statSimulate += std::chrono::duration<double, std::micro>(middle - start).count();
			_statDraw += std::chrono::duration<double, std::micro>(end - middle).count();
			_statWorst = std::max(_statWorst, std::chrono::duration<double, std::micro>(end - start).count());
			++_statFrames;
			_statTime += a_delta;
			if (_statTime >= 10.0f) {
				for (const auto& domain : _domains) {
					logger::info("milk pool {}: {} particles, {} pooled, {} blobs, {} ropes, {} spinning resting pills", domain->id,
						domain->particles.size(), domain->pooled, domain->shownBlobs, domain->shownRopes, domain->spinning);
					if (!domain->spinSample.empty()) {
						logger::info("milk pool {} spin sample: {}", domain->id, domain->spinSample);
					}
					domain->spinning = 0;
					domain->spinSample.clear();
				}
				logger::info("fluid cost: simulate {:.0f} us, draw {:.0f} us per frame, worst frame {:.0f} us; {} mesh(es) created; {} emitter(s)",
					_statSimulate / std::max(_statFrames, 1), _statDraw / std::max(_statFrames, 1), _statWorst, _statClones, _emitters.size());
				_statWorst = 0.0;
				_statClones = 0;

				_statTime = 0.0f;
				_statSimulate = 0.0;
				_statDraw = 0.0;
				_statFrames = 0;
			}
		}
	}
}
