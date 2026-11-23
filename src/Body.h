#pragma once

#include "fluid/Skin.h"

#include <unordered_set>

namespace OStrands::Body
{
	struct Capsule
	{
		RE::NiPoint3 a;
		RE::NiPoint3 b;
		float        radius{ 0.0f };
		RE::NiPoint3 velocityA;
		RE::NiPoint3 velocityB;
	};

	struct Hole
	{
		RE::NiPoint3 center;
		float        radius{ 0.0f };
	};

	struct Tip
	{
		RE::NiPoint3 point;
		RE::NiPoint3 axis;
	};

	struct Patch
	{
		RE::NiPoint3 position;
		RE::NiPoint3 normal;
		RE::NiPoint3 velocity;
	};

	struct Surface
	{
		std::span<const Patch>   patches;
		std::span<const Capsule> extras;
	};

	void                     BeginFrame(float a_delta);
	void                     RequestRefit();
	std::uint64_t            DebugMeshSignature(RE::Actor* a_actor);
	std::span<const Capsule> Of(RE::Actor* a_actor);
	std::span<const Capsule> ShaftOf(RE::Actor* a_actor);
	std::optional<Tip>       TipOf(RE::Actor* a_actor);
	// Nipple on the body mesh (a_side 0 = left, 1 = right); follows skinning, physics and BodyMorph.
	std::optional<Tip>       NippleOf(RE::Actor* a_actor, int a_side);
	Surface                  SurfaceOf(RE::Actor* a_actor);
	// Same, but only the skin near a world-space box has to be exact this frame: points far from it are refreshed in
	// turn every few frames (they are filtered out by the caller anyway). Cuts the per-frame skinning cost.
	// a_refresh: skin away from the milk is re-posed every this many frames (bystanders can use a slower refresh)
	Surface                  SurfaceOf(RE::Actor* a_actor, const std::unordered_set<std::int64_t>& a_cells, float a_cell, std::uint64_t a_refresh = 3);
	std::int64_t             CellKey(const RE::NiPoint3& a_point, float a_cell);
	const Fluid::Skin*       GridOf(RE::Actor* a_actor);
	void                     Reset();

	RE::NiPoint3 Closest(const Capsule& a_capsule, const RE::NiPoint3& a_point, float* a_along = nullptr);
	bool         PushOut(std::span<const Capsule> a_capsules, std::span<const Hole> a_holes, RE::NiPoint3& a_point, float a_margin);
	bool         PushOut(const Fluid::Skin& a_skin, RE::NiPoint3& a_point, float a_margin);
}
