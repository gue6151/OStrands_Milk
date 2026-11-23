#pragma once

#include "fluid/Particle.h"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>

namespace OStrands::Fluid
{
	// Floor heights on a grid. Each grid point keeps every surface found under it (table top, seat, floor ...), so milk
	// under a table falls to the floor and milk on the table stays on it. Near edges a finer grid takes over.
	class Ground
	{
	public:
		explicit Ground(float a_cell);

		float Cell() const { return _cell; }
		float FineCell() const { return _cell * 0.25f; }
		int   Node(float a_value) const;
		int   FineNode(float a_value) const;
		// measured from at least this high (a point above that may have surfaces under it not yet seen)
		bool  Known(int a_x, int a_y, float a_z) const;
		bool  FineKnown(int a_x, int a_y, float a_z) const;
		// a_layers: surfaces top to bottom; a_top: where the measuring ray started
		void  Set(int a_x, int a_y, float a_top, std::span<const float> a_layers);
		void  SetFine(int a_x, int a_y, float a_top, std::span<const float> a_layers);
		// a surface accepted by the coarse grid near this point at about this height (a real shelf, seat or floor)
		bool  NearLayer(float a_x, float a_y, float a_height, float a_tolerance) const;
		// the surface the coarse grid found under a_height around this point (lowest of the four), if any
		std::optional<float> Under(float a_x, float a_y, float a_height) const;
		// The ground a point at height a_z can be standing on: only surfaces at or below it count.
		float Below(float a_x, float a_y, float a_z) const;
		// The four grid points around are at different levels (an edge of a table, seat or step).
		bool  Step(float a_x, float a_y, float a_z) const;
		// All four grid points around agree: a real floor or table top, not the edge of something.
		bool  Flat(float a_x, float a_y, float a_z) const;
		Vec3  Gradient(float a_x, float a_y, float a_z) const;

	private:
		struct Point
		{
			float                top{};
			std::uint8_t         count{};
			std::array<float, 4> layers{};  // highest first
		};
		using Map = std::unordered_map<std::uint64_t, Point>;

		static std::uint64_t Key(int a_x, int a_y);
		static void          Store(Map& a_map, int a_x, int a_y, float a_top, std::span<const float> a_layers);
		static std::optional<float> Resolve(const Map& a_map, int a_x, int a_y, float a_z);
		std::array<std::optional<float>, 4> Corners(const Map& a_map, float a_cell, float a_x, float a_y, float a_z, float& a_fu, float& a_fv) const;

		float _cell;
		Map   _nodes;
		Map   _fine;
	};
}
