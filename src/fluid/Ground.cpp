#include "fluid/Ground.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace OStrands::Fluid
{
	namespace
	{
		constexpr float kCliff = 0.03f;  // spread of a smooth floor or slope across one cell
		constexpr float kStep = 0.06f;   // more than this: an edge (table, seat, step), not a slope
		constexpr float kAbove = 0.02f;  // a surface this far above still counts (it is standing in it)
		constexpr float kRecheck = 0.07f;  // re-measure a point once milk is this close below where it was measured from

		float Bilinear(const std::array<std::optional<float>, 4>& a_c, float a_fu, float a_fv)
		{
			const float bottom = *a_c[0] + (*a_c[1] - *a_c[0]) * a_fu;
			const float top = *a_c[2] + (*a_c[3] - *a_c[2]) * a_fu;
			return bottom + (top - bottom) * a_fv;
		}

		float Nearest(const std::array<std::optional<float>, 4>& a_c, float a_fu, float a_fv)
		{
			return *a_c[static_cast<std::size_t>((a_fu < 0.5f ? 0 : 1) + (a_fv < 0.5f ? 0 : 2))];
		}

		bool Complete(const std::array<std::optional<float>, 4>& a_c, float& a_low, float& a_high)
		{
			a_low = std::numeric_limits<float>::max();
			a_high = std::numeric_limits<float>::lowest();
			bool complete = true;
			for (const auto& corner : a_c) {
				if (!corner) {
					complete = false;
					continue;
				}
				a_low = std::min(a_low, *corner);
				a_high = std::max(a_high, *corner);
			}
			return complete;
		}
	}

	Ground::Ground(float a_cell) :
		_cell(a_cell)
	{}

	std::uint64_t Ground::Key(int a_x, int a_y)
	{
		return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a_x)) << 32) | static_cast<std::uint32_t>(a_y);
	}

	int Ground::Node(float a_value) const { return static_cast<int>(std::floor(a_value / _cell)); }
	int Ground::FineNode(float a_value) const { return static_cast<int>(std::floor(a_value / FineCell())); }

	bool Ground::Known(int a_x, int a_y, float a_z) const
	{
		const auto it = _nodes.find(Key(a_x, a_y));
		return it != _nodes.end() && a_z <= it->second.top - kRecheck;
	}

	bool Ground::FineKnown(int a_x, int a_y, float a_z) const
	{
		const auto it = _fine.find(Key(a_x, a_y));
		return it != _fine.end() && a_z <= it->second.top - kRecheck;
	}

	void Ground::Store(Map& a_map, int a_x, int a_y, float a_top, std::span<const float> a_layers)
	{
		auto& node = a_map[Key(a_x, a_y)];
		node.top = a_top;
		node.count = 0;
		for (const float layer : a_layers) {
			if (node.count < node.layers.size() && std::isfinite(layer)) {
				node.layers[node.count++] = layer;
			}
		}
		std::sort(node.layers.begin(), node.layers.begin() + node.count, std::greater<>());
		if (node.count == 0) {
			node.layers[node.count++] = 0.0f;
		}
	}

	void Ground::Set(int a_x, int a_y, float a_top, std::span<const float> a_layers) { Store(_nodes, a_x, a_y, a_top, a_layers); }
	void Ground::SetFine(int a_x, int a_y, float a_top, std::span<const float> a_layers) { Store(_fine, a_x, a_y, a_top, a_layers); }

	std::optional<float> Ground::Resolve(const Map& a_map, int a_x, int a_y, float a_z)
	{
		const auto it = a_map.find(Key(a_x, a_y));
		if (it == a_map.end()) {
			return std::nullopt;
		}
		const auto& node = it->second;
		for (std::uint8_t k = 0; k < node.count; ++k) {
			if (node.layers[k] <= a_z + kAbove) {
				return node.layers[k];  // the highest surface at or under it
			}
		}
		return a_z - 1.0f;  // every surface measured here is above it (it already went below the floor): nothing holds it
	}

	std::array<std::optional<float>, 4> Ground::Corners(const Map& a_map, float a_cell, float a_x, float a_y, float a_z, float& a_fu, float& a_fv) const
	{
		const float u = a_x / a_cell;
		const float v = a_y / a_cell;
		const int   x = static_cast<int>(std::floor(u));
		const int   y = static_cast<int>(std::floor(v));
		a_fu = u - static_cast<float>(x);
		a_fv = v - static_cast<float>(y);
		return { Resolve(a_map, x, y, a_z), Resolve(a_map, x + 1, y, a_z), Resolve(a_map, x, y + 1, a_z), Resolve(a_map, x + 1, y + 1, a_z) };
	}

	bool Ground::NearLayer(float a_x, float a_y, float a_height, float a_tolerance) const
	{
		const int x = Node(a_x);
		const int y = Node(a_y);
		for (int dx = 0; dx <= 1; ++dx) {
			for (int dy = 0; dy <= 1; ++dy) {
				const auto it = _nodes.find(Key(x + dx, y + dy));
				if (it == _nodes.end()) {
					continue;
				}
				for (std::uint8_t k = 0; k < it->second.count; ++k) {
					if (std::abs(it->second.layers[k] - a_height) <= a_tolerance) {
						return true;
					}
				}
			}
		}
		return false;
	}

	std::optional<float> Ground::Under(float a_x, float a_y, float a_height) const
	{
		const int            x = Node(a_x);
		const int            y = Node(a_y);
		std::optional<float> lowest;
		for (int dx = 0; dx <= 1; ++dx) {
			for (int dy = 0; dy <= 1; ++dy) {
				const auto it = _nodes.find(Key(x + dx, y + dy));
				if (it == _nodes.end()) {
					continue;
				}
				for (std::uint8_t k = 0; k < it->second.count; ++k) {
					if (it->second.layers[k] < a_height) {
						lowest = lowest ? std::min(*lowest, it->second.layers[k]) : it->second.layers[k];
						break;
					}
				}
			}
		}
		return lowest;
	}

	float Ground::Below(float a_x, float a_y, float a_z) const
	{
		float      fu{}, fv{};
		const auto corners = Corners(_nodes, _cell, a_x, a_y, a_z, fu, fv);
		float      lowest{}, highest{};
		const bool complete = Complete(corners, lowest, highest);
		if (highest == std::numeric_limits<float>::lowest()) {
			return 0.0f;
		}
		if (!complete) {
			return highest;
		}
		if (highest - lowest <= kCliff && highest <= a_z + kAbove) {
			return Bilinear(corners, fu, fv);  // smooth floor or slope: as before
		}
		if (highest - lowest <= kStep) {
			return highest;
		}
		// An edge runs through this cell: use the fine grid there if it has been measured, else the nearest point.
		float      gu{}, gv{};
		const auto fine = Corners(_fine, FineCell(), a_x, a_y, a_z, gu, gv);
		float      fineLow{}, fineHigh{};
		if (Complete(fine, fineLow, fineHigh)) {
			if (fineHigh - fineLow <= kCliff && fineHigh <= a_z + kAbove) {
				return Bilinear(fine, gu, gv);
			}
			return Nearest(fine, gu, gv);
		}
		return Nearest(corners, fu, fv);
	}

	bool Ground::Step(float a_x, float a_y, float a_z) const
	{
		float      fu{}, fv{};
		const auto corners = Corners(_nodes, _cell, a_x, a_y, a_z, fu, fv);
		float      lowest{}, highest{};
		return Complete(corners, lowest, highest) && highest - lowest > kStep;
	}

	bool Ground::Flat(float a_x, float a_y, float a_z) const
	{
		float      fu{}, fv{};
		const auto corners = Corners(_nodes, _cell, a_x, a_y, a_z, fu, fv);
		float      lowest{}, highest{};
		if (!Complete(corners, lowest, highest)) {
			return true;  // not measured yet: as before
		}
		return highest - lowest <= kCliff && highest <= a_z + kAbove;
	}

	Vec3 Ground::Gradient(float a_x, float a_y, float a_z) const
	{
		float      fu{}, fv{};
		const auto c = Corners(_nodes, _cell, a_x, a_y, a_z, fu, fv);
		float      lowest{}, highest{};
		if (!Complete(c, lowest, highest) || highest - lowest > kCliff) {
			return {};
		}
		return { ((*c[1] - *c[0]) * (1.0f - fv) + (*c[3] - *c[2]) * fv) / _cell, ((*c[2] - *c[0]) * (1.0f - fu) + (*c[3] - *c[1]) * fu) / _cell, 0.0f };
	}
}
