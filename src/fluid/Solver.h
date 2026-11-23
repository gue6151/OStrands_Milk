#pragma once

#include "fluid/Ground.h"
#include "fluid/Particle.h"
#include "fluid/Skin.h"

#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace OStrands::Fluid
{
	struct Parameters
	{
		std::uint32_t iterations{ 4 };
		float         cohesion{ 4.0f };
		float         viscosity{ 40.0f };
		float         gravity{ 9.81f };
		float         stretch{ 3.0f };
		float         friction{ 30.0f };
		float         grip{ 0.6f };
		float         pressure{ 1.0f };       // scales how hard overlapping particles push apart each iteration
		float         bounce{ 1.0f };         // share of collision push-out that turns into velocity
		float         floorFriction{ 8.0f };
		float         airCohesion{ 1.0f };    // cohesion share for particles in free flight (low keeps jets from beading up)
		float         skinGravity{ 0.25f };   // share of gravity while running over skin
		float         pullSpeed{ 1e9f };      // faster than this (m/s) a particle is not held onto the skin
		float         drag{ 0.0f };           // air drag (1/s), bends fast jets into natural arcs
		bool          pushLinks{ true };      // false: strand links only pull (a liquid thread never pushes its head forward)
		float         wallGrip{ -1.0f };      // grip on world objects (walls, pillars, furniture); below 0 = same as grip
		float         maxSpeed{ 6.0f };       // m/s a particle may reach (raised only for jets set above the old maximum)
	};

	struct Anchor
	{
		std::size_t   particle{};
		Vec3          point;
		float         rest{};
		std::uint32_t owner{};
	};

	struct Capsule
	{
		Vec3  a;
		Vec3  b;
		float radius{};
		Vec3  velocityA;
		Vec3  velocityB;
	};

	struct Hole
	{
		Vec3  center;
		float radius{};
	};

	// A surface found on a particle's own flight path this frame (by a ray against the real geometry).
	struct Plane
	{
		Vec3 point;
		Vec3 normal;
		bool valid{ false };
	};

	struct Colliders
	{
		std::span<const Capsule> capsules;
		std::span<const Hole>    holes;
		const Skin*              skin{ nullptr };
		bool                     walls{ false };  // the skin holds wall / object samples (else the swept wall test is skipped)
		std::span<const Plane>   guards;          // per particle (may be empty)
	};

	class Solver
	{
	public:
		Solver(float a_spacing, Parameters a_parameters);

		static bool Valid(const Parameters& a_parameters);
		void        Configure(const Parameters& a_parameters);

		void  Step(std::span<Particle> a_particles, std::span<Anchor> a_anchors, const Colliders& a_colliders, const Ground& a_ground, float a_seconds);
		float Spacing() const { return _spacing; }

	private:
		struct Cell
		{
			int  x, y, z;
			bool operator==(const Cell&) const = default;
		};

		struct CellHash
		{
			std::size_t operator()(Cell a_cell) const noexcept;
		};

		Cell  CellOf(Vec3 a_point) const;
		float Weight(Vec3 a_delta) const;
		Vec3  Gradient(Vec3 a_delta) const;
		void  FindNeighbors(std::span<const Particle> a_particles, bool a_all);
		void  Collide(std::span<const Particle> a_particles, const Colliders& a_colliders, bool a_pull);
		static std::optional<Vec3> Carrier(const Colliders& a_colliders, int a_surface, float a_along, Vec3 a_position);

		float                                                _spacing;
		float                                                _radiusSquared;
		float                                                _restWeight{};
		Parameters                                           _parameters;
		std::vector<Vec3>                                    _predicted;
		std::vector<Vec3>                                    _velocity;
		std::vector<Vec3>                                    _correction;
		std::vector<Vec3>                                    _pushed;
		std::vector<float>                                   _lambda;
		std::vector<bool>                                    _anchored;
		std::vector<float>                                   _floor;
		std::vector<float>                                   _weights;
		std::vector<int>                                     _contact;
		std::vector<Vec3>                                    _guarded;  // face a drop was stopped on this step (zero = none)
		std::vector<float>                                   _along;
		std::vector<Vec3>                                    _normal;
		std::vector<std::vector<std::size_t>>                _neighbors;
		std::unordered_map<Cell, std::vector<std::size_t>, CellHash> _grid;
	};
}
