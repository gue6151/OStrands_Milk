#pragma once

#include "Body.h"

namespace OStrands::Lactation
{
	// Soul Gem Oven 4 IF (Sgo4IF.esp) integration and nipple placement.
	void Connect();
	void Update(float a_delta);
	void Reset();

	// SGO keeps filling to twice its own "max milk" setting, so the menu shows SGO percentages against that real maximum
	// (display only: the percentages used inside stay relative to SGO's setting, as before).
	inline constexpr float kSGOTrueMax = 2.0f;

	bool SGOInstalled();
	// Live SGO milk level text for the menu, built on the game thread.
	std::string Readout();
	// SGO milk fullness in percent (FactionMilkPercent rank), or -1 when unknown.
	int  MilkPercent(RE::Actor* a_actor);
	bool IsFemale(RE::Actor* a_actor);
	bool ClimaxAllowed(RE::Actor* a_actor);
	// a_side: 0 = left, 1 = right. Point sits just outside the skin, axis points out of the nipple.
	std::optional<Body::Tip> NippleOf(RE::Actor* a_actor, int a_side);
	// Milk starts spraying from the nipples: at the top leak stages it also starts dripping off the vulva a moment later
	// and keeps dripping while it flows. a_flow = how long this spray lasts (s).
	void OnSpray(RE::ActorHandle a_actor, float a_flow);
	// Share (0..1) of the spray strength and amount for this milk level: the minimum at the leak threshold,
	// 100% at the 'full' level. 1 when the option is off or the level is unknown.
	float SprayPower(RE::Actor* a_actor, bool a_regularLeak);
}
