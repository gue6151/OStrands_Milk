#include "Milk.h"

#include "FluidManager.h"
#include "Lactation.h"
#include "Settings.h"

#include "SKSEMenuFramework.h"

#include <SimpleIni.h>
#include <charconv>
#include <cmath>
#include <format>
#include <numbers>

namespace OStrands::Milk
{
	namespace
	{
		constexpr const char* kPath = "Data/SKSE/Plugins/OStrands_Milk.ini";
		constexpr const char* kUserPath = "Data/SKSE/Plugins/OStrands_Milk_User.ini";
		constexpr const char* kSection = "Milk";

		Config config;

		constexpr float kStrengthTop = 60.0f;  // was 20

		float SpeedOf(float a_strength)
		{
			// 0 = slow trickle that runs down the breast, 20 = hard jet, up to 60 = three times that.
			// Same straight line as before, so every value up to 20 gives exactly the jet it always did.
			return 0.03f + 0.09f * std::clamp(a_strength, 0.0f, kStrengthTop);
		}

		bool Read(Config& a_config, const char* a_path)
		{
			CSimpleIniA ini;
			ini.SetUnicode();
			if (ini.LoadFile(a_path) < 0) {
				return false;
			}
			auto f = [&](const char* key, float& value) {
				const double parsed = ini.GetDoubleValue(kSection, key, value);
				if (std::isfinite(parsed)) {
					value = static_cast<float>(parsed);
				}
			};
			auto i = [&](const char* key, int& value) { value = static_cast<int>(ini.GetLongValue(kSection, key, value)); };
			auto b = [&](const char* key, bool& value) { value = ini.GetBoolValue(kSection, key, value); };
			i("Opacity", a_config.opacity);
			i("Glow", a_config.glow);
			f("Interval", a_config.interval);
			f("StrengthMin", a_config.strengthMin);
			f("StrengthMax", a_config.strengthMax);
			i("SprayMode", a_config.mode);
			f("FlowTime", a_config.flow);
			i("StreamsMin", a_config.streamsMin);
			i("StreamsMax", a_config.streamsMax);
			f("StreamAngle", a_config.cone);
			i("StreamThickness", a_config.thickness);
			i("LeakTextureFrom", a_config.stageFrom);
			i("LeakTextureTo", a_config.stageTo);
			f("Viscosity", a_config.viscosity);
			f("Stickiness", a_config.stickiness);
			if (ini.GetValue(kSection, "StickinessWorld") == nullptr) {
				if (ini.GetValue(kSection, "Stickiness") != nullptr) {
					a_config.stickinessWorld = a_config.stickiness;  // saved before this option existed: objects keep behaving like skin
				}
			} else {
				f("StickinessWorld", a_config.stickinessWorld);
			}
			b("SGOSync", a_config.sgoSync);
			b("Clothed", a_config.clothed);
			b("Sex", a_config.sex);
			f("Lifetime", a_config.lifetime);
			f("FadeTime", a_config.fade);
			i("MaxParticles", a_config.maxParticles);
			f("Amount", a_config.amount);
			f("NippleForward", a_config.nippleForward);
			f("NippleUp", a_config.nippleUp);
			f("NippleSide", a_config.nippleSide);
			i("Merge", a_config.merge);
			i("FullPercent", a_config.fullPercent);
			b("Overlay", a_config.overlay);
			i("OverlayR", a_config.overlayR);
			i("OverlayG", a_config.overlayG);
			i("OverlayB", a_config.overlayB);
			f("OverlayGloss", a_config.overlayGloss);
			f("OverlaySpecular", a_config.overlaySpecular);
			f("OverlayBoost", a_config.overlayBoost);
			b("FillRamp", a_config.ramp);
			i("FillRampFull", a_config.rampFull);
			i("FillRampMin", a_config.rampMin);
			b("FillRampAll", a_config.rampAll);
			b("VulvaDrip", a_config.crotch);
			i("VulvaDripStage", a_config.crotchStage);
			f("VulvaDripInterval", a_config.crotchInterval);
			f("VulvaDripAmount", a_config.crotchAmount);
			f("VulvaDripDelay", a_config.crotchDelay);
			b("DebugLog", a_config.debugLog);
			return true;
		}
	}

	void Config::Validate()
	{
		const Config defaults;
		auto bounded = [](float value, float fallback, float min, float max) { return std::isfinite(value) ? std::clamp(value, min, max) : fallback; };
		opacity = std::clamp(opacity, 0, 100);
		glow = std::clamp(glow, 0, 100);
		interval = bounded(interval, defaults.interval, 0.0f, 60.0f);
		merge = std::clamp(merge, 0, 100);
		fullPercent = std::clamp(fullPercent, 10, 300);
		crotchStage = std::clamp(crotchStage, 1, 8);
		rampFull = std::clamp(rampFull, 10, 400);
		rampMin = std::clamp(rampMin, 0, 100);
		crotchInterval = bounded(crotchInterval, defaults.crotchInterval, 1.0f, 30.0f);
		crotchAmount = bounded(crotchAmount, defaults.crotchAmount, 0.0f, 3.0f);
		crotchDelay = bounded(crotchDelay, defaults.crotchDelay, 0.0f, 10.0f);
		overlayR = std::clamp(overlayR, 0, 255);
		overlayG = std::clamp(overlayG, 0, 255);
		overlayB = std::clamp(overlayB, 0, 255);
		overlayGloss = bounded(overlayGloss, defaults.overlayGloss, 0.0f, 100.0f);
		overlaySpecular = bounded(overlaySpecular, defaults.overlaySpecular, 0.0f, 10.0f);
		overlayBoost = bounded(overlayBoost, defaults.overlayBoost, 1.0f, 4.0f);
		strengthMin = bounded(strengthMin, defaults.strengthMin, 0.0f, 60.0f);
		strengthMax = bounded(strengthMax, defaults.strengthMax, 0.0f, 60.0f);
		mode = std::clamp(mode, 0, 4);
		flow = bounded(flow, defaults.flow, 0.5f, 30.0f);
		streamsMin = std::clamp(streamsMin, 1, 12);
		streamsMax = std::clamp(streamsMax, 1, 12);
		cone = bounded(cone, defaults.cone, 0.0f, 45.0f);
		thickness = std::clamp(thickness, 20, 150);
		stageFrom = std::clamp(stageFrom, 1, 300);
		stageTo = std::clamp(stageTo, stageFrom, 300);
		viscosity = bounded(viscosity, defaults.viscosity, 0.0f, 30.0f);
		stickiness = bounded(stickiness, defaults.stickiness, 0.0f, 20.0f);
		stickinessWorld = bounded(stickinessWorld, stickiness, 0.0f, 20.0f);
		lifetime = bounded(lifetime, defaults.lifetime, 0.0f, 60.0f);
		fade = bounded(fade, defaults.fade, 0.0f, 10.0f);
		maxParticles = std::clamp(maxParticles, 0, 3000);
		amount = bounded(amount, defaults.amount, 0.0f, 6.0f);
		nippleForward = bounded(nippleForward, 0.0f, -2.0f, 4.0f);
		nippleUp = bounded(nippleUp, 0.0f, -2.0f, 4.0f);
		nippleSide = bounded(nippleSide, 0.0f, -2.0f, 4.0f);
	}

	Config& Get() { return config; }

	std::unique_lock<std::recursive_mutex> Lock()
	{
		// Shares OStrands' settings lock so the fluid engine never sees a half-applied change.
		return Settings::Lock();
	}

	void Load()
	{
		auto   guard = Lock();
		Config next;
		const bool shipped = Read(next, kPath);
		const bool user = Read(next, kUserPath);
		next.Validate();
		config = next;
		logger::info("milk settings: {}{}", shipped ? "OStrands_Milk.ini" : "built-in defaults", user ? " + OStrands_Milk_User.ini" : "");
		Apply();
	}

	bool Save()
	{
		auto guard = Lock();
		try {
			CSimpleIniA ini;
			ini.SetUnicode();
			ini.LoadFile(kUserPath);
			const auto f = [&](const char* key, float value) { ini.SetValue(kSection, key, std::format("{}", value).c_str()); };
			const auto i = [&](const char* key, int value) { ini.SetValue(kSection, key, std::to_string(value).c_str()); };
			const auto b = [&](const char* key, bool value) { ini.SetValue(kSection, key, value ? "true" : "false"); };
			i("Opacity", config.opacity);
			i("Glow", config.glow);
			f("Interval", config.interval);
			f("StrengthMin", config.strengthMin);
			f("StrengthMax", config.strengthMax);
			i("SprayMode", config.mode);
			f("FlowTime", config.flow);
			i("StreamsMin", config.streamsMin);
			i("StreamsMax", config.streamsMax);
			f("StreamAngle", config.cone);
			i("StreamThickness", config.thickness);
			i("LeakTextureFrom", config.stageFrom);
			i("LeakTextureTo", config.stageTo);
			f("Viscosity", config.viscosity);
			f("Stickiness", config.stickiness);
			f("StickinessWorld", config.stickinessWorld);
			b("SGOSync", config.sgoSync);
			b("Clothed", config.clothed);
			b("Sex", config.sex);
			f("Lifetime", config.lifetime);
			f("FadeTime", config.fade);
			i("MaxParticles", config.maxParticles);
			f("Amount", config.amount);
			f("NippleForward", config.nippleForward);
			f("NippleUp", config.nippleUp);
			f("NippleSide", config.nippleSide);
			i("Merge", config.merge);
			i("FullPercent", config.fullPercent);
			b("Overlay", config.overlay);
			i("OverlayR", config.overlayR);
			i("OverlayG", config.overlayG);
			i("OverlayB", config.overlayB);
			f("OverlayGloss", config.overlayGloss);
			f("OverlaySpecular", config.overlaySpecular);
			f("OverlayBoost", config.overlayBoost);
			b("FillRamp", config.ramp);
			i("FillRampFull", config.rampFull);
			i("FillRampMin", config.rampMin);
			b("FillRampAll", config.rampAll);
			b("VulvaDrip", config.crotch);
			i("VulvaDripStage", config.crotchStage);
			f("VulvaDripInterval", config.crotchInterval);
			f("VulvaDripAmount", config.crotchAmount);
			f("VulvaDripDelay", config.crotchDelay);
			b("DebugLog", config.debugLog);
			std::string text;
			if (ini.Save(text) < 0) {
				return false;
			}
			const std::filesystem::path target(kUserPath);
			const auto                  temporary = std::filesystem::path(target).concat(".tmp");
			{
				std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
				out << text;
				if (!out) {
					return false;
				}
			}
			std::error_code error;
			std::filesystem::rename(temporary, target, error);
			if (error) {
				std::filesystem::remove(temporary, error);
				return false;
			}
			return true;
		} catch (const std::exception& error) {
			logger::warn("could not save OStrands_Milk_User.ini: {}", error.what());
			return false;
		}
	}

	void Apply()
	{
		auto  guard = Lock();
		auto& s = Settings::Get();
		const float thick = config.viscosity / 30.0f;
		s.milk = true;
		s.debugLog = config.debugLog;
		s.milkOpacity = std::max(config.opacity / 100.0f, 0.02f);
		s.milkInterval = config.interval;
		s.milkSpeedMin = SpeedOf(std::min(config.strengthMin, config.strengthMax));
		s.milkSpeedMax = SpeedOf(std::max(config.strengthMin, config.strengthMax));
		s.milkAlwaysStrong = false;
		s.milkMode = config.mode;
		s.milkFlow = config.flow;
		s.milkStreamsMin = config.streamsMin;
		s.milkStreamsMax = config.streamsMax;
		s.milkCone = config.cone * std::numbers::pi_v<float> / 180.0f;
		s.milkThickness = config.thickness / 100.0f;
		s.overlayStageFrom = config.stageFrom;
		s.overlayStageTo = config.stageTo;
		// 0 = water, 30 = honey. A little XSPH viscosity even at 0 keeps the stream together.
		s.milkViscosity = 12.0f + 88.0f * thick;
		// Enough cohesion that drops merge into one puddle; streams stay linked until they really break.
		s.milkMerge = config.merge / 100.0f;
		s.milkCohesion = std::min(20.0f, (6.0f + 12.0f * thick) * (0.4f + 1.2f * s.milkMerge));
		s.milkStretch = 20.0f - 13.0f * thick;  // links lengthen with the flow so trickles stay one strand
		s.milkGlow = config.glow / 100.0f;
		s.milkStickiness = 5.0f * (config.stickiness / 20.0f);
		s.milkStickinessWorld = 5.0f * (config.stickinessWorld / 20.0f);
		s.milkSGOMilking = config.sgoSync;
		s.milkSGOFull = config.sgoSync;
		s.milkClimaxNeedsSGO = config.sgoSync;
		s.milkClimaxMinPercent = 10;
		s.milkSGOFullPercent = config.fullPercent;
		s.overlay = config.overlay && config.sgoSync;
		s.overlayColor = (config.overlayR << 16) | (config.overlayG << 8) | config.overlayB;
		s.overlayGloss = config.overlayGloss;
		s.overlaySpecular = config.overlaySpecular;
		s.overlayBoost = config.overlayBoost;
		{
			// Redraw the leak layer only when its look changed (not on every slider move elsewhere in the menu).
			static std::string lastOverlayLook;
			const std::string  look = std::format("{}|{}|{}|{}|{}", s.overlay, s.overlayColor, s.overlayGloss, s.overlaySpecular, s.overlayBoost);
			if (look != lastOverlayLook) {
				lastOverlayLook = look;
				++s.overlayRevision;
			}
		}
		s.milkNeedsExposed = !config.clothed;
		s.milkCrotch = config.crotch && config.sgoSync;
		s.milkRamp = config.ramp && config.sgoSync;
		s.milkRampFull = config.rampFull;
		s.milkRampMin = config.rampMin / 100.0f;
		s.milkRampAll = config.rampAll;
		s.milkCrotchStage = config.crotchStage;
		s.milkCrotchInterval = config.crotchInterval;
		s.milkCrotchAmount = config.crotchAmount;
		s.milkCrotchDelay = config.crotchDelay;
		s.milkOnClimax = config.sex;
		s.milkLifetime = config.lifetime;
		s.fluidLifetime = config.lifetime;
		s.fluidFade = std::max(config.fade, 0.05f);
		s.fluidMaxParticles = config.maxParticles;
		s.milkAmount = config.amount;
		s.milkNippleOffset = config.nippleForward;
		s.milkNippleUp = config.nippleUp;
		s.milkNippleSide = config.nippleSide;
	}

	void ApplyRecommended()
	{
		{
			auto guard = Lock();
			// Measured cost in this engine: about 2 ms per frame for ~1,000 milk particles and rising steeply after
			// ~1,500. One squeeze at amount 1.5 makes roughly 300 particles per nipple, and a steady leak about 35
			// per second per nipple, so 1,200 particles with a 15 s lifetime holds a full scene without recycling much.
			config.amount = 1.5f;
			config.maxParticles = 1500;
			config.lifetime = 15.0f;
			config.fade = 3.0f;
			config.merge = 70;
			config.interval = std::min(config.interval, 1.0f);
			config.streamsMax = std::min(config.streamsMax, 3);
			config.streamsMin = std::min(config.streamsMin, 2);
			config.Validate();
			Apply();
		}
		Save();
	}

	// ---------------------------------------------------------------- SexLab
	// Each actor's orgasms are read from SexLab's own counter (sslActorAlias.GetOrgasmCount), which
	// SexLab and SLSO both keep, so every separate orgasm is caught without any script patch.
	namespace
	{
		constexpr std::string_view kSexLab = "SexLab.esm";
		constexpr float            kCountPoll = 0.5f;

		struct Seat
		{
			RE::ActorHandle actor;
			int             count{ -1 };
		};

		struct Thread
		{
			RE::FormID                                quest{ 0 };
			std::int32_t                              tid{ -1 };
			std::unordered_map<std::uintptr_t, Seat>  seats;  // keyed by alias
			bool                                      counted{ false };
		};

		struct Signal
		{
			enum class Kind
			{
				kStart,
				kEnd,
				kOrgasm
			} kind;
			RE::FormID   quest{ 0 };
			std::int32_t tid{ -1 };
		};

		struct Count
		{
			RE::FormID      quest{ 0 };
			std::uintptr_t  alias{ 0 };
			RE::ActorHandle actor;
			int             count{ 0 };
		};

		std::mutex          sexLock;
		std::vector<Signal> signals;
		std::vector<Count>  counts;
		std::vector<Thread> threads;
		float               pollWait{ 0.0f };

		bool FromSexLab(const RE::TESForm* a_form)
		{
			const auto* file = a_form ? a_form->GetFile(0) : nullptr;
			return file && file->GetFilename() == kSexLab;
		}

		class SexLabListener : public RE::BSTEventSink<SKSE::ModCallbackEvent>
		{
		public:
			static SexLabListener* Get()
			{
				static SexLabListener listener;
				return &listener;
			}

			RE::BSEventNotifyControl ProcessEvent(const SKSE::ModCallbackEvent* a_event, RE::BSTEventSource<SKSE::ModCallbackEvent>*) override
			{
				if (!a_event || !a_event->sender || a_event->sender->GetFormType() != RE::FormType::Quest || !FromSexLab(a_event->sender)) {
					return RE::BSEventNotifyControl::kContinue;
				}
				const std::string_view name = a_event->eventName.c_str();
				std::optional<Signal::Kind> kind;
				if (name == "AnimationStart" || name == "StageStart") {
					kind = Signal::Kind::kStart;
				} else if (name == "AnimationEnd") {
					kind = Signal::Kind::kEnd;
				} else if (name == "OrgasmStart") {
					kind = Signal::Kind::kOrgasm;
				}
				if (!kind) {
					return RE::BSEventNotifyControl::kContinue;
				}
				const std::string_view text = a_event->strArg.c_str();
				std::int32_t           tid = -1;
				std::from_chars(text.data(), text.data() + text.size(), tid);
				std::scoped_lock guard(sexLock);
				signals.push_back({ *kind, a_event->sender->GetFormID(), tid });
				return RE::BSEventNotifyControl::kContinue;
			}
		};

		class CountCallback : public RE::BSScript::IStackCallbackFunctor
		{
		public:
			CountCallback(RE::FormID a_quest, std::uintptr_t a_alias, RE::ActorHandle a_actor) :
				_quest(a_quest), _alias(a_alias), _actor(a_actor)
			{}

			void operator()(RE::BSScript::Variable a_result) override
			{
				if (!a_result.IsInt()) {
					return;
				}
				std::scoped_lock guard(sexLock);
				counts.push_back({ _quest, _alias, _actor, a_result.GetSInt() });
			}

			void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}

		private:
			RE::FormID      _quest;
			std::uintptr_t  _alias;
			RE::ActorHandle _actor;
		};

		std::vector<std::pair<RE::BGSRefAlias*, RE::Actor*>> SeatsOf(RE::TESQuest* a_quest)
		{
			std::vector<std::pair<RE::BGSRefAlias*, RE::Actor*>> seats;
			for (auto* base : a_quest->aliases) {
				if (!base || base->GetVMTypeID() != RE::BGSRefAlias::VMTYPEID) {
					continue;
				}
				auto* alias = static_cast<RE::BGSRefAlias*>(base);
				if (auto* actor = alias->GetActorReference()) {
					seats.emplace_back(alias, actor);
				}
			}
			return seats;
		}

		void RequestCounts(const Thread& a_thread)
		{
			auto* quest = RE::TESForm::LookupByID<RE::TESQuest>(a_thread.quest);
			auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
			auto* policy = vm ? vm->GetObjectHandlePolicy() : nullptr;
			if (!quest || !policy) {
				return;
			}
			for (const auto& [alias, actor] : SeatsOf(quest)) {
				const auto handle = policy->GetHandleForObject(RE::BGSRefAlias::VMTYPEID, alias);
				if (handle == policy->EmptyHandle()) {
					continue;
				}
				RE::BSTSmartPointer<RE::BSScript::Object> object;
				if (!vm->FindBoundObject(handle, "sslActorAlias", object) || !object) {
					continue;
				}
				RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback(
					new CountCallback(a_thread.quest, reinterpret_cast<std::uintptr_t>(alias), actor->GetHandle()));
				vm->DispatchMethodCall(object, "GetOrgasmCount", RE::MakeFunctionArguments(), callback);
			}
		}

		void Squirt(RE::ActorHandle a_handle, std::int32_t a_tid, const std::vector<RE::ActorHandle>& a_hosts)
		{
			const auto& settings = Settings::Get();
			const auto  actor = a_handle.get();
			if (!actor || !Lactation::ClimaxAllowed(actor.get())) {
				return;
			}
			const int   level = Lactation::MilkPercent(actor.get());
			const float amount = settings.milkAmount * (level >= 0 ? std::clamp(0.5f + level / 100.0f, 0.5f, 1.5f) : 1.0f);
			if (settings.debugLog) {
				logger::info("milk: {} climaxes in SexLab thread {}", actor->GetDisplayFullName(), a_tid);
			}
			const auto  climaxing = a_handle.get();
			const float power = Lactation::SprayPower(climaxing.get(), false);
			FluidManager::Get().Lactate(a_handle, amount * power, FluidManager::kClimax, 0x10000 + std::max(a_tid, 0), a_hosts, power);
			Lactation::OnSpray(a_handle, std::min(Settings::Get().milkFlow, 2.5f));
		}
	}

	void ConnectSexLab()
	{
		auto* data = RE::TESDataHandler::GetSingleton();
		if (!data || !data->LookupModByName(kSexLab)) {
			logger::info("SexLab not found; milk on sex is off");
			return;
		}
		if (auto* source = SKSE::GetModCallbackEventSource()) {
			source->AddEventSink(SexLabListener::Get());
			logger::info("SexLab found; women squirt milk on every orgasm SexLab or SLSO counts");
		}
	}

	void Update(float a_delta)
	{
		std::vector<Signal> batch;
		std::vector<Count>  results;
		{
			std::scoped_lock guard(sexLock);
			batch.swap(signals);
			results.swap(counts);
		}
		auto        guard = Lock();
		const auto& settings = Settings::Get();
		const bool  active = settings.enabled && settings.milk && settings.milkOnClimax;
		const auto  find = [](RE::FormID a_quest) { return std::ranges::find_if(threads, [&](const Thread& t) { return t.quest == a_quest; }); };

		for (const auto& signal : batch) {
			auto thread = find(signal.quest);
			switch (signal.kind) {
			case Signal::Kind::kStart:
				if (thread == threads.end()) {
					threads.push_back({ signal.quest, signal.tid });
					if (settings.debugLog) {
						logger::info("milk: following SexLab thread {}", signal.tid);
					}
				}
				break;
			case Signal::Kind::kEnd:
				if (thread != threads.end()) {
					threads.erase(thread);
				}
				break;
			case Signal::Kind::kOrgasm:
				// Fallback only for threads whose orgasm counters cannot be read.
				if (active && (thread == threads.end() || !thread->counted)) {
					if (auto* quest = RE::TESForm::LookupByID<RE::TESQuest>(signal.quest)) {
						std::vector<RE::ActorHandle> hosts;
						for (const auto& [alias, actor] : SeatsOf(quest)) {
							hosts.push_back(actor->GetHandle());
						}
						for (const auto& host : hosts) {
							Squirt(host, signal.tid, hosts);
						}
					}
				}
				break;
			}
		}

		for (const auto& result : results) {
			auto thread = find(result.quest);
			if (thread == threads.end()) {
				continue;
			}
			thread->counted = true;
			auto& seat = thread->seats[result.alias];
			if (seat.actor != result.actor || seat.count < 0 || result.count < seat.count) {
				seat = { result.actor, result.count };  // first reading or a new actor in this seat: baseline only
				continue;
			}
			if (result.count > seat.count) {
				seat.count = result.count;
				if (active) {
					std::vector<RE::ActorHandle> hosts;
					for (const auto& [key, other] : thread->seats) {
						hosts.push_back(other.actor);
					}
					Squirt(result.actor, thread->tid, hosts);
				}
			}
		}

		pollWait -= a_delta;
		if (pollWait > 0.0f || threads.empty()) {
			return;
		}
		pollWait = kCountPoll;
		for (const auto& thread : threads) {
			RequestCounts(thread);
		}
	}

	void Reset()
	{
		{
			std::scoped_lock guard(sexLock);
			signals.clear();
			counts.clear();
		}
		threads.clear();
		pollWait = 0.0f;
	}

	// ---------------------------------------------------------------- Menu
	namespace
	{
		namespace UI = ImGuiMCP;

		// Tooltip wrapped to a readable width, so long lines never run off the screen.
		void Hint(const char* a_text)
		{
			if (a_text && *a_text && UI::IsItemHovered(UI::ImGuiHoveredFlags_AllowWhenDisabled)) {
				if (UI::BeginTooltip()) {
					UI::PushTextWrapPos(UI::GetFontSize() * 32.0f);
					UI::TextUnformatted(a_text);
					UI::PopTextWrapPos();
					UI::EndTooltip();
				}
			}
		}

		void Commit()
		{
			auto guard = Lock();
			config.Validate();
			Apply();
		}

		void Persist()
		{
			Commit();
			if (!Save()) {
				logger::warn("could not write OStrands_Milk_User.ini");
			}
		}

		template <class T>
		void SliderRow(const char* a_name, const char* a_tooltip, T Config::* a_field, float a_min, float a_max, float a_step)
		{
			float value = 0.0f;
			{
				auto guard = Lock();
				value = static_cast<float>(config.*a_field);
			}
			const int         digits = a_step >= 1.0f ? 0 : a_step >= 0.1f ? 1 : 2;
			const std::string format = std::format("%.{}f", digits);
			if (UI::SliderFloat(a_name, &value, a_min, a_max, format.c_str()) && std::isfinite(value)) {
				value = std::clamp(std::round(value / a_step) * a_step, a_min, a_max);
				auto guard = Lock();
				if constexpr (std::is_integral_v<T>) {
					config.*a_field = static_cast<T>(std::lround(value));
				} else {
					config.*a_field = static_cast<T>(value);
				}
				Commit();
			}
			if (UI::IsItemDeactivatedAfterEdit()) {
				Persist();
			}
			Hint(a_tooltip);
		}

		// SGO milk % options are shown against the real maximum, which is twice SGO's own "max" setting
		// (Lactation::kSGOTrueMax). Only the display changes: the stored value (and the INI) keep the old unit, so saved
		// settings and the behaviour stay exactly as they were. 0.5 steps on screen = whole steps of the stored value.
		void SGOPercentRow(const char* a_name, const char* a_tooltip, int Config::* a_field, int a_min, int a_max)
		{
			constexpr float scale = 1.0f / Lactation::kSGOTrueMax;
			int             stored = 0;
			{
				auto guard = Lock();
				stored = config.*a_field;
			}
			float       value = static_cast<float>(stored) * scale;
			const char* format = stored % 2 == 0 ? "%.0f" : "%.1f";
			if (UI::SliderFloat(a_name, &value, static_cast<float>(a_min) * scale, static_cast<float>(a_max) * scale, format) && std::isfinite(value)) {
				const int next = std::clamp(static_cast<int>(std::lround(value / scale)), a_min, a_max);
				auto      guard = Lock();
				config.*a_field = next;
				Commit();
			}
			if (UI::IsItemDeactivatedAfterEdit()) {
				Persist();
			}
			Hint(a_tooltip);
		}

		void YesNo(const char* a_name, const char* a_tooltip, bool Config::* a_field)
		{
			bool value = false;
			{
				auto guard = Lock();
				value = config.*a_field;
			}
			int               index = value ? 0 : 1;
			const char* const labels[]{ "예", "아니오" };
			if (UI::Combo(a_name, &index, labels, 2)) {
				{
					auto guard = Lock();
					config.*a_field = index == 0;
				}
				Persist();
			}
			Hint(a_tooltip);
		}

		void __stdcall DrawMilk()
		{
			UI::SeparatorText("모유 외형");
			SliderRow("모유 불투명도", "100에 가까울수록 불투명한 흰색이 됩니다.", &Config::opacity, 0.0f, 100.0f, 1.0f);
			SliderRow("모유 양", "모든 모유 효과의 양 배율입니다.", &Config::amount, 0.0f, 6.0f, 0.05f);
			SliderRow("조명 보정 (흰색 유지)", "주변 조명과 상관없이 모유가 흰색으로 보이도록 스스로 밝히는 정도입니다. 어두운 곳이나 횃불 아래에서 갈색·회색으로 보이면 올리세요. 0이면 조명만 받습니다.",
				&Config::glow, 0.0f, 100.0f, 1.0f);

			UI::SeparatorText("분출");
			SliderRow("모유 분출 빈도 (초)",
				"분출과 분출 사이 쉬는 시간입니다. 0에 가까울수록 더 자주 분출합니다. 평상시 흘러내림은 최대 4초 동안 이어진 뒤 이 시간만큼 멈췄다가 다시 나옵니다. 단위: 초.",
				&Config::interval, 0.0f, 60.0f, 0.1f);
			SliderRow("최대 분출 세기", "세차게 분출할 때의 세기입니다. 20은 이전 최댓값과 같은 세찬 분출이고, 60까지 올리면 그 3배 속도로 멀리 뿜어져 나갑니다. 세기가 높을수록 같은 시간에 나오는 모유 입자도 많아집니다.", &Config::strengthMax, 0.0f, 60.0f, 0.1f);
			SliderRow("최소 분출 세기", "평상시 흘러나올 때의 세기입니다. 0이면 가슴을 타고 줄줄 흘러내립니다. (0~60)", &Config::strengthMin, 0.0f, 60.0f, 0.1f);
			SliderRow("모유가 나오는 시간 (초)",
				"한 번 나올 때 모유가 이어지는 시간입니다. 이 시간이 끝나면 '모유 분출 빈도'만큼 쉬었다가 다시 나옵니다. 예: 4초 / 60초 → 4초 동안 나오고 60초 쉬기를 반복.",
				&Config::flow, 0.5f, 30.0f, 0.1f);
			{
				int mode = 1;
				{
					auto guard = Lock();
					mode = config.mode;
				}
				const char* const labels[]{ "질질샘", "조건부 젖소 련", "풍부한 젖소 련", "넘쳐흐르는 젖소 련", "발정난 젖소 련" };
				if (UI::Combo("분출 방식", &mode, labels, 5)) {
					{
						auto guard = Lock();
						config.mode = mode;
					}
					Persist();
				}
				Hint("[질질샘]\n"
					 "나오는 시간 내내 최소 세기·최소 가닥으로만 흘러나옵니다.\n"
					 "성관계나 SGO 젖 짜기에서도 최소 세기입니다.\n"
					 "\n"
					 "[조건부 젖소 련]\n"
					 "평상시에는 최소 세기로 흘러나옵니다.\n"
					 "성관계·SGO 젖 짜기에서는 최대 세기·최대 가닥으로 뿜어져 나옵니다.\n"
					 "\n"
					 "[풍부한 젖소 련]\n"
					 "나오는 시간 동안 최대 세기로 시작해 점점 최소 세기로 약해집니다. (가닥 수도 함께 줄어듦)\n"
					 "성관계·젖 짜기는 최대 세기입니다.\n"
					 "\n"
					 "[넘쳐흐르는 젖소 련]\n"
					 "풍부한 젖소 련처럼 약해지다가, 나오는 시간이 끝난 뒤 쉬는 동안에도 최소 세기로 조금씩 계속 새어 나옵니다.\n"
					 "성관계·젖 짜기는 최대 세기입니다.\n"
					 "\n"
					 "[발정난 젖소 련]\n"
					 "나오는 시간 동안 세기는 최대로 고정되고, 가닥 수만 최대에서 최소로 점점 줄어듭니다.\n"
					 "쉬는 동안에도 최소 가닥·최대 세기로 계속 뿜다가, 분출 빈도 시간이 되면 다시 최대 가닥으로 돌아가는 반복입니다.\n"
					 "성관계·젖 짜기는 최대 세기입니다.\n"
					 "\n"
					 "※ 발정난 젖소 련은 최대 세기로 계속 뿜기 때문에 입자가 가장 많이 나옵니다. 끊기면 가닥 수나 최대 입자 수를 낮추세요.");
				static const char* const details[]{
					"질질샘 - 언제나 최소 세기로 흘러나옵니다.",
					"조건부 젖소 련 - 평상시 최소 세기, 성관계·젖 짜기 때만 최대 세기.",
					"풍부한 젖소 련 - 최대 세기에서 시작해 점점 약해집니다.",
					"넘쳐흐르는 젖소 련 - 점점 약해진 뒤 쉬는 동안에도 조금씩 새어 나옵니다.",
					"발정난 젖소 련 - 최대 세기 유지, 가닥 수만 줄었다가 주기마다 다시 최대 가닥.",
				};
				UI::TextWrapped("%s", details[std::clamp(mode, 0, 4)]);
			}

			UI::SeparatorText("가닥");
			SliderRow("최대 가닥 수", "최대 분출 세기일 때 젖꼭지 한 점에서 퍼져 나가는 모유 줄기 수입니다. 세기가 약해지면 최소 가닥 수까지 차례로 줄어듭니다.", &Config::streamsMax,
				1.0f, 12.0f, 1.0f);
			SliderRow("최소 가닥 수", "최소 분출 세기일 때의 줄기 수입니다.", &Config::streamsMin, 1.0f, 12.0f, 1.0f);
			SliderRow("가닥 퍼짐 각도 (도)", "여러 가닥이 샤워기처럼 퍼지는 각도입니다. 0이면 한 방향으로 모입니다.", &Config::cone, 0.0f, 45.0f, 0.5f);
			SliderRow("가닥 굵기 (%)", "공중으로 뿜어지는 줄기의 굵기입니다. 바닥에 고이는 모유의 퍼짐에는 영향을 주지 않습니다.", &Config::thickness, 20.0f, 150.0f, 1.0f);
			UI::TextWrapped("%s", "가닥이 많을수록 입자도 그만큼 많아집니다. 끊김이 느껴지면 최대 가닥 수를 낮추거나 '권장값 적용'을 누르세요.");

			UI::SeparatorText("물성");
			SliderRow("모유 점성", "0은 물처럼 묽고, 30은 꿀처럼 걸쭉합니다.", &Config::viscosity, 0.0f, 30.0f, 0.1f);
			SliderRow("피부 점착성", "몸(피부)에 닿았을 때 달라붙는 정도입니다. 0은 물처럼 흘러내리고, 20은 꿀처럼 달라붙어 잘 흘러내리지 않습니다.", &Config::stickiness, 0.0f, 20.0f, 0.1f);
			SliderRow("사물 점착성", "사물에 닿았을 때 달라붙는 정도입니다. 0은 표면을 타고 빠르게 흘러내리고, 20은 묻은 자리에 오래 머뭅니다.", &Config::stickinessWorld, 0.0f, 20.0f, 0.1f);
			SliderRow("모유 뭉침 정도", "서로 닿은 모유가 하나로 합쳐지는 정도입니다. 0은 방울들이 따로따로, 100은 닿는 순간 한 덩어리처럼 합쳐집니다.", &Config::merge, 0.0f,
				100.0f, 1.0f);

			UI::SeparatorText("조건");
			UI::TextWrapped("%s",
				"SGO 모유량 단위 안내\n"
				"SGO MCM의 '최대 모유량'(예: 4)은 실제로 쌓이는 최대치의 절반입니다. 실제로는 그 설정값의 2배(예: 8)까지 찹니다.\n"
				"그래서 아래의 SGO 관련 %와 오버레이의 누출 단계 기준은 모두 이 실제 최대치(설정값 × 2)를 100%로 표시합니다.\n"
				"예: SGO 최대 모유량 4 → 모유 8 = 100%, 모유 4 = 50%.");
			UI::Spacing();
			YesNo("SGO 모드 동기화",
				"예: Soul Gem Oven과 연동합니다. 모유가 가득 차면 흘러나오고, 젖 짜기 모션에서 분출하며, 성관계 시에는 SGO 모유가 있을 때만 분출합니다.",
				&Config::sgoSync);
			SGOPercentRow("SGO 가득 참 기준 (%)",
				"SGO 모유량이 이 값 이상이면 저절로 흘러내립니다.\n100 = 실제 최대치(SGO 최대 모유량 설정값의 2배)에 가득 찼을 때입니다.",
				&Config::fullPercent, 10, 300);
			YesNo("모유량에 따른 분출 강화",
				"예: 'SGO 가득 참 기준'에서 막 흘러나오기 시작할 때는 약하고 적게 나오다가, 모유가 더 찰수록 분출 세기와 양이 점점 늘어나\n"
				"'최대 분출 기준'에 도달하면 설정한 최대 세기·양(100%)으로 나옵니다.\n"
				"세기와 양만 바뀌며, 점성·뭉침·피부 흐름·바닥 웅덩이의 동작은 그대로입니다.",
				&Config::ramp);
			SGOPercentRow("최대 분출 기준 (SGO %)",
				"SGO 모유량이 이 값 이상이면 100%로 분출합니다.\n보통 '누출 최대 단계 기준'과 같은 값(실제로 가득 찼을 때 = 100)을 넣습니다.",
				&Config::rampFull, 10, 400);
			SliderRow("시작 시 분출 비율 (%)",
				"'SGO 가득 참 기준'에서 처음 흘러나올 때의 세기·양 비율입니다. 그 사이는 모유량에 비례해 고르게 늘어납니다.\n"
				"예: 가득 참 기준 50, 최대 분출 기준 100, 시작 비율 10 → 50%일 때 10%, 75%일 때 55%, 100%일 때 100%.",
				&Config::rampMin, 0.0f, 100.0f, 1.0f);
			YesNo("착유·성관계 분출에도 적용",
				"예: SGO 착유와 성관계 절정 분출도 모유량에 따라 약해집니다. 아니오: 저절로 흘러나오는 분출에만 적용하고, 착유·절정은 항상 100%입니다.",
				&Config::rampAll);
			{
				const std::string text = Lactation::Readout();
				UI::TextWrapped("%s", text.empty() ? "현재 SGO 모유량: 측정 대기 중" : text.c_str());
			}
			YesNo("옷을 입은 상태에서 모유 분출", "예: 옷 위로도 모유가 흘러나옵니다. 아니오: 상의를 벗고 있어야 보입니다 (SGO 젖 짜기 제외).", &Config::clothed);
			YesNo("성관계 시 모유 분출", "예: SexLab 장면에서 여성이 절정하면 모유가 분출합니다.", &Config::sex);
			UI::TextWrapped("%s", Lactation::SGOInstalled() ? "Soul Gem Oven 4 (Sgo4IF.esp)가 감지되었습니다." : "Sgo4IF.esp가 없습니다. SGO 동기화는 동작하지 않습니다.");

			UI::SeparatorText("유지 시간과 최적화");
			SliderRow("모유 유지 시간 (초)",
				"모유 입자 하나가 생긴 뒤 사라지기 시작할 때까지의 시간입니다. 입자마다 따로(±15% 무작위) 계산되므로 먼저 나온 모유부터 차례로 사라집니다.\n"
				"바닥에 고인 웅덩이도 그 웅덩이에 합쳐진 모유 중 가장 늦게 사라질 입자의 남은 시간만큼 유지됩니다.\n"
				"길수록 바닥에 오래 남지만, 그만큼 동시에 존재하는 입자가 많아져 아래 '최대 입자 수'에 더 빨리 닿습니다.",
				&Config::lifetime, 0.0f, 60.0f, 0.5f);
			SliderRow("소멸 전환 시간 (초)",
				"유지 시간이 끝난 입자가 크기를 줄이며 완전히 사라지기까지의 시간입니다. 0에 가까우면 갑자기 사라지고, 길면 서서히 말라 없어지듯 보입니다.\n"
				"이 시간 동안에도 입자는 계산되므로 지나치게 길면 부담이 늘어납니다.",
				&Config::fade, 0.0f, 10.0f, 0.1f);
			SliderRow("최대 입자 수 (모유 영역당)",
				"한 모유 영역(한 사람 주변의 모유 묶음)에서 동시에 물리 계산되는 입자 수의 상한입니다.\n"
				"상한에 닿으면 새 모유를 막지 않고, 가장 오래된 입자부터 0.25초 안에 빠르게 지워 자리를 비웁니다. 그래서 바닥에 남는 양이 일정 이상 늘지 않습니다.\n"
				"바닥에 내려앉아 멈춘 모유는 입자에서 빠져 '고정 웅덩이'(약 3.5cm 칸, 영역당 최대 260칸 / 그리는 모양 최대 120개)로 합쳐지므로 이 상한에 포함되지 않습니다.",
				&Config::maxParticles, 0.0f, 3000.0f, 10.0f);
			UI::TextWrapped("%s",
				"최적화 팁\n"
				"- 프레임을 가장 크게 좌우하는 것은 '공중과 피부 위에 동시에 떠 있는 입자 수'입니다. 바닥 웅덩이는 거의 비용이 없습니다.\n"
				"- 동시 입자 수 ≈ 초당 나오는 양(모유 양 × 가닥 수 × 세기) × 유지 시간. 셋 중 하나만 줄여도 효과가 큽니다.\n"
				"- 끊김이 있으면: 최대 입자 수 1000~1500, 유지 시간 10~15초, 최대 가닥 수 2~3 순서로 낮춰 보세요.\n"
				"- 바닥에 오래 남기고 싶으면 유지 시간을 늘리되 최대 입자 수는 그대로 두세요. 웅덩이는 상한과 별개로 남습니다.\n"
				"- 여러 NPC가 동시에 모유를 흘리면 영역이 사람마다 따로 생기므로, 상한 × 사람 수만큼 부담이 늘어납니다.");

			UI::SeparatorText("유즙 누출 오버레이 (흘러내림 텍스쳐)");
			YesNo("누출 오버레이 사용",
				"SGO의 모유량에 맞춰 피부에 흘러내림 자국(1~8단계 텍스쳐)을 그립니다.\n"
				"단계는 '실제 최대 모유량(SGO 설정값의 2배) 대비 현재 모유량'으로 정해지며, 분출 시작 기준('SGO 가득 참 기준')과는 무관합니다.\n"
				"이 애드온이 SGO와 별도의 오버레이 층에 직접 그리므로 SGO MCM의 '모유 오버레이 퍼센트' 설정과도 상관없이 '누출 1단계 기준'부터 보입니다.\n"
				"SGO 자체의 흘러내림 층은 겹쳐 보이지 않도록 투명하게 처리합니다. SGO 동기화가 '예'일 때만 동작합니다.",
				&Config::overlay);
			SliderRow("색상 - R", "오버레이 색상의 빨강 채널입니다. R·G·B 모두 255면 순백색입니다.", &Config::overlayR, 0.0f, 255.0f, 1.0f);
			SliderRow("색상 - G", "오버레이 색상의 초록 채널입니다.", &Config::overlayG, 0.0f, 255.0f, 1.0f);
			SliderRow("색상 - B", "오버레이 색상의 파랑 채널입니다. 살짝 낮추면 크림빛 유백색이 됩니다.", &Config::overlayB, 0.0f, 255.0f, 1.0f);
			SliderRow("광택도 (Glossiness)", "반사 하이라이트의 날카로움입니다. 높을수록 하이라이트가 작고 선명해져 갓 흘러내린 젖은 표면처럼 보입니다.",
				&Config::overlayGloss, 0.0f, 100.0f, 1.0f);
			SliderRow("반사 강도 (Specular)", "반사 하이라이트의 밝기입니다. 높을수록 빛을 받을 때 번들거림이 강해집니다.", &Config::overlaySpecular, 0.0f, 10.0f, 0.1f);
			SliderRow("불투명도 배율", "흘러내림 자국의 진하기입니다. 실제 불투명도 = 배율 × 60% (최대 100%). 1.0이면 60%, 1.7 이상이면 100%(가장 진함)입니다.",
				&Config::overlayBoost, 1.0f, 4.0f, 0.05f);
			SGOPercentRow("누출 1단계 기준 (최대량 대비 %)",
				"실제 최대 모유량 대비 현재 모유량이 이 값에 도달하면 1단계(유두 주변의 소량 누출) 자국이 나타납니다. 이보다 적으면 자국이 없습니다.\n"
				"이 값과 '누출 최대 단계 기준' 사이를 1~7단계로 고르게 나누고, 최대 단계 기준 이상이면 8단계입니다.",
				&Config::stageFrom, 1, 300);
			SGOPercentRow("누출 최대 단계 기준 (최대량 대비 %)",
				"실제 최대 모유량 대비 현재 모유량이 이 값 이상이면 8단계(가슴·복부를 지나 허벅지와 질 입구까지 흘러내린 다량 누출) 자국이 됩니다.\n"
				"100 = 실제 최대치(SGO 최대 모유량 설정값의 2배)에 가득 찼을 때입니다.", &Config::stageTo, 1, 300);
			UI::TextWrapped("%s",
				"참고\n"
				"- 위의 '현재 SGO 모유량' 줄에 지금의 '최대량 대비 %'와 단계가 표시됩니다. 예: 1단계 기준 5, 최대 단계 기준 50 → 5%부터 자국이 생기고 약 6.4%마다 한 단계씩 올라 50%에서 8단계.\n"
				"- 가득 찼을 때 '현재 SGO 모유량' 줄에 보이는 숫자를 '누출 최대 단계 기준'에 넣으면 가득 찬 순간 8단계가 됩니다.\n"
				"- 젖을 짜거나 모유가 줄면 단계도 그만큼 내려갑니다. 자국은 1초 간격으로 확인해서 단계가 바뀔 때만 다시 그립니다.");

			UI::SeparatorText("회음부 유즙 낙하");
			YesNo("회음부 유즙 낙하",
				"누출 오버레이가 지정한 단계 이상일 때, 유두에서 모유가 나오기 시작하면 복부를 타고 흘러내린 모유가 잠시 뒤 질 입구에서 방울져 떨어집니다.\n"
				"낙하는 분출(평상시 흘러내림, SGO 착유, 성관계 절정)과 함께 시작해서 그 분출이 끝나면 멈춥니다.\n"
				"SGO 동기화가 '예'일 때만 동작하며, '옷을 입은 상태에서 분출'이 '아니오'면 하의를 벗은 상태에서만 보입니다.",
				&Config::crotch);
			SliderRow("시작 단계", "누출 오버레이가 이 단계 이상일 때만 낙하합니다. 8 = 최대 단계에서만.", &Config::crotchStage, 1.0f, 8.0f, 1.0f);
			SliderRow("분출 후 지연 (초)", "유두에서 분출이 시작된 뒤, 흘러내린 모유가 질 입구에 도달해 첫 방울이 떨어지기까지의 시간입니다.", &Config::crotchDelay, 0.0f, 10.0f, 0.1f);
			SliderRow("낙하 간격 (초)", "분출이 이어지는 동안 방울이 떨어지는 평균 간격입니다. 실제 간격은 ±30% 범위에서 무작위로 바뀝니다.", &Config::crotchInterval, 1.0f, 30.0f, 0.5f);
			SliderRow("낙하량", "한 번에 떨어지는 방울의 양입니다. '모유 양' 배율이 함께 곱해집니다.", &Config::crotchAmount, 0.0f, 3.0f, 0.05f);

			UI::SeparatorText("모유 위치 보정");
			SliderRow("앞뒤 위치", "양수면 젖꼭지 바깥(앞)으로, 음수면 가슴 안쪽으로 이동합니다. 단위: 게임 거리.", &Config::nippleForward, -2.0f, 4.0f, 0.05f);
			SliderRow("상하 위치", "양수면 머리 방향으로, 음수면 발 방향으로 이동합니다.", &Config::nippleUp, -2.0f, 4.0f, 0.05f);
			SliderRow("좌우 위치", "양수면 몸 바깥쪽으로, 음수면 가슴 사이 방향으로 이동합니다.", &Config::nippleSide, -2.0f, 4.0f, 0.05f);
			if (UI::Button("위치 보정 초기화")) {
				{
					auto guard = Lock();
					config.nippleForward = config.nippleUp = config.nippleSide = 0.0f;
				}
				Persist();
			}

			UI::SeparatorText("성능");
			if (UI::Button("권장값 적용 (성능 최적)")) {
				ApplyRecommended();
			}
			Hint("모유 양 1.5, 최대 파티클 1500, 소멸 시간 15초, 완전 소멸 3초, 뭉침 70, 최대 가닥 3, 최소 가닥 2 이하로 맞춥니다. 모유 처리 비용이 프레임당 약 2~3ms로 유지됩니다.");

			UI::SeparatorText("기타");
			{
				bool value = false;
				{
					auto guard = Lock();
					value = config.debugLog;
				}
				if (UI::Checkbox("디버그 로그 (OStrands_Milk.log)", &value)) {
					{
						auto guard = Lock();
						config.debugLog = value;
					}
					Persist();
				}
			}
			UI::TextWrapped("%s", "변경 사항은 Data\\SKSE\\Plugins\\OStrands_Milk_User.ini에만 저장됩니다. OStrands 본편 설정 파일은 읽기만 하고 수정하지 않습니다.");
		}
	}

	void RegisterMenu()
	{
		if (!SKSEMenuFramework::IsInstalled() || !GetMenuFrameworkModule()) {
			logger::info("SKSE Menu Framework not found; change milk settings in OStrands_Milk_User.ini");
			return;
		}
		SKSEMenuFramework::SetSection("OStrands");
		SKSEMenuFramework::AddSectionItem("모유", DrawMilk);
		logger::info("milk page added to SKSE Menu Framework under OStrands");
	}
}
