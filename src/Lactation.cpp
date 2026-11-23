#include "Lactation.h"

#include "Anatomy.h"
#include "FluidManager.h"
#include "Revealing.h"
#include "Settings.h"

#include <unordered_map>

namespace OStrands::Lactation
{
	namespace
	{
		constexpr std::string_view kSGOPlugin = "Sgo4IF.esp";
		// SGO's FactionMilkPercent rank is only refreshed when milk is added, never when it is milked or drains,
		// so it goes stale. Ask SGO's database script directly instead: ActorMilkTotalPercent() = amount / max.
		constexpr std::string_view kSGOBasePlugin = "dse-soulgem-oven.esp";
		constexpr RE::FormID       kDatabaseQuest = 0x00182A;  // dse_sgo_QuestDatabase
		constexpr const char*      kDatabaseScript = "dse_sgo_QuestDatabase_Main";
		constexpr const char*      kMilkFunction = "ActorMilkTotalPercent";
		constexpr RE::FormID       kProduceMilkFaction = 0x0048C3;  // dse_sgo_FactionProduceMilk
		// SGO's overlay helpers (slot bookkeeping shared with SGO's own overlays): our own leak layer goes through them.
		constexpr RE::FormID       kUtilQuest = 0x000D64;  // dse_sgo_QuestUtil
		constexpr const char*      kUtilScript = "dse_sgo_QuestUtil_Main";
		constexpr const char*      kOwnOverlay = "OStrandsMilk";
		constexpr const char*      kOwnOverlayKey = "SGO4.Actor.Overlay.OStrandsMilk";
		constexpr float            kOwnOverlayVerify = 5.0f;  // s between checks that our layer is still there
		constexpr std::size_t      kMaxLevelRequests = 6;           // per poll, to keep SGO's script queue light
		constexpr auto             kLevelFresh = std::chrono::seconds(6);
		constexpr float            kPoll = 1.0f;
		constexpr float            kMilkingWindow = 12.0f;
		constexpr std::string_view kSpawnMilk = "sgo4.actorspawnmilk";
		constexpr std::string_view kActorDone = "sgo4.actordone";
		constexpr std::string_view kSpawnSemen = "sgo4.actorspawnsemen";
		constexpr std::string_view kSpawnGem = "sgo4.actorspawngem";
		// SGO's event preloader animation fires SpawnSemen/SpawnGem/SpawnMilk within a few
		// hundredths of a second; a real milking squeeze fires SpawnMilk alone.
		constexpr auto             kPreloadWindow = std::chrono::milliseconds(250);

		using Clock = std::chrono::steady_clock;

		enum class Signal
		{
			kSpawnMilk,
			kDone,
			kOther
		};

		struct Pending
		{
			RE::ActorHandle   actor;
			Signal            signal{ Signal::kSpawnMilk };
			Clock::time_point at;
		};

		struct Milking
		{
			RE::ActorHandle actor;
			float           idle{ 0.0f };
			float           dribble{ 0.0f };
		};

		RE::TESQuest*                            database{ nullptr };
		RE::TESQuest*                            utilQuest{ nullptr };
		RE::TESFaction*                          produceMilk{ nullptr };
		std::mutex                               readoutLock;
		std::string                              readout;
		std::size_t                              roundRobin{ 0 };

		struct Level
		{
			float                                 ratio{ 0.0f };
			std::chrono::steady_clock::time_point at;
		};
		std::mutex                               levelLock;
		std::unordered_map<std::uint32_t, Level> levels;

		class MilkLevelCallback : public RE::BSScript::IStackCallbackFunctor
		{
		public:
			explicit MilkLevelCallback(std::uint32_t a_actor) :
				_actor(a_actor)
			{}

			void operator()(RE::BSScript::Variable a_result) override
			{
				float ratio = 0.0f;
				if (a_result.IsFloat()) {
					ratio = a_result.GetFloat();
				}
				if (!std::isfinite(ratio) || ratio < 0.0f) {
					ratio = 0.0f;
				}
				std::scoped_lock guard(levelLock);
				levels[_actor] = { ratio, std::chrono::steady_clock::now() };
			}

			void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}

		private:
			std::uint32_t _actor;
		};

		// ------------------------------------------------ SGO "MilkLeak" overlay tuning
		// SGO paints the leak overlay with tint 1 and its own alpha every time it updates an actor.
		// We read the overlay node SGO chose, and when SGO has just (re)applied it, replace tint,
		// alpha, glossiness and specular with the user's values through NiOverride.
		constexpr const char* kOverlayKey = "SGO4.Actor.Overlay.MilkLeak";
		constexpr int         kTint = 7, kAlpha = 8, kGloss = 2, kSpecular = 3, kTexture = 9;

		struct OverlayStep
		{
			enum class Kind
			{
				kNode,
				kTint,
				kAlpha
			} kind;
			RE::ActorHandle actor;
			std::string     node;
			int             tint{ 0 };
			float           alpha{ 0.0f };
		};

		std::mutex                                overlayLock;
		std::vector<OverlayStep>                  overlaySteps;
		std::unordered_map<std::uint32_t, int>    overlayApplied;  // actor -> settings revision applied

		class OverlayCallback : public RE::BSScript::IStackCallbackFunctor
		{
		public:
			OverlayCallback(OverlayStep::Kind a_kind, RE::ActorHandle a_actor, std::string a_node, int a_tint = 0) :
				_kind(a_kind), _actor(a_actor), _node(std::move(a_node)), _tint(a_tint)
			{}

			void operator()(RE::BSScript::Variable a_result) override
			{
				OverlayStep step{ _kind, _actor, _node, _tint };
				switch (_kind) {
				case OverlayStep::Kind::kNode:
					if (!a_result.IsString()) {
						return;
					}
					step.node = std::string(a_result.GetString());
					if (step.node.empty()) {
						return;
					}
					break;
				case OverlayStep::Kind::kTint:
					if (!a_result.IsInt()) {
						return;
					}
					step.tint = a_result.GetSInt();
					break;
				case OverlayStep::Kind::kAlpha:
					if (!a_result.IsFloat()) {
						return;
					}
					step.alpha = a_result.GetFloat();
					break;
				}
				std::scoped_lock guard(overlayLock);
				overlaySteps.push_back(std::move(step));
			}

			void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}

		private:
			OverlayStep::Kind _kind;
			RE::ActorHandle   _actor;
			std::string       _node;
			int               _tint;
		};

		void Static(const char* a_class, const char* a_function, RE::BSScript::IFunctionArguments* a_arguments,
			RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> a_callback = {})
		{
			auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
			if (!vm) {
				delete a_arguments;
				return;
			}
			vm->DispatchStaticCall(a_class, a_function, a_arguments, a_callback);
		}

		void RequestOverlayNode(RE::Actor* a_actor)
		{
			RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback(new OverlayCallback(OverlayStep::Kind::kNode, a_actor->GetHandle(), {}));
			Static("StorageUtil", "GetStringValue",
				RE::MakeFunctionArguments(static_cast<RE::TESForm*>(a_actor), RE::BSFixedString(kOverlayKey), RE::BSFixedString("")), callback);
		}

		std::unordered_map<std::uint32_t, int> lastStage;

		int StageFor(RE::Actor* a_actor)
		{
			const auto& settings = Settings::Get();
			const int   level = MilkPercent(a_actor);
			const auto  key = a_actor->GetHandle().native_handle();
			if (level < 0) {
				const auto known = lastStage.find(key);
				return known != lastStage.end() ? known->second : 0;  // level not measured yet: keep what is shown
			}
			const auto remember = [&](int a_stage) {
				lastStage[key] = a_stage;
				return a_stage;
			};
			if (level < settings.overlayStageFrom) {
				return remember(0);  // below the first stage: no leak texture at all
			}
			if (level >= settings.overlayStageTo || settings.overlayStageTo <= settings.overlayStageFrom) {
				return remember(8);
			}
			const float t = static_cast<float>(level - settings.overlayStageFrom) / static_cast<float>(settings.overlayStageTo - settings.overlayStageFrom);
			return remember(std::clamp(1 + static_cast<int>(t * 7.0f), 1, 7));
		}

		void ProcessOverlay(const OverlayStep& a_step)
		{
			const auto& settings = Settings::Get();
			const auto  actor = a_step.actor.get();
			if (!actor || !settings.overlay || !actor->Get3D(false)) {
				return;
			}
			auto*                   ref = static_cast<RE::TESObjectREFR*>(actor.get());
			const bool              female = IsFemale(actor.get());
			const RE::BSFixedString node(a_step.node);
			const auto              key = a_step.actor.native_handle();
			const int               stage = StageFor(actor.get());
			switch (a_step.kind) {
			case OverlayStep::Kind::kNode:
				{
					RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback(
						new OverlayCallback(OverlayStep::Kind::kTint, a_step.actor, a_step.node));
					Static("NiOverride", "GetNodeOverrideInt", RE::MakeFunctionArguments(std::move(ref), bool(female), RE::BSFixedString(node), int(kTint), int(-1)),
						callback);
				}
				break;
			case OverlayStep::Kind::kTint:
				{
					// SGO always paints tint 1 (almost black, a hint of blue). Seeing it means SGO just repainted the
					// overlay, so ours must go back on at once, and persistently, so NiOverride never restores SGO's.
					const bool fresh = a_step.tint == 1;
					const auto applied = overlayApplied.find(key);
					// with our own layer SGO's is only kept invisible, which does not depend on the stage
					const int  stamp = settings.overlayRevision * 16 + (utilQuest ? 0 : stage);
					if (settings.debugLog && !utilQuest && applied != overlayApplied.end() && applied->second % 16 != stage) {
						logger::info("SGO leak overlay on {}: milk {}% -> texture stage {} (was {})", actor->GetDisplayFullName(), MilkPercent(actor.get()), stage,
							applied->second % 16);
					}
					if (!fresh && applied != overlayApplied.end() && applied->second == stamp) {
						return;
					}
					RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback(
						new OverlayCallback(OverlayStep::Kind::kAlpha, a_step.actor, a_step.node, fresh ? 1 : 0));
					Static("NiOverride", "GetNodeOverrideFloat", RE::MakeFunctionArguments(std::move(ref), bool(female), RE::BSFixedString(node), int(kAlpha), int(-1)),
						callback);
				}
				break;
			case OverlayStep::Kind::kAlpha:
				{
					const bool        fresh = a_step.tint != 0;
					// With our own leak layer, SGO's layer is only kept invisible so the two never stack.
					const bool        own = utilQuest != nullptr;
					const float       alpha = own ? 0.0f :
					                          fresh ? std::clamp(a_step.alpha * settings.overlayBoost, 0.0f, 1.0f) : std::clamp(a_step.alpha, 0.0f, 1.0f);
					const std::string texture = std::format("textures\\OStrandsMilk\\MilkLeak_{}.dds", std::max(stage, 1));
					Static("NiOverride", "AddNodeOverrideString",
						RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kTexture), int(0), RE::BSFixedString(texture), bool(true)));
					Static("NiOverride", "AddNodeOverrideInt",
						RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kTint), int(-1), int(settings.overlayColor), bool(true)));
					Static("NiOverride", "AddNodeOverrideFloat",
						RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kAlpha), int(-1), float(alpha), bool(true)));
					Static("NiOverride", "AddNodeOverrideFloat",
						RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kGloss), int(-1), float(settings.overlayGloss), bool(true)));
					Static("NiOverride", "AddNodeOverrideFloat",
						RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kSpecular), int(-1), float(settings.overlaySpecular), bool(true)));
					Static("NiOverride", "ApplyNodeOverrides", RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref)));
					overlayApplied[key] = settings.overlayRevision * 16 + (own ? 0 : stage);
					if (settings.debugLog) {
						logger::info("SGO leak overlay on {} ({}): {}{}", actor->GetDisplayFullName(), a_step.node,
							own ? std::string("hidden (our own layer shows the leak)") : std::format("stage {}, tint #{:06X}, alpha {:.2f}", stage, settings.overlayColor, alpha),
							fresh ? " (SGO repainted)" : "");
					}
				}
				break;
			}
		}

		void RequestLevel(RE::Actor* a_actor)
		{
			auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
			auto* policy = vm ? vm->GetObjectHandlePolicy() : nullptr;
			if (!policy || !database || !a_actor) {
				return;
			}
			const auto handle = policy->GetHandleForObject(RE::TESQuest::FORMTYPE, database);
			if (handle == policy->EmptyHandle()) {
				return;
			}
			RE::BSTSmartPointer<RE::BSScript::Object> object;
			if (!vm->FindBoundObject(handle, kDatabaseScript, object) || !object) {
				return;
			}
			RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback(new MilkLevelCallback(a_actor->GetHandle().native_handle()));
			auto* arguments = RE::MakeFunctionArguments(static_cast<RE::Actor*>(a_actor));
			vm->DispatchMethodCall(object, kMilkFunction, arguments, callback);
		}

		// ------------------------------------------------ our own leak layer
		// SGO only draws its leak overlay above its own MCM threshold, so the stages are drawn on a layer of our own,
		// staged by SGO's current milk / SGO's maximum (ActorMilkTotalPercent). Slots go through SGO's helpers so
		// SGO and other overlays never get the same node.
		std::uint32_t overlayEpoch{ 1 };

		class TaskCallback : public RE::BSScript::IStackCallbackFunctor
		{
		public:
			using Fn = std::function<void(const RE::BSScript::Variable&)>;
			explicit TaskCallback(Fn a_fn) :
				_fn(std::move(a_fn))
			{}

			void operator()(RE::BSScript::Variable a_result) override
			{
				// results arrive on a script thread: continue on the game thread
				if (auto* tasks = SKSE::GetTaskInterface()) {
					auto fn = _fn;
					tasks->AddTask([fn, a_result]() {
						try {
							fn(a_result);
						} catch (...) {
						}
					});
				}
			}

			void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}

		private:
			Fn _fn;
		};

		RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> Then(TaskCallback::Fn a_fn)
		{
			return RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor>(new TaskCallback(std::move(a_fn)));
		}

		bool CallUtil(const char* a_function, RE::BSScript::IFunctionArguments* a_arguments,
			RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> a_callback = {})
		{
			auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
			auto* policy = vm ? vm->GetObjectHandlePolicy() : nullptr;
			RE::BSTSmartPointer<RE::BSScript::Object> object;
			if (policy && utilQuest) {
				const auto handle = policy->GetHandleForObject(RE::TESQuest::FORMTYPE, utilQuest);
				if (handle != policy->EmptyHandle()) {
					vm->FindBoundObject(handle, kUtilScript, object);
				}
			}
			if (!object) {
				delete a_arguments;
				return false;
			}
			return vm->DispatchMethodCall(object, a_function, a_arguments, a_callback);
		}

		struct OwnOverlay
		{
			int   stamp{ 0 };      // settings revision * 16 + stage shown, 0 = none
			bool  known{ false };  // checked once this session (a layer may be left over from a save)
			bool  busy{ false };
			float verify{ 0.0f };
		};
		std::unordered_map<std::uint32_t, OwnOverlay> ownOverlays;

		void ClearOwnOverlay(RE::Actor* a_actor)
		{
			const auto key = a_actor->GetHandle().native_handle();
			auto&      state = ownOverlays[key];
			if (state.busy) {
				return;
			}
			state.busy = true;
			const auto epoch = overlayEpoch;
			const bool sent = CallUtil("ActorOverlayClear", RE::MakeFunctionArguments(static_cast<RE::Actor*>(a_actor), RE::BSFixedString(kOwnOverlay)),
				Then([key, epoch](const RE::BSScript::Variable&) {
					if (epoch != overlayEpoch) {
						return;
					}
					auto& s = ownOverlays[key];
					s.busy = false;
					s.stamp = 0;
					s.known = true;
				}));
			if (!sent) {
				state.busy = false;
			}
		}

		void ApplyOwnOverlay(RE::Actor* a_actor, int a_stage)
		{
			const auto& settings = Settings::Get();
			const auto  key = a_actor->GetHandle().native_handle();
			auto&       state = ownOverlays[key];
			if (state.busy) {
				return;
			}
			state.busy = true;
			const auto        epoch = overlayEpoch;
			const auto        handle = a_actor->GetHandle();
			const bool        female = IsFemale(a_actor);
			const int         stamp = settings.overlayRevision * 16 + a_stage;
			const std::string texture = std::format("textures\\OStrandsMilk\\MilkLeak_{}.dds", a_stage);
			const int         color = settings.overlayColor;
			const float       alpha = std::clamp(settings.overlayBoost * 0.6f, 0.3f, 1.0f);
			const float       gloss = settings.overlayGloss;
			const float       specular = settings.overlaySpecular;
			const bool        debug = settings.debugLog;
			const auto fail = [key, epoch]() {
				if (epoch == overlayEpoch) {
					ownOverlays[key].busy = false;
				}
			};
			// 1) the actor needs NiOverride's overlay nodes (SGO adds them the same way)
			Static("NiOverride", "HasOverlays", RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(a_actor)),
				Then([=](const RE::BSScript::Variable& a_has) {
					if (epoch != overlayEpoch) {
						return;
					}
					const auto actor = handle.get();
					if (!actor) {
						fail();
						return;
					}
					if (a_has.IsBool() && !a_has.GetBool()) {
						Static("NiOverride", "AddOverlays", RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(actor.get())));
					}
					// 2) texture, colour and opacity on a slot SGO hands out and books for us
					const bool sent = CallUtil("ActorOverlayApply",
						RE::MakeFunctionArguments(static_cast<RE::Actor*>(actor.get()), RE::BSFixedString(kOwnOverlay), RE::BSFixedString(texture), int(color),
							float(alpha), int(0), float(0.0f)),
						Then([=](const RE::BSScript::Variable&) {
							if (epoch != overlayEpoch) {
								return;
							}
							const auto who = handle.get();
							if (!who) {
								fail();
								return;
							}
							// 3) which node it went to, for the wet look
							Static("StorageUtil", "GetStringValue",
								RE::MakeFunctionArguments(static_cast<RE::TESForm*>(who.get()), RE::BSFixedString(kOwnOverlayKey), RE::BSFixedString("")),
								Then([=](const RE::BSScript::Variable& a_node) {
									if (epoch != overlayEpoch) {
										return;
									}
									auto& s = ownOverlays[key];
									s.busy = false;
									s.known = true;
									const auto target = handle.get();
									const std::string node = a_node.IsString() ? std::string(a_node.GetString()) : std::string();
									if (!target || node.empty()) {
										s.stamp = 0;  // no free overlay slot right now; try again on a later poll
										return;
									}
									auto* ref = static_cast<RE::TESObjectREFR*>(target.get());
									Static("NiOverride", "AddNodeOverrideFloat",
										RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kGloss), int(-1), float(gloss), bool(true)));
									Static("NiOverride", "AddNodeOverrideFloat",
										RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref), bool(female), RE::BSFixedString(node), int(kSpecular), int(-1), float(specular), bool(true)));
									Static("NiOverride", "ApplyNodeOverrides", RE::MakeFunctionArguments(static_cast<RE::TESObjectREFR*>(ref)));
									s.stamp = stamp;
									s.verify = kOwnOverlayVerify;
									if (debug) {
										logger::info("leak layer on {} ({}): stage {} of 8, milk {}% of SGO max", target->GetDisplayFullName(), node, stamp % 16,
											MilkPercent(target.get()));
									}
								}));
						}));
					if (!sent) {
						fail();
					}
				}));
		}

		void VerifyOwnOverlay(RE::Actor* a_actor)
		{
			const auto key = a_actor->GetHandle().native_handle();
			const auto epoch = overlayEpoch;
			Static("StorageUtil", "GetStringValue",
				RE::MakeFunctionArguments(static_cast<RE::TESForm*>(a_actor), RE::BSFixedString(kOwnOverlayKey), RE::BSFixedString("")),
				Then([key, epoch](const RE::BSScript::Variable& a_node) {
					if (epoch != overlayEpoch) {
						return;
					}
					if (!a_node.IsString() || std::string_view(a_node.GetString()).empty()) {
						auto& s = ownOverlays[key];
						if (!s.busy) {
							s.stamp = 0;  // something cleared our layer (SGO reset, overlay mod): put it back
						}
					}
				}));
		}

		// Called on every poll for each lactating actor whose level is being read.
		void UpdateOwnOverlay(RE::Actor* a_actor, float a_elapsed)
		{
			if (!utilQuest || !a_actor) {
				return;
			}
			const auto& settings = Settings::Get();
			const int   level = MilkPercent(a_actor);
			if (level < 0) {
				return;
			}
			const int stage = settings.overlay && IsFemale(a_actor) ? StageFor(a_actor) : 0;
			const int stamp = stage > 0 ? settings.overlayRevision * 16 + stage : 0;
			auto&     state = ownOverlays[a_actor->GetHandle().native_handle()];
			if (state.busy) {
				return;
			}
			if (stamp == 0) {
				if (state.stamp != 0 || !state.known) {
					ClearOwnOverlay(a_actor);
				}
				return;
			}
			if (state.stamp != stamp) {
				ApplyOwnOverlay(a_actor, stage);
				return;
			}
			state.verify -= a_elapsed;
			if (state.verify <= 0.0f) {
				state.verify = kOwnOverlayVerify;
				VerifyOwnOverlay(a_actor);
			}
		}
		std::mutex                               lock;
		std::vector<Pending>                     pending;
		std::vector<Pending>                     deferred;
		std::vector<Pending>                     others;
		std::vector<Milking>                     milking;
		std::unordered_map<std::uint32_t, std::pair<RE::ActorHandle, float>> leakTimers;
		std::unordered_map<std::uint32_t, std::pair<RE::ActorHandle, float>> crotchTimers;  // actors at a dripping stage
		struct VulvaFlow
		{
			RE::ActorHandle actor;
			float           wait{ 0.0f };    // until the milk has run down to the vulva
			float           remain{ 0.0f };  // dripping time left
			float           next{ 0.0f };    // until the next drop
		};
		std::vector<VulvaFlow> vulvaFlows;
		float                                    wait{ 0.0f };
		float                                    overlayPoll{ 0.0f };
		std::mt19937                             rng{ std::random_device{}() };
		bool                                     hooked{ false };

		bool SameText(std::string_view a_text, std::string_view a_lower)
		{
			if (a_text.size() != a_lower.size()) {
				return false;
			}
			for (std::size_t i = 0; i < a_text.size(); ++i) {
				if (static_cast<char>(std::tolower(static_cast<unsigned char>(a_text[i]))) != a_lower[i]) {
					return false;
				}
			}
			return true;
		}

		void Observe(const RE::BSAnimationGraphEvent* a_event)
		{
			if (!a_event || !a_event->holder) {
				return;
			}
			const char* raw = a_event->tag.c_str();
			if (!raw || (raw[0] != 'S' && raw[0] != 's')) {
				return;
			}
			const std::string_view tag(raw);
			std::optional<Signal>  signal;
			if (SameText(tag, kSpawnMilk)) {
				signal = Signal::kSpawnMilk;
			} else if (SameText(tag, kActorDone)) {
				signal = Signal::kDone;
			} else if (SameText(tag, kSpawnSemen) || SameText(tag, kSpawnGem)) {
				signal = Signal::kOther;
			}
			if (!signal) {
				return;
			}
			auto* actor = a_event->holder->As<RE::Actor>();
			if (!actor) {
				return;
			}
			std::scoped_lock guard(lock);
			if (pending.size() < 64) {
				pending.push_back({ const_cast<RE::Actor*>(actor)->GetHandle(), *signal, Clock::now() });
			}
		}

		using Sink = RE::BSTEventSink<RE::BSAnimationGraphEvent>;
		using Source = RE::BSTEventSource<RE::BSAnimationGraphEvent>;

		struct CharacterHook
		{
			static RE::BSEventNotifyControl Thunk(Sink* a_this, const RE::BSAnimationGraphEvent* a_event, Source* a_source)
			{
				Observe(a_event);
				return original(a_this, a_event, a_source);
			}
			static inline REL::Relocation<decltype(Thunk)> original;
		};

		struct PlayerHook
		{
			static RE::BSEventNotifyControl Thunk(Sink* a_this, const RE::BSAnimationGraphEvent* a_event, Source* a_source)
			{
				Observe(a_event);
				return original(a_this, a_event, a_source);
			}
			static inline REL::Relocation<decltype(Thunk)> original;
		};

		void InstallHooks()
		{
			if (hooked) {
				return;
			}
			hooked = true;
			REL::Relocation<std::uintptr_t> character{ RE::VTABLE_Character[2] };
			CharacterHook::original = character.write_vfunc(0x1, CharacterHook::Thunk);
			REL::Relocation<std::uintptr_t> player{ RE::VTABLE_PlayerCharacter[2] };
			PlayerHook::original = player.write_vfunc(0x1, PlayerHook::Thunk);
			logger::info("watching animation events for SGO milking");
		}

		RE::NiAVObject* First(RE::NiAVObject* a_root, std::initializer_list<const char*> a_names)
		{
			for (const char* name : a_names) {
				if (auto* node = a_root->GetObjectByName(name)) {
					return node;
				}
			}
			return nullptr;
		}

		RE::NiPoint3 Unit(RE::NiPoint3 a_vector, const RE::NiPoint3& a_fallback)
		{
			return a_vector.Unitize() > 1e-5f ? a_vector : a_fallback;
		}

		bool Eligible(RE::Actor* a_actor, const RE::NiPoint3& a_origin, float a_reach)
		{
			return a_actor && !a_actor->IsDead() && !a_actor->IsChild() && a_actor->Is3DLoaded() && a_actor->GetPosition().GetDistance(a_origin) <= a_reach &&
			       Anatomy::IsPerson(a_actor);
		}
	}

	void Connect()
	{
		auto* data = RE::TESDataHandler::GetSingleton();
		const bool sgo4 = data && data->LookupModByName(kSGOPlugin);
		database = sgo4 ? data->LookupForm<RE::TESQuest>(kDatabaseQuest, kSGOBasePlugin) : nullptr;
		produceMilk = sgo4 ? data->LookupForm<RE::TESFaction>(kProduceMilkFaction, kSGOBasePlugin) : nullptr;
		utilQuest = sgo4 ? data->LookupForm<RE::TESQuest>(kUtilQuest, kSGOBasePlugin) : nullptr;
		logger::info("leak textures: {}", utilQuest ? "own overlay layer, staged by SGO milk / SGO max" : "SGO's own leak overlay (SGO util quest not found)");
		logger::info("{}", database ? "Soul Gem Oven 4 (Sgo4IF.esp) found; milk follows its milking and its real milk level" :
		                              "Sgo4IF.esp not found; milk only comes with climaxes");
		InstallHooks();
	}

	bool SGOInstalled()
	{
		return database != nullptr;
	}

	std::string Readout()
	{
		std::scoped_lock guard(readoutLock);
		return readout;
	}

	int MilkPercent(RE::Actor* a_actor)
	{
		if (!database || !a_actor) {
			return -1;
		}
		std::scoped_lock guard(levelLock);
		const auto it = levels.find(a_actor->GetHandle().native_handle());
		if (it == levels.end() || std::chrono::steady_clock::now() - it->second.at > kLevelFresh) {
			return -1;
		}
		return static_cast<int>(std::floor(it->second.ratio * 100.0f + 0.001f));
	}

	bool IsFemale(RE::Actor* a_actor)
	{
		auto* base = a_actor ? a_actor->GetActorBase() : nullptr;
		return base && base->GetSex() == RE::SEXES::kFemale;
	}

	bool ClimaxAllowed(RE::Actor* a_actor)
	{
		const auto& settings = Settings::Get();
		if (!a_actor || !IsFemale(a_actor) || !Anatomy::IsPerson(a_actor)) {
			return false;
		}
		if (settings.milkNeedsExposed && !Revealing::Exposed(a_actor)) {
			return false;
		}
		if (settings.milkClimaxNeedsSGO && SGOInstalled() && MilkPercent(a_actor) < settings.milkClimaxMinPercent) {
			if (settings.debugLog) {
				logger::info("{} climaxes without milk (SGO milk {}%, needs {}%)", a_actor->GetDisplayFullName(), MilkPercent(a_actor),
					settings.milkClimaxMinPercent);
			}
			return false;
		}
		return true;
	}

	float SprayPower(RE::Actor* a_actor, bool a_regularLeak)
	{
		const auto& settings = Settings::Get();
		if (!a_actor || !settings.milkRamp || (!a_regularLeak && !settings.milkRampAll)) {
			return 1.0f;
		}
		const int level = MilkPercent(a_actor);
		if (level < 0) {
			return 1.0f;
		}
		const float from = static_cast<float>(settings.milkSGOFullPercent);
		const float to = std::max(static_cast<float>(settings.milkRampFull), from + 1.0f);
		const float t = std::clamp((static_cast<float>(level) - from) / (to - from), 0.0f, 1.0f);
		const float low = std::clamp(settings.milkRampMin, 0.0f, 1.0f);
		return low + (1.0f - low) * t;
	}

	void OnSpray(RE::ActorHandle a_actor, float a_flow)
	{
		const auto& settings = Settings::Get();
		if (!settings.milkCrotch || !crotchTimers.contains(a_actor.native_handle())) {
			return;  // not at a dripping leak stage (or not exposed)
		}
		const float delay = std::max(settings.milkCrotchDelay, 0.0f);
		const float flow = std::clamp(a_flow, 0.3f, 60.0f);
		for (auto& f : vulvaFlows) {
			if (f.actor == a_actor) {
				// already dripping: keep going for this spray too
				f.remain = std::max(f.remain, flow + (f.wait > 0.0f ? 0.0f : delay));
				return;
			}
		}
		vulvaFlows.push_back({ a_actor, delay, flow + 0.5f, 0.0f });
	}

	std::optional<Body::Tip> NippleOf(RE::Actor* a_actor, int a_side)
	{
		auto* root = a_actor ? a_actor->Get3D(false) : nullptr;
		if (!root) {
			return std::nullopt;
		}
		const auto& settings = Settings::Get();
		if (a_side == 2) {
			// Milk that ran down to the crotch drips off the vulva.
			std::vector<Anatomy::Point> rim;
			RE::NiPoint3                center;
			RE::NiPoint3                inward;
			if (Anatomy::FindOpening(root, Anatomy::Opening::kVagina, rim, center, inward)) {
				return std::nullopt;
			}
			const RE::NiPoint3 down{ 0.0f, 0.0f, -1.0f };
			const RE::NiPoint3 outward = Unit(-inward, down);
			return Body::Tip{ center + outward * 0.8f + down * 0.4f, Unit(outward * 0.35f + down, down) };
		}
		const float offset = settings.milkNippleOffset;
		const float scale = root->world.scale > 0.0f ? root->world.scale : 1.0f;
		const RE::NiPoint3 up = Unit(root->world.rotate * RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f });
		const RE::NiPoint3 right = Unit(root->world.rotate * RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f });
		const RE::NiPoint3 shift = (up * settings.milkNippleUp + right * (a_side == 0 ? -settings.milkNippleSide : settings.milkNippleSide)) * scale;
		if (const auto nipple = Body::NippleOf(a_actor, a_side)) {
			return Body::Tip{ nipple->point + nipple->axis * (offset * scale) + shift, nipple->axis };
		}
		// No readable body mesh: fall back to the outermost breast bone.
		const bool      left = a_side == 0;
		RE::NiAVObject* tip = left ? First(root, { "L Breast03", "L Breast02", "NPC L Breast01", "NPC L Breast" }) :
		                             First(root, { "R Breast03", "R Breast02", "NPC R Breast01", "NPC R Breast" });
		RE::NiAVObject* base = left ? First(root, { "NPC L Breast", "NPC L Breast01" }) : First(root, { "NPC R Breast", "NPC R Breast01" });
		RE::NiAVObject* chest = First(root, { "NPC Spine2 [Spn2]", "NPC Spine1 [Spn1]" });
		if (!tip || !base || !chest) {
			return std::nullopt;
		}
		const RE::NiPoint3 forward = Unit(root->world.rotate * RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f });
		const RE::NiPoint3 out = Unit(base->world.translate - chest->world.translate, forward);
		RE::NiPoint3       along = tip->world.translate - base->world.translate;
		const RE::NiPoint3 axis = along.Length() > 0.5f ? Unit(Unit(along, out) * 0.6f + out * 0.4f, out) : out;
		return Body::Tip{ tip->world.translate + axis * (offset * scale) + shift, axis };
	}

	void Update(float a_delta)
	{
		std::vector<Pending> batch;
		{
			std::scoped_lock guard(lock);
			batch.swap(pending);
		}
		auto        settingsGuard = Settings::Lock();
		const auto& settings = Settings::Get();
		if (!settings.enabled || !settings.milk) {
			deferred.clear();
			others.clear();
			milking.clear();
			leakTimers.clear();
			crotchTimers.clear();
			vulvaFlows.clear();
			return;
		}
		auto& fluids = FluidManager::Get();
		const auto now = Clock::now();
		{
			std::vector<OverlayStep> steps;
			{
				std::scoped_lock guard(overlayLock);
				steps.swap(overlaySteps);
			}
			for (const auto& step : steps) {
				ProcessOverlay(step);
			}
		}
		for (const auto& entry : batch) {
			if (entry.signal == Signal::kSpawnMilk) {
				deferred.push_back(entry);
			} else if (entry.signal == Signal::kOther) {
				others.push_back(entry);
			}
		}
		std::erase_if(others, [&](const Pending& a_other) { return now - a_other.at > 4 * kPreloadWindow; });
		batch.erase(std::remove_if(batch.begin(), batch.end(), [](const Pending& a_entry) { return a_entry.signal != Signal::kDone; }), batch.end());
		std::vector<Pending> ready;
		for (auto it = deferred.begin(); it != deferred.end();) {
			if (now - it->at < kPreloadWindow) {
				++it;
				continue;
			}
			const auto& entry = *it;
			const bool  preload = std::ranges::any_of(others, [&](const Pending& a_other) {
				const auto gap = a_other.at > entry.at ? a_other.at - entry.at : entry.at - a_other.at;
				return a_other.actor == entry.actor && gap <= kPreloadWindow;
			});
			if (!preload) {
				ready.push_back(entry);
			} else if (settings.debugLog) {
				logger::info("SGO event preloader ignored");
			}
			it = deferred.erase(it);
		}
		batch.insert(batch.begin(), ready.begin(), ready.end());
		for (const auto& entry : batch) {
			auto existing = std::ranges::find_if(milking, [&](const Milking& m) { return m.actor == entry.actor; });
			if (entry.signal == Signal::kDone) {
				if (existing != milking.end()) {
					milking.erase(existing);
				}
				continue;
			}
			if (!settings.milkSGOMilking) {
				continue;
			}
			const auto actor = entry.actor.get();
			if (!actor || !actor->Get3D(false)) {
				continue;
			}
			if (settings.debugLog) {
				logger::info("SGO milking: {} gives milk", actor->GetDisplayFullName());
			}
			{
				const float power = SprayPower(actor.get(), false);
				fluids.Lactate(entry.actor, settings.milkSGOMilkingAmount * settings.milkAmount * power, FluidManager::kSqueeze, FluidManager::kNoThread, {},
					power);
			}
			OnSpray(entry.actor, std::min(settings.milkFlow, 1.0f));
			if (existing == milking.end()) {
				milking.push_back({ entry.actor });
			} else {
				existing->idle = 0.0f;
				existing->dribble = 0.0f;
			}
		}

		// Between squeezes the breasts keep dribbling until SGO says the milking is done.
		std::uniform_real_distribution<float> unit(0.0f, 1.0f);
		std::erase_if(milking, [&](Milking& m) {
			m.idle += a_delta;
			m.dribble += a_delta;
			if (m.idle > kMilkingWindow || !m.actor.get()) {
				return true;
			}
			if (m.dribble >= std::clamp(2.0f * settings.milkInterval, 0.3f, 3.0f)) {
				m.dribble = -0.4f * settings.milkInterval * unit(rng);
				fluids.Lactate(m.actor, 0.5f * settings.milkAmount, FluidManager::kDribble);
			}
			return false;
		});

		// Full breasts leak on their own timers, advanced every frame.
		for (auto it = leakTimers.begin(); it != leakTimers.end();) {
			const auto handle = it->second.first;
			if (!handle.get()) {
				it = leakTimers.erase(it);
				continue;
			}
			auto& timer = it->second.second;
			timer -= a_delta;
			if (timer <= 0.0f) {
				// One cycle = the flow time, then the pause ("분출 빈도"), then the next cycle.
				timer = std::max(settings.milkFlow + settings.milkInterval, 0.3f) * (0.95f + 0.1f * unit(rng));
				if (std::ranges::none_of(milking, [&](const Milking& m) { return m.actor == handle; })) {
					const auto  leaking = handle.get();
					const float power = SprayPower(leaking.get(), true);
					fluids.Lactate(handle, settings.milkSGOFullAmount * settings.milkAmount * power, FluidManager::kCycle, FluidManager::kNoThread, {}, power);
					OnSpray(handle, settings.milkFlow);
				}
			}
			++it;
		}

		// Vulva drips run with the spraying: they start a moment after milk starts coming out and stop after it.
		std::erase_if(vulvaFlows, [&](VulvaFlow& f) {
			if (!settings.milkCrotch || !f.actor.get()) {
				return true;
			}
			if (f.wait > 0.0f) {
				f.wait -= a_delta;
				return false;
			}
			f.remain -= a_delta;
			f.next -= a_delta;
			if (f.next <= 0.0f) {
				f.next = std::max(settings.milkCrotchInterval, 0.5f) * (0.7f + 0.6f * unit(rng));
				fluids.Lactate(f.actor, settings.milkCrotchAmount * settings.milkAmount, FluidManager::kCrotch);
			}
			return f.remain <= 0.0f;
		});

		wait -= a_delta;
		if (wait > 0.0f) {
			return;
		}
		wait = kPoll;
		if (!database) {
			leakTimers.clear();
			crotchTimers.clear();
			vulvaFlows.clear();
			return;
		}
		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player) {
			return;
		}
		const auto origin = player->GetPosition();
		overlayPoll -= kPoll;
		std::unordered_map<std::uint32_t, std::pair<RE::ActorHandle, float>> next;
		std::unordered_map<std::uint32_t, std::pair<RE::ActorHandle, float>> nextCrotch;
		std::vector<RE::Actor*> candidates;
		const auto consider = [&](RE::Actor* a_actor) {
			if (!Eligible(a_actor, origin, settings.arousalRadius) || !IsFemale(a_actor)) {
				return;
			}
			if (produceMilk && !a_actor->IsInFaction(produceMilk)) {
				// not lactating (any more): take our leak layer off if one is still on
				if (const auto own = ownOverlays.find(a_actor->GetHandle().native_handle()); own != ownOverlays.end() && own->second.stamp != 0) {
					ClearOwnOverlay(a_actor);
				}
				return;  // nothing to ask SGO about
			}
			candidates.push_back(a_actor);
			if (settings.milkCrotch && MilkPercent(a_actor) >= 0 && StageFor(a_actor) >= settings.milkCrotchStage &&
				(!settings.milkNeedsExposed || Revealing::Exposed(a_actor))) {
				const auto handle = a_actor->GetHandle();
				const auto key = handle.native_handle();
				if (auto it = crotchTimers.find(key); it != crotchTimers.end()) {
					nextCrotch[key] = it->second;
				} else {
					nextCrotch[key] = { handle, 0.5f + 2.0f * unit(rng) };
					if (settings.debugLog) {
						logger::info("SGO: {} is at leak stage {}: milk drips off the vulva", a_actor->GetDisplayFullName(), StageFor(a_actor));
					}
				}
			}
			if (!settings.milkSGOFull || MilkPercent(a_actor) < settings.milkSGOFullPercent) {
				return;
			}
			if (settings.milkNeedsExposed && !Revealing::Exposed(a_actor)) {
				return;
			}
			const auto handle = a_actor->GetHandle();
			const auto key = handle.native_handle();
			if (auto it = leakTimers.find(key); it != leakTimers.end()) {
				next[key] = it->second;
			} else {
				next[key] = { handle, std::max(settings.milkInterval, 0.1f) * unit(rng) };
				if (settings.debugLog) {
					logger::info("SGO: {} is full of milk ({}%) and starts leaking", a_actor->GetDisplayFullName(), MilkPercent(a_actor));
				}
			}
		};
		consider(player);
		if (auto* lists = RE::ProcessLists::GetSingleton()) {
			for (const auto& handle : lists->highActorHandles) {
				const auto actor = handle.get();
				consider(actor.get());
			}
		}
		leakTimers = std::move(next);
		crotchTimers = std::move(nextCrotch);
		// Ask SGO about a few lactating actors per second, the player and console-selected actor first.
		{
			RE::Actor* selected = nullptr;
			if (const auto ref = RE::Console::GetSelectedRef()) {
				selected = ref->As<RE::Actor>();
			}
			std::vector<RE::Actor*> first;
			std::vector<RE::Actor*> rest;
			for (auto* actor : candidates) {
				(actor == player || actor == selected ? first : rest).push_back(actor);
			}
			const auto ask = [&](RE::Actor* a_actor) {
				RequestLevel(a_actor);
				UpdateOwnOverlay(a_actor, kPoll);
				if (settings.overlay && overlayPoll <= 0.0f) {
					RequestOverlayNode(a_actor);
				}
			};
			for (auto* actor : first) {
				ask(actor);
			}
			const std::size_t budget = std::min(rest.size(), kMaxLevelRequests);
			for (std::size_t k = 0; k < budget; ++k) {
				ask(rest[(roundRobin + k) % rest.size()]);
			}
			roundRobin += kMaxLevelRequests;
			auto*       shown = selected ? selected : player;
			const int   level = MilkPercent(shown);
			std::string text = level >= 0 ? std::format("현재 SGO 모유량 ({}): 최대량 대비 {:g}%  →  흘러내림 텍스쳐 {}  ·  분출 세기·양 {}%", shown->GetDisplayFullName(),
			                                    static_cast<float>(level) / kSGOTrueMax,
			                                    StageFor(shown) > 0 ? std::format("{}단계 / 8", StageFor(shown)) : std::string("없음"),
			                                    static_cast<int>(std::lround(SprayPower(shown, true) * 100.0f))) :
			                                std::string("현재 SGO 모유량: 측정 안 됨 (SGO에서 모유를 생산 중인 근처 여성만 측정, 콘솔로 선택 가능)");
			std::scoped_lock guard(readoutLock);
			readout = std::move(text);
		}
		if (overlayPoll <= 0.0f) {
			overlayPoll = 1.0f;
		}
	}

	void Reset()
	{
		{
			std::scoped_lock guard(lock);
			pending.clear();
		}
		{
			std::scoped_lock guard(levelLock);
			levels.clear();
		}
		deferred.clear();
		others.clear();
		milking.clear();
		leakTimers.clear();
			crotchTimers.clear();
			vulvaFlows.clear();
		{
			std::scoped_lock guard(overlayLock);
			overlaySteps.clear();
		}
		overlayApplied.clear();
		lastStage.clear();
		ownOverlays.clear();
		++overlayEpoch;
		wait = 0.0f;
	}
}
