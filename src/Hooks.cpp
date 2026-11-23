#include "Hooks.h"

#include "Body.h"
#include "FluidManager.h"
#include "Lactation.h"
#include "Milk.h"
#include "Settings.h"

#include <atomic>

namespace OStrands::Hooks
{
	namespace
	{
		bool g_faulted = false;  // a hard fault happened: milk stays off until the next load

		void RunFrame(float a_delta)
		{
			// Never let an error escape into the game's task queue: log it, drop the fluid and keep playing.
			try {
				Body::BeginFrame(a_delta);
				Milk::Update(a_delta);
				Lactation::Update(a_delta);
				FluidManager::Get().Update(a_delta);
			} catch (const std::exception& error) {
				logger::error("milk update failed: {}; clearing milk and continuing", error.what());
				ResetSession();
			} catch (...) {
				logger::error("milk update failed; clearing milk and continuing");
				ResetSession();
			}
		}

		void RunReset(float)
		{
			FluidManager::Get().Reset();
			Body::Reset();
			Lactation::Reset();
		}

		// Structured exception guard: an access violation inside the milk code turns milk off for this session
		// instead of taking the whole game down. (No C++ objects with destructors may live in this function.)
		bool Guarded(void (*a_function)(float), float a_delta)
		{
			__try {
				a_function(a_delta);
				return true;
			} __except (1 /* EXCEPTION_EXECUTE_HANDLER */) {
				return false;
			}
		}

		class FrameDriver : public RE::BSTEventSink<RE::InputEvent*>
		{
		public:
			static FrameDriver* Get()
			{
				static FrameDriver driver;
				return &driver;
			}

			RE::BSEventNotifyControl ProcessEvent(RE::InputEvent* const*, RE::BSTEventSource<RE::InputEvent*>*) override
			{
				_frame.fetch_add(1, std::memory_order_acq_rel);
				auto* tasks = SKSE::GetTaskInterface();
				if (tasks && _pending.load(std::memory_order_acquire) < kMaxPending) {
					_pending.fetch_add(1, std::memory_order_acq_rel);
					tasks->AddTask([this]() {
						Update();
						_pending.fetch_sub(1, std::memory_order_acq_rel);
					});
				}
				return RE::BSEventNotifyControl::kContinue;
			}

		private:
			static constexpr int           kMaxPending = 2;
			static constexpr std::uint64_t kMaxCatchUp = 4;

			void Update()
			{
				const std::uint64_t frame = _frame.load(std::memory_order_acquire);
				if (frame == _handled) {
					return;
				}
				const std::uint64_t passed = frame - _handled;
				_handled = frame;
				auto*      ui = RE::UI::GetSingleton();
				const bool mainMenu = ui && ui->IsMenuOpen(RE::MainMenu::MENU_NAME);
				if (mainMenu != _mainMenu) {
					_mainMenu = mainMenu;
					if (mainMenu) {
						ResetSession();
						g_faulted = false;
					}
				}
				if (mainMenu) {
					return;
				}
				const bool editingBody = ui && ui->IsMenuOpen(RE::RaceSexMenu::MENU_NAME);
				if (editingBody != _editingBody) {
					auto guard = Settings::Lock();
					FluidManager::Get().WaitForBodyEditor(editingBody);
					Body::Reset();
					_editingBody = editingBody;
				}
				if (editingBody || (ui && ui->GameIsPaused())) {
					return;
				}
				const float delta = RE::GetSecondsSinceLastFrame() * static_cast<float>(std::min(passed, kMaxCatchUp));
				if (g_faulted) {
					return;
				}
				if (!Guarded(&RunFrame, delta)) {
					g_faulted = true;
					logger::critical("milk hit a hard fault (access violation) and is turned off until the next save load. "
									 "Please send this log and the crash log.");
					Guarded(&RunReset, 0.0f);
				}
			}

			std::atomic<std::uint64_t> _frame{ 0 };
			std::atomic<int>           _pending{ 0 };
			std::uint64_t              _handled{ 0 };
			bool                       _mainMenu{ false };
			bool                       _editingBody{ false };
		};
	}

	void ResetSession()
	{
		g_faulted = false;
		FluidManager::Get().Reset();
		Body::Reset();
		Lactation::Reset();
		Milk::Reset();
	}

	void Install()
	{
		auto* input = RE::BSInputDeviceManager::GetSingleton();
		if (!SKSE::GetTaskInterface() || !input) {
			logger::error("no SKSE task interface or input manager; milk disabled");
			return;
		}
		input->PrependEventSink(FrameDriver::Get());
		logger::info("frame driver registered");
	}
}
