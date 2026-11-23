#include "Hooks.h"
#include "Lactation.h"
#include "Milk.h"
#include "Settings.h"

namespace
{
	void InitLogging()
	{
		try {
			auto path = logger::log_directory();
			if (!path) {
				return;
			}
			*path /= "OStrands_Milk.log";
			auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
			auto log = std::make_shared<spdlog::logger>("global", std::move(sink));
			log->set_level(spdlog::level::info);
			log->flush_on(spdlog::level::info);
			log->set_pattern("[%H:%M:%S.%e] [%l] %v");
			spdlog::set_default_logger(std::move(log));
		} catch (const std::exception&) {
		}
	}

	void OnMessage(SKSE::MessagingInterface::Message* message)
	{
		switch (message->type) {
		case SKSE::MessagingInterface::kInputLoaded:
			OStrands::Hooks::Install();
			break;
		case SKSE::MessagingInterface::kPostPostLoad:
			OStrands::Milk::RegisterMenu();
			break;
		case SKSE::MessagingInterface::kDataLoaded:
			OStrands::Lactation::Connect();
			OStrands::Milk::ConnectSexLab();
			break;
		case SKSE::MessagingInterface::kPreLoadGame:
		case SKSE::MessagingInterface::kNewGame:
			OStrands::Hooks::ResetSession();
			break;
		default:
			break;
		}
	}
}

SKSEPluginLoad(const SKSE::LoadInterface* skse)
{
	SKSE::Init(skse, { .log = false });
	InitLogging();
	const auto* plugin = SKSE::PluginDeclaration::GetSingleton();
	logger::info("{} {} loaded on Skyrim {}", plugin->GetName(), plugin->GetVersion().string("."sv), REL::Module::get().version().string("."sv));

	// Read OStrands' own settings (collision, revealing armor...) without ever writing them, then the milk settings.
	OStrands::Settings::Get().Load();
	OStrands::Milk::Load();
	auto* messaging = SKSE::GetMessagingInterface();
	return messaging && messaging->RegisterListener(OnMessage);
}
