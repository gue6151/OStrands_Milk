#pragma once

#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace OStrands
{
	struct Settings
	{
		bool  enabled{ true };
		bool  debugLog{ false };

		float bridgeRate{ 2.1f };
		int   maxPerStroke{ 2 };
		int   maxPerPair{ 5 };
		int   maxTotal{ 20 };
		float cooldown{ 0.25f };

		float radius{ 0.10f };
		float minRadius{ 0.035f };
		float shaftRadius{ 1.0f };
		std::vector<std::pair<std::string, float>> shaftRadiusOverrides;
		float tipExtension{ 2.9f };
		float transparency{ 0.8f };
		float shine{ 20.0f };
		float gloss{ 100.0f };

		bool  glints{ true };
		float glintRate{ 1.5f };
		float glintSize{ 1.3f };
		float glintBrightness{ 5.0f };
		float glintLife{ 0.25f };
		int   glintMax{ 24 };

		float gravity{ 350.0f };
		float damping{ 0.9f };
		float viscosity{ 4.0f };
		float maxLength{ 14.0f };
		float maxLife{ 4.0f };
		float fadeTime{ 0.4f };

		bool  crotch{ true };
		float crotchRate{ 2.1f };
		int   crotchMaxPerContact{ 3 };
		int   crotchMaxPerPair{ 5 };
		float crotchMaxGap{ 10.0f };
		float crotchSpread{ 1.5f };
		float crotchSkin{ 1.0f };
		float crotchRadius{ 0.09f };
		float crotchMinRadius{ 0.03f };
		float crotchMaxLength{ 12.0f };
		float crotchMaxLife{ 4.0f };

		bool  tribbing{ true };
		float tribbingRate{ 1.0f };
		int   tribbingMaxPerContact{ 3 };
		int   tribbingMaxPerPair{ 5 };
		float tribbingMaxGap{ 6.0f };
		float tribbingSpread{ 1.5f };

		bool  oral{ true };
		bool  oralDrips{ true };
		float mouthBlend{ 0.7f };
		float lipWidth{ 0.9f };
		float lipHeight{ 0.45f };
		float oralRate{ 0.8f };
		int   oralMaxPerContact{ 3 };
		int   oralMaxPerPair{ 5 };
		float oralMaxGap{ 6.0f };
		float oralSpread{ 1.0f };

		bool  fingering{ true };
		float fingerRadius{ 0.35f };
		float fingerTipExtension{ 0.9f };
		bool  fingerDrips{ true };
		float fingerRate{ 0.8f };
		int   fingerMaxPerContact{ 2 };
		int   fingerMaxPerPair{ 4 };
		float fingerMaxGap{ 4.0f };
		float fingerSpread{ 0.8f };

		bool  toys{ true };
		float toyRadius{ 0.8f };
		float toyMaxRadius{ 20.0f };

		std::vector<std::pair<std::string, std::string>> actions;

		bool  drips{ true };
		bool  wearerDrips{ true };
		float dripInterval{ 4.0f };
		float dripLinger{ 15.0f };
		int   dripMaxPerSource{ 3 };
		float dripRadius{ 0.09f };
		float dripMinRadius{ 0.025f };
		float dripStretchTime{ 1.0f };
		float dripWeight{ 2.5f };
		float dripMaxLength{ 18.0f };
		float dripMaxLife{ 7.0f };
		float dripFadeTime{ 0.9f };

		std::vector<std::string>                         revealingKeywords{ "ostrands_revealing" };
		std::vector<std::pair<std::string, std::string>> revealingArmor;

		bool  arousal{ true };
		bool  arousalWetness{ true };
		bool  arousalPrecum{ true };
		float arousalThreshold{ 50.0f };
		float arousalInterval{ 3.0f };
		float arousalRadius{ 2048.0f };

		int   semen{ -1 };
		float semenAmount{ 5.0f };
		float semenStrength{ 1.1f };
		int   semenPulses{ 10 };
		float semenOpacity{ 1.0f };
		float semenCohesion{ 15.0f };
		float semenViscosity{ 50.0f };
		float semenStretch{ 5.0f };
		float semenStickiness{ 0.4f };
		float semenOverflow{ 0.5f };
		float ocumWait{ 0.5f };

		bool  collision{ true };
		float collisionThickness{ 1.0f };
		bool  showShapes{ false };

		float fluidLifetime{ 40.0f };
		float fluidFade{ 6.0f };
		int   fluidMaxParticles{ 3072 };

		// Milk: nipples squirt during climax, SGO milking and while SGO breasts are full.
		bool  milkRamp{ true };        // spray strength/amount grow with the SGO milk level
		int   milkRampFull{ 200 };     // SGO % where the spray reaches 100%
		float milkRampMin{ 0.1f };     // share at the leak threshold
		bool  milkRampAll{ false };    // also SGO milking and sex
		bool  milkCrotch{ true };
		int   milkCrotchStage{ 8 };
		float milkCrotchInterval{ 4.0f };
		float milkCrotchDelay{ 1.5f };
		float milkCrotchAmount{ 1.0f };
		bool  milk{ true };
		bool  milkOnClimax{ true };
		bool  milkClimaxNeedsSGO{ true };
		int   milkClimaxMinPercent{ 30 };
		bool  milkNeedsExposed{ true };
		float milkAmount{ 1.6f };
		float milkStrength{ 1.0f };
		int   milkPulses{ 6 };
		float milkOpacity{ 1.0f };
		float milkCohesion{ 2.0f };
		float milkViscosity{ 6.0f };
		float milkStretch{ 18.0f };
		float milkStickiness{ 0.08f };
		float milkStickinessWorld{ -1.0f };  // set by the milk menu only (never read from or written to OStrands.ini); -1 = same as skin
		float milkLifetime{ 20.0f };
		float milkNippleOffset{ 0.0f };
		float milkNippleUp{ 0.0f };
		float milkNippleSide{ 0.0f };
		bool  milkSGOMilking{ true };
		float milkSGOMilkingAmount{ 1.8f };
		bool  milkSGOFull{ true };
		int   milkSGOFullPercent{ 95 };
		float milkSGOFullInterval{ 5.0f };
		float milkSGOFullAmount{ 1.0f };
		// OStrands_Milk: user-facing values mapped by Milk::Config::Apply.
		float milkInterval{ 0.6f };
		float milkSpeedMin{ 0.09f };
		float milkSpeedMax{ 1.47f };
		bool  milkAlwaysStrong{ false };
		float milkGlow{ 0.4f };
		float milkShine{ 1.5f };
		float milkMerge{ 0.6f };
		int   milkMode{ 1 };             // 0 dribbling, 1 conditional cow, 2 generous, 3 overflowing, 4 in heat
		float milkFlow{ 4.0f };          // seconds each flow lasts
		int   milkStreamsMin{ 1 };
		int   milkStreamsMax{ 3 };
		float milkCone{ 0.26f };         // radians
		float milkThickness{ 0.7f };           // 0..1: how strongly touching milk merges into one mass
		// SGO "MilkLeak" skin overlay tuning
		bool  overlay{ true };
		int   overlayColor{ 0xFFFFFF };
		float overlayGloss{ 30.0f };
		float overlaySpecular{ 1.5f };
		float overlayBoost{ 1.8f };
		int   overlayRevision{ 0 };
		int   overlayStageFrom{ 50 };
		int   overlayStageTo{ 100 };
		int   fluidTestKey{ 0 };
		int   revealingKey{ 0 };

		static Settings&       Get();
		static const Settings& Defaults();
		static std::unique_lock<std::recursive_mutex> Lock();
		static constexpr float kRateMin = 0.0f, kRateMax = 3.0f;
		static constexpr float kIntervalMin = 0.2f, kIntervalMax = 6.0f;
		static constexpr int kCountMin = 0, kCountMax = 32;
		static constexpr float kTransparencyMin = 0.0f, kTransparencyMax = 0.95f;
		static constexpr float kShineMin = 0.0f, kShineMax = 20.0f;
		static constexpr float kGlossMin = 10.0f, kGlossMax = 1000.0f;
		static constexpr float kGlintRateMin = 0.0f, kGlintRateMax = 6.0f;
		static constexpr float kGlintSizeMin = 0.1f, kGlintSizeMax = 3.0f;
		static constexpr float kGlintBrightnessMin = 0.0f, kGlintBrightnessMax = 5.0f;
		static constexpr float kLingerMin = 0.0f, kLingerMax = 30.0f;
		static constexpr float kThresholdMin = 0.0f, kThresholdMax = 100.0f;
		static constexpr float kArousalIntervalMin = 0.5f, kArousalIntervalMax = 20.0f;
		static constexpr float kReachMin = 256.0f, kReachMax = 8192.0f;
		static constexpr float kAmountMin = 0.0f, kAmountMax = 12.0f;
		static constexpr float kStrengthMin = 0.2f, kStrengthMax = 2.0f;
		static constexpr int kPulsesMin = 1, kPulsesMax = 20;
		static constexpr float kOverflowMin = 0.0f, kOverflowMax = 1.0f;
		static constexpr float kViscosityMin = 0.0f, kViscosityMax = 100.0f;
		static constexpr float kCohesionMin = 0.0f, kCohesionMax = 20.0f;
		static constexpr float kStretchMin = 0.0f, kStretchMax = 30.0f;
		static constexpr float kStickinessMin = 0.0f, kStickinessMax = 5.0f;
		static constexpr float kOpacityMin = 0.05f, kOpacityMax = 1.0f;
		static constexpr float kLifetimeMin = 5.0f, kLifetimeMax = 300.0f;
		static constexpr float kFadeMin = 0.5f, kFadeMax = 30.0f;
		static constexpr int kParticlesMin = 64, kParticlesMax = 4096;
		static constexpr float kWaitMin = 0.0f, kWaitMax = 2.0f;
		static constexpr float kThicknessMin = 0.5f, kThicknessMax = 2.0f;
		static constexpr float kMilkAmountMin = 0.0f, kMilkAmountMax = 6.0f;
		static constexpr float kMilkStrengthMin = 0.2f, kMilkStrengthMax = 2.5f;
		static constexpr float kNippleOffsetMin = -2.0f, kNippleOffsetMax = 4.0f;
		static constexpr int kFullPercentMin = 1, kFullPercentMax = 100;
		static constexpr float kFullIntervalMin = 1.0f, kFullIntervalMax = 60.0f;
		static constexpr int kKeyMax = 255;
		bool Load();
		bool Save() const;
		void Validate();

	private:
		bool ReadFile(const char* a_path);
		void Report() const;
	};
}
