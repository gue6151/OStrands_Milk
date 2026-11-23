#pragma once

namespace OStrands::Milk
{
	// User-facing milk options (OStrands_Milk.ini / OStrands_Milk_User.ini).
	struct Config
	{
		int   opacity{ 100 };        // 0..100
		int   glow{ 40 };            // 0..100 self-lit white
		float interval{ 0.5f };      // 0..60 s
		float strengthMin{ 0.5f };   // 0..60
		float strengthMax{ 12.0f };  // 0..60 (20 was the old top)
		int   mode{ 1 };             // 0 dribbling .. 4 cow in heat
		float flow{ 4.0f };          // 0.5..30 s each flow lasts
		int   streamsMin{ 1 };       // 1..12 streams at the minimum strength
		int   streamsMax{ 3 };       // 1..12 streams at the maximum strength
		float cone{ 15.0f };         // 0..45 degrees fan of the streams
		int   thickness{ 70 };       // 20..150 % stream thickness (puddles unaffected)
		int   stageFrom{ 10 };       // % of SGO max milk where the lightest leak texture starts
		int   stageTo{ 100 };        // % of SGO max milk where the heaviest leak texture starts
		float viscosity{ 3.0f };     // 0..30
		float stickiness{ 0.5f };    // 0..20 on the body
		float stickinessWorld{ 0.5f };  // 0..20 on walls, pillars and furniture
		bool  sgoSync{ true };
		bool  clothed{ false };
		bool  sex{ true };
		float lifetime{ 20.0f };     // 0..60 s
		float fade{ 3.0f };          // 0..10 s
		int   maxParticles{ 2000 };  // 0..3000
		float amount{ 1.6f };        // 0..6
		float nippleForward{ 0.0f }; // -2..4
		float nippleUp{ 0.0f };
		float nippleSide{ 0.0f };
		int   merge{ 60 };           // 0..100: how strongly touching milk merges into one mass
		int   fullPercent{ 100 };    // SGO milk level (% of SGO's own max) where breasts count as full
		bool  overlay{ true };       // tune SGO's MilkLeak skin overlay
		int   overlayR{ 255 };
		int   overlayG{ 255 };
		int   overlayB{ 255 };
		float overlayGloss{ 30.0f };    // 0..100
		float overlaySpecular{ 1.5f };  // 0..10
		float overlayBoost{ 1.8f };     // 1..4
		bool  ramp{ true };            // spray grows with the SGO milk level
		int   rampFull{ 200 };         // SGO % for 100% spray
		int   rampMin{ 10 };           // % of the spray at the leak threshold
		bool  rampAll{ false };        // also SGO milking and sex
		bool  crotch{ true };          // milk drips off the vulva at the top leak stages
		int   crotchStage{ 8 };        // 1..8: leak texture stage where the dripping starts
		float crotchInterval{ 4.0f };  // 1..30 s between drips
		float crotchAmount{ 1.0f };    // 0..3
		float crotchDelay{ 1.5f };     // 0..10 s after the spray starts
		bool  debugLog{ false };

		void Validate();
	};

	Config& Get();
	std::unique_lock<std::recursive_mutex> Lock();
	void Load();
	bool Save();
	// Copies the options into the fluid engine's settings.
	void Apply();

	void RegisterMenu();
	void ConnectSexLab();
	void Update(float a_delta);
	void Reset();
	// Values chosen for a good look at a low frame cost (see README).
	void ApplyRecommended();
}
