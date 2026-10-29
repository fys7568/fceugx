/****************************************************************************
 * Platform Abstraction Layer (OGC driver)
 * Daryl Borth 2009-2026
 * OgcInputDriver.cpp
 *
 * Wii/GameCube controller management and normalization
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ogcsys.h>
#include <unistd.h>
#include <ogc/conf.h>

#ifdef HW_RVL
#include <wiiuse/wpad.h>
#endif

#include "OgcInputDriver.h"
#include "../Platform.h"
#include "../InputController.h"

#ifdef HW_RVL
#include "wii/wiidrc.h"
#include "wii/WiiPlatform.h"
#endif

extern "C" {
s32 __STM_Close();
s32 __STM_Init();
}

bool isWiiVC = false;

OgcInputDriver::OgcInputDriver() {
	for (int i = 0; i < 4; i++) {
		rumbleRequest[i] = false;
		menuRumbleFrames[i] = 0;
		menuRumbleGapFrames[i] = 0;
	}
}

OgcInputDriver::~OgcInputDriver() {
	shutdown();
}

#ifdef HW_RVL
static void WiimotePowerButtonCallback(s32) { NotifyWiiShutdownRequested(); }
#endif

void OgcInputDriver::init() {
	PAD_Init();

	#ifdef HW_RVL
	// Wii Power/Reset buttons
	__STM_Close();
	__STM_Init();
	SYS_SetPowerCallback(NotifyWiiShutdownRequested);

	WiiDRC_Init();
	isWiiVC = WiiDRC_Inited();
	WPAD_Init();
	WPAD_SetDataFormat(WPAD_CHAN_ALL, WPAD_FMT_BTNS_ACC_IR);
	WPAD_SetVRes(WPAD_CHAN_ALL, platform->getVideo()->getScreenWidth(), platform->getVideo()->getScreenHeight());
	WPAD_SetPowerButtonCallback(WiimotePowerButtonCallback);
	#endif

	InitUserInputControllers();
}

void OgcInputDriver::shutdown() {
	for (int i = 0; i < 4; i++) {
		#ifdef HW_RVL
		WPAD_Rumble(i, 0);
		#endif
		PAD_ControlMotor(i, PAD_MOTOR_STOP);
		rumbleRequest[i] = false;
		menuRumbleFrames[i] = 0;
		menuRumbleGapFrames[i] = 0;
	}
}

void OgcInputDriver::setRumble(int channel, bool rumble) {
	if (channel >= 0 && channel < 4) {
		rumbleRequest[channel] = rumble;
	}
}

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/****************************************************************************
 * Hardware Mapping Helpers
 * Translates raw libogc hardware bits to our generic UI masks
 ***************************************************************************/
static uint32_t MapPADToGeneric(uint32_t pad_btns) {
	uint32_t mask = INPUT_BTN_NONE;
	if (pad_btns & PAD_BUTTON_A)      mask |= INPUT_BTN_A;
	if (pad_btns & PAD_BUTTON_B)      mask |= INPUT_BTN_B;
	if (pad_btns & PAD_BUTTON_X)      mask |= INPUT_BTN_X;
	if (pad_btns & PAD_BUTTON_Y)      mask |= INPUT_BTN_Y;
	if (pad_btns & PAD_BUTTON_UP)     mask |= INPUT_BTN_UP;
	if (pad_btns & PAD_BUTTON_DOWN)   mask |= INPUT_BTN_DOWN;
	if (pad_btns & PAD_BUTTON_LEFT)   mask |= INPUT_BTN_LEFT;
	if (pad_btns & PAD_BUTTON_RIGHT)  mask |= INPUT_BTN_RIGHT;
	if (pad_btns & PAD_BUTTON_START)  mask |= INPUT_BTN_PLUS;
	if (pad_btns & PAD_TRIGGER_L)     mask |= INPUT_TRIGGER_L;
	if (pad_btns & PAD_TRIGGER_R)     mask |= INPUT_TRIGGER_R;
	if (pad_btns & PAD_TRIGGER_Z)     mask |= INPUT_TRIGGER_ZR;
	return mask;
}

#ifdef HW_RVL
static uint32_t MapWiimoteToGeneric(uint32_t wpad_btns)
{
	uint32_t mask = INPUT_BTN_NONE;

	if (wpad_btns & WPAD_BUTTON_A)     mask |= INPUT_BTN_A;
	if (wpad_btns & WPAD_BUTTON_B)     mask |= INPUT_BTN_B;
	if (wpad_btns & WPAD_BUTTON_1)     mask |= INPUT_BTN_1;
	if (wpad_btns & WPAD_BUTTON_2)     mask |= INPUT_BTN_2;
	if (wpad_btns & WPAD_BUTTON_UP)    mask |= INPUT_BTN_UP;
	if (wpad_btns & WPAD_BUTTON_DOWN)  mask |= INPUT_BTN_DOWN;
	if (wpad_btns & WPAD_BUTTON_LEFT)  mask |= INPUT_BTN_LEFT;
	if (wpad_btns & WPAD_BUTTON_RIGHT) mask |= INPUT_BTN_RIGHT;
	if (wpad_btns & WPAD_BUTTON_PLUS)  mask |= INPUT_BTN_PLUS;
	if (wpad_btns & WPAD_BUTTON_MINUS) mask |= INPUT_BTN_MINUS;
	if (wpad_btns & WPAD_BUTTON_HOME)  mask |= INPUT_BTN_HOME;

	return mask;
}

static uint32_t MapNunchukToGeneric(uint32_t wpad_btns)
{
	uint32_t mask = INPUT_BTN_NONE;

	if (wpad_btns & WPAD_NUNCHUK_BUTTON_Z) mask |= INPUT_TRIGGER_ZL;
	if (wpad_btns & WPAD_NUNCHUK_BUTTON_C) mask |= INPUT_TRIGGER_L;

	return mask;
}

static uint32_t MapClassicToGeneric(uint32_t wpad_btns)
{
	uint32_t mask = INPUT_BTN_NONE;

	// Classic Controller inputs (upper 16 bits)
	if (wpad_btns & WPAD_CLASSIC_BUTTON_A) mask |= INPUT_BTN_A;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_B) mask |= INPUT_BTN_B;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_X) mask |= INPUT_BTN_X;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_Y) mask |= INPUT_BTN_Y;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_UP) mask |= INPUT_BTN_UP;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_DOWN) mask |= INPUT_BTN_DOWN;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_LEFT) mask |= INPUT_BTN_LEFT;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_RIGHT) mask |= INPUT_BTN_RIGHT;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_PLUS) mask |= INPUT_BTN_PLUS;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_MINUS) mask |= INPUT_BTN_MINUS;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_HOME) mask |= INPUT_BTN_HOME;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_FULL_L) mask |= INPUT_TRIGGER_L;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_FULL_R) mask |= INPUT_TRIGGER_R;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_ZL) mask |= INPUT_TRIGGER_ZL;
	if (wpad_btns & WPAD_CLASSIC_BUTTON_ZR) mask |= INPUT_TRIGGER_ZR;

	return mask;
}

static uint32_t MapWiiUGamepadToGeneric(uint32_t drc_btns) {
	uint32_t mask = INPUT_BTN_NONE;
	if (drc_btns & WIIDRC_BUTTON_A) mask |= INPUT_BTN_A;
	if (drc_btns & WIIDRC_BUTTON_B) mask |= INPUT_BTN_B;
	if (drc_btns & WIIDRC_BUTTON_X) mask |= INPUT_BTN_X;
	if (drc_btns & WIIDRC_BUTTON_Y) mask |= INPUT_BTN_Y;
	if (drc_btns & WIIDRC_BUTTON_UP) mask |= INPUT_BTN_UP;
	if (drc_btns & WIIDRC_BUTTON_DOWN) mask |= INPUT_BTN_DOWN;
	if (drc_btns & WIIDRC_BUTTON_LEFT) mask |= INPUT_BTN_LEFT;
	if (drc_btns & WIIDRC_BUTTON_RIGHT) mask |= INPUT_BTN_RIGHT;
	if (drc_btns & WIIDRC_BUTTON_PLUS) mask |= INPUT_BTN_PLUS;
	if (drc_btns & WIIDRC_BUTTON_MINUS) mask |= INPUT_BTN_MINUS;
	if (drc_btns & WIIDRC_BUTTON_HOME) mask |= INPUT_BTN_HOME;
	if (drc_btns & WIIDRC_BUTTON_L) mask |= INPUT_TRIGGER_L;
	if (drc_btns & WIIDRC_BUTTON_R) mask |= INPUT_TRIGGER_R;
	if (drc_btns & WIIDRC_BUTTON_ZL) mask |= INPUT_TRIGGER_ZL;
	if (drc_btns & WIIDRC_BUTTON_ZR) mask |= INPUT_TRIGGER_ZR;
	return mask;
}

/****************************************************************************
 * Analog Normalization Helpers
 ***************************************************************************/
static float NormalizeWPADAnalog(int pos, int min, int max, int center) {
	if (min == max) return 0.0f;

	// Broken/unusable calibration data (common on 3rd party and clone pads).
	// Do NOT invent a range here: guessing 0..64/32 turns a pad that simply has
	// no analog stick into a permanent full-scale deflection, and the emulator
	// then translates that into a direction key that is always held down.
	// Treat the axis as non-existent instead.
	if ((min >= center) || (max <= center)) {
		return 0.0f;
	}
	int offset = pos - center;
	if (offset > 0) {
		return clampf((float)offset / (float)(max - center), 0.0f, 1.0f);
	} else {
		return clampf((float)offset / (float)(center - min), -1.0f, 0.0f);
	}
}
#endif

/****************************************************************************
 * Analog Axis Plausibility
 *
 * A pad that presents itself as a Classic Controller does not necessarily
 * have analog sticks. Cheap FC/NES replicas (the "China NES controller" that
 * shows up as a Classic Controller) and NES/SFC-to-Wiimote adapters report a
 * zeroed calibration block and park the raw position on a rail. Decoding that
 * yields a full-scale deflection nobody commanded, which the emulator then
 * turns into a D-pad direction that is held down forever - and the button
 * mapping screen becomes unusable because a direction is always "pressed".
 *
 * An axis is trusted once it has actually been seen to move, or while it is
 * properly calibrated and sitting away from the rails. Evaluated per axis, so
 * a pad that only misreports one of them is still usable on the other.
 *
 * Lives outside the HW_RVL block because the GameCube ports need it too.
 ***************************************************************************/
enum { AXIS_X = 0, AXIS_Y = 1, AXIS_RX = 2, AXIS_RY = 3 };

// 4 channels x 4 axes, plus a second block for the GameCube ports.
// noise is the per-axis movement, in raw units, that counts as "the player
// really moved it" - a pad parked on a rail still jitters by a unit or two.
static bool AxisTrusted(int pos, int min, int max, int center, int slot, int noise) {
	static int  last[32];
	static bool known[32];
	static bool moved[32];

	if (slot < 0 || slot >= 32) return true;

	// A genuine stick changes value as soon as it is touched; latch that and
	// stop second-guessing it for the rest of the session.
	if (known[slot]) {
		int delta = pos - last[slot];
		if (delta < 0) delta = -delta;
		if (delta > noise) moved[slot] = true;
	}
	known[slot] = true;
	last[slot] = pos;

	if (moved[slot]) return true;

	bool calOk = (min < center) && (max > center);
	bool atRail = (pos <= min + 2) || (pos >= max - 2);
	return calOk && !atRail;
}

void OgcInputDriver::update() {
	#ifdef HW_RVL
	WPAD_ScanPads();
	WiiDRC_ScanPads();
	bool systemRumbleAllowed = (CONF_GetPadMotorMode() != 0);
	#else
	bool systemRumbleAllowed = true;
	#endif

	u32 activeGamecubePads = PAD_ScanPads();
	bool allowRumble = isRumbleEnabled() && systemRumbleAllowed;

	for(int i = 3; i >= 0; i--) {
		InputPadData padData;

		bool gamecubeActive = (activeGamecubePads & (1 << i)) != 0;

		if(gamecubeActive) {
			padData.hw_connected[INPUT_HW_GAMECUBE] = true;
			padData.hw_buttons_d[INPUT_HW_GAMECUBE] = MapPADToGeneric(PAD_ButtonsDown(i));
			padData.hw_buttons_h[INPUT_HW_GAMECUBE] = MapPADToGeneric(PAD_ButtonsHeld(i));
			padData.hw_buttons_r[INPUT_HW_GAMECUBE] = MapPADToGeneric(PAD_ButtonsUp(i));
			// Same plausibility rule for the GameCube ports: an FC replica with
			// a GameCube plug answers the analog poll with a pegged value.
			int gsx = PAD_StickX(i),  gsy = PAD_StickY(i);
			int grx = PAD_SubStickX(i), gry = PAD_SubStickY(i);

			if(AxisTrusted(gsx, -128, 128, 0, 16 + i * 4 + AXIS_X, 8))
				padData.hw_stickX[INPUT_HW_GAMECUBE] = clampf((float)gsx / 128.0f, -1.0f, 1.0f);
			if(AxisTrusted(gsy, -128, 128, 0, 16 + i * 4 + AXIS_Y, 8))
				padData.hw_stickY[INPUT_HW_GAMECUBE] = clampf((float)gsy / 128.0f, -1.0f, 1.0f);
			if(AxisTrusted(grx, -128, 128, 0, 16 + i * 4 + AXIS_RX, 8))
				padData.hw_substickX[INPUT_HW_GAMECUBE] = clampf((float)grx / 128.0f, -1.0f, 1.0f);
			if(AxisTrusted(gry, -128, 128, 0, 16 + i * 4 + AXIS_RY, 8))
				padData.hw_substickY[INPUT_HW_GAMECUBE] = clampf((float)gry / 128.0f, -1.0f, 1.0f);
		}

		#ifdef HW_RVL
		// Process Wiimote and Extensions
		uint32_t exp_type = WPAD_EXP_NONE;

		if (WPAD_Probe(i, &exp_type) == WPAD_ERR_NONE) {
			WPADData* wpad = WPAD_Data(i);

			// Always process base Wiimote
			padData.hw_connected[INPUT_HW_WIIMOTE] = true;
			padData.battery_level = wpad->battery_level;
			padData.hw_battery_level[INPUT_HW_WIIMOTE] = wpad->battery_level;
			
			padData.hw_gforceX[INPUT_HW_WIIMOTE] = wpad->gforce.x;
			padData.hw_gforceY[INPUT_HW_WIIMOTE] = wpad->gforce.y;
			padData.hw_gforceZ[INPUT_HW_WIIMOTE] = wpad->gforce.z;
			padData.hw_pitch[INPUT_HW_WIIMOTE]   = wpad->orient.pitch;
			padData.hw_roll[INPUT_HW_WIIMOTE]    = wpad->orient.roll;
			padData.hw_yaw[INPUT_HW_WIIMOTE]     = wpad->orient.yaw;

			if (exp_type == WPAD_EXP_NONE) {
				padData.hw_buttons_d[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_d);
				padData.hw_buttons_h[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_h);
				padData.hw_buttons_r[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_u);

				if (wpad->ir.valid) {
					padData.validPointer = true;
					padData.isTouch = false;
					padData.cursor_x = wpad->ir.x;
					padData.cursor_y = wpad->ir.y;
					padData.cursor_angle = wpad->ir.angle;
				}

				controller[i]->setSideways(getWiimoteOrientation() == WIIMOTE_ORIENTATION_HORIZONTAL);
			}
			else if (exp_type == WPAD_EXP_NUNCHUK) {
				padData.hw_buttons_d[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_d);
				padData.hw_buttons_h[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_h);
				padData.hw_buttons_r[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_u);

				padData.hw_connected[INPUT_HW_NUNCHUK] = true;
				padData.hw_buttons_d[INPUT_HW_NUNCHUK] = MapNunchukToGeneric(wpad->btns_d);
				padData.hw_buttons_h[INPUT_HW_NUNCHUK] = MapNunchukToGeneric(wpad->btns_h);
				padData.hw_buttons_r[INPUT_HW_NUNCHUK] = MapNunchukToGeneric(wpad->btns_u);
				joystick_t* js = &wpad->exp.nunchuk.js;
				padData.hw_stickX[INPUT_HW_NUNCHUK] = NormalizeWPADAnalog(js->pos.x, js->min.x, js->max.x, js->center.x);
				padData.hw_stickY[INPUT_HW_NUNCHUK] = NormalizeWPADAnalog(js->pos.y, js->min.y, js->max.y, js->center.y);
				
				padData.hw_gforceX[INPUT_HW_NUNCHUK] = wpad->exp.nunchuk.gforce.x;
				padData.hw_gforceY[INPUT_HW_NUNCHUK] = wpad->exp.nunchuk.gforce.y;
				padData.hw_gforceZ[INPUT_HW_NUNCHUK] = wpad->exp.nunchuk.gforce.z;
				padData.hw_pitch[INPUT_HW_NUNCHUK]   = wpad->exp.nunchuk.orient.pitch;
				padData.hw_roll[INPUT_HW_NUNCHUK]    = wpad->exp.nunchuk.orient.roll;
				padData.hw_yaw[INPUT_HW_NUNCHUK]     = wpad->exp.nunchuk.orient.yaw;
				
				controller[i]->setSideways(false);
			}
			else if (exp_type == WPAD_EXP_CLASSIC) {
				bool isWUPC = (wpad->exp.classic.type == 2);
				int hw = isWUPC ? INPUT_HW_WUPC : INPUT_HW_CLASSIC;

				// Build with -DFCEUGX_PAD_DEBUG to dump what a pad really sends.
				// Needed when buttons come out shifted: the log tells you which
				// bit each physical button lands on.
				#ifdef FCEUGX_PAD_DEBUG
				{
					static unsigned dbgTick = 0;
					if ((dbgTick++ % 60) == 0) {
						const joystick_t& d = wpad->exp.classic.ljs;
						printf("[pad%d] exp=%u type=%d btn=%08x ljs pos=%d,%d min=%d,%d max=%d,%d ctr=%d,%d\n",
							i, (unsigned)exp_type, (int)wpad->exp.classic.type,
							(unsigned)wpad->btns_h, d.pos.x, d.pos.y,
							d.min.x, d.min.y, d.max.x, d.max.y, d.center.x, d.center.y);
					}
				}
				#endif

				padData.hw_connected[hw] = true;
				padData.hw_buttons_d[hw] = MapClassicToGeneric(wpad->btns_d);
				padData.hw_buttons_h[hw] = MapClassicToGeneric(wpad->btns_h);
				padData.hw_buttons_r[hw] = MapClassicToGeneric(wpad->btns_u);

				// Only trust an axis once it has proven itself. A clone FC/NES
				// pad has no sticks at all, and decoding its parked axes holds
				// a direction down forever.
				joystick_t* ljs = &wpad->exp.classic.ljs;
				joystick_t* rjs = &wpad->exp.classic.rjs;
				const int slot = i * 4;

				bool useLX = isWUPC || AxisTrusted(ljs->pos.x, ljs->min.x, ljs->max.x, ljs->center.x, slot + AXIS_X, 2);
				bool useLY = isWUPC || AxisTrusted(ljs->pos.y, ljs->min.y, ljs->max.y, ljs->center.y, slot + AXIS_Y, 2);
				bool useRX = isWUPC || AxisTrusted(rjs->pos.x, rjs->min.x, rjs->max.x, rjs->center.x, slot + AXIS_RX, 2);
				bool useRY = isWUPC || AxisTrusted(rjs->pos.y, rjs->min.y, rjs->max.y, rjs->center.y, slot + AXIS_RY, 2);

				if (useLX) padData.hw_stickX[hw] = NormalizeWPADAnalog(ljs->pos.x, ljs->min.x, ljs->max.x, ljs->center.x);
				if (useLY) padData.hw_stickY[hw] = NormalizeWPADAnalog(ljs->pos.y, ljs->min.y, ljs->max.y, ljs->center.y);
				if (useRX) padData.hw_substickX[hw] = NormalizeWPADAnalog(rjs->pos.x, rjs->min.x, rjs->max.x, rjs->center.x);
				if (useRY) padData.hw_substickY[hw] = NormalizeWPADAnalog(rjs->pos.y, rjs->min.y, rjs->max.y, rjs->center.y);
				controller[i]->setSideways(false);
			}
			else {
				// Unrecognised extension (unknown clone / adapter). Its own
				// buttons are unreadable to us, but the Wiimote it is plugged
				// into still works - don't swallow that.
				padData.hw_buttons_d[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_d);
				padData.hw_buttons_h[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_h);
				padData.hw_buttons_r[INPUT_HW_WIIMOTE] = MapWiimoteToGeneric(wpad->btns_u);
				controller[i]->setSideways(false);
			}
		}

		// Process Wii U Gamepad
		if(i == 0 && WiiDRC_Inited() && WiiDRC_Connected()) {
			padData.hw_connected[INPUT_HW_DRC] = true;
			DrcBatteryFromRaw(WiiDRC_Data()->battery, padData.hw_battery_level[INPUT_HW_DRC], padData.hw_charging[INPUT_HW_DRC]);
			padData.hw_buttons_d[INPUT_HW_DRC] = MapWiiUGamepadToGeneric(WiiDRC_ButtonsDown());
			padData.hw_buttons_h[INPUT_HW_DRC] = MapWiiUGamepadToGeneric(WiiDRC_ButtonsHeld());
			padData.hw_buttons_r[INPUT_HW_DRC] = MapWiiUGamepadToGeneric(WiiDRC_ButtonsUp());
			padData.hw_stickX[INPUT_HW_DRC] = clampf((float)WiiDRC_lStickX() / 128.0f, -1.0f, 1.0f);
			padData.hw_stickY[INPUT_HW_DRC] = clampf((float)WiiDRC_lStickY() / 128.0f, -1.0f, 1.0f);
			padData.hw_substickX[INPUT_HW_DRC] = clampf((float)WiiDRC_rStickX() / 128.0f, -1.0f, 1.0f);
			padData.hw_substickY[INPUT_HW_DRC] = clampf((float)WiiDRC_rStickY() / 128.0f, -1.0f, 1.0f);
		}
		#endif

		// Merge into unified aggregate state for UI Elements
		for (uint32_t hw = 0; hw < INPUT_HW_MAX; hw++) {
			if (!padData.hw_connected[hw])
				continue;

			padData.buttons_d |= padData.hw_buttons_d[hw];
			padData.buttons_h |= padData.hw_buttons_h[hw];
			padData.buttons_r |= padData.hw_buttons_r[hw];

			if (std::abs(padData.hw_stickX[hw]) > std::abs(padData.stickX)) padData.stickX = padData.hw_stickX[hw];
			if (std::abs(padData.hw_stickY[hw]) > std::abs(padData.stickY)) padData.stickY = padData.hw_stickY[hw];
			if (std::abs(padData.hw_substickX[hw]) > std::abs(padData.substickX)) padData.substickX = padData.hw_substickX[hw];
			if (std::abs(padData.hw_substickY[hw]) > std::abs(padData.substickY)) padData.substickY = padData.hw_substickY[hw];
			if (std::abs(padData.hw_gforceX[hw]) > std::abs(padData.gforceX)) padData.gforceX = padData.hw_gforceX[hw];
			if (std::abs(padData.hw_gforceY[hw]) > std::abs(padData.gforceY)) padData.gforceY = padData.hw_gforceY[hw];
			if (std::abs(padData.hw_gforceZ[hw]) > std::abs(padData.gforceZ)) padData.gforceZ = padData.hw_gforceZ[hw];
			if (std::abs(padData.hw_pitch[hw]) > std::abs(padData.pitch)) padData.pitch = padData.hw_pitch[hw];
			if (std::abs(padData.hw_roll[hw]) > std::abs(padData.roll)) padData.roll = padData.hw_roll[hw];
			if (std::abs(padData.hw_yaw[hw]) > std::abs(padData.yaw)) padData.yaw = padData.hw_yaw[hw];
		}

		// Push the finalized, merged payload to the controller abstraction
		controller[i]->update(padData, platform->getVideo()->getDeltaTime());
		
		// Menu (hover) rumble: a short tick with an enforced silent gap afterward
		static constexpr int kMenuRumbleOnFrames = 2; // ~33ms motor-on burst
		static constexpr int kMenuRumbleGapFrames = 6; // ~100ms enforced silence after a tick

		if (rumbleRequest[i]) {
			rumbleRequest[i] = false;
			if (menuRumbleFrames[i] == 0 && menuRumbleGapFrames[i] == 0) {
				menuRumbleFrames[i] = kMenuRumbleOnFrames;
			}
		}

		bool wantRumble = menuRumbleFrames[i] > 0;
		bool doRumble = wantRumble && allowRumble;

		if (menuRumbleFrames[i] > 0) {
			menuRumbleFrames[i]--;
			if (menuRumbleFrames[i] == 0) menuRumbleGapFrames[i] = kMenuRumbleGapFrames;
		} else if (menuRumbleGapFrames[i] > 0) {
			menuRumbleGapFrames[i]--;
		}

		#ifdef HW_RVL
		WPAD_Rumble(i, doRumble ? 1 : 0);
		#endif

		if (gamecubeActive || !doRumble) {
			PAD_ControlMotor(i, doRumble ? PAD_MOTOR_RUMBLE : PAD_MOTOR_STOP);
		}
	}
}
