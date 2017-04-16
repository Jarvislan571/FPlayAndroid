//
// FPlayAndroid is distributed under the FreeBSD License
//
// Copyright (c) 2013-2014, Carlos Rafael Gimenes das Neves
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
// either expressed or implied, of the FreeBSD Project.
//
// https://github.com/carlosrafaelgn/FPlayAndroid
//

#include "EffectsImplMacros.h"
#include "iir/Iir.h"
#define TAG "Equalizer16Band"
#include <android/log.h>
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG,__VA_ARGS__)
//https://en.wikipedia.org/wiki/Dynamic_range_compression
//https://en.wikipedia.org/wiki/Dynamic_range_compression#Limiting
//as the article states, brick-wall limiting are harsh and unpleasant.. also... reducing the gain abruptly causes audible clicks!
#define GAIN_REDUCTION_PER_SECOND_DB -40.0 //-30.0dB/s
#define GAIN_RECOVERY_PER_SECOND_DB 3.0 //+3.0dB/s
static uint32_t bassBoostStrength, virtualizerStrength;
static int32_t equalizerGainInMillibels[BAND_COUNT];
static EFFECTPROC effectProc;
static float* effectsFloatSamplesOriginal;

uint32_t effectsEnabled, equalizerMaxBandCount, effectsGainEnabled;
int32_t effectsFramesBeforeRecoveringGain,
	effectsMinimumAmountOfFramesToReduce,
	effectsTemp[4] __attribute__((aligned(16))),
	equalizerActuallyUsedGainInMillibels[BAND_COUNT];
float effectsGainRecoveryOne[4] __attribute__((aligned(16))) = { 1.0f, 1.0f, 0.0f, 0.0f },
	effectsGainReductionPerFrame[4] __attribute__((aligned(16))),
	effectsGainRecoveryPerFrame[4] __attribute__((aligned(16))),
	effectsGainClip[4] __attribute__((aligned(16))),
	equalizerLastBandGain[4] __attribute__((aligned(16)));
float preAmpVal = 1;
float *effectsFloatSamples;

#ifdef FPLAY_X86
static const uint32_t effectsAbsSample[4] __attribute__((aligned(16))) = { 0x7FFFFFFF, 0x7FFFFFFF, 0, 0 };
#else
extern void processEffectsNeon(int16_t* buffer, uint32_t sizeInFrames);
#endif
Iir::Butterworth::LowShelf<4,Iir::DirectFormII> BassBoostl;
Iir::Butterworth::LowShelf<4,Iir::DirectFormII> BassBoostr;
Iir::Butterworth::LowShelf<4,Iir::DirectFormII> lsl;
Iir::Butterworth::LowShelf<4,Iir::DirectFormII> lsr;
Iir::Butterworth::BandShelf<4,Iir::DirectFormII> bs1l;
Iir::Butterworth::BandShelf<4,Iir::DirectFormII> bs1r;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs2l;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs2r;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs3l;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs3r;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs4l;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs4r;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs5l;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs5r;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs6l;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs6r;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs7l;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs7r;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs8l;
Iir::Butterworth::BandShelf<3,Iir::DirectFormII> bs8r;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs9l;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs9r;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs10l;
Iir::Butterworth::BandShelf<2,Iir::DirectFormII> bs10r;
Iir::Butterworth::HighShelf<2,Iir::DirectFormII> bs11l;
Iir::Butterworth::HighShelf<2,Iir::DirectFormII> bs11r;
void updateEffectProc();
int32_t JNICALL getCurrentAutomaticEffectsGainInMB(JNIEnv* env, jclass clazz) {
	return ((effectsGainEnabled && effectsEnabled) ? (int32_t)(2000.0 * log10(effectsGainClip[0])) : 0);
}

void JNICALL enableAutomaticEffectsGain(JNIEnv* env, jclass clazz, uint32_t enabled) {
	::effectsGainEnabled = enabled;

	if (!enabled) {
		effectsGainClip[0] = 1.0f;
		effectsGainClip[1] = 1.0f;
		effectsGainClip[2] = 0.0f;
		effectsGainClip[3] = 0.0f;
		effectsFramesBeforeRecoveringGain = 0x7FFFFFFF;
		effectsMinimumAmountOfFramesToReduce = 0;
	}
}	

void resetAutomaticEffectsGain() {
	effectsGainClip[0] = 1.0f;
	effectsGainClip[1] = 1.0f;
	effectsGainClip[2] = 0.0f;
	effectsGainClip[3] = 0.0f;
	effectsFramesBeforeRecoveringGain = 0x7FFFFFFF;
	effectsMinimumAmountOfFramesToReduce = 0;
}

uint32_t JNICALL isAutomaticEffectsGainEnabled(JNIEnv* env, jclass clazz) {
	return effectsGainEnabled;
}
void computeFilter() {
/*	LOGI("PreAmp: %d",equalizerGainInMillibels[0]/100);
	LOGI("Band1: %d",equalizerGainInMillibels[1]/100);
	LOGI("Band2: %d",equalizerGainInMillibels[2]/100);
	LOGI("Band3: %d",equalizerGainInMillibels[3]/100);
	LOGI("Band4: %d",equalizerGainInMillibels[4]/100);
	LOGI("Band5: %d",equalizerGainInMillibels[5]/100);
	LOGI("Band6: %d",equalizerGainInMillibels[6]/100);
	LOGI("Band7: %d",equalizerGainInMillibels[7]/100);
	LOGI("Band8: %d",equalizerGainInMillibels[8]/100);
	LOGI("Band9: %d",equalizerGainInMillibels[9]/100);
	LOGI("Band10: %d",equalizerGainInMillibels[10]/100);
	LOGI("Band11: %d",equalizerGainInMillibels[11]/100);
	LOGI("Band12: %d",equalizerGainInMillibels[12]/100);*/
	preAmpVal = equalizerGainInMillibels[0];
	if(preAmpVal>0.0) {
		preAmpVal = preAmpVal/100;
	}
	if(preAmpVal>6.0) {
		preAmpVal = 6.0;
	}
	if(preAmpVal==0.0)
		preAmpVal = 1;
	if(preAmpVal<0) {
		preAmpVal = 99/abs(preAmpVal);
	}
	lsl.setup(4, dstSampleRate, 32.0, equalizerGainInMillibels[1]/100);
	lsr.setup(4, dstSampleRate, 32.0, equalizerGainInMillibels[1]/100);
	lsl.reset();
	lsr.reset();
	bs1l.setup(4, dstSampleRate, 64.0f, 52.0, equalizerGainInMillibels[2]/100);
	bs1r.setup(4, dstSampleRate, 64.0f, 52.0, equalizerGainInMillibels[2]/100);
	bs1l.reset();
	bs1r.reset();
	bs2l.setup(3, dstSampleRate, 126.0f, 66.0, equalizerGainInMillibels[3]/100);
	bs2r.setup(3, dstSampleRate, 126.0f, 66.0, equalizerGainInMillibels[3]/100);
	bs2l.reset();
	bs2r.reset();
	bs3l.setup(2, dstSampleRate, 220.0f, 110.0, equalizerGainInMillibels[4]/100);
	bs3r.setup(2, dstSampleRate, 220.0f, 110.0, equalizerGainInMillibels[4]/100);
	bs3l.reset();
	bs3r.reset();
	bs4l.setup(2, dstSampleRate, 380.0f, 180.0, equalizerGainInMillibels[5]/100);
	bs4r.setup(2, dstSampleRate, 380.0f, 180.0, equalizerGainInMillibels[5]/100);
	bs4l.reset();
	bs4r.reset();
	bs5l.setup(3, dstSampleRate, 750.0f, 600.0, equalizerGainInMillibels[6]/100);
	bs5r.setup(3, dstSampleRate, 750.0f, 600.0, equalizerGainInMillibels[6]/100);
	bs5l.reset();
	bs5r.reset();
	bs6l.setup(3, dstSampleRate, 1600.0f, 1000.0, equalizerGainInMillibels[7]/100);
	bs6r.setup(3, dstSampleRate, 1600.0f, 1000.0, equalizerGainInMillibels[7]/100);
	bs6l.reset();
	bs6r.reset();
	bs7l.setup(3, dstSampleRate, 3000.0f, 1800.0, equalizerGainInMillibels[8]/100);
	bs7r.setup(3, dstSampleRate, 3000.0f, 1800.0, equalizerGainInMillibels[8]/100);
	bs7l.reset();
	bs7r.reset();
	bs8l.setup(3, dstSampleRate, 4800.0f, 1800.0, equalizerGainInMillibels[9]/100);
	bs8r.setup(3, dstSampleRate, 4800.0f, 1800.0, equalizerGainInMillibels[9]/100);
	bs8l.reset();
	bs8r.reset();
	bs9l.setup(2, dstSampleRate, 7000.0f, 2800.0, equalizerGainInMillibels[10]/100);
	bs9r.setup(2, dstSampleRate, 7000.0f, 2800.0, equalizerGainInMillibels[10]/100);
	bs9l.reset();
	bs9r.reset();
	bs10l.setup(2, dstSampleRate, 11000.0f, 3400.0, equalizerGainInMillibels[11]/100);
	bs10r.setup(2, dstSampleRate, 11000.0f, 3400.0, equalizerGainInMillibels[11]/100);
	bs10l.reset();
	bs10r.reset();
	bs11l.setup(2, dstSampleRate, 15000.0f, equalizerGainInMillibels[12]/100);
	bs11r.setup(2, dstSampleRate, 15000.0f, equalizerGainInMillibels[12]/100);
	bs11l.reset();
	bs11r.reset();
}

void resetEqualizer() {
	memset(effectsTemp, 0, 4 * sizeof(int32_t));
	lsl.reset();
	lsr.reset();
	bs1l.reset();
	bs1r.reset();
	bs2l.reset();
	bs2r.reset();
	bs3l.reset();
	bs3r.reset();
	bs4l.reset();
	bs4r.reset();
	bs5l.reset();
	bs5r.reset();
	bs6l.reset();
	bs6r.reset();
	bs7l.reset();
	bs7r.reset();
	bs8l.reset();
	bs8r.reset();
	bs9l.reset();
	bs9r.reset();
	bs10l.reset();
	bs10r.reset();
	bs11l.reset();
	bs11r.reset();
}

void equalizerConfigChanged() {
	//this only happens in two moments: upon initialization and when the sample rate changes (even when the equalizer is not enabled!)

	if (dstSampleRate > (2 * 6000))
		equalizerMaxBandCount = BAND_COUNT;
	else
		equalizerMaxBandCount = BAND_COUNT - 1; //Android's minimum allowed sample rate is 4000 Hz

	effectsGainReductionPerFrame[0] = (float)pow(10.0, GAIN_REDUCTION_PER_SECOND_DB / (double)(dstSampleRate * 20));
	effectsGainReductionPerFrame[1] = effectsGainReductionPerFrame[0];
	effectsGainRecoveryPerFrame[0] = (float)pow(10.0, GAIN_RECOVERY_PER_SECOND_DB / (double)(dstSampleRate * 20));
	effectsGainRecoveryPerFrame[1] = effectsGainRecoveryPerFrame[0];

	resetAutomaticEffectsGain();
	resetEqualizer();
}
void virtualizerConfigChanged() {
}
void resetVirtualizer() {
}
#define BBFreq 400
void initializeEffects() {
	effectsEnabled = 0;
	bassBoostStrength = 0;
	virtualizerStrength = 0;
	equalizerMaxBandCount = BAND_COUNT;
	effectsFloatSamplesOriginal = 0;
	effectsFloatSamples = 0;
	effectsGainEnabled = 1;
	effectsGainReductionPerFrame[0] = 1.0f;
	effectsGainReductionPerFrame[1] = 1.0f;
	effectsGainReductionPerFrame[2] = 0.0f;
	effectsGainReductionPerFrame[3] = 0.0f;
	effectsGainRecoveryPerFrame[0] = 1.0f;
	effectsGainRecoveryPerFrame[1] = 1.0f;
	effectsGainRecoveryPerFrame[2] = 0.0f;
	effectsGainRecoveryPerFrame[3] = 0.0f;
	equalizerLastBandGain[0] = 1.0f;
	equalizerLastBandGain[1] = 1.0f;
	equalizerLastBandGain[2] = 0.0f;
	equalizerLastBandGain[3] = 0.0f;

	memset(equalizerGainInMillibels, 0, BAND_COUNT * sizeof(int32_t));
	memset(equalizerActuallyUsedGainInMillibels, 0, BAND_COUNT * sizeof(int32_t));
	BassBoostl.setup(4, dstSampleRate, BBFreq, 0);
	BassBoostl.reset();
	BassBoostr.setup(4, dstSampleRate, BBFreq, 0);
	BassBoostr.reset();
	lsl.reset();
	lsr.reset();
	bs1l.reset();
	bs1r.reset();
	bs2l.reset();
	bs2r.reset();
	bs3l.reset();
	bs3r.reset();
	bs4l.reset();
	bs4r.reset();
	bs5l.reset();
	bs5r.reset();
	bs6l.reset();
	bs6r.reset();
	bs7l.reset();
	bs7r.reset();
	bs8l.reset();
	bs8r.reset();
	bs9l.reset();
	bs9r.reset();
	bs10l.reset();
	bs10r.reset();
	bs11l.reset();
	bs11r.reset();

	resetAutomaticEffectsGain();
	resetEqualizer();
	updateEffectProc();
}
void terminateEffects() {
	if (effectsFloatSamplesOriginal) {
		delete effectsFloatSamplesOriginal;
		effectsFloatSamplesOriginal = 0;
		effectsFloatSamples = 0;
	}
}

void processNull(int16_t* buffer, uint32_t sizeInFrames) {
	//nothing to be done :)
}

void processEffects(int16_t* buffer, uint32_t sizeInFrames) {
    //a few buggy devices change the audio sink so fast and so repeatedly,
    //that this case indeed happens... :(
	if (!effectsFloatSamples)
		return;

	if (effectsMinimumAmountOfFramesToReduce <= 0)
		effectsFramesBeforeRecoveringGain -= sizeInFrames;
	else
		effectsMinimumAmountOfFramesToReduce -= sizeInFrames;

	if (!(effectsEnabled & (EQUALIZER_ENABLED | BASSBOOST_ENABLED)) || !equalizerActuallyUsedGainInMillibels[BAND_COUNT - 1]) {
		for (int32_t i = ((sizeInFrames << 1) - 1); i >= 0; i--)
			effectsFloatSamples[i] = (float)buffer[i];
	}
	float* samples = effectsFloatSamples;
	if ((effectsEnabled & EQUALIZER_ENABLED)) {
		for (int32_t i = sizeInFrames - 1; i >= 0; i--) {
			const float inL = samples[0] * preAmpVal;
			const float inR = samples[1] * preAmpVal;
			float outL = lsl.filter(inL);
			float outR = lsr.filter(inR);
			outL = bs1l.filter(outL);
			outR = bs1r.filter(outR);
			outL = bs2l.filter(outL);
			outR = bs2r.filter(outR);
			outL = bs3l.filter(outL);
			outR = bs3r.filter(outR);
			outL = bs4l.filter(outL);
			outR = bs4r.filter(outR);
			outL = bs5l.filter(outL);
			outR = bs5r.filter(outR);
			outL = bs6l.filter(outL);
			outR = bs6r.filter(outR);
			outL = bs7l.filter(outL);
			outR = bs7r.filter(outR);
			outL = bs8l.filter(outL);
			outR = bs8r.filter(outR);
			outL = bs9l.filter(outL);
			outR = bs9r.filter(outR);
			outL = bs10l.filter(outL);
			outR = bs10r.filter(outR);
			outL = bs11l.filter(outL);
			outR = bs11r.filter(outR);
			samples[0] = outL;
			samples[1] = outR;
			samples += 2;
		}
	}
	if ((effectsEnabled & BASSBOOST_ENABLED)) {
		for (int32_t i = sizeInFrames - 1; i >= 0; i--) {
			const float inL = samples[0];
			const float inR = samples[1];
			const float outL = BassBoostl.filter(inL);
			const float outR = BassBoostr.filter(inR);
			samples[0] = outL;
			samples[1] = outR;
			samples += 2;
		}
	}
	float gainClip = effectsGainClip[0];
	float maxAbsSample = 0.0f;
	float* floatSamples = effectsFloatSamples;

	while ((sizeInFrames--)) {
		float inL = floatSamples[0] * gainClip;
		float inR = floatSamples[1] * gainClip;
		floatSamples += 2;

		if (effectsMinimumAmountOfFramesToReduce > 0) {
			gainClip *= effectsGainReductionPerFrame[0];
		} else if (effectsFramesBeforeRecoveringGain <= 0) {
			gainClip *= effectsGainRecoveryPerFrame[0];
			if (gainClip > 1.0f)
				gainClip = 1.0f;
		}
		//abs
		const uint32_t tmpAbsL = *((uint32_t*)&inL) & 0x7FFFFFFF;
		if (maxAbsSample < *((float*)&tmpAbsL))
			maxAbsSample = *((float*)&tmpAbsL);
		const uint32_t tmpAbsR = *((uint32_t*)&inL) & 0x7FFFFFFF;;
		if (maxAbsSample < *((float*)&tmpAbsR))
			maxAbsSample = *((float*)&tmpAbsR);

		const int32_t iL = (int32_t)inL;
		const int32_t iR = (int32_t)inR;
		buffer[0] = (iL >= 32767 ? 32767 : (iL <= -32768 ? -32768 : (int16_t)iL));
		buffer[1] = (iR >= 32767 ? 32767 : (iR <= -32768 ? -32768 : (int16_t)iR));

		buffer += 2;
	}

	if (!effectsGainEnabled) {
		effectsFramesBeforeRecoveringGain = 0x7FFFFFFF;
		effectsMinimumAmountOfFramesToReduce = 0;
		return;
	}

	effectsGainClip[0] = gainClip;
	if (maxAbsSample > MAX_ALLOWED_SAMPLE_VALUE) {
		effectsFramesBeforeRecoveringGain = dstSampleRate << 2; //wait some time before starting to recover the gain
		effectsMinimumAmountOfFramesToReduce = (MAXIMUM_BUFFER_SIZE_IN_FRAMES_FOR_PROCESSING * 3) >> 1;
	} else if (effectsMinimumAmountOfFramesToReduce <= 0) {
		if (effectsGainClip[0] >= 1.0f)
			effectsFramesBeforeRecoveringGain = 0x7FFFFFFF;
	}
}

void JNICALL enableEqualizer(JNIEnv* env, jclass clazz, uint32_t enabled) {
	const uint32_t oldEffects = effectsEnabled;
	if (enabled)
		effectsEnabled |= EQUALIZER_ENABLED;
	else
		effectsEnabled &= ~EQUALIZER_ENABLED;

	if (!oldEffects && effectsEnabled)
		resetAutomaticEffectsGain();

	computeFilter();
	updateEffectProc();
}

uint32_t JNICALL isEqualizerEnabled(JNIEnv* env, jclass clazz) {
	return (effectsEnabled & EQUALIZER_ENABLED);
}

void JNICALL setEqualizerBandLevel(JNIEnv* env, jclass clazz, uint32_t band, int32_t level) {
	if (band >= BAND_COUNT)
		return;

	equalizerGainInMillibels[band] = ((level <= -DB_RANGE) ? -DB_RANGE : ((level >= DB_RANGE) ? DB_RANGE : level));
	computeFilter();
}

void JNICALL setEqualizerBandLevels(JNIEnv* env, jclass clazz, jshortArray jlevels) {
	int16_t* const levels = (int16_t*)env->GetPrimitiveArrayCritical(jlevels, 0);
	if (!levels)
		return;

	for (int32_t i = 0; i < BAND_COUNT; i++)
		equalizerGainInMillibels[i] = ((levels[i] <= -DB_RANGE) ? -DB_RANGE : ((levels[i] >= DB_RANGE) ? DB_RANGE : levels[i]));
	computeFilter();
	env->ReleasePrimitiveArrayCritical(jlevels, levels, JNI_ABORT);
}

void JNICALL enableBassBoost(JNIEnv* env, jclass clazz, uint32_t enabled) {
	const uint32_t oldEffects = effectsEnabled;
	if (enabled)
		effectsEnabled |= BASSBOOST_ENABLED;
	else
		effectsEnabled &= ~BASSBOOST_ENABLED;

	if (!oldEffects && effectsEnabled)
		resetAutomaticEffectsGain();
	updateEffectProc();
}

uint32_t JNICALL isBassBoostEnabled(JNIEnv* env, jclass clazz) {
	return ((effectsEnabled & BASSBOOST_ENABLED) >> 1);
}

void JNICALL setBassBoostStrength(JNIEnv* env, jclass clazz, int32_t strength) {
	bassBoostStrength = ((strength <= 0) ? 0 : ((strength >= 1000) ? 1000 : strength));
	BassBoostl.setup (4, dstSampleRate, BBFreq, bassBoostStrength/100);
	BassBoostl.reset();
	BassBoostr.setup (4, dstSampleRate, BBFreq, bassBoostStrength/100);
	BassBoostr.reset();
}

int32_t JNICALL getBassBoostRoundedStrength(JNIEnv* env, jclass clazz) {
	return bassBoostStrength;
}
void JNICALL enableVirtualizer(JNIEnv* env, jclass clazz, int32_t enabled) {
	const uint32_t oldEffects = effectsEnabled;
	if (enabled)
		effectsEnabled |= VIRTUALIZER_ENABLED;
	else
		effectsEnabled &= ~VIRTUALIZER_ENABLED;

	if (!oldEffects && effectsEnabled)
		resetAutomaticEffectsGain();

	//recreate the filter if the virtualizer is enabled
	if ((effectsEnabled & VIRTUALIZER_ENABLED)) {
	}
	updateEffectProc();
}

uint32_t JNICALL isVirtualizerEnabled(JNIEnv* env, jclass clazz) {
	return ((effectsEnabled & VIRTUALIZER_ENABLED) >> 2);
}

void JNICALL setVirtualizerStrength(JNIEnv* env, jclass clazz, int32_t strength) {
	virtualizerStrength = ((strength <= 0) ? 0 : ((strength >= 1000) ? 1000 : strength));

	//recompute the filter if the virtualizer is enabled
	if ((effectsEnabled & VIRTUALIZER_ENABLED)) {
	}
}

int32_t JNICALL getVirtualizerRoundedStrength(JNIEnv* env, jclass clazz) {
	return virtualizerStrength;
}

void updateEffectProc() {
	if ((effectsEnabled & (EQUALIZER_ENABLED | BASSBOOST_ENABLED | VIRTUALIZER_ENABLED))) {
#ifdef FPLAY_X86
		effectProc = processEffects;
#else
		effectProc = (neonMode ? processEffectsNeon : processEffects);
#endif
		if (!effectsFloatSamplesOriginal) {
			//MAXIMUM_BUFFER_SIZE_IN_FRAMES_FOR_PROCESSING * 2, because audioTrack allows up to MAXIMUM_BUFFER_SIZE_IN_FRAMES_FOR_PROCESSING * 2 frames
			effectsFloatSamplesOriginal = new float[4 + (MAXIMUM_BUFFER_SIZE_IN_FRAMES_FOR_PROCESSING * 2 * 2)];
			//align memory on a 16-byte boundary
			if (((size_t)effectsFloatSamplesOriginal & 15))
				effectsFloatSamples = (float*)((size_t)effectsFloatSamplesOriginal + 16 - ((size_t)effectsFloatSamplesOriginal & 15));
			else
				effectsFloatSamples = effectsFloatSamplesOriginal;
		}
	} else {
		effectProc = processNull;
		if (effectsFloatSamplesOriginal) {
			delete effectsFloatSamplesOriginal;
			effectsFloatSamplesOriginal = 0;
			effectsFloatSamples = 0;
		}
	}
}
