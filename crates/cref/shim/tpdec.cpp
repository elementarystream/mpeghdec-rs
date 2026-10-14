// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libMpegTPDec (tpdec_lib.h). The transport decoder is driven with recording dummy
// callbacks; its complete internal state and the callback log are exported as INT arrays whose
// layout is mirrored by crates/cref/tests/tpdec_common/mod.rs.

#include "tpdec_lib.h"
#include "FDK_crc.h"

/* Copy of the private definitions in tpdec_lib.cpp (identical definition, used to read the
   state). */
typedef union {
  struct {
    INT bufferFullness;
    UINT endOfFrame;
    UINT mhasLabels[TPDEC_MAX_TRACKS];
    UINT substreamState[TPDEC_MAX_TRACKS];
    UINT flags[TPDEC_MAX_TRACKS];
  } mhas;
} transportdec_parser_t;

struct TRANSPORTDEC {
  TRANSPORT_TYPE transportFmt;
  CSTpCallBacks callbacks;
  FDK_BITSTREAM bitStream[TPDEC_MAX_LAYERS];
  UCHAR* bsBuffer;
  transportdec_parser_t parser;
  CSAudioSpecificConfig asc[CSAUDIOSPECIFICCONFIG_SIZE];
  CCtrlCFGChange ctrlCFGChange[TPDEC_MAX_TRACKS];
  AUDIO_SCENE_INFO* pASI;
  UINT globalFramePos;
  UINT accessUnitAnchor[TPDEC_MAX_LAYERS];
  INT auLength[TPDEC_MAX_LAYERS];
  INT numberOfRawDataBlocks;
  UINT avgBitRate;
  UINT lastValidBufferFullness;
  INT remainder;
  INT missingAccessUnits;
  UINT burstPeriod;
  UINT holdOffFrames;
  UINT flags;
  INT targetLayout;
  INT* pLoudnessInfoSetPosition;
};

/* Registration mask bits of optional callbacks */
#define REG_TRUNC 1
#define REG_UNIDRC 2
#define REG_UI 4
#define REG_EARCON_BS 8
#define REG_EARCON_CFG 16
#define REG_EARCON_INFO 32
#define REG_ASI 64

struct Ctx {
  HANDLE_TRANSPORTDEC tp;
  INT* log;
  INT logCap;
  INT logLen;
  UINT counter;
  UINT failMask;
  UINT failPeriod;
  UINT failPhase;
  AUDIO_SCENE_INFO asi;
  INT pos[3];
  INT pointerQuirks; /* signal group speaker pointers into another ASC */
  INT fullAscLog;    /* debugging: log the complete ASC instead of a checksum */
  INT ascBuf[40000]; /* scratch for ASC dumps in callbacks (no statics: tests run in threads) */
};

static void logv(Ctx* c, INT v) {
  if (c->logLen < c->logCap) c->log[c->logLen] = v;
  c->logLen++;
}

/* Returns the callback result: 0 or TRANSPORTDEC_UNKOWN_ERROR according to the fail schedule. */
static INT failRet(Ctx* c, int type) {
  UINT n = c->counter++;
  if ((c->failMask >> type) & 1) {
    if (c->failPeriod != 0 && (n % c->failPeriod) == c->failPhase) return TRANSPORTDEC_UNKOWN_ERROR;
  }
  return 0;
}

static UINT hashBytes(const UCHAR* p, int n) {
  UINT h = 0;
  for (int i = 0; i < n; i++) h = h * 31 + p[i];
  return h;
}

/* ---- state dump ---- */

static INT* putSpeaker(INT* o, const mpegh_speaker_t* s) {
  *o++ = s->Lfe;
  *o++ = s->Az;
  *o++ = s->El;
  return o;
}

/* Classify the speakers pointer of a signal group; *avail receives the number of speakers that
   can be read in bounds of the array it points to. */
static void classifySpeakers(Ctx* c, const CSUsacConfig* own, const mpegh_speaker_t* p, INT* kind,
                             INT* idx, INT* avail) {
  *kind = 0;
  *idx = 0;
  *avail = 0;
  if (p == NULL) return;
  for (int k = -1; k < CSAUDIOSPECIFICCONFIG_SIZE; k++) {
    const CSUsacConfig* u =
        (k < 0) ? own : (c->tp ? &c->tp->asc[k].m_sc.m_usacConfig : (const CSUsacConfig*)NULL);
    if (u == NULL) continue;
    if (p >= u->speakers && p < u->speakers + TP_USAC_MAX_SPEAKERS) {
      *kind = 1;
      *idx = (INT)(p - u->speakers);
      *avail = TP_USAC_MAX_SPEAKERS - *idx;
    } else if (p >= u->m_signalGroupTypeSpeakers && p < u->m_signalGroupTypeSpeakers + 2 * 28) {
      *kind = 2;
      *idx = (INT)(p - u->m_signalGroupTypeSpeakers);
      *avail = 2 * 28 - *idx;
    } else {
      continue;
    }
    if (u != own) c->pointerQuirks++;
    return;
  }
  *kind = 3;
}

static INT* putAsc(Ctx* c, INT* o, const CSAudioSpecificConfig* a) {
  const CSUsacConfig* u = &a->m_sc.m_usacConfig;
  *o++ = u->m_frameLengthFlag;
  *o++ = u->m_coreSbrFrameLengthIndex;
  *o++ = u->m_sbrRatioIndex;
  *o++ = u->m_nUsacChannels;
  *o++ = u->m_channelConfigurationIndex;
  *o++ = (INT)u->m_usacNumElements;
  for (int i = 0; i < TP_USAC_MAX_ELEMENTS; i++) {
    const CSUsacElementConfig* e = &u->element[i];
    *o++ = e->usacElementType;
    *o++ = e->fullbandLpd;
    *o++ = e->lpdStereoIndex;
    *o++ = e->m_noiseFilling;
    *o++ = e->enhancedNoiseFilling;
    *o++ = e->igfUseEnf;
    *o++ = e->igfUseWhitening;
    *o++ = e->igfAfterTnsSynth;
    *o++ = e->igfIndependentTiling;
    *o++ = e->igfStartIndex;
    *o++ = e->igfUseHighRes;
    *o++ = e->igfStopIndex;
    *o++ = e->igfNTiles;
    *o++ = e->shiftIndex1;
    *o++ = e->shiftChannel1;
    const CSUsacExtElementConfig* x = &e->extElement;
    *o++ = x->usacExtElementType;
    *o++ = x->usacExtElementDefaultLength;
    *o++ = x->usacExtElementPayloadFrag;
    *o++ = x->usacExtElementHasAudioPreRoll;
    if (x->usacExtElementType == ID_EXT_ELE_OBJ_METADATA) {
      *o++ = x->extConfig.oam.OAMframeLength;
      *o++ = x->extConfig.oam.lowDelayMetadataCoding;
      *o++ = x->extConfig.oam.hasDynamicObjectPriority;
      *o++ = x->extConfig.oam.hasUniformSpread;
      *o++ = x->extConfig.oam.numObjectSignals;
    } else if (x->usacExtElementType == ID_EXT_ELE_MCT) {
      *o++ = (INT)x->extConfig.mct.mctChanMask;
    } else if (x->usacExtElementType == ID_EXT_ELE_PROD_METADATA) {
      *o++ = (INT)x->extConfig.prodMetadata.hasObjectDistance;
      *o++ = x->extConfig.prodMetadata.hasReferenceDistance;
      *o++ = x->extConfig.prodMetadata.bsReferenceDistance;
    }
  }
  for (int g = 0; g < TP_MPEGH_MAX_SIGNAL_GROUPS; g++) {
    const CSSignalGroup* s = &u->m_signalGroupType[g];
    INT kind, idx, avail;
    *o++ = s->type;
    *o++ = s->count;
    *o++ = s->Layout;
    classifySpeakers(c, u, s->speakers, &kind, &idx, &avail);
    *o++ = kind;
    *o++ = idx;
    *o++ = s->firstSigIdx;
    *o++ = s->bUseCustomDownmixMatrix;
    int n = s->count < avail ? s->count : avail;
    for (int i = 0; i < n; i++) o = putSpeaker(o, &s->speakers[i]);
  }
  for (int i = 0; i < 2 * 28; i++) o = putSpeaker(o, &u->m_signalGroupTypeSpeakers[i]);
  *o++ = u->referenceLayout;
  for (int i = 0; i < TP_USAC_MAX_SPEAKERS; i++) o = putSpeaker(o, &u->speakers[i]);
  *o++ = u->bsNumSignalGroups;
  *o++ = u->mpegh3daProfileLevelIndication;
  *o++ = u->numAudioObjects;
  *o++ = u->numHOATransportChannels;
  *o++ = u->chNumSignalGroups;
  *o++ = u->objNumSignalGroups;
  for (int i = 0; i < TP_MPEGH_MAX_SIGNAL_GROUPS; i++) *o++ = u->objSignalGroupsIndices[i];
  for (int i = 0; i < TP_MPEGH_MAX_SIGNAL_GROUPS; i++) *o++ = u->sigIdx2GrpIdx[i];
  *o++ = u->passiveDownmixFlag;
  *o++ = u->phaseAlignStrength;
  *o++ = u->immersiveDownmixFlag;
  *o++ = u->downmixConfigType;
  *o++ = u->downmixIdCount;
  *o++ = u->subStreamIndex;
  *o++ = u->targetLayout;
  *o++ = u->uiManagerActive;
  *o++ = u->numAudioChannels;
  *o++ = u->m_usacConfigExtensionPresent;
  *o++ = u->elementLengthPresent;

  *o++ = a->m_aot;
  *o++ = (INT)a->m_samplingFrequency;
  *o++ = (INT)a->m_samplesPerFrame;
  *o++ = (INT)a->m_directMapping;
  *o++ = a->m_extensionAudioObjectType;
  *o++ = (INT)a->m_extensionSamplingFrequency;
  *o++ = a->m_channelConfiguration;
  *o++ = a->m_epConfig;
  *o++ = a->m_vcb11Flag;
  *o++ = a->m_rvlcFlag;
  *o++ = a->m_hcrFlag;
  *o++ = a->m_sbrPresentFlag;
  *o++ = a->m_psPresentFlag;
  *o++ = a->m_samplingFrequencyIndex;
  *o++ = a->m_extensionSamplingFrequencyIndex;
  *o++ = a->m_extensionChannelConfiguration;
  *o++ = a->configMode;
  *o++ = a->AacConfigChanged;
  *o++ = a->SbrConfigChanged;
  *o++ = a->SacConfigChanged;
  return o;
}

static INT* putDescription(INT* o, const ASI_DESCRIPTION* d) {
  *o++ = d->present;
  *o++ = d->numLanguages;
  *o++ = d->prefLangIdx;
  for (int i = 0; i < ASI_MAX_DESCRIPTION_LANGUAGES; i++) {
    for (int k = 0; k < 3; k++) *o++ = d->language[i][k];
    for (int k = 0; k < ASI_MAX_STORED_DESCRIPTION_LEN + 1; k++) *o++ = d->data[i][k];
  }
  return o;
}

static INT* putScreen(INT* o, const ASI_PRODUCTION_SCREEN_SIZE_DATA* s) {
  *o++ = s->hasNonStandardScreenSize;
  *o++ = s->isCenteredInAzimuth;
  *o++ = s->bsScreenSizeAz;
  *o++ = s->bsScreenSizeLeftAz;
  *o++ = s->bsScreenSizeRightAz;
  *o++ = s->bsScreenSizeTopEl;
  *o++ = s->bsScreenSizeBottomEl;
  return o;
}

static INT* putCondition(INT* o, const ASI_GROUP_PRESET_CONDITION* c) {
  *o++ = c->referenceID;
  *o++ = c->conditionOnOff;
  *o++ = c->disableGainInteractivity;
  *o++ = c->gainFlag;
  *o++ = c->gain;
  *o++ = c->disablePositionInteractivity;
  *o++ = c->positionFlag;
  *o++ = c->azOffset;
  *o++ = c->elOffset;
  *o++ = c->distFactor;
  *o++ = c->isSwitchGroupCondition;
  return o;
}

static INT* putAsi(INT* o, const AUDIO_SCENE_INFO* a) {
  for (int i = 0; i < 16; i++) *o++ = a->isMainStream[i];
  *o++ = a->audioSceneInfoID;
  *o++ = a->numGroups;
  for (int g = 0; g < ASI_MAX_GROUPS; g++) {
    const ASI_GROUP* p = &a->groups[g];
    *o++ = p->groupID;
    *o++ = p->allowOnOff;
    *o++ = p->defaultOnOff;
    *o++ = p->allowPositionInteractivity;
    *o++ = p->interactivityMinAzOffset;
    *o++ = p->interactivityMaxAzOffset;
    *o++ = p->interactivityMinElOffset;
    *o++ = p->interactivityMaxElOffset;
    *o++ = p->interactivityMinDistFactor;
    *o++ = p->interactivityMaxDistFactor;
    *o++ = p->allowGainInteractivity;
    *o++ = p->interactivityMinGain;
    *o++ = p->interactivityMaxGain;
    *o++ = p->isAvailable;
    *o++ = p->priority;
    *o++ = p->closestSpeakerPlayout;
    *o++ = p->numMembers;
    *o++ = p->hasConjunctMembers;
    *o++ = p->startID;
    for (int i = 0; i < ASI_MAX_GROUP_MEMBERS; i++) *o++ = p->metaDataElementID[i];
    *o++ = p->extPresent;
    *o++ = p->contPresent;
    *o++ = p->contentData.contentKind;
    for (int k = 0; k < 3; k++) *o++ = p->contentData.contentLanguage[k];
    *o++ = p->switchGroupID;
  }
  *o++ = a->numSwitchGroups;
  for (int s = 0; s < ASI_MAX_SWITCH_GROUPS; s++) {
    const ASI_SWITCH_GROUP* p = &a->switchGroups[s];
    *o++ = p->switchGroupID;
    *o++ = p->allowOnOff;
    *o++ = p->defaultOnOff;
    *o++ = p->numMembers;
    for (int i = 0; i < ASI_MAX_SWITCH_GROUP_MEMBERS; i++) *o++ = p->memberID[i];
    *o++ = p->defaultGroupID;
  }
  *o++ = a->numGroupPresets;
  for (int s = 0; s < ASI_MAX_GROUP_PRESETS; s++) {
    const ASI_GROUP_PRESET* p = &a->groupPresets[s];
    *o++ = p->groupPresetID;
    *o++ = p->kind;
    *o++ = p->numConditions;
    for (int i = 0; i < ASI_MAX_GROUP_PRESET_CONDITIONS; i++) o = putCondition(o, &p->conditions[i]);
    *o++ = p->extPresent;
    *o++ = p->hasDownmixIdExtension;
    *o++ = p->downmixIdExtension.downmixId;
    *o++ = p->downmixIdExtension.numConditions;
    for (int i = 0; i < ASI_MAX_GROUP_PRESET_CONDITIONS; i++)
      o = putCondition(o, &p->downmixIdExtension.conditions[i]);
    o = putScreen(o, &p->productionScreenSize);
  }
  *o++ = a->pDescriptions != NULL;
  if (a->pDescriptions) {
    for (int i = 0; i < ASI_MAX_GROUPS; i++) o = putDescription(o, &a->pDescriptions->groups[i]);
    for (int i = 0; i < ASI_MAX_SWITCH_GROUPS; i++)
      o = putDescription(o, &a->pDescriptions->switchGroups[i]);
    for (int i = 0; i < ASI_MAX_GROUP_PRESETS; i++)
      o = putDescription(o, &a->pDescriptions->groupPresets[i]);
  }
  for (int i = 0; i < 16; i++) *o++ = a->metaDataElementIDoffset[i];
  for (int i = 0; i < 16; i++) *o++ = a->metaDataElementIDmaxAvail[i];
  *o++ = a->compPairsPresent;
  *o++ = a->numCompositePairs;
  for (int i = 0; i < ASI_MAX_COMPOSITE_PAIRS; i++) {
    *o++ = a->compositePairs[i][0];
    *o++ = a->compositePairs[i][1];
  }
  *o++ = a->scrnSizePresent;
  *o++ = a->scrnSizeExtPresent;
  o = putScreen(o, &a->productionScreenSizeData);
  *o++ = a->loudCompPresent;
  const ASI_LOUDNESS_COMPENSATION_DATA* l = &a->loudnessCompensationData;
  *o++ = l->groupLoudnessPresent;
  for (int i = 0; i < ASI_MAX_GROUPS; i++) *o++ = l->groupLoudness[i];
  *o++ = l->defaultParamsPresent;
  for (int i = 0; i < ASI_MAX_GROUPS; i++) *o++ = l->defaultIncludeGroup[i];
  *o++ = l->defaultMinMaxGainPresent;
  *o++ = l->defaultMinGain;
  *o++ = l->defaultMaxGain;
  for (int p = 0; p < ASI_MAX_GROUP_PRESETS; p++) {
    *o++ = l->presetParamsPresent[p];
    for (int i = 0; i < ASI_MAX_GROUPS; i++) *o++ = l->presetIncludeGroup[p][i];
    *o++ = l->presetMinMaxGainPresent[p];
    *o++ = l->presetMinGain[p];
    *o++ = l->presetMaxGain[p];
  }
  *o++ = a->drcUiInfoPresent;
  *o++ = a->drcUiInfo.numTargetLoudnessConditions;
  for (int i = 0; i < ASI_MAX_TARGET_LOUDNESS_CONDITIONS; i++) {
    *o++ = a->drcUiInfo.bsTargetLoudnessValueUpper[i];
    *o++ = a->drcUiInfo.drcSetEffectAvailable[i];
  }
  *o++ = a->sceneDispPresent;
  *o++ = (INT)a->diffFlags;
  for (int i = 0; i < ASI_MAX_PREF_DESCR_LANGUAGES; i++)
    for (int k = 0; k < 3; k++) *o++ = a->prefDescrLanguages[i][k];
  *o++ = a->activeDmxId;
  *o++ = a->crcForUid;
  *o++ = a->crcForCompare;
  return o;
}

static INT* putCtrl(INT* o, const CCtrlCFGChange* c) {
  *o++ = (INT)hashBytes(c->Mpegh3daConfig, TP_MPEGH3DA_MAX_CONFIG_LEN);
  *o++ = (INT)c->Mpegh3daConfigLen;
  *o++ = c->flushCnt;
  *o++ = c->flushStatus;
  *o++ = c->buildUpCnt;
  *o++ = c->buildUpStatus;
  *o++ = c->cfgChanged;
  *o++ = c->contentChanged;
  *o++ = c->forceCfgChange;
  *o++ = c->truncationPresent;
  *o++ = c->forceCrossfade;
  return o;
}

static INT* putBs(INT* o, const FDK_BITSTREAM* b) {
  *o++ = (INT)b->CacheWord;
  *o++ = (INT)b->BitsInCache;
  *o++ = (INT)b->hBitBuf.BitNdx;
  *o++ = b->hBitBuf.ValidBits;
  *o++ = (INT)b->hBitBuf.ReadOffset;
  *o++ = (INT)b->hBitBuf.WriteOffset;
  *o++ = (INT)b->hBitBuf.bufSize;
  *o++ = (INT)b->ConfigCache;
  return o;
}

static INT ascChecksum(Ctx* c, const CSAudioSpecificConfig* a) {
  INT* buf = c->ascBuf;
  INT* e = putAsc(c, buf, a);
  UINT h = 0;
  for (INT* p = buf; p < e; p++) h = h * 1000003u + (UINT)*p;
  return (INT)h;
}

/* ---- callbacks ---- */

static INT cbUpdateConfig(void* self, const CSAudioSpecificConfig* asc, const UCHAR configMode,
                          UCHAR* configChanged) {
  Ctx* c = (Ctx*)self;
  logv(c, 1);
  logv(c, configMode);
  logv(c, *configChanged);
  if (c->fullAscLog) {
    INT* buf = c->ascBuf;
    INT* e = putAsc(c, buf, asc);
    for (INT* p = buf; p < e; p++) logv(c, *p);
  } else {
    logv(c, ascChecksum(c, asc));
  }
  if ((configMode & AC_CM_DET_CFG_CHANGE) && (c->counter % 7) == 3) *configChanged = 1;
  return failRet(c, 1);
}

static INT cbFreeMem(void* self, const CSAudioSpecificConfig* asc) {
  Ctx* c = (Ctx*)self;
  logv(c, 2);
  logv(c, asc->configMode);
  logv(c, asc->m_sc.m_usacConfig.subStreamIndex);
  logv(c, asc->m_aot);
  return failRet(c, 2);
}

static INT cbCtrlCFGChange(void* self, const CCtrlCFGChange* p) {
  Ctx* c = (Ctx*)self;
  INT tmp[16];
  logv(c, 3);
  INT* e = putCtrl(tmp, p);
  for (INT* q = tmp; q < e; q++) logv(c, *q);
  return failRet(c, 3);
}

static INT cbTruncationMsg(void* self, INT nTruncSamples, INT truncFromBegin) {
  Ctx* c = (Ctx*)self;
  logv(c, 4);
  logv(c, nTruncSamples);
  logv(c, truncFromBegin);
  return failRet(c, 4);
}

static INT cbUniDrc(void* self, HANDLE_FDK_BITSTREAM hBs, const INT fullPayloadLength,
                    const INT payloadType, const INT subStreamIndex, const INT payloadStart,
                    const AUDIO_OBJECT_TYPE aot) {
  Ctx* c = (Ctx*)self;
  logv(c, 5);
  logv(c, hBs != NULL);
  logv(c, hBs ? FDKgetValidBits(hBs) : 0);
  logv(c, fullPayloadLength);
  logv(c, payloadType);
  logv(c, subStreamIndex);
  logv(c, payloadStart);
  logv(c, aot);
  if (hBs != NULL) {
    if (payloadType == 1) {
      c->pos[1] = payloadStart;
      c->pos[2] = fullPayloadLength;
    }
    /* consume part of the payload like a real parser would */
    INT n = (INT)((c->counter * 13u) % (UINT)(fullPayloadLength * 8 + 1));
    while (n > 0) {
      INT k = n > 32 ? 32 : n;
      logv(c, (INT)FDKreadBits(hBs, k));
      n -= k;
    }
  }
  return failRet(c, 5);
}

static INT cbUserInteract(void* self, HANDLE_FDK_BITSTREAM hBs, const mha_pactyp_t type,
                          const UINT len) {
  Ctx* c = (Ctx*)self;
  logv(c, 6);
  logv(c, type);
  logv(c, (INT)len);
  logv(c, FDKgetValidBits(hBs));
  INT n = (INT)(c->counter % 3) * 8;
  if (n <= ((INT)len - 2) * 8) logv(c, (INT)FDKreadBits(hBs, n));
  return failRet(c, 6);
}

static INT cbParseDmxMatrix(void* self, HANDLE_FDK_BITSTREAM hBs, CSUsacConfig* usc) {
  Ctx* c = (Ctx*)self;
  logv(c, 7);
  logv(c, FDKgetValidBits(hBs));
  logv(c, usc->targetLayout);
  logv(c, usc->bsNumSignalGroups);
  logv(c, usc->downmixConfigType);
  logv(c, (INT)FDKreadBits(hBs, 8));
  usc->m_signalGroupType[0].bUseCustomDownmixMatrix = c->counter & 1;
  return failRet(c, 7);
}

static INT cbEarconBS(void* self, HANDLE_FDK_BITSTREAM bs) {
  Ctx* c = (Ctx*)self;
  logv(c, 8);
  logv(c, FDKgetValidBits(bs));
  c->counter++;
  return 0;
}

static INT cbEarconConfig(void* self, EarconConfig* cf) {
  Ctx* c = (Ctx*)self;
  logv(c, 9);
  logv(c, (INT)cf->m_numPcmSignals);
  logv(c, (INT)cf->m_bsPcmFrameSize_index);
  logv(c, (INT)cf->m_bsPcmFrameSize);
  logv(c, (INT)cf->m_bsPcmLoudnessValue);
  logv(c, (INT)cf->m_pcmHasAttenuationGain);
  logv(c, (INT)cf->m_bsPcmAttenuationGain);
  logv(c, (INT)cf->m_bsPcmTruncationLength);
  logv(c, (INT)cf->m_bsMHASTruncationLength);
  logv(c, (INT)cf->sampling_frequency);
  logv(c, (INT)cf->TruncationFlag);
  logv(c, (INT)cf->EarconFlag);
  logv(c, (INT)cf->EarconParseError);
  c->counter++;
  return 0;
}

static INT cbEarconInfo(void* self, EarconInfo* info) {
  Ctx* c = (Ctx*)self;
  logv(c, 10);
  logv(c, (INT)info->m_numEarcons);
  c->counter++;
  return 0;
}

extern "C" {

void* cref_tp_open(INT fmt, UINT nrOfLayers, UINT regMask, UINT failMask, UINT failPeriod, UINT failPhase) {
  Ctx* c = (Ctx*)FDKcalloc(1, sizeof(Ctx));
  c->logCap = 1 << 22;
  c->log = (INT*)FDKcalloc(c->logCap, sizeof(INT));
  c->failMask = failMask;
  c->failPeriod = failPeriod;
  c->failPhase = failPhase;
  c->asi.pDescriptions = (ASI_DESCRIPTIONS*)FDKcalloc(1, sizeof(ASI_DESCRIPTIONS));
  c->tp = transportDec_Open((TRANSPORT_TYPE)fmt, 0, nrOfLayers);
  if (c->tp == NULL) return c;
  transportDec_RegisterAscCallback(c->tp, cbUpdateConfig, c);
  transportDec_RegisterFreeMemCallback(c->tp, cbFreeMem, c);
  transportDec_RegisterCtrlCFGChangeCallback(c->tp, cbCtrlCFGChange, c);
  transportDec_RegisterTruncationMsgCallback(c->tp, (regMask & REG_TRUNC) ? cbTruncationMsg : NULL, c);
  transportDec_RegisterUniDrcConfigCallback(c->tp, (regMask & REG_UNIDRC) ? cbUniDrc : NULL, c, c->pos);
  transportDec_RegisterUserInteractCallback(c->tp, (regMask & REG_UI) ? cbUserInteract : NULL, c);
  transportDec_RegisterParseDmxMatrixCallback(c->tp, cbParseDmxMatrix, c);
  transportDec_RegisterEarconSetBSCallBack(c->tp, (regMask & REG_EARCON_BS) ? cbEarconBS : NULL, (regMask & REG_EARCON_BS) ? c : NULL);
  transportDec_RegisterEarconConfigCallBack(c->tp, (regMask & REG_EARCON_CFG) ? cbEarconConfig : NULL, (regMask & REG_EARCON_CFG) ? c : NULL);
  transportDec_RegisterEarconInfoCallBack(c->tp, (regMask & REG_EARCON_INFO) ? cbEarconInfo : NULL, c);
  transportDec_SetAsiParsing(c->tp, (regMask & REG_ASI) ? &c->asi : NULL);
  return c;
}
INT cref_tp_isOpen(void* h) { return ((Ctx*)h)->tp != NULL; }
void cref_tp_close(void* h) {
  Ctx* c = (Ctx*)h;
  transportDec_Close(&c->tp);
  FDKfree(c->asi.pDescriptions);
  FDKfree(c->log);
  FDKfree(c);
}
INT cref_tp_fillData(void* h, const UCHAR* buf, UINT size, UINT* bytesValid, INT layer) { return transportDec_FillData(((Ctx*)h)->tp, buf, size, bytesValid, layer); }
INT cref_tp_readAccessUnit(void* h, UINT layer) { return transportDec_ReadAccessUnit(((Ctx*)h)->tp, layer); }
INT cref_tp_endAccessUnit(void* h) { return transportDec_EndAccessUnit(((Ctx*)h)->tp); }
INT cref_tp_getAuBitsRemaining(void* h, UINT layer) { return transportDec_GetAuBitsRemaining(((Ctx*)h)->tp, layer); }
INT cref_tp_getAuBitsTotal(void* h, UINT layer) { return transportDec_GetAuBitsTotal(((Ctx*)h)->tp, layer); }
INT cref_tp_getMissingAccessUnitCount(void* h) {
  INT n = 0;
  transportDec_GetMissingAccessUnitCount(&n, ((Ctx*)h)->tp);
  return n;
}
INT cref_tp_getFormat(void* h) { return transportDec_GetFormat(((Ctx*)h)->tp); }
INT cref_tp_setParam(void* h, INT param, INT value) { return transportDec_SetParam(((Ctx*)h)->tp, (TPDEC_PARAM)param, value); }
void* cref_tp_getBitstream(void* h, UINT layer) { return transportDec_GetBitstream(((Ctx*)h)->tp, layer); }
INT cref_tp_outOfBandConfig(void* h, UCHAR* conf, UINT length, UINT layer) { return transportDec_OutOfBandConfig(((Ctx*)h)->tp, conf, length, layer); }
INT cref_tp_inBandConfig(void* h, UCHAR* conf, UINT length, UINT buildUpStatus, UINT* configChanged, UINT layer) {
  UCHAR cc = (UCHAR)*configChanged;
  INT err = transportDec_InBandConfig(((Ctx*)h)->tp, conf, length, (UCHAR)buildUpStatus, &cc, layer);
  *configChanged = cc;
  return err;
}
INT cref_tp_crcStartReg(void* h, INT mBits) { return transportDec_CrcStartReg(((Ctx*)h)->tp, mBits); }
INT cref_tp_crcCheck(void* h) { return transportDec_CrcCheck(((Ctx*)h)->tp); }

/* Complete state; returns the number of values written. */
INT cref_tp_state(void* h, INT* out) {
  Ctx* c = (Ctx*)h;
  HANDLE_TRANSPORTDEC t = c->tp;
  INT* o = out;
  *o++ = t->transportFmt;
  for (int i = 0; i < TPDEC_MAX_LAYERS; i++) o = putBs(o, &t->bitStream[i]);
  *o++ = t->parser.mhas.bufferFullness;
  *o++ = (INT)t->parser.mhas.endOfFrame;
  for (int i = 0; i < TPDEC_MAX_TRACKS; i++) *o++ = (INT)t->parser.mhas.mhasLabels[i];
  for (int i = 0; i < TPDEC_MAX_TRACKS; i++) *o++ = (INT)t->parser.mhas.substreamState[i];
  for (int i = 0; i < TPDEC_MAX_TRACKS; i++) *o++ = (INT)t->parser.mhas.flags[i];
  for (int i = 0; i < CSAUDIOSPECIFICCONFIG_SIZE; i++) o = putAsc(c, o, &t->asc[i]);
  for (int i = 0; i < TPDEC_MAX_TRACKS; i++) o = putCtrl(o, &t->ctrlCFGChange[i]);
  *o++ = (INT)t->globalFramePos;
  for (int i = 0; i < TPDEC_MAX_LAYERS; i++) *o++ = (INT)t->accessUnitAnchor[i];
  for (int i = 0; i < TPDEC_MAX_LAYERS; i++) *o++ = t->auLength[i];
  *o++ = t->numberOfRawDataBlocks;
  *o++ = (INT)t->avgBitRate;
  *o++ = (INT)t->lastValidBufferFullness;
  *o++ = t->remainder;
  *o++ = t->missingAccessUnits;
  *o++ = (INT)t->burstPeriod;
  *o++ = (INT)t->holdOffFrames;
  *o++ = (INT)t->flags;
  *o++ = t->targetLayout;
  *o++ = (INT)hashBytes(t->bsBuffer, 65536);
  o = putAsi(o, &c->asi);
  for (int i = 0; i < 3; i++) *o++ = c->pos[i];
  *o++ = (INT)c->counter;
  return (INT)(o - out);
}

/* Copies the callback log (at most cap values) and clears it; returns the log length. */
INT cref_tp_takeLog(void* h, INT* out, INT cap) {
  Ctx* c = (Ctx*)h;
  INT n = c->logLen;
  for (INT i = 0; i < n && i < cap && i < c->logCap; i++) out[i] = c->log[i];
  c->logLen = 0;
  return n;
}
INT cref_tp_pointerQuirks(void* h) { return ((Ctx*)h)->pointerQuirks; }
void cref_tp_setFullAscLog(void* h, INT on) { ((Ctx*)h)->fullAscLog = on; }

/* Direct access to the ASC/ASI parsers on a separate bitstream. */
INT cref_tp_parseConfig(void* h, UCHAR* buf, UINT size, UINT validBits, INT subStreamIndex, INT* out) {
  Ctx* c = (Ctx*)h;
  FDK_BITSTREAM bs;
  CSAudioSpecificConfig asc;
  FDKmemclear(&asc, sizeof(asc));
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = Mpegh3daConfig_Parse(&asc, c->tp->pASI, &bs, 0, &c->tp->callbacks, AC_CM_DET_CFG_CHANGE,
                                 0, c->tp->targetLayout, subStreamIndex, c->pos);
  INT* o = out;
  *o++ = err;
  *o++ = FDKgetValidBits(&bs);
  o = putAsc(c, o, &asc);
  return (INT)(o - out);
}

INT cref_tp_parseAsi(void* h, UCHAR* buf, UINT size, UINT validBits, INT numMaxElementIDs, INT streamIndex, INT* validOut) {
  Ctx* c = (Ctx*)h;
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = mae_AudioSceneInfo(&c->asi, &bs, numMaxElementIDs, streamIndex);
  *validOut = FDKgetValidBits(&bs);
  return err;
}
INT cref_tp_checkAsi(void* h, UINT ascIdx, INT streamIndex) {
  Ctx* c = (Ctx*)h;
  const CSUsacConfig* u = &c->tp->asc[ascIdx].m_sc.m_usacConfig;
  return checkASI(&c->asi, u->bsNumSignalGroups, u->m_signalGroupType, streamIndex);
}
void cref_tp_asiReset(void* h) { asiReset(&((Ctx*)h)->asi); }
void cref_tp_asiResetAvailability(void* h) { asiResetAvailability(&((Ctx*)h)->asi); }
void cref_tp_asiSetPrefLang(void* h, INT idx, const UCHAR* lang) {
  for (int k = 0; k < 3; k++) ((Ctx*)h)->asi.prefDescrLanguages[idx][k] = (char)lang[k];
}
void cref_tp_asiSetActiveDmxId(void* h, INT id) { ((Ctx*)h)->asi.activeDmxId = (UCHAR)id; }
INT cref_tp_asiIds(void* h, INT groupID, INT switchGroupID, INT presetID) {
  Ctx* c = (Ctx*)h;
  return ((asiGroupID2idx(&c->asi, groupID) + 1) << 16) | ((asiSwitchGroupID2idx(&c->asi, switchGroupID) + 1) << 8) |
         (asiGroupPresetID2idx(&c->asi, presetID) + 1);
}
INT cref_tp_isoMap(UCHAR* buf) {
  char b[3] = {(char)buf[0], (char)buf[1], (char)buf[2]};
  asiMapISO639_2_T2B_and_tolower(b);
  INT r = asiCheckISO639_2(b);
  for (int k = 0; k < 3; k++) buf[k] = (UCHAR)b[k];
  return r;
}
INT cref_tp_earconInfo(UCHAR* buf, UINT size, UINT validBits, INT* out) {
  FDK_BITSTREAM bs;
  EarconInfo info;
  FDKmemclear(&info, sizeof(info));
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = earconInfo(&bs, &info);
  out[0] = (INT)info.m_numEarcons;
  out[1] = FDKgetValidBits(&bs);
  return err;
}
INT cref_tp_pcmDataConfig(UCHAR* buf, UINT size, UINT validBits, INT* out) {
  FDK_BITSTREAM bs;
  EarconConfig cf;
  FDKmemclear(&cf, sizeof(cf));
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = pcmDataConfig(&bs, &cf);
  out[0] = (INT)cf.m_numPcmSignals;
  out[1] = (INT)cf.m_bsPcmFrameSize_index;
  out[2] = (INT)cf.m_bsPcmFrameSize;
  out[3] = (INT)cf.m_bsPcmLoudnessValue;
  out[4] = (INT)cf.m_pcmHasAttenuationGain;
  out[5] = (INT)cf.m_bsPcmAttenuationGain;
  out[6] = FDKgetValidBits(&bs);
  return err;
}
INT cref_tp_programConfig(INT* elTypes, INT n, INT* out) {
  CProgramConfig pce;
  CProgramConfig_Init(&pce);
  MP4_ELEMENT_ID elList[64];
  UCHAR chMapping[8], chIndex[8];
  AUDIO_CHANNEL_TYPE chType[8];
  INT* o = out;
  for (int i = 0; i < 64; i++) elList[i] = ID_NONE;
  for (int i = 0; i < n; i++) {
    UCHAR elMapping = 0xAA;
    *o++ = CProgramConfig_LookupElement(&pce, 0, 0, 0, chMapping, chType, chIndex, 8, &elMapping, elList, (MP4_ELEMENT_ID)elTypes[i]);
    *o++ = elMapping;
    *o++ = pce.elCounter;
    if (i == n / 2) CProgramConfig_Reset(&pce);
  }
  *o++ = CProgramConfig_IsValid(&pce);
  for (int i = 0; i < 64; i++) *o++ = elList[i];
  return (INT)(o - out);
}

}  // extern "C"
