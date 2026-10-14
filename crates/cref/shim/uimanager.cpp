// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libUIManager (uiManager.h, uiXml.h, uiPersistence.h) and libMpeghUIMan
// (mpeghUIManager.h). Internal states are exported as INT arrays whose layout is mirrored by
// crates/cref/tests/uimanager.rs.

#include "mpeghUIManager.h"
#include "uiManager.h"
#include "uiManagerInternal.h"
#include "uiXml.h"
#include "uiPersistence.h"
#include "tpdec_lib.h"

/* Copies of the private definitions in mpeghUIManager.cpp and uiPersistence.cpp (identical
   definitions, used to read the state). */
struct MPEGH_UI_MANAGER {
  HANDLE_UI_MANAGER hUiManager;
  UCHAR isActive;
  UCHAR configFound;
  UINT substreamLabel[MAX_NUMBER_SUBSTREAMS];
  UINT insertOffset;
  UCHAR bs_buffer[64 * 1024];
};

typedef struct {
  UCHAR uuid[16];
  USHORT firstCmdIdx;
  USHORT nextKeyIdx;
} UI_PERSISTENCE_KEY;

typedef struct {
  UCHAR type;
  UCHAR id;
  SHORT param;
  USHORT nextCmdIdx;
  USHORT flags;
} UI_PERSISTENCE_COMMAND;

struct UI_PERSISTENCE_MANAGER {
  void* memory;
  USHORT memSize;
  USHORT maxKeys;
  UI_PERSISTENCE_KEY* keys;
  USHORT maxCommands;
  UI_PERSISTENCE_COMMAND* commands;
  USHORT firstKeyIdx;
  USHORT nextGetCmdIdx;
  UCHAR nextGetCmdFlags;
  UCHAR accessibilitySetting;
  char preferredLanguage[3];
};

/* ---- state dumps ---- */

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

/* same layout as put_asi() in crates/cref/tests/tpdec_common */
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

static INT* putUiState(INT* o, const UI_STATE* s) {
  for (int i = 0; i < 16; i++) *o++ = s->uuid[i];
  for (int g = 0; g < ASI_MAX_GROUPS; g++) {
    const UI_STATE_GROUP* p = &s->groups[g];
    *o++ = p->onOff;
    *o++ = p->allowOnOff;
    *o++ = p->gain;
    *o++ = p->defaultGain;
    *o++ = p->allowGainInteractivity;
    *o++ = p->azOffset;
    *o++ = p->elOffset;
    *o++ = p->distFactor;
    *o++ = p->defaultAzOffset;
    *o++ = p->defaultElOffset;
    *o++ = p->defaultDistFactor;
    *o++ = p->allowPositionInteractivity;
    *o++ = p->isAvailable;
  }
  for (int g = 0; g < ASI_MAX_SWITCH_GROUPS; g++) {
    const UI_STATE_SWITCH_GROUP* p = &s->switchGroups[g];
    *o++ = p->onOff;
    *o++ = p->activeMemberIndex;
    *o++ = p->allowSwitch;
    *o++ = p->isAvailable;
  }
  for (int g = 0; g < ASI_MAX_GROUP_PRESETS; g++) *o++ = s->groupPresets[g].isAvailable;
  *o++ = s->activePresetIndex;
  for (int i = 0; i < NUM_PREF_LANGUAGES; i++)
    for (int k = 0; k < 3; k++) *o++ = s->prefAudioLanguages[i][k];
  *o++ = s->accessibilityPreference;
  *o++ = s->targetLoudness;
  *o++ = (INT)s->drcSelected;
  *o++ = s->albumMode;
  *o++ = s->boost;
  *o++ = s->compress;
  return o;
}

static INT* putPersistence(INT* o, const UI_PERSISTENCE_MANAGER* p) {
  *o++ = p->memory != NULL;
  *o++ = p->memSize;
  *o++ = p->maxKeys;
  *o++ = p->maxCommands;
  *o++ = p->memory ? (INT)((UCHAR*)p->commands - (UCHAR*)p->memory) : -1;
  *o++ = p->firstKeyIdx;
  *o++ = p->nextGetCmdIdx;
  *o++ = p->nextGetCmdFlags;
  *o++ = p->accessibilitySetting;
  for (int k = 0; k < 3; k++) *o++ = p->preferredLanguage[k];
  /* raw memory content (FNV-1a) */
  UINT h = 2166136261u;
  if (p->memory) {
    for (int i = 0; i < p->memSize; i++) h = (h ^ ((UCHAR*)p->memory)[i]) * 16777619u;
  }
  *o++ = (INT)h;
  return o;
}

static INT* putUiManager(INT* o, const UI_MANAGER* h) {
  o = putAsi(o, &h->asi);
  o = putUiState(o, &h->uiState);
  *o++ = h->xmlStateChanged;
  *o++ = h->uiStateChanged;
  *o++ = h->drcStateChanged;
  *o++ = h->configChanged;
  *o++ = h->isActive;
  *o++ = (INT)h->xmlWriter.nLeft;
  *o++ = h->xmlWriter.nextPresetIdx;
  *o++ = h->xmlWriter.nextGroupIdx;
  *o++ = h->xmlWriter.nextSwitchGroupMemberIdx;
  o = putPersistence(o, h->hPersistence);
  return o;
}

/* action encoding: uuid[16], actionType, paramInt, paramFloat, paramBool, paramText[41],
   presentFlags */
static INT* putAction(INT* o, const UI_MANAGER_ACTION* a) {
  for (int i = 0; i < 16; i++) *o++ = a->uuid[i];
  *o++ = a->actionType;
  *o++ = a->paramInt;
  *o++ = a->paramFloat;
  *o++ = a->paramBool;
  for (int i = 0; i < MAX_XML_TEXT_LENGTH + 1; i++) *o++ = a->paramText[i];
  *o++ = a->presentFlags;
  return o;
}

static const INT* getAction(const INT* in, UI_MANAGER_ACTION* a) {
  for (int i = 0; i < 16; i++) a->uuid[i] = (UCHAR)*in++;
  a->actionType = (UCHAR)*in++;
  a->paramInt = *in++;
  a->paramFloat = *in++;
  a->paramBool = (UCHAR)*in++;
  for (int i = 0; i < MAX_XML_TEXT_LENGTH + 1; i++) a->paramText[i] = (char)*in++;
  a->presentFlags = (UCHAR)*in++;
  return in;
}

extern "C" {

/* ---- mpeghUIManager.h ---- */

void* cref_uim_open() { return mpegh_UI_Manager_Open(); }
void cref_uim_close(void* h) { mpegh_UI_Manager_Close((HANDLE_MPEGH_UI_MANAGER)h); }
INT cref_uim_getXml(void* h, char* out, UINT size, UINT flagsIn, UINT* flagsOut) {
  return mpegh_UI_GetXmlSceneState((HANDLE_MPEGH_UI_MANAGER)h, out, size, flagsIn, flagsOut);
}
INT cref_uim_applyXml(void* h, const char* in, UINT size, UINT* flagsOut) {
  return mpegh_UI_ApplyXmlAction((HANDLE_MPEGH_UI_MANAGER)h, in, size, flagsOut);
}
INT cref_uim_feed(void* h, UCHAR* buf, UINT len) {
  return mpegh_UI_FeedMHAS((HANDLE_MPEGH_UI_MANAGER)h, buf, len);
}
INT cref_uim_update(void* h, UCHAR* buf, UINT bufLen, UINT* len) {
  return mpegh_UI_UpdateMHAS((HANDLE_MPEGH_UI_MANAGER)h, buf, bufLen, len);
}
INT cref_uim_setPersist(void* h, UCHAR* mem, USHORT size) {
  return mpegh_UI_SetPersistenceMemory((HANDLE_MPEGH_UI_MANAGER)h, mem, size);
}
INT cref_uim_getPersist(void* h, UCHAR** mem, USHORT* size) {
  return mpegh_UI_GetPersistenceMemory((HANDLE_MPEGH_UI_MANAGER)h, (void**)mem, size);
}
void* cref_uim_uiManager(void* h) { return ((HANDLE_MPEGH_UI_MANAGER)h)->hUiManager; }
INT cref_uim_state(void* h, INT* out) {
  HANDLE_MPEGH_UI_MANAGER s = (HANDLE_MPEGH_UI_MANAGER)h;
  INT* o = out;
  *o++ = s->isActive;
  *o++ = s->configFound;
  for (int i = 0; i < MAX_NUMBER_SUBSTREAMS; i++) *o++ = (INT)s->substreamLabel[i];
  *o++ = (INT)s->insertOffset;
  o = putUiManager(o, s->hUiManager);
  return (INT)(o - out);
}

/* ---- uiManager.h ---- */

void* cref_uimgr_create(INT storeAsiDescriptions) {
  HANDLE_UI_MANAGER h = NULL;
  UI_Manager_Create(&h, storeAsiDescriptions);
  return h;
}
void cref_uimgr_delete(void* h) {
  HANDLE_UI_MANAGER p = (HANDLE_UI_MANAGER)h;
  UI_Manager_Delete(&p);
}
INT cref_uimgr_parseAsi(void* h, UCHAR* buf, UINT size, UINT validBits, INT streamIndex) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  return mae_AudioSceneInfo(UI_Manager_GetAsiPointer((HANDLE_UI_MANAGER)h), &bs, 2 * 28, streamIndex);
}
void cref_uimgr_asiReset(void* h) { asiReset(UI_Manager_GetAsiPointer((HANDLE_UI_MANAGER)h)); }
void cref_uimgr_setActiveDmxId(void* h, INT id) { UI_Manager_GetAsiPointer((HANDLE_UI_MANAGER)h)->activeDmxId = (UCHAR)id; }
void cref_uimgr_asiResetAvailability(void* h) { asiResetAvailability(UI_Manager_GetAsiPointer((HANDLE_UI_MANAGER)h)); }
INT cref_uimgr_setIsActive(void* h, INT isActive) { return UI_Manager_SetIsActive((HANDLE_UI_MANAGER)h, (UCHAR)isActive); }
INT cref_uimgr_getXmlChanged(void* h) { return UI_Manager_GetXmlChanged((HANDLE_UI_MANAGER)h); }
void cref_uimgr_resetXmlChanged(void* h) { UI_Manager_ResetXmlChanged((HANDLE_UI_MANAGER)h); }
INT cref_uimgr_getXml(void* h, char* out, UINT size, UINT flagsIn, UINT* flagsOut) {
  return UI_Manager_GetXmlSceneState((HANDLE_UI_MANAGER)h, out, size, flagsIn, flagsOut);
}
INT cref_uimgr_applyXml(void* h, const char* in, UINT size, UINT* flagsOut) {
  return UI_Manager_ApplyXmlAction((HANDLE_UI_MANAGER)h, in, size, flagsOut);
}
INT cref_uimgr_setUUID(void* h, UCHAR* uuid, INT applyAsiCrc) { return UI_Manager_SetUUID((HANDLE_UI_MANAGER)h, uuid, (UCHAR)applyAsiCrc); }
INT cref_uimgr_getUUID(void* h, UCHAR* uuid) { return UI_Manager_GetUUID((HANDLE_UI_MANAGER)h, uuid); }
/* status: interactionMode, numGroups, groupPresetID, 28 x 10 group fields (in/out) */
INT cref_uimgr_getInteractivityStatus(void* h, INT* status, UCHAR* signalChanged) {
  USER_INTERACTIVITY_STATUS s;
  INT* p = status;
  s.interactionMode = (UCHAR)*p++;
  s.numGroups = (UCHAR)*p++;
  s.groupPresetID = (UCHAR)*p++;
  for (int g = 0; g < ASI_MAX_GROUPS; g++) {
    s.groupData[g].groupID = (UCHAR)*p++;
    s.groupData[g].onOff = (UCHAR)*p++;
    s.groupData[g].routeToWIRE = (UCHAR)*p++;
    s.groupData[g].routeToWireID = (UCHAR)*p++;
    s.groupData[g].changePosition = (UCHAR)*p++;
    s.groupData[g].azOffset = (UCHAR)*p++;
    s.groupData[g].elOffset = (UCHAR)*p++;
    s.groupData[g].distFact = (UCHAR)*p++;
    s.groupData[g].changeGain = (UCHAR)*p++;
    s.groupData[g].gain = (UCHAR)*p++;
  }
  INT err = UI_Manager_GetInteractivityStatus((HANDLE_UI_MANAGER)h, &s, signalChanged);
  p = status;
  *p++ = s.interactionMode;
  *p++ = s.numGroups;
  *p++ = s.groupPresetID;
  for (int g = 0; g < ASI_MAX_GROUPS; g++) {
    *p++ = s.groupData[g].groupID;
    *p++ = s.groupData[g].onOff;
    *p++ = s.groupData[g].routeToWIRE;
    *p++ = s.groupData[g].routeToWireID;
    *p++ = s.groupData[g].changePosition;
    *p++ = s.groupData[g].azOffset;
    *p++ = s.groupData[g].elOffset;
    *p++ = s.groupData[g].distFact;
    *p++ = s.groupData[g].changeGain;
    *p++ = s.groupData[g].gain;
  }
  return err;
}
/* out: targetLoudness, drcSelected, albumMode, boost, compress (in/out) */
INT cref_uimgr_getDrcStatus(void* h, INT* out) {
  UI_DRC_LOUDNESS_STATUS s;
  s.targetLoudness = (SCHAR)out[0];
  s.drcSelected = (ULONG)out[1];
  s.albumMode = (SCHAR)out[2];
  s.boost = (SHORT)out[3];
  s.compress = (SHORT)out[4];
  INT err = UI_Manager_GetDrcLoudnessStatus((HANDLE_UI_MANAGER)h, &s);
  out[0] = s.targetLoudness;
  out[1] = (INT)s.drcSelected;
  out[2] = s.albumMode;
  out[3] = s.boost;
  out[4] = s.compress;
  return err;
}
INT cref_uimgr_getStatusChanged(void* h, UCHAR* ui, UCHAR* drc) { return UI_Manager_GetStatusChanged((HANDLE_UI_MANAGER)h, ui, drc); }
INT cref_uimgr_setPersist(void* h, UCHAR* mem, USHORT size) { return UI_Manager_SetPersistenceMemory((HANDLE_UI_MANAGER)h, mem, size); }
INT cref_uimgr_getPersist(void* h, UCHAR** mem, USHORT* size) { return UI_Manager_GetPersistenceMemory((HANDLE_UI_MANAGER)h, (void**)mem, size); }
INT cref_uimgr_state(void* h, INT* out) { return (INT)(putUiManager(out, (UI_MANAGER*)h) - out); }

/* ---- uiXml.h ---- */

UINT cref_uimgr_parseXmlAction(const char* in, UINT size, INT* out) {
  UI_MANAGER_ACTION a;
  FDKmemclear(&a, sizeof(a));
  UINT r = uiManagerParseXmlAction(in, size, &a);
  putAction(out, &a);
  return r;
}

/* ---- uiPersistence.h ---- */

void* cref_pers_create() { return persistenceManagerCreate(); }
void cref_pers_delete(void* h) { persistenceManagerDelete((HANDLE_UI_PERSISTENCE_MANAGER)h); }
INT cref_pers_setMemory(void* h, UCHAR* mem, USHORT size) { return persistenceManagerSetMemory((HANDLE_UI_PERSISTENCE_MANAGER)h, mem, size); }
void cref_pers_getMemory(void* h, UCHAR** mem, USHORT* size) { persistenceManagerGetMemory((HANDLE_UI_PERSISTENCE_MANAGER)h, (void**)mem, size); }
void cref_pers_save(void* h, const INT* action) {
  UI_MANAGER_ACTION a;
  getAction(action, &a);
  persistenceManagerSaveCommand((HANDLE_UI_PERSISTENCE_MANAGER)h, &a);
}
/* action: in/out (fields not written by persistenceManagerGetCommand() keep their values) */
INT cref_pers_get(void* h, const UCHAR* uuid, INT* action) {
  UI_MANAGER_ACTION a;
  getAction(action, &a);
  INT r = persistenceManagerGetCommand((HANDLE_UI_PERSISTENCE_MANAGER)h, uuid, &a);
  putAction(action, &a);
  return r;
}
INT cref_pers_state(void* h, INT* out) { return (INT)(putPersistence(out, (UI_PERSISTENCE_MANAGER*)h) - out); }

} /* extern "C" */
