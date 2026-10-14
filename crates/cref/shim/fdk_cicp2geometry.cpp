// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libFDK FDK_cicp2geometry.h. A CICP2GEOMETRY_CHANNEL_GEOMETRY is exchanged as 10
// INTs in member order.

#include "FDK_cicp2geometry.h"
#include "genericStds.h"

static void geoToInts(const CICP2GEOMETRY_CHANNEL_GEOMETRY* g, INT* o) {
  o[0] = g->cicpLoudspeakerIndex;
  o[1] = g->Az;
  o[2] = g->El;
  o[3] = g->LFE;
  o[4] = g->screenRelative;
  o[5] = g->hasDistance;
  o[6] = (INT)g->distance;
  o[7] = g->hasLoudspeakerCalibrationGain;
  o[8] = g->loudspeakerCalibrationGain;
  o[9] = (INT)g->loudspeakerType;
}

static void intsToGeo(const INT* o, CICP2GEOMETRY_CHANNEL_GEOMETRY* g) {
  g->cicpLoudspeakerIndex = o[0];
  g->Az = o[1];
  g->El = o[2];
  g->LFE = o[3];
  g->screenRelative = o[4];
  g->hasDistance = o[5];
  g->distance = (unsigned int)o[6];
  g->hasLoudspeakerCalibrationGain = o[7];
  g->loudspeakerCalibrationGain = o[8];
  g->loudspeakerType = (CICP2GEOMETRY_LOUDSPEAKER_TYPE)o[9];
}

extern "C" {

/* out: 32 * 10 INTs (in/out: preset content is kept where not written) */
INT cref_cicp_geometryFromCicp(INT cicpIndex, INT* out, INT* numChannels, INT* numLfes, INT useNull) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  for (int i = 0; i < CICP2GEOMETRY_MAX_LOUDSPEAKERS; i++) intsToGeo(out + 10 * i, &g[i]);
  INT err = cicp2geometry_get_geometry_from_cicp(cicpIndex, useNull ? NULL : g, numChannels, numLfes);
  for (int i = 0; i < CICP2GEOMETRY_MAX_LOUDSPEAKERS; i++) geoToInts(&g[i], out + 10 * i);
  return err;
}
INT cref_cicp_numChannelsFromCicp(INT cicpIndex, INT* numTotalChannels) { return cicp2geometry_get_numChannels_from_cicp(cicpIndex, numTotalChannels); }
INT cref_cicp_numChannelsFromCicpNoErr(INT cicpIndex) { return cicp2geometry_get_numChannels_from_cicp(cicpIndex); }
INT cref_cicp_cicpIndexFromGeometry(const INT* in, INT n, INT numChannels, INT numLfes, INT* cicpIndex) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  FDKmemclear(g, sizeof(g));
  for (int i = 0; i < n; i++) intsToGeo(in + 10 * i, &g[i]);
  return cicp2geometry_get_cicpIndex_from_geometry(g, numChannels, numLfes, cicpIndex);
}
INT cref_cicp_geometryFromLsIndex(INT idx, INT* out, INT useNull) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g;
  intsToGeo(out, &g);
  INT err = cicp2geometry_get_geometry_from_cicp_loudspeaker_index(idx, useNull ? NULL : &g);
  geoToInts(&g, out);
  return err;
}
INT cref_cicp_numberOfLfes(const INT* in, UINT n, UINT* numLfes) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  for (UINT i = 0; i < n; i++) intsToGeo(in + 10 * i, &g[i]);
  return cicp2geometry_get_number_of_lfes(g, n, numLfes);
}
INT cref_cicp_frontSpeakers(INT targetLayout, INT numSpeakers, INT* numSignalsMix, INT* pos, INT* gains) {
  return cicp2geometry_get_front_speakers(targetLayout, numSpeakers, numSignalsMix, pos, gains);
}

}  // extern "C"
