// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims for libgVBAPRenderer: renderer API (gVBAPRenderer.h), VBAP core functions, quickhull
// triangulation, Cartesian math and a serializer of the renderer state for differential testing.
// The serializer writes the fields of GVBAPRENDERER as INT values in a fixed order;
// tests/gvbap.rs contains the matching Rust serializer.

#include <stdlib.h>
#include <string.h>

#include "gVBAPRenderer.h"
#include "quickHull.h"
#include "vbap_core.h"
#include "cartesianMath.h"
#include "tables.h"

extern "C" int aziCmp(const void* a, const void* b);

#define P(x) (out[n++] = (INT)(x))

/* Fields that gVBAPRenderer_Open() leaves uninitialized (FDKmalloc) are set to 0 (like in the
 * Rust port). None of them is read before it is written. */
static void zero_uninitialized(HANDLE_GVBAPRENDERER h) {
  int nof = h->numOamFrames, nobj = h->numObjects;
  for (int k = 0; k < nof; k++) {
    for (int i = 0; i < nobj; i++) {
      OAM_SAMPLE* s = &h->oamSamples[k][i];
      OAM_SAMPLE* v = &h->oamSamplesValid[k][i];
      memset(&s->cart, 0, sizeof(s->cart));
      memset(&v->cart, 0, sizeof(v->cart));
      if (k < nof - 1) {
        FIXP_DBL rad = s->sph.rad;
        memset(s, 0, sizeof(*s));
        s->sph.rad = rad;
        memset(v, 0, sizeof(*v));
      }
    }
  }
  if (h->renderMode == GVBAP_LEGACY) {
    memset(h->gainCache, 0, h->gainCacheLength * sizeof(FIXP_DBL));
  }
  memset(h->startGainsMax, 0, h->numChannels * sizeof(FIXP_DBL));
  memset(h->prevGainsMax, 0, h->numChannels * sizeof(FIXP_DBL));
}

static int ser_point(const PointCartesian* p, INT* out, int n) {
  P(p->x);
  P(p->y);
  P(p->z);
  return n;
}

static int ser_oam(const OAM_SAMPLE* s, INT* out, int n) {
  P(s->sph.azi);
  P(s->sph.ele);
  P(s->sph.rad);
  P(s->gain);
  P(s->spreadAngle);
  P(s->spreadHeight);
  P(s->spreadDepth);
  P(s->goa_bsObjectDistance);
  return n;
}

extern "C" {

/* geo: outChannels triplets (Az, El, LFE). On error the handle is closed and NULL returned. */
void* cref_gvbap_open(INT numObjects, INT frameLength, INT oamFrameLength, const INT* geo, INT outChannels, INT outCICPIndex, INT hasUniformSpread, INT renderMode, INT* err) {
  CICP2GEOMETRY_CHANNEL_GEOMETRY g[32];
  memset(g, 0, sizeof(g));
  for (int i = 0; i < outChannels && i < 32; i++) {
    g[i].cicpLoudspeakerIndex = -1;
    g[i].Az = geo[3 * i];
    g[i].El = geo[3 * i + 1];
    g[i].LFE = geo[3 * i + 2];
  }
  HANDLE_GVBAPRENDERER h = NULL;
  *err = gVBAPRenderer_Open(&h, numObjects, frameLength, oamFrameLength, g, outChannels, outCICPIndex, hasUniformSpread, renderMode);
  if (*err != 0) {
    if (h) gVBAPRenderer_Close(h);
    return NULL;
  }
  zero_uninitialized(h);
  return h;
}
void cref_gvbap_close(void* h) { gVBAPRenderer_Close((HANDLE_GVBAPRENDERER)h); }

INT cref_gvbap_ser(void* hv, INT* out) {
  HANDLE_GVBAPRENDERER h = (HANDLE_GVBAPRENDERER)hv;
  int n = 0, i, j, k;
  int nch = h->numChannels, nobj = h->numObjects, nof = h->numOamFrames, gcl = h->gainCacheLength;
  P(h->numChannels);
  P(h->numLFE);
  P(h->numGhosts);
  P(h->numObjects);
  P(h->frameLength);
  P(h->oamFrameLength);
  P(h->numOamFrames);
  P(h->gainCacheLength);
  P(h->startGainsFilled);
  P(h->renderMode);
  P(h->hasUniformSpread);
  P(h->oamDataValid);
  P(h->ghostVoiceOfGodSpeakerIndex);
  P(h->ghostVoiceOfHellSpeakerIndex);
  for (i = 0; i < 16; i++) P(h->metadataPresent[i]);
  for (i = 0; i < 16; i++) P(h->metadataPresentValid[i]);
  for (i = 0; i < 8; i++) P(h->fixed_val[i]);
  const SPEAKERSETUP* ss = &h->speakerSetup;
  P(ss->speakerListSize);
  P(ss->speakerTripletSize);
  P(ss->greatestInverseMatrixExponent);
  P(ss->coversUpperHalfSphere);
  P(ss->coversLowerHalfSphere);
  P(ss->hasHeightSpeakers);
  for (i = 0; i < ss->speakerListSize; i++) {
    P(ss->speakerList[i].sph.azi);
    P(ss->speakerList[i].sph.ele);
    P(ss->speakerList[i].sph.rad);
    P(ss->speakerList[i].isLFE);
    P(ss->speakerList[i].isGhost);
    P(ss->mapping[i]);
  }
  for (i = 0; i < ss->speakerTripletSize; i++) {
    const SPEAKERTRIPLET* t = &ss->speakerTriplet[i];
    for (j = 0; j < 3; j++) P(t->triangle[j]);
    for (j = 0; j < 3; j++) n = ser_point(&t->matrix[j], out, n);
    for (j = 0; j < 3; j++) n = ser_point(&t->inverseMatrix[j], out, n);
    P(t->exponentInverseMatrix);
  }
  P(h->downmixMatrix != NULL);
  P(h->downmixMatrixNumRows);
  P(h->downmixMatrixNumCols);
  if (h->downmixMatrix) {
    for (i = 0; i < h->downmixMatrixNumRows; i++)
      for (j = 0; j < h->downmixMatrixNumCols; j++) P(h->downmixMatrix[i][j]);
  }
  for (i = 0; i < 20; i++) n = ser_point(&h->baseVectors[i], out, n);
  for (i = 0; i < nobj * 8; i++) P(h->OAM_parsed_data[i]);
  for (k = 0; k < nof; k++)
    for (i = 0; i < nobj; i++) n = ser_oam(&h->oamSamples[k][i], out, n);
  for (k = 0; k < nof; k++)
    for (i = 0; i < nobj; i++) n = ser_oam(&h->oamSamplesValid[k][i], out, n);
  for (i = 0; i < nobj; i++)
    for (j = 0; j < nch; j++) {
      P(h->startGains[i][j]);
      P(h->endGains[i][j]);
      P(h->stepState[i][j]);
      P(h->scaleState[i][j]);
    }
  for (j = 0; j < nch; j++) {
    P(h->startGainsMax[j]);
    P(h->prevGainsMax[j]);
  }
  for (i = 0; i < gcl; i++) P(h->gainCache[i]);
  for (i = 0; i < GVBAP_SPREAD_NUM_VSO; i++) P(h->spread_gainsSpread[i]);
  P(h->spread_gainArray != NULL);
  if (h->spread_gainArray) {
    for (i = 0; i < gcl; i++) P(h->spread_gainArray[i]);
    for (k = 0; k < GVBAP_SPREAD_NUM_VSO; k++)
      for (i = 0; i < gcl; i++) P(h->spread_gainsVSO[k][i]);
  }
  return n;
}

INT cref_gvbap_render(void* h, INT* in, INT* out, INT startSamplePosition, INT inOffset) {
  return gVBAPRenderer_RenderFrame_Time((HANDLE_GVBAPRENDERER)h, (FIXP_DBL*)in, (FIXP_DBL*)out, startSamplePosition, inOffset);
}
INT cref_gvbap_oam_frame(void* h, UCHAR* buf, UINT size, UINT validBits, INT payloadLen, INT lowDelay, UINT numObjects, INT hasDynPrio, INT hasUniformSpread, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = objectMetadataFrame((HANDLE_GVBAPRENDERER)h, &bs, payloadLen, (UCHAR)lowDelay, numObjects, (UCHAR)hasDynPrio, (UCHAR)hasUniformSpread);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
INT cref_gvbap_prod_frame(void* h, UCHAR* buf, UINT size, UINT validBits, UINT numObjects, INT refDist, INT hasObjDist, INT* bitsLeft) {
  FDK_BITSTREAM bs;
  FDKinitBitStream(&bs, buf, size, validBits, BS_READER);
  INT err = prodMetadataFrameGroup((HANDLE_GVBAPRENDERER)h, &bs, numObjects, refDist, hasObjDist);
  *bitsLeft = (INT)FDKgetValidBits(&bs);
  return err;
}
/* v: azi, ele, rad, gain, spreadAngle, spreadHeight, spreadDepth */
void cref_gvbap_set_oam(void* hv, INT k, INT o, const INT* v) {
  HANDLE_GVBAPRENDERER h = (HANDLE_GVBAPRENDERER)hv;
  OAM_SAMPLE* s = &h->oamSamples[k][o];
  s->sph.azi = v[0];
  s->sph.ele = v[1];
  s->sph.rad = v[2];
  s->gain = v[3];
  s->spreadAngle = v[4];
  s->spreadHeight = v[5];
  s->spreadDepth = v[6];
}
void cref_gvbap_set_flags(void* hv, INT k, INT present, INT oamDataValid, INT hasUniformSpread) {
  HANDLE_GVBAPRENDERER h = (HANDLE_GVBAPRENDERER)hv;
  h->metadataPresent[k] = (UCHAR)present;
  h->oamDataValid = (UCHAR)oamDataValid;
  h->hasUniformSpread = (UCHAR)hasUniformSpread;
}
/* v: azi, ele, rad, gain, spreadAngle, spreadHeight, spreadDepth; cart from sphericalToCartesian */
void cref_gvbap_calculate_vbap(void* h, const INT* v, INT* gains, INT noDownmix) {
  OAM_SAMPLE s = {};
  s.sph.azi = v[0];
  s.sph.ele = v[1];
  s.sph.rad = v[2];
  s.gain = v[3];
  s.spreadAngle = v[4];
  s.spreadHeight = v[5];
  s.spreadDepth = v[6];
  s.cart = sphericalToCartesian(s.sph);
  calculateVbap((HANDLE_GVBAPRENDERER)h, s, (FIXP_DBL*)gains, noDownmix);
}

/* ---- math and quickhull ---- */

void cref_gvbap_sph2cart(INT azi, INT ele, INT rad, INT* out) {
  PointSpherical s = {azi, ele, rad};
  PointCartesian c = sphericalToCartesian(s);
  out[0] = c.x;
  out[1] = c.y;
  out[2] = c.z;
}
void cref_gvbap_cross_dot(const INT* a, const INT* b, INT* out) {
  PointCartesian pa = {a[0], a[1], a[2]}, pb = {b[0], b[1], b[2]};
  PointCartesian c = crossProduct(&pa, &pb);
  out[0] = c.x;
  out[1] = c.y;
  out[2] = c.z;
  out[3] = dotProduct(&pa, &pb);
}
INT cref_gvbap_normalize(INT* xyz) {
  PointCartesian p = {xyz[0], xyz[1], xyz[2]};
  INT r = normalize(&p);
  xyz[0] = p.x;
  xyz[1] = p.y;
  xyz[2] = p.z;
  return r;
}
/* pts: x1, x2, x3, p (xyz each) */
INT cref_gvbap_dist_point_plane(const INT* pts) {
  PointCartesian p[4];
  for (int i = 0; i < 4; i++) {
    p[i].x = pts[3 * i];
    p[i].y = pts[3 * i + 1];
    p[i].z = pts[3 * i + 2];
  }
  return distPointPlane(p[0], p[1], p[2], p[3]);
}
/* out: x, y, z, index, azi, ele, rad */
void cref_gvbap_init_vertex(INT azi, INT ele, INT* out) {
  vertex v = init_vertex(azi, ele);
  out[0] = v.xyz.x;
  out[1] = v.xyz.y;
  out[2] = v.xyz.z;
  out[3] = v.index;
  out[4] = v.sph.azi;
  out[5] = v.sph.ele;
  out[6] = v.sph.rad;
}
/* pairs: n (azi, index) pairs, sorted in place with qsort() and aziCmp() */
void cref_gvbap_qsort(INT* pairs, INT n) {
  surroundList* l = (surroundList*)malloc((n > 0 ? n : 1) * sizeof(surroundList));
  for (int i = 0; i < n; i++) {
    l[i].azi = pairs[2 * i];
    l[i].index = pairs[2 * i + 1];
  }
  qsort(l, n, sizeof(surroundList), aziCmp);
  for (int i = 0; i < n; i++) {
    pairs[2 * i] = l[i].azi;
    pairs[2 * i + 1] = l[i].index;
  }
  free(l);
}
/* azel: n (azi, ele) pairs (FIXP_DBL). Outputs the vertex list (7 values per vertex, see
 * cref_gvbap_init_vertex), the triangles and the downmix matrix (up to 32 x 32). */
INT cref_gvbap_triangulate(INT cicp, const INT* azel, INT n, INT* vout, INT* nv, INT* tout, INT* nt, INT* dmx, INT* rows, INT* cols) {
  vertexList vL;
  triangleList tL;
  FIXP_DBL az[32], el[32];
  FIXP_DBL** dm = NULL;
  for (int i = 0; i < n; i++) {
    az[i] = azel[2 * i];
    el[i] = azel[2 * i + 1];
  }
  *rows = 0;
  *cols = 0;
  resetVertexList(&vL);
  qh_gen_VertexList(n, az, el, &vL);
  resetTriangleList(&tL);
  INT err = qh_sphere_triangulation(cicp, &vL, &tL, &dm, rows, cols);
  *nv = vL.size;
  for (int i = 0; i < vL.size; i++) {
    vertex* v = &vL.element[i];
    vout[7 * i + 0] = v->xyz.x;
    vout[7 * i + 1] = v->xyz.y;
    vout[7 * i + 2] = v->xyz.z;
    vout[7 * i + 3] = v->index;
    vout[7 * i + 4] = v->sph.azi;
    vout[7 * i + 5] = v->sph.ele;
    vout[7 * i + 6] = v->sph.rad;
  }
  *nt = tL.size;
  for (int i = 0; i < tL.size; i++)
    for (int j = 0; j < 3; j++) tout[3 * i + j] = tL.element[i].index[j];
  if (dm) {
    for (int i = 0; i < *rows; i++)
      for (int j = 0; j < *cols; j++) dmx[i * *cols + j] = dm[i][j];
    fdkFreeMatrix2D((void**)dm);
  } else {
    *rows = -1;
    *cols = -1;
  }
  return err;
}

}  // extern "C"
