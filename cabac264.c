
/*!
 * \copy
 *     Copyright (c)  2009-2013, Cisco Systems
 *     All rights reserved.
 *
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the following conditions
 *     are met:
 *
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *     POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * \file    set_mb_syn_cabac.cpp
 *
 * \brief   cabac coding engine
 *
 * \date    10/11/2014 Created
 *
 *************************************************************************************
 */
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>


/*
#include "typedefs.h"
#include "macros.h"
#include "set_mb_syn_cabac.h"
#include "encoder.h"
#include "golomb_common.h"
*/

const int8_t g_kiClz5Table[32] = {
  6, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

void PropagateCarry (uint8_t* pBufCur, uint8_t* pBufStart) {
  for (; pBufCur > pBufStart; --pBufCur)
    if (++ * (pBufCur - 1))
      break;
}
#define WELS_CLIP3(iX, iY, iZ) ((iX) < (iY) ? (iY) : ((iX) > (iZ) ? (iZ) : (iX)))

#define WRITE_BE_32(ptr, val) do { \
        (ptr)[0] = (val) >> 24; \
        (ptr)[1] = (val) >> 16; \
        (ptr)[2] = (val) >>  8; \
        (ptr)[3] = (val) >>  0; \
    } while (0)


typedef enum TagWelsErr {
  ERR_NONE                = 0,
  ERR_INVALID_PARAMETERS  = 1,
  ERR_MALLOC_FAILED       = 2,
  ERR_API_FAILED          = 3,

  ERR_BOUND               = 31
} EWelsErr;

/*
 * Specified error format:
 * ERR_NO = (ERR_LEVEL_FROM (HIGH WORD) << 16) | (ERR_INFO_FROM (LOW WORD))
 *
 */
#define GENERATE_ERROR_NO(iErrLevel, iErrInfo) ((iErrLevel << 16) | (iErrInfo & 0xFFFF))

/* ERR_LEVEL */
//-----------------------------------------------------------------------------------------------------------
enum {
  ERR_LEVEL_ACCESS_UNIT = 1,
  ERR_LEVEL_NAL_UNIT_HEADER,
  ERR_LEVEL_PREFIX_NAL,
  ERR_LEVEL_PARAM_SETS,
  ERR_LEVEL_SLICE_HEADER,
  ERR_LEVEL_SLICE_DATA,
  ERR_LEVEL_MB_DATA
};
/* More detailed error information, maximal value is 65535 */
//-----------------------------------------------------------------------------------------------------------
#define ERR_INFO_COMMON_BASE        1
#define ERR_INFO_SYNTAX_BASE        1001
#define ERR_INFO_LOGIC_BASE         10001
enum {
//for CABAC
  ERR_CABAC_NO_BS_TO_READ = ERR_INFO_LOGIC_BASE,
  ERR_CABAC_UNEXPECTED_VALUE
};

typedef uint64_t cabac_low_t;
#define  CABAC_LOW_WIDTH  (sizeof (cabac_low_t) / sizeof (uint8_t) * 8)
typedef struct TagCabacCtx {
  cabac_low_t m_uiLow;
  int32_t   m_iLowBitCnt;
  int32_t   m_iRenormCnt;
  uint32_t  m_uiRange;
  uint8_t Mps;
  uint8_t State; 
  uint8_t*   m_pBufStart;
  uint8_t*   m_pBufEnd;
  uint8_t*   m_pBufCur;
} SCabacCtx;


#if  defined(__LP64__) || defined(_WIN64)
typedef int64_t intX_t;
#else
typedef int32_t intX_t;
#endif

// private functions used by public inline functions.
static void WelsCabacEncodeDecisionLps_ (SCabacCtx* pCbCtx);
void WelsCabacEncodeBypassOne (SCabacCtx* pCbCtx, int32_t uiBin);
static void WelsCabacEncodeUpdateLowNontrivial_ (SCabacCtx* pCbCtx);

static inline void WelsCabacEncodeUpdateLow_ (SCabacCtx* pCbCtx) {
  if (pCbCtx->m_iLowBitCnt + pCbCtx->m_iRenormCnt < CABAC_LOW_WIDTH) {
    pCbCtx->m_iLowBitCnt  += pCbCtx->m_iRenormCnt;
    pCbCtx->m_uiLow      <<= pCbCtx->m_iRenormCnt;
  } else {
    WelsCabacEncodeUpdateLowNontrivial_ (pCbCtx);
  }
  pCbCtx->m_iRenormCnt = 0;
}


void  WelsCabacEncodeInit (SCabacCtx* pCbCtx, uint8_t* pBuf,  size_t len) {
  pCbCtx->m_uiLow     = 0;
  pCbCtx->m_iLowBitCnt = 9;
  pCbCtx->m_iRenormCnt = 0;
  pCbCtx->m_uiRange   = 510;
  pCbCtx->m_pBufStart = pBuf;
  pCbCtx->m_pBufEnd = pBuf + len;
  pCbCtx->m_pBufCur = pBuf;
  pCbCtx->Mps = 0;
  pCbCtx->State = 0;
}

static void WelsCabacEncodeUpdateLowNontrivial_ (SCabacCtx* pCbCtx) {
  int32_t iLowBitCnt = pCbCtx->m_iLowBitCnt;
  int32_t iRenormCnt = pCbCtx->m_iRenormCnt;
  cabac_low_t uiLow = pCbCtx->m_uiLow;

  do {
    uint8_t* pBufCur = pCbCtx->m_pBufCur;
    const int32_t kiInc = CABAC_LOW_WIDTH - 1 - iLowBitCnt;

    uiLow <<= kiInc;
    if (uiLow & (cabac_low_t) (1) << (CABAC_LOW_WIDTH - 1))
      PropagateCarry (pBufCur, pCbCtx->m_pBufStart);

    if (CABAC_LOW_WIDTH > 32) {
      WRITE_BE_32 (pBufCur, (uint32_t) (uiLow >> 31));
      pBufCur += 4;
    }
    *pBufCur++ = (uint8_t) (uiLow >> 23);
    *pBufCur++ = (uint8_t) (uiLow >> 15);
    iRenormCnt -= kiInc;
    iLowBitCnt = 15;
    uiLow &= (1u << iLowBitCnt) - 1;
    pCbCtx->m_pBufCur = pBufCur;
  } while (iLowBitCnt + iRenormCnt > CABAC_LOW_WIDTH - 1);

  pCbCtx->m_iLowBitCnt = iLowBitCnt + iRenormCnt;
  pCbCtx->m_uiLow = uiLow << iRenormCnt;
}

/*Table 9-44 – Specification of rangeTabLPS depending on pStateIdx and qCodIRangeIdx */

static const uint8_t g_kuiCabacRangeLps[64][4] = {
  { 128, 176, 208, 240}, { 128, 167, 197, 227}, { 128, 158, 187, 216}, { 123, 150, 178, 205}, { 116, 142, 169, 195}, { 111, 135, 160, 185}, { 105, 128, 152, 175}, { 100, 122, 144, 166},
  {  95, 116, 137, 158}, {  90, 110, 130, 150}, {  85, 104, 123, 142}, {  81,  99, 117, 135}, {  77,  94, 111, 128}, {  73,  89, 105, 122}, {  69,  85, 100, 116}, {  66,  80,  95, 110},
  {  62,  76,  90, 104}, {  59,  72,  86,  99}, {  56,  69,  81,  94}, {  53,  65,  77,  89}, {  51,  62,  73,  85}, {  48,  59,  69,  80}, {  46,  56,  66,  76}, {  43,  53,  63,  72},
  {  41,  50,  59,  69}, {  39,  48,  56,  65}, {  37,  45,  54,  62}, {  35,  43,  51,  59}, {  33,  41,  48,  56}, {  32,  39,  46,  53}, {  30,  37,  43,  50}, {  29,  35,  41,  48},
  {  27,  33,  39,  45}, {  26,  31,  37,  43}, {  24,  30,  35,  41}, {  23,  28,  33,  39}, {  22,  27,  32,  37}, {  21,  26,  30,  35}, {  20,  24,  29,  33}, {  19,  23,  27,  31},
  {  18,  22,  26,  30}, {  17,  21,  25,  28}, {  16,  20,  23,  27}, {  15,  19,  22,  25}, {  14,  18,  21,  24}, {  14,  17,  20,  23}, {  13,  16,  19,  22}, {  12,  15,  18,  21},
  {  12,  14,  17,  20}, {  11,  14,  16,  19}, {  11,  13,  15,  18}, {  10,  12,  15,  17}, {  10,  12,  14,  16}, {   9,  11,  13,  15}, {   9,  11,  12,  14}, {   8,  10,  12,  14},
  {   8,   9,  11,  13}, {   7,   9,  11,  12}, {   7,   9,  10,  12}, {   7,   8,  10,  11}, {   6,   8,   9,  11}, {   6,   7,   9,  10}, {   6,   7,   8,   9}, {   2,   2,   2,   2}
};

/*Table 9-45 – State transition table*/

static const uint8_t g_kuiStateTransTable[64][2] = {

  {0, 1}, {0, 2}, {1, 3}, {2, 4}, {2, 5}, {4, 6}, {4, 7}, {5, 8}, {6, 9}, {7, 10},
  {8, 11}, {9, 12}, {9, 13}, {11, 14}, {11, 15}, {12, 16}, {13, 17}, {13, 18}, {15, 19}, {15, 20},
  {16, 21}, {16, 22}, {18, 23}, {18, 24}, {19, 25}, {19, 26}, {21, 27}, {21, 28}, {22, 29}, {22, 30},
  {23, 31}, {24, 32}, {24, 33}, {25, 34}, {26, 35}, {26, 36}, {27, 37}, {27, 38}, {28, 39}, {29, 40},
  {29, 41}, {30, 42}, {30, 43}, {30, 44}, {31, 45}, {32, 46}, {32, 47}, {33, 48}, {33, 49}, {33, 50},
  {34, 51}, {34, 52}, {35, 53}, {35, 54}, {35, 55}, {36, 56}, {36, 57}, {36, 58}, {37, 59}, {37, 60},
  {37, 61}, {38, 62}, {38, 62}, {63, 63}
};

static void WelsCabacEncodeDecisionLps_ (SCabacCtx* pCbCtx) {
  const int32_t kiState = pCbCtx->State;
  uint32_t uiRange = pCbCtx->m_uiRange;
  uint32_t uiRangeLps = g_kuiCabacRangeLps[kiState][ (uiRange & 0xff) >> 6];
  uiRange -= uiRangeLps;
  pCbCtx->State = g_kuiStateTransTable[kiState][0];
  pCbCtx->Mps   ^=  (kiState == 0);

  WelsCabacEncodeUpdateLow_ (pCbCtx);
  pCbCtx->m_uiLow += uiRange;

  const int32_t kiRenormAmount = g_kiClz5Table[uiRangeLps >> 3];
  pCbCtx->m_uiRange = uiRangeLps << kiRenormAmount;
  pCbCtx->m_iRenormCnt = kiRenormAmount;
}

void WelsCabacEncodeTerminate (SCabacCtx* pCbCtx, uint32_t uiBin) {
  pCbCtx->m_uiRange -= 2;
  if (uiBin) {
    WelsCabacEncodeUpdateLow_ (pCbCtx);
    pCbCtx->m_uiLow  += pCbCtx->m_uiRange;

    const int32_t kiRenormAmount = 7;
    pCbCtx->m_uiRange = 2 << kiRenormAmount;
    pCbCtx->m_iRenormCnt = kiRenormAmount;

    WelsCabacEncodeUpdateLow_ (pCbCtx);
    pCbCtx->m_uiLow |= 0x80;
  } else {
    const int32_t kiRenormAmount = pCbCtx->m_uiRange >> 8 ^ 1;
    pCbCtx->m_uiRange = pCbCtx->m_uiRange << kiRenormAmount;
    pCbCtx->m_iRenormCnt += kiRenormAmount;
  }
}
void WelsCabacEncodeUeBypass (SCabacCtx* pCbCtx, int32_t iExpBits, uint32_t uiVal) {
  int32_t iSufS = uiVal;
  int32_t iStopLoop = 0;
  int32_t k = iExpBits;
  do {
    if (iSufS >= (1 << k)) {
      WelsCabacEncodeBypassOne (pCbCtx, 1);
      iSufS = iSufS - (1 << k);
      k++;
    } else {
      WelsCabacEncodeBypassOne (pCbCtx, 0);
      while (k--)
        WelsCabacEncodeBypassOne (pCbCtx, (iSufS >> k) & 1);
      iStopLoop = 1;
    }
  } while (!iStopLoop);
}

void WelsCabacEncodeFlush (SCabacCtx* pCbCtx) {
  WelsCabacEncodeTerminate (pCbCtx, 1);

  cabac_low_t uiLow = pCbCtx->m_uiLow;
  int32_t iLowBitCnt = pCbCtx->m_iLowBitCnt;
  uint8_t* pBufCur = pCbCtx->m_pBufCur;

  uiLow <<= CABAC_LOW_WIDTH - 1 - iLowBitCnt;
  if (uiLow & (cabac_low_t) (1) << (CABAC_LOW_WIDTH - 1))
    PropagateCarry (pBufCur, pCbCtx->m_pBufStart);
  for (; (iLowBitCnt -= 8) >= 0; uiLow <<= 8)
    * pBufCur++ = (uint8_t) (uiLow >> (CABAC_LOW_WIDTH - 9));

  pCbCtx->m_pBufCur = pBufCur;
}

uint8_t* WelsCabacEncodeGetPtr (SCabacCtx* pCbCtx) {
  return pCbCtx->m_pBufCur;
}


void WelsCabacEncodeBypassOne (SCabacCtx* pCbCtx, int32_t uiBin) {
  const uint32_t kuiBinBitmask = -uiBin;
  pCbCtx->m_iRenormCnt++;
  WelsCabacEncodeUpdateLow_ (pCbCtx);
  pCbCtx->m_uiLow += kuiBinBitmask & pCbCtx->m_uiRange;
}

void WelsCabacEncodeDecision (SCabacCtx* pCbCtx, uint32_t uiBin) {
  if (uiBin == pCbCtx->Mps) {
    const int32_t kiState = pCbCtx->State;
    uint32_t uiRange = pCbCtx->m_uiRange;
    uint32_t uiRangeLps = g_kuiCabacRangeLps[kiState][(uiRange & 0xff) >> 6];
    uiRange -= uiRangeLps;

    const int32_t kiRenormAmount = uiRange >> 8 ^ 1;
    pCbCtx->m_uiRange = uiRange << kiRenormAmount;
    pCbCtx->m_iRenormCnt += kiRenormAmount;
    pCbCtx->State = g_kuiStateTransTable[kiState][1];
    pCbCtx->Mps = uiBin;
  } else {
    WelsCabacEncodeDecisionLps_ (pCbCtx);
  }
}

/* decoder */

typedef struct {
  uint64_t uiRange;
  uint64_t uiOffset;
  int32_t iBitsLeft;
  uint8_t* pBuffCurr;
  uint8_t* pBuffEnd;
  uint8_t uiState;
  uint8_t uiMPS;
  uint8_t buffer[8];
} SWelsCabacDecEngine, *PWelsCabacDecEngine;



// ------------------- 3. actual decoding
static int32_t Read32BitsCabac (PWelsCabacDecEngine pDecEngine, uint32_t* uiValue, int32_t* iNumBitsRead) {
  intX_t iLeftBytes = pDecEngine->pBuffEnd - pDecEngine->pBuffCurr;
  *iNumBitsRead = 0;
  *uiValue = 0;
  if (iLeftBytes <= 0) {
    return GENERATE_ERROR_NO (ERR_LEVEL_MB_DATA, ERR_CABAC_NO_BS_TO_READ);
  }
  switch (iLeftBytes) {
  case 3:
    *uiValue = ((pDecEngine->pBuffCurr[0]) << 16 | (pDecEngine->pBuffCurr[1]) << 8 | (pDecEngine->pBuffCurr[2]));
    pDecEngine->pBuffCurr += 3;
    *iNumBitsRead = 24;
    break;
  case 2:
    *uiValue = ((pDecEngine->pBuffCurr[0]) << 8 | (pDecEngine->pBuffCurr[1]));
    pDecEngine->pBuffCurr += 2;
    *iNumBitsRead = 16;
    break;
  case 1:
    *uiValue = pDecEngine->pBuffCurr[0];
    pDecEngine->pBuffCurr += 1;
    *iNumBitsRead = 8;
    break;
  default:
    *uiValue = ((pDecEngine->pBuffCurr[0] << 24) | (pDecEngine->pBuffCurr[1]) << 16 | (pDecEngine->pBuffCurr[2]) << 8 |
               (pDecEngine->pBuffCurr[3]));
    pDecEngine->pBuffCurr += 4;
    *iNumBitsRead = 32;
    break;
  }
  return ERR_NONE;
}

static const uint8_t g_kRenormTable256[256] = {
  6, 6, 6, 6, 6, 6, 6, 6,
  5, 5, 5, 5, 5, 5, 5, 5,
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 4,
  3, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1
};

#define WELS_CABAC_HALF    0x01FE
#define WELS_CABAC_QUARTER 0x0100
#define WELS_CABAC_FALSE_RETURN(iErrorInfo) \
if(iErrorInfo) { \
  return iErrorInfo; \
}

// ------------------- 2. decoding Engine initialization
int32_t InitCabacDecEngine(PWelsCabacDecEngine pDecEngine, uint8_t *buf, size_t len) {

   if (len == 0) {
    return GENERATE_ERROR_NO (ERR_LEVEL_MB_DATA, ERR_CABAC_NO_BS_TO_READ);
  }

  if (len <= 8) {
    memset(pDecEngine->buffer,0,sizeof(pDecEngine->buffer));
    memcpy(pDecEngine->buffer, buf, len);
    buf = pDecEngine->buffer;
    len = 8;
  }
  pDecEngine->pBuffEnd = buf+len;
  pDecEngine->uiOffset = ((buf[0] << 16) | (buf[1] << 8) | buf[2]);
  pDecEngine->uiOffset <<= 16;
  pDecEngine->uiOffset |= (buf[3] << 8) | buf[4];
  pDecEngine->iBitsLeft = 31;
  pDecEngine->pBuffCurr = buf + 5;
  
  pDecEngine->uiRange = WELS_CABAC_HALF;
  return ERR_NONE;
}

 int32_t DecodeBypassCabac (PWelsCabacDecEngine pDecEngine, uint32_t* uiBinVal) {
  int32_t iErrorInfo = ERR_NONE;
  int32_t iBitsLeft = pDecEngine->iBitsLeft;
  uint64_t uiOffset = pDecEngine->uiOffset;
  uint64_t uiRangeValue;


  if (iBitsLeft <= 0) {
    uint32_t uiVal = 0;
    int32_t iNumBitsRead = 0;
    iErrorInfo = Read32BitsCabac (pDecEngine, &uiVal, &iNumBitsRead);
    uiOffset = (uiOffset << iNumBitsRead) | uiVal;
    iBitsLeft = iNumBitsRead;
    if (iErrorInfo && iBitsLeft == 0) {
      return iErrorInfo;
    }
  }
  iBitsLeft--;
  uiRangeValue = (pDecEngine->uiRange << iBitsLeft);
  if (uiOffset >= uiRangeValue) {
    pDecEngine->iBitsLeft = iBitsLeft;
    pDecEngine->uiOffset = uiOffset - uiRangeValue;
    *uiBinVal = 1;
    return ERR_NONE;
  }
  pDecEngine->iBitsLeft = iBitsLeft;
  pDecEngine->uiOffset = uiOffset;
  *uiBinVal = 0;
  return ERR_NONE;
}


int32_t DecodeBinCabac (PWelsCabacDecEngine pDecEngine, uint32_t* uiBinVal) {
  int32_t iErrorInfo = ERR_NONE;
  uint32_t uiState = pDecEngine->uiState;
  *uiBinVal = pDecEngine->uiMPS;
  uint64_t uiOffset = pDecEngine->uiOffset;
  uint64_t uiRange = pDecEngine->uiRange;

  int32_t iRenorm = 1;
  uint32_t uiRangeLPS = g_kuiCabacRangeLps[uiState][ (uiRange >> 6) & 0x03];
  uiRange -= uiRangeLPS;
  if (uiOffset >= (uiRange << pDecEngine->iBitsLeft)) { //LPS
    uiOffset -= (uiRange << pDecEngine->iBitsLeft);
    *uiBinVal ^= 0x0001;
    if (!uiState)
      pDecEngine->uiMPS ^= 0x01;
    pDecEngine->uiState = g_kuiStateTransTable[uiState][0];
    iRenorm = g_kRenormTable256[uiRangeLPS];
    uiRange = (uiRangeLPS << iRenorm);
  } else {  //MPS
    pDecEngine->uiState = g_kuiStateTransTable[uiState][1];
    if (uiRange >= WELS_CABAC_QUARTER) {
      pDecEngine->uiRange = uiRange;
      return ERR_NONE;
    } else {
      uiRange <<= 1;
    }
  }
  //Renorm
  pDecEngine->uiRange = uiRange;
  pDecEngine->iBitsLeft -= iRenorm;
  if (pDecEngine->iBitsLeft > 0) {
    pDecEngine->uiOffset = uiOffset;
    return ERR_NONE;
  }
  if (pDecEngine->pBuffCurr >= pDecEngine->pBuffEnd) {
    pDecEngine->iBitsLeft = -pDecEngine->iBitsLeft;
    pDecEngine->uiOffset = uiOffset << pDecEngine->iBitsLeft;
    //printf("<no more data %016llx - %d!>",uiOffset,pDecEngine->iBitsLeft);
    return ERR_NONE;
  }
  uint32_t uiVal = 0;
  int32_t iNumBitsRead = 0;
  iErrorInfo = Read32BitsCabac (pDecEngine, &uiVal, &iNumBitsRead);
  pDecEngine->uiOffset = (uiOffset << iNumBitsRead) | uiVal;
  pDecEngine->iBitsLeft += iNumBitsRead;
  if (iErrorInfo && pDecEngine->iBitsLeft < 0) {
    return iErrorInfo;
  }
  return ERR_NONE;
}

int32_t DecodeTerminateCabac (PWelsCabacDecEngine pDecEngine, uint32_t* uiBinVal) {
  int32_t iErrorInfo = ERR_NONE;
  uint64_t uiRange = pDecEngine->uiRange - 2;
  uint64_t uiOffset = pDecEngine->uiOffset;

  if (uiOffset >= (uiRange << pDecEngine->iBitsLeft)) {
    *uiBinVal = 1;
  } else {
    *uiBinVal = 0;
    // Renorm
    if (uiRange < WELS_CABAC_QUARTER) {
      int32_t iRenorm = g_kRenormTable256[uiRange];
      pDecEngine->uiRange = (uiRange << iRenorm);
      pDecEngine->iBitsLeft -= iRenorm;
      if (pDecEngine->iBitsLeft < 0) {
        uint32_t uiVal = 0;
        int32_t iNumBitsRead = 0;
        iErrorInfo = Read32BitsCabac (pDecEngine, &uiVal, &iNumBitsRead);
        pDecEngine->uiOffset = (pDecEngine->uiOffset << iNumBitsRead) | uiVal;
        pDecEngine->iBitsLeft += iNumBitsRead;
      }
      if (iErrorInfo && pDecEngine->iBitsLeft < 0) {
        return iErrorInfo;
      }
      return ERR_NONE;
    } else {
      pDecEngine->uiRange = uiRange;
      return ERR_NONE;
    }
  }
  return ERR_NONE;
}

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#if defined(__APPLE__) || defined(__MACH__)
#include <mach/mach_time.h>
#define absolute_time mach_absolute_time
#elif defined(_WIN32)
   #include <windows.h>
static inline absolute_time() {
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  return (ft.dwHighDateTime << 32) | ft.dwLowDateTime;
}
#endif
int main() {
  srand(absolute_time());
  SCabacCtx enc = {0};
  static uint8_t buffer[65536] = {0};
  WelsCabacEncodeInit(&enc,buffer,sizeof(buffer));
  uint32_t data[2048];
  //populate data
  for (int i=0;i<2048;i++) {
    data[i] = rand() % 8 == 0; 
  }
  int BIT_COUNT = rand() % 2048;
  if (BIT_COUNT == 0)
      BIT_COUNT = 1;
  //compress data
  for (int i=0; i<BIT_COUNT; i++) {
    // if (i&1)
          WelsCabacEncodeDecision(&enc,data[i]);
    //  else
    //     WelsCabacEncodeBypassOne(&enc,data[i]);
  }
  WelsCabacEncodeFlush(&enc);
  size_t encoded_sz = WelsCabacEncodeGetPtr(&enc) - buffer;

  //printf("encoded sz=%d bc=%d\n", encoded_sz,BIT_COUNT);
  // for (int i=0;i<encoded_sz;i++) {
  //   printf(" %02.2x",buffer[i]);
  // }
  // printf("\n");
  //decompress
  SWelsCabacDecEngine dec = {0};
  int err = InitCabacDecEngine(&dec, buffer, encoded_sz);
  assert(err == ERR_NONE);
  //printf("%016llx - %d\n", dec.uiOffset, dec.iBitsLeft);
  uint32_t bit;
  for (int i=0;i<BIT_COUNT;i++) {
    
    // if (i&1)
       err = DecodeBinCabac(&dec,&bit);
    // else
    //  err = DecodeBypassCabac(&dec,&bit);
    assert(err == ERR_NONE);
    //verify
    assert(bit == data[i]);
  }
  err = DecodeTerminateCabac(&dec,&bit);
  assert(err == ERR_NONE);
  assert(bit == 1);
  // printf("{");
  // for (int i=0; i<80; i++) {
  //   uint32_t bit;
  //   err = DecodeBinCabac(&dec,&bit);
  //   printf("%u", bit);
  // }
  // printf("}\n");
   printf("%4d -> %4d bits   -   passed!\n", BIT_COUNT, encoded_sz*8);
 return 0;
}
