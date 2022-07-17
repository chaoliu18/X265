#include "common.h"
#include "x265.h"
#include "slicetype.h"
#include "frame.h"
#include "mctf.h"

using namespace X265_NS;

Mctf::Mctf()
{
    m_datOriBfr    = (pixel (*)                     [NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel                         [NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datRefBfr    = (pixel (*) [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel     [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datRefAft    = (pixel (*) [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel     [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datOriAft    = (pixel (*)                     [NUMB_CHN][SIZE_FRA_MAX_Y    ][SIZE_FRA_MAX_X    ])
        new           pixel                         [NUMB_CHN][SIZE_FRA_MAX_Y    ][SIZE_FRA_MAX_X    ];
    memset(*m_datOriBfr   , 0, sizeof(*m_datOriBfr   ));
    memset(*m_datRefBfr   , 0, sizeof(*m_datRefBfr   ));
    memset(*m_datRefAft   , 0, sizeof(*m_datRefAft   ));
    memset(*m_datOriAft   , 0, sizeof(*m_datOriAft   ));

    m_datOriLuSub2 = (pixel (*)                     [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel                         [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datOriLuSub4 = (pixel (*)                     [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel                         [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datRefLuSub2 = (pixel (*) [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel     [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    m_datRefLuSub4 = (pixel (*) [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
        new           pixel     [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    memset(*m_datOriLuSub2, 0, sizeof(*m_datOriLuSub2));
    memset(*m_datOriLuSub4, 0, sizeof(*m_datOriLuSub4));
    memset(*m_datRefLuSub2, 0, sizeof(*m_datRefLuSub2));
    memset(*m_datRefLuSub4, 0, sizeof(*m_datRefLuSub4));
}

Mctf::~Mctf()
{
    delete[] m_datOriBfr;
    delete[] m_datRefBfr;
    delete[] m_datRefAft;
    delete[] m_datOriAft;

    delete[] m_datOriLuSub2;
    delete[] m_datOriLuSub4;
    delete[] m_datRefLuSub2;
    delete[] m_datRefLuSub4;
}

void Mctf::init(Frame* srcFrame, Lookahead* lookahead)
{
    m_FrameSkip = 0;
    m_sourceWidth = srcFrame->m_fencPic->m_picWidth;
    m_sourceHeight = srcFrame->m_fencPic->m_picHeight;
    m_QP = srcFrame->m_param->rc.qp;
    m_temporalFilterStrengths[0] = 0.95;
    m_gopBasedTemporalFilterFutureReference = 1;
    m_lookahead = lookahead;
    m_frameEnc = srcFrame;
}

bool Mctf::filter(Frame* srcFrame, Lookahead* lookahead)
{
  init(srcFrame, lookahead);

  bool isFilterThisFrame = false;
  if (m_QP >= 17)  // disable filter for QP < 17
  {
    if (m_frameEnc->m_lowres.sliceType != X265_TYPE_B && m_frameEnc->m_lowres.sliceType != X265_TYPE_BREF)
    {
      isFilterThisFrame = true;
    }
  }

  if (isFilterThisFrame)
  {
    int offset = m_FrameSkip;

    std::deque<TemporalFilterSourcePicInfo> srcFrameInfo;

    int range_past = (s_range <= m_lookahead->m_outputQueue.size()) ? s_range : m_lookahead->m_outputQueue.size();
    int firstFrame = m_frameEnc->m_poc + offset - range_past;
    int lastFrame = m_frameEnc->m_poc + offset + s_range;
    if (!m_gopBasedTemporalFilterFutureReference)
    {
      lastFrame = m_frameEnc->m_poc + offset - 1;
    }
    // future frames are not available
    lastFrame = (lastFrame >= m_frameEnc->m_param->totalFrames) ? (m_frameEnc->m_param->totalFrames - 1) : lastFrame;
    int origOffset = - range_past;
    // check IDR frame
    if (m_frameEnc->m_lowres.sliceType == X265_TYPE_IDR)
    {
      firstFrame = m_frameEnc->m_poc + offset + 1;
      origOffset = 1;
    }
    int numRefPast = (firstFrame < m_frameEnc->m_poc + offset) ? m_frameEnc->m_poc + offset - firstFrame : 0;
    int numRefFutr = (lastFrame > m_frameEnc->m_poc + offset) ? lastFrame - (m_frameEnc->m_poc + offset) : 0;

    loadFileFtpOri(m_frameEnc->m_poc, &(*m_datOriBfr), &(*m_datOriLuSub2), &(*m_datOriLuSub4));

    // determine motion vectors
    for (int poc = firstFrame; poc <= lastFrame; poc++)
    {
      if (poc < 0)
      {
        origOffset++;
        continue; // frame not available
      }
      else if (poc == offset + m_frameEnc->m_poc)
      { // hop over frame that will be filtered
        origOffset++;
        continue;
      }
      srcFrameInfo.push_back(TemporalFilterSourcePicInfo());
      TemporalFilterSourcePicInfo &srcPic=srcFrameInfo.back();

      int tpfIdxFra = (poc < m_frameEnc->m_poc + offset) ? (origOffset + s_range) : (origOffset + s_range - 1);
      loadFileFtpOri(poc, &(*m_datRefBfr)[tpfIdxFra], &(*m_datRefLuSub2)[tpfIdxFra], &(*m_datRefLuSub4)[tpfIdxFra]);

      srcPic.mvs.allocate(m_sourceWidth / 4, m_sourceHeight / 4);

      // global pointer
      pixel (*pDatOriBfr)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
      pDatOriBfr = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datOriBfr)[0][TPF_MARGIN][TPF_MARGIN];
      pixel (*pDatRefBfr)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
      pDatRefBfr = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datRefBfr)[tpfIdxFra][0][TPF_MARGIN][TPF_MARGIN];
      pixel     (*pDatOriLuSub2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
      pDatOriLuSub2 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datOriLuSub2)[TPF_MARGIN][TPF_MARGIN];
      pixel     (*pDatOriLuSub4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
      pDatOriLuSub4 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datOriLuSub4)[TPF_MARGIN][TPF_MARGIN];

      motionEstimation(srcPic.mvs, &(*pDatOriBfr), &(*pDatRefBfr), &(*pDatOriLuSub2), &(*pDatOriLuSub4), tpfIdxFra);
      srcPic.origOffset = origOffset;
      origOffset++;
    }

    // filter
    double overallStrength = -1.0;
    for (std::map<int, double>::iterator it = m_temporalFilterStrengths.begin(); it != m_temporalFilterStrengths.end(); ++it)
    {
      int layer = it->first;
      double strength = it->second;
      if (layer == 0 && (IS_X265_TYPE_I(m_frameEnc->m_lowres.sliceType) || m_frameEnc->m_lowres.sliceType == X265_TYPE_P))
      {
        overallStrength = strength;
      }
      if (layer == 1 && m_frameEnc->m_lowres.sliceType == X265_TYPE_BREF)
      {
        overallStrength = strength;
      }
    }

    bilateralFilter(&(*m_datOriBfr), srcFrameInfo, &(*m_datOriAft), overallStrength, numRefPast, numRefFutr);

    copyTpfFra();

    return true;
  }
  return false;
}

void Mctf::motionEstimation(
    Array2D<MotionVector> &mv,
    const pixel (*orgPic)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const pixel (*origSubsampled2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const pixel (*origSubsampled4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
          int idxFra
  ) const
{
  const int width = m_sourceWidth;
  const int height = m_sourceHeight;
  Array2D<MotionVector> mv_0(width / 16, height / 16);
  Array2D<MotionVector> mv_1(width / 16, height / 16);
  Array2D<MotionVector> mv_2(width / 16, height / 16);

  // global pointer
  pixel     (*pDatRefLuSub2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
  pDatRefLuSub2 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datRefLuSub2)[idxFra][TPF_MARGIN][TPF_MARGIN];
  pixel     (*pDatRefLuSub4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
  pDatRefLuSub4 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*m_datRefLuSub4)[idxFra][TPF_MARGIN][TPF_MARGIN];

  motionEstimationLuma(mv_0, origSubsampled4, &(*pDatRefLuSub4), m_sourceWidth / 4, m_sourceHeight / 4, 16, NULL, 1, false);
  motionEstimationLuma(mv_1, origSubsampled2, &(*pDatRefLuSub2), m_sourceWidth / 2, m_sourceHeight / 2, 16, &mv_0, 2, false);
  motionEstimationLuma(mv_2, orgPic, buffer, m_sourceWidth, m_sourceHeight, 16, &mv_1, 2, false);

  motionEstimationLuma(mv, orgPic, buffer, m_sourceWidth, m_sourceHeight, 8, &mv_2, 1, true);
}

void Mctf::motionEstimationLuma(
    Array2D<MotionVector> &mvs,
    const pixel (*orig)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const int sizX,
    const int sizY,
    const int blockSize,
    const Array2D<MotionVector> *previous,
    const int factor,
    const bool doubleRes
  ) const
{
#if JVET_V0056_MCTF
  int range = doubleRes ? 0 : 5;
#else
  int range = 5;
#endif
  const int stepSize = blockSize;

  const int origWidth  = sizX;
  const int origHeight = sizY;

#if JVET_V0056_MCTF
  for (int blockY = 0; blockY + blockSize <= origHeight; blockY += stepSize)
  {
    for (int blockX = 0; blockX + blockSize <= origWidth; blockX += stepSize)
#else
  for (int blockY = 0; blockY + blockSize < origHeight; blockY += stepSize)
  {
    for (int blockX = 0; blockX + blockSize < origWidth; blockX += stepSize)
#endif
    {
      MotionVector best;

      if (previous == NULL)
      {
        range = 8;
      }
      else
      {
#if JVET_V0056_MCTF
        for (int py = -1; py <= 1; py++)
#else
        for (int py = -2; py <= 2; py++)
#endif
        {
          int testy = blockY / (2 * blockSize) + py;
#if JVET_V0056_MCTF
          for (int px = -1; px <= 1; px++)
#else
          for (int px = -2; px <= 2; px++)
#endif
          {
            int testx = blockX / (2 * blockSize) + px;
            if ((testx >= 0) && (testx < origWidth / (2 * blockSize)) && (testy >= 0) && (testy < origHeight / (2 * blockSize)))
            {
              MotionVector old = previous->get(testx, testy);
              int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, old.x * factor, old.y * factor, blockSize, best.error);
              if (error < best.error)
              {
                best.set(old.x * factor, old.y * factor, error);
              }
            }
          }
        }
#if JVET_V0056_MCTF
        int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, 0, 0, blockSize, best.error);
        if (error < best.error)
        {
          best.set(0, 0, error);
        }
#endif
      }
      MotionVector prevBest = best;
      for (int y2 = prevBest.y / s_motionVectorFactor - range; y2 <= prevBest.y / s_motionVectorFactor + range; y2++)
      {
        for (int x2 = prevBest.x / s_motionVectorFactor - range; x2 <= prevBest.x / s_motionVectorFactor + range; x2++)
        {
          int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, x2 * s_motionVectorFactor, y2 * s_motionVectorFactor, blockSize, best.error);
          if (error < best.error)
          {
            best.set(x2 * s_motionVectorFactor, y2 * s_motionVectorFactor, error);
          }
        }
      }
      if (doubleRes)
      { // merge into one loop, probably with precision array (here [12, 3] or maybe [4, 1]) with setable number of iterations
        prevBest = best;
        int doubleRange = 3 * 4;
        for (int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2 += 4)
        {
          for (int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2 += 4)
          {
            int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }
          }
        }

        prevBest = best;
        doubleRange = 3;
        for (int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2++)
        {
          for (int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2++)
          {
            int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }
          }
        }
      }
#if JVET_V0056_MCTF
      if (blockY > 0)
      {
        MotionVector aboveMV = mvs.get(blockX / stepSize, (blockY - stepSize) / stepSize);
        int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, aboveMV.x, aboveMV.y, blockSize, best.error);
        if (error < best.error)
        {
          best.set(aboveMV.x, aboveMV.y, error);
        }
      }
      if (blockX > 0)
      {
        MotionVector leftMV = mvs.get((blockX - stepSize) / stepSize, blockY / stepSize);
        int error = motionErrorLuma(orig, buffer, sizX, sizY, blockX, blockY, leftMV.x, leftMV.y, blockSize, best.error);
        if (error < best.error)
        {
          best.set(leftMV.x, leftMV.y, error);
        }
      }

      const pixel* origOrigin = &(*orig)[0][0];
      const int origStride  = TPF_SIZE_FRA_MAX_X;

      // calculate average
      double avg = 0.0;
      for (int x1 = 0; x1 < blockSize; x1++)
      {
        for (int y1 = 0; y1 < blockSize; y1++)
        {
          avg = avg + *(origOrigin + (blockX + x1 + origStride * (blockY + y1)));
        }
      }
      avg = avg / (blockSize * blockSize);

      // calculate variance
      double variance = 0;
      for (int x1 = 0; x1 < blockSize; x1++)
      {
        for (int y1 = 0; y1 < blockSize; y1++)
        {
          int pix = *(origOrigin + (blockX + x1 + origStride * (blockY + y1)));
          variance = variance + (pix - avg) * (pix - avg);
        }
      }
      best.error = (int) (20 * ((best.error + 5.0) / (variance + 5.0)) + (best.error / (blockSize * blockSize)) / 50);
#endif
      mvs.get(blockX / stepSize, blockY / stepSize) = best;
    }
  }
}

int Mctf::motionErrorLuma(
    const pixel (*orig)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    const int sizX,
    const int sizY,
    const int x,
    const int y,
          int dx,
          int dy,
    const int bs,
    const int besterror = 8 * 8 * 1024 * 1024) const
{

  const pixel* origOrigin = &(*orig)[0][0];
  const int origStride  = TPF_SIZE_FRA_MAX_X;
  const pixel *buffOrigin = &(*buffer)[0][0];
  const int buffStride  = TPF_SIZE_FRA_MAX_X;
  int error = 0;// dx * 10 + dy * 10;
  if (((dx | dy) & 0xF) == 0)
  {
    dx /= s_motionVectorFactor;
    dy /= s_motionVectorFactor;
    for (int y1 = 0; y1 < bs; y1++)
    {
      const pixel* origRowStart = origOrigin + (y+y1)*origStride + x;
      const pixel* bufferRowStart = buffOrigin + (y+y1+dy)*buffStride + (x+dx);
      for (int x1 = 0; x1 < bs; x1 += 2)
      {
        int diff = origRowStart[x1] - bufferRowStart[x1];
        error += diff * diff;
        diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
        error += diff * diff;
      }
      if (error > besterror)
      {
        return error;
      }
    }
  }
  else
  {
    const int *xFilter = s_interpolationFilter[dx & 0xF];
    const int *yFilter = s_interpolationFilter[dy & 0xF];
    int tempArray[64 + 8][64];

    int iSum, iBase;
    for (int y1 = 1; y1 < bs + 7; y1++)
    {
      const int yOffset = y + y1 + (dy >> 4) - 3;
      const pixel *sourceRow = buffOrigin + (yOffset)*buffStride + 0;
      for (int x1 = 0; x1 < bs; x1++)
      {
        iSum = 0;
        iBase = x + x1 + (dx >> 4) - 3;
        const pixel *rowStart = sourceRow + iBase;

        iSum += xFilter[1] * rowStart[1];
        iSum += xFilter[2] * rowStart[2];
        iSum += xFilter[3] * rowStart[3];
        iSum += xFilter[4] * rowStart[4];
        iSum += xFilter[5] * rowStart[5];
        iSum += xFilter[6] * rowStart[6];

        tempArray[y1][x1] = iSum;
      }
    }

    for (int y1 = 0; y1 < bs; y1++)
    {
      const pixel *origRow = origOrigin + (y+y1)*origStride + 0;
      for (int x1 = 0; x1 < bs; x1++)
      {
        iSum = 0;
        iSum += yFilter[1] * tempArray[y1 + 1][x1];
        iSum += yFilter[2] * tempArray[y1 + 2][x1];
        iSum += yFilter[3] * tempArray[y1 + 3][x1];
        iSum += yFilter[4] * tempArray[y1 + 4][x1];
        iSum += yFilter[5] * tempArray[y1 + 5][x1];
        iSum += yFilter[6] * tempArray[y1 + 6][x1];

        iSum = (iSum + (1 << 11)) >> 12;
        iSum = iSum < 0 ? 0 : (iSum > DATA_PXL_MAX ? DATA_PXL_MAX : iSum);

        error += (iSum - origRow[x + x1]) * (iSum - origRow[x + x1]);
      }
      if (error > besterror)
      {
        return error;
      }
    }
  }
  return error;
}

void Mctf::bilateralFilter(const pixel (*orgPic)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
#if JVET_V0056_MCTF
                                std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
#else
                                std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
#endif
                                pixel (*newOrgPic)[NUMB_CHN][SIZE_FRA_MAX_Y][SIZE_FRA_MAX_X],
                                double overallStrength,
                                int numRefPast,
                                int numRefFutr) const
{
  const int numRefs = int(srcFrameInfo.size());
  for (int i = 0; i < numRefs; i++)
  {
    int tpfIdxFra = (srcFrameInfo[i].origOffset < 0) ? (srcFrameInfo[i].origOffset + s_range) : (srcFrameInfo[i].origOffset + s_range - 1);
    applyMotion(srcFrameInfo[i].mvs, &(*m_datRefBfr)[tpfIdxFra], &(*m_datRefAft)[tpfIdxFra]);
  }

  int refStrengthRow = 2;
  if ((numRefFutr > 0) && (numRefPast > 0))
  {
    refStrengthRow = 0;
  }
  else if ((numRefFutr > 0) || (numRefPast > 0))
  {
    refStrengthRow = 1;
  }

  const double lumaSigmaSq = (m_QP - s_sigmaZeroPoint) * (m_QP - s_sigmaZeroPoint) * s_sigmaMultiplier;
  const double chromaSigmaSq = 30 * 30;
  
  for(int c=0; c<NUMB_CHN; c++)
  {
    int datShf = c == 0 ? 0 : DATA_SHF_CH;
    const int height = m_sourceHeight >> datShf;
    const int width  = m_sourceWidth >> datShf;
    const pixel *srcPelRow = &(*orgPic)[c][TPF_MARGIN >> datShf][TPF_MARGIN >> datShf];
    const int srcStride = TPF_SIZE_FRA_MAX_X;
          pixel *dstPelRow = &(*newOrgPic)[c][0][0];
    const int dstStride = SIZE_FRA_MAX_X;
    const double sigmaSq = (c > 0) ? chromaSigmaSq : lumaSigmaSq;
    const double weightScaling = overallStrength * ((c > 0) ? s_chromaFactor : 0.4);
    const double bitDepthDiffWeighting=1024.0 / (DATA_PXL_MAX+1);
#if JVET_V0056_MCTF
    static const int lumaBlockSize=8;    
    const int csx = c == 0 ? 0 : DATA_SHF_CH;
    const int csy = c == 0 ? 0 : DATA_SHF_CH;
    const int blkSizeX = lumaBlockSize>>csx;
    const int blkSizeY = lumaBlockSize>>csy;    
#endif

    for (int y = 0; y < height; y++, srcPelRow+=srcStride, dstPelRow+=dstStride)
    {
      const pixel *srcPel=srcPelRow;
            pixel *dstPel=dstPelRow;
      for (int x = 0; x < width; x++, srcPel++, dstPel++)
      {
        const int orgVal = (int) *srcPel;
        double temporalWeightSum = 1.0;
        double newVal = (double) orgVal;
#if JVET_V0056_MCTF
        if ((y % blkSizeY == 0) && (x % blkSizeX == 0))
        {
          for (int i = 0; i < numRefs; i++)
          {
            int tpfIdxFra = (srcFrameInfo[i].origOffset < 0) ? (srcFrameInfo[i].origOffset + s_range) : (srcFrameInfo[i].origOffset + s_range - 1);
            double variance = 0, diffsum = 0;
            for (int y1 = 0; y1 < blkSizeY - 1; y1++)
            {
              for (int x1 = 0; x1 < blkSizeX - 1; x1++)
              {
                int pix  = *(srcPel + x1);
                int pixR = *(srcPel + x1 + 1);
                int pixD = *(srcPel + x1 + srcStride);
                int ref  = (*m_datRefAft)[tpfIdxFra][c][(TPF_MARGIN >> datShf) + y + y1    ][(TPF_MARGIN >> datShf) + x + x1    ];
                int refR = (*m_datRefAft)[tpfIdxFra][c][(TPF_MARGIN >> datShf) + y + y1    ][(TPF_MARGIN >> datShf) + x + x1 + 1];
                int refD = (*m_datRefAft)[tpfIdxFra][c][(TPF_MARGIN >> datShf) + y + y1 + 1][(TPF_MARGIN >> datShf) + x + x1    ];

                int diff  = pix  - ref;
                int diffR = pixR - refR;
                int diffD = pixD - refD;

                variance += diff * diff;
                diffsum  += (diffR - diff) * (diffR - diff);
                diffsum  += (diffD - diff) * (diffD - diff);
              }
            }
            srcFrameInfo[i].mvs.get(x / blkSizeX, y / blkSizeY).noise = (int) round((300 * variance + 50) / (10 * diffsum + 50));
          }
        }

        double minError = 9999999;
        for (int i = 0; i < numRefs; i++)
        {
          minError = std::min(minError, (double) srcFrameInfo[i].mvs.get(x / blkSizeX, y / blkSizeY).error);
        }
#endif
        for (int i = 0; i < numRefs; i++)
        {
          int tpfIdxFra = (srcFrameInfo[i].origOffset < 0) ? (srcFrameInfo[i].origOffset + s_range) : (srcFrameInfo[i].origOffset + s_range - 1);
#if JVET_V0056_MCTF
          const int error = srcFrameInfo[i].mvs.get(x / blkSizeX, y / blkSizeY).error;
          const int noise = srcFrameInfo[i].mvs.get(x / blkSizeX, y / blkSizeY).noise;
#endif
          const pixel *pCorrectedPelPtr=&(*m_datRefAft)[tpfIdxFra][c][(TPF_MARGIN >> datShf) + y][(TPF_MARGIN >> datShf) + x];
          const int refVal = (int) *pCorrectedPelPtr;
          double diff = (double)(refVal - orgVal);
          diff *= bitDepthDiffWeighting;
          double diffSq = diff * diff;
#if JVET_V0056_MCTF
          const int index = std::min(3, std::abs(srcFrameInfo[i].origOffset) - 1);
          double ww = 1, sw = 1;
          ww *= (noise < 25) ? 1 : 1.2;
          sw *= (noise < 25) ? 1.3 : 0.8;
          ww *= (error < 50) ? 1.2 : ((error > 100) ? 0.8 : 1);
          sw *= (error < 50) ? 1.3 : 1;
          ww *= ((minError + 1) / (error + 1));
          const double weight = weightScaling * s_refStrengths[refStrengthRow][index] * ww * exp(-diffSq / (2 * sw * sigmaSq));
#else
          const int index = std::min(1, std::abs(srcFrameInfo[i].origOffset) - 1);
          const double weight = weightScaling * s_refStrengths[refStrengthRow][index] * exp(-diffSq / (2 * sigmaSq));
#endif
          newVal += weight * refVal;
          temporalWeightSum += weight;
        }
        newVal /= temporalWeightSum;
        pixel sampleVal = (pixel)round(newVal);
        sampleVal=(sampleVal<0?0 : (sampleVal>DATA_PXL_MAX ? DATA_PXL_MAX : sampleVal));
        *dstPel = sampleVal;
      }
    }
  }
}

void Mctf::applyMotion(const Array2D<MotionVector> &mvs, const pixel (*input)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X], pixel (*output)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) const
{
  static const int lumaBlockSize=8;

  for (int c=0; c<NUMB_CHN; c++)
  {
    const int csx = c == 0 ? 0 : DATA_SHF_CH;
    const int csy = c == 0 ? 0 : DATA_SHF_CH;
    const int blockSizeX = lumaBlockSize>>csx;
    const int blockSizeY = lumaBlockSize>>csy;
    const int height = m_sourceHeight >> csy;
    const int width  = m_sourceWidth >> csx;

    const pixel *pSrcImage=&(*input)[c][TPF_MARGIN >> csy][TPF_MARGIN >> csx];
    const int srcStride=TPF_SIZE_FRA_MAX_X;

          pixel *pDstImage=&(*output)[c][TPF_MARGIN >> csy][TPF_MARGIN >> csx];
          int dstStride=TPF_SIZE_FRA_MAX_X;

    for (int y = 0, blockNumY = 0; y + blockSizeY <= height; y += blockSizeY, blockNumY++)
    {
      for (int x = 0, blockNumX = 0; x + blockSizeX <= width; x += blockSizeX, blockNumX++)
      {
        const MotionVector &mv = mvs.get(blockNumX,blockNumY);
        const int dx = mv.x >> csx;
        const int dy = mv.y >> csy;
        const int xInt = mv.x >> (4+csx);
        const int yInt = mv.y >> (4+csy);

        const int *xFilter = s_interpolationFilter[dx & 0xf];
        const int *yFilter = s_interpolationFilter[dy & 0xf]; // will add 6 bit.
        const int numFilterTaps=7;
        const int centreTapOffset=3;

        int tempArray[lumaBlockSize + numFilterTaps][lumaBlockSize];

        for (int by = 1; by < blockSizeY + numFilterTaps; by++)
        {
          const int yOffset = y + by + yInt - centreTapOffset;
          const pixel *sourceRow = pSrcImage+yOffset*srcStride;
          for (int bx = 0; bx < blockSizeX; bx++)
          {
            int iBase = x + bx + xInt - centreTapOffset;
            const pixel *rowStart = sourceRow + iBase;

            int iSum = 0;
            iSum += xFilter[1] * rowStart[1];
            iSum += xFilter[2] * rowStart[2];
            iSum += xFilter[3] * rowStart[3];
            iSum += xFilter[4] * rowStart[4];
            iSum += xFilter[5] * rowStart[5];
            iSum += xFilter[6] * rowStart[6];

            tempArray[by][bx] = iSum;
          }
        }

        pixel *pDstRow=pDstImage+y*dstStride;
        for (int by = 0; by < blockSizeY; by++, pDstRow+=dstStride)
        {
          pixel *pDstPel=pDstRow+x;
          for (int bx = 0; bx < blockSizeX; bx++, pDstPel++)
          {
            int iSum = 0;

            iSum += yFilter[1] * tempArray[by + 1][bx];
            iSum += yFilter[2] * tempArray[by + 2][bx];
            iSum += yFilter[3] * tempArray[by + 3][bx];
            iSum += yFilter[4] * tempArray[by + 4][bx];
            iSum += yFilter[5] * tempArray[by + 5][bx];
            iSum += yFilter[6] * tempArray[by + 6][bx];

            iSum = (iSum + (1 << 11)) >> 12;
            iSum = iSum < 0 ? 0 : (iSum > DATA_PXL_MAX ? DATA_PXL_MAX : iSum);
            *pDstPel = iSum;
          }
        }
      }
    }
  }
}

void Mctf::loadFileFtpOri(
    int poc,
    pixel (*pDst)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    pixel (*pDstSub2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
    pixel (*pDstSub4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X])
{
    Frame* curFrame;
    if (poc == m_frameEnc->m_poc)
        curFrame = m_frameEnc;
    else if (poc > m_frameEnc->m_poc)
        curFrame = m_lookahead->m_inputQueue.getPOC(poc);
    else
        curFrame = m_lookahead->m_outputQueue.getPOC(poc);

    for (int idxChn = 0; idxChn < NUMB_CHN; ++idxChn) {
        int datShf = idxChn == 0 ? 0 : DATA_SHF_CH;
        int datStr = idxChn == 0 ? curFrame->m_fencPic->m_stride : curFrame->m_fencPic->m_strideC;
        // global pointer
        pixel (*pDatDst)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
        pDatDst = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*pDst)[idxChn][TPF_MARGIN >> datShf][TPF_MARGIN >> datShf];
        
        for (int j = 0; j < m_sourceHeight >> datShf; ++j) {
            for (int i = 0; i < m_sourceWidth >> datShf; ++i) {
                (*pDatDst)[j][i] = *(curFrame->m_fencPic->m_picOrg[idxChn] + j * datStr + i);
            }
        }
    }

    // extend
    for (int idxChn = 0; idxChn < NUMB_CHN; ++idxChn) {
        int datShf = idxChn == 0 ? 0 : DATA_SHF_CH;
        // global pointer
        pixel (*pDatDst)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
        pDatDst = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*pDst)[idxChn][TPF_MARGIN >> datShf][TPF_MARGIN >> datShf];

        for (int j = 0; j < m_sourceHeight >> datShf; ++j) {
            for (int i = m_sourceWidth >> datShf; i < m_sourceWidth >> datShf; ++i)
                (*pDatDst)[j][i] = (*pDatDst)[j][(m_sourceWidth >> datShf) - 1];
        }
        for (int j = m_sourceHeight >> datShf; j < m_sourceHeight >> datShf; ++j) {
            for (int i = 0; i < m_sourceWidth >> datShf; ++i)
                (*pDatDst)[j][i] = (*pDatDst)[(m_sourceHeight >> datShf) - 1][i];
        }
    }

    // global pointer
    pixel     (*pDatDst)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pDatDst = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*pDst)[0][TPF_MARGIN][TPF_MARGIN];
    pixel     (*pDatDstSub2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pDatDstSub2 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*pDstSub2)[TPF_MARGIN][TPF_MARGIN];
    pixel     (*pDatDstSub4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pDatDstSub4 = (pixel (*)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) &(*pDstSub4)[TPF_MARGIN][TPF_MARGIN];

    // 2 times subsample
    for (int j = 0; j < m_sourceHeight >> 1; ++j) {
        for (int i = 0; i < m_sourceWidth >> 1; ++i) {
            int sum = 0;
            for (int n = 0; n < TPF_SIZE_STD_Y; n++) {
                for (int m = 0; m < TPF_SIZE_STD_X; m++) {
                    sum += (*pDatDst)[j * TPF_SIZE_STD_Y + n][i * TPF_SIZE_STD_X + m];
                }
            }
            (*pDatDstSub2)[j][i] = (sum + 2) >> 2; // average pooling
        }
    }

    // 4 times subsample
    for (int j = 0; j < m_sourceHeight >> 2; ++j) {
        for (int i = 0; i < m_sourceWidth >> 2; ++i) {
            int sum = 0;
            for (int n = 0; n < TPF_SIZE_STD_Y; n++) {
                for (int m = 0; m < TPF_SIZE_STD_X; m++) {
                    sum += (*pDatDstSub2)[j * TPF_SIZE_STD_Y + n][i * TPF_SIZE_STD_X + m];
                }
            }
            (*pDatDstSub4)[j][i] = (sum + 2) >> 2; // average pooling
        }
    }

    // padding
    for (int c = 0; c < NUMB_CHN; ++c) {
        extendPicBorder(&(*pDst)[c], c, 1);
    }
    extendPicBorder(&(*pDstSub2), 0, 2);
    extendPicBorder(&(*pDstSub4), 0, 4);
}

void Mctf::extendPicBorder(pixel (*input)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X], int idxChn, int factor)
{
    const int csx = idxChn == 0 ? 0 : DATA_SHF_CH;
    const int csy = idxChn == 0 ? 0 : DATA_SHF_CH;
    pixel *piTxt=&(*input)[TPF_MARGIN >> csy][TPF_MARGIN >> csx]; // piTxt = point to (0,0) of image within bigger picture.
    const int stride=TPF_SIZE_FRA_MAX_X;
    const int width=(m_sourceWidth >> csx) / factor;
    const int height=(m_sourceHeight >> csy) / factor;
    const int marginX=TPF_MARGIN >> csx;
    const int marginY=TPF_MARGIN >> csy;

    pixel*  pi = piTxt;
    // do left and right margins
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < marginX; x++ )
        {
        pi[ -marginX + x ] = pi[0];
        pi[    width + x ] = pi[width-1];
        }
        pi += stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (stride + marginX);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < marginY; y++ )
    {
        ::memcpy( pi + (y+1)*stride, pi, sizeof(pixel)*(width + (marginX<<1)) );
    }

    // pi is still (-marginX, height-1)
    pi -= ((height-1) * stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < marginY; y++ )
    {
        ::memcpy( pi - (y+1)*stride, pi, sizeof(pixel)*(width + (marginX<<1)) );
    }
}

void Mctf::copyTpfFra()
{
    for (int idxChn = 0; idxChn < NUMB_CHN; ++idxChn) {
        int datShf = idxChn == 0 ? 0 : DATA_SHF_CH;
        int datStr = idxChn == 0 ? m_frameEnc->m_fencPic->m_stride : m_frameEnc->m_fencPic->m_strideC;
        for (int j = 0; j < m_sourceHeight >> datShf; ++j) {
            for (int i = 0; i < m_sourceWidth >> datShf; ++i) {
                *(m_frameEnc->m_fencPic->m_picOrg[idxChn] + j * datStr + i) = (*m_datOriAft)[idxChn][j][i];
            }
        }
    }
}
