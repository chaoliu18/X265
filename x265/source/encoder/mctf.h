#ifndef X265_MCTF_H
#define X265_MCTF_H

#include "common.h"
#include <map>
#include <deque>
#include <vector>

namespace X265_NS {
// private namespace

//*** DEFINE *******************************************************************
    #define JVET_V0056_MCTF      1       // JVET-V0056: Changes to MCTF

    // global
    #define NUMB_CHN             3
    #define SIZE_FRA_MAX_X       4096
    #define SIZE_FRA_MAX_Y       4096
    #define DATA_PXL_WD          8
    #define DATA_PXL_MAX         ((1 << (DATA_PXL_WD)) - 1)
    #define DATA_SHF_CH          1

    // pixel
    #define TPF_SIZE_STD_Y       2                                       // vertical down-sample stride
    #define TPF_SIZE_STD_X       2                                       // horizontal down-sample stride
    #define TPF_SIZE_RANGE       4                                       // size of temporal reference range
    #define TPF_MARGIN           (128 + 16)                              // margin
    #define TPF_SIZE_FRA_MAX_X   (2 * TPF_MARGIN + SIZE_FRA_MAX_X)
    #define TPF_SIZE_FRA_MAX_Y   (2 * TPF_MARGIN + SIZE_FRA_MAX_Y)


//*** STRUCT *******************************************************************
    // HM
    struct MotionVector
    {
        int x, y;
        int error;
    #if JVET_V0056_MCTF
        int noise;
        MotionVector() : x(0), y(0), error(INT_LEAST32_MAX), noise(0) {}
    #else
        MotionVector() : x(0), y(0), error(INT_LEAST32_MAX)
        {
        }
    #endif
        void set(int nx, int ny, int ne)
        {
            x = nx;
            y = ny;
            error = ne;
        }
    };

    template <class T>
    struct Array2D
    {
    private:
        unsigned int m_width, m_height;
        std::vector<T> v;

    public:
        Array2D() : m_width(0), m_height(0), v() {}
        Array2D(unsigned int width, unsigned int height, const T &value = T()) : m_width(0), m_height(0), v() { allocate(width, height, value); }

        void allocate(unsigned int width, unsigned int height, const T &value = T())
        {
            m_width = width;
            m_height = height;
            v.resize(std::size_t(m_width * m_height), value);
        }

        T &get(unsigned int x, unsigned int y)
        {
            assert(x < m_width && y < m_height);
            return v[y * m_width + x];
        }

        const T &get(unsigned int x, unsigned int y) const
        {
            assert(x < m_width && y < m_height);
            return v[y * m_width + x];
        }
    };

    struct TemporalFilterSourcePicInfo
    {
        TemporalFilterSourcePicInfo() : mvs(), origOffset(0) {}
        Array2D<MotionVector> mvs;
        int origOffset;
    };

//*** CLASS ********************************************************************
class Mctf
{
//--- PUBLIC FUNCTION ------------------
public:
    Mctf();
    ~Mctf();
    bool filter(Frame* srcFrame, Lookahead* lookahead);

private:
//--- FUNCTION -----------------
    // load pixels
    void loadFileFtpOri(
        int poc,
        pixel (*pDst)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        pixel (*pDstSub2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        pixel (*pDstSub4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]); // bool m_loadFileOriFlg = 0; FILE *m_loadFileOriFpt;
    void extendPicBorder(pixel (*input)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X], int idxChn, int factor);
    void copyTpfFra();

    // HM
    void init(Frame* srcFrame, Lookahead* lookahead);
    void motionEstimation(
        Array2D<MotionVector> &mv,
        const pixel (*orgPic)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const pixel (*origSubsampled2)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const pixel (*origSubsampled4)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        int idxFra) const;
    void motionEstimationLuma(
        Array2D<MotionVector> &mvs,
        const pixel (*orig)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const int sizX,
        const int sizY,
        const int blockSize,
        const Array2D<MotionVector> *previous,
        const int factor,
        const bool doubleRes) const;
    int motionErrorLuma(
        const pixel (*orig)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const pixel (*buffer)[TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
        const int sizX,
        const int sizY,
        const int x,
        const int y,
                int dx,
                int dy,
        const int bs,
        const int besterror) const;
    #if JVET_V0056_MCTF
        void bilateralFilter(
        const pixel (*orgPic)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
                std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
                pixel (*newOrgPic)[NUMB_CHN][SIZE_FRA_MAX_Y][SIZE_FRA_MAX_X],
                double overallStrength,
                int numRefPast,
                int numRefFutr) const;
    #else
        void bilateralFilter(
        const pixel (*orgPic)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X],
                std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
                pixel (*newOrgPic)[NUMB_CHN][SIZE_FRA_MAX_Y][SIZE_FRA_MAX_Y],
                double overallStrength,
                int numRefPast,
                int numRefFutr) const;
    #endif
    void applyMotion(const Array2D<MotionVector> &mvs, const pixel (*input)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X], pixel (*output)[NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X]) const;


//--- VARIABLE -----------------
    // HM
    int m_FrameSkip;
    int m_sourceWidth;
    int m_sourceHeight;
    int m_QP;
    std::map<int, double> m_temporalFilterStrengths;
    bool m_gopBasedTemporalFilterFutureReference;

    // pixel (ori, ref, mc, filtered)
    pixel (*m_datOriBfr)                     [NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datRefBfr) [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datRefAft) [TPF_SIZE_RANGE * 2][NUMB_CHN][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datOriAft)                     [NUMB_CHN][SIZE_FRA_MAX_Y    ][SIZE_FRA_MAX_X    ];

    // pixel (lumaSubsample)
    pixel (*m_datOriLuSub2)                     [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datOriLuSub4)                     [TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datRefLuSub2) [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];
    pixel (*m_datRefLuSub4) [TPF_SIZE_RANGE * 2][TPF_SIZE_FRA_MAX_Y][TPF_SIZE_FRA_MAX_X];

    // interface with x265
    Lookahead* m_lookahead;
    Frame* m_frameEnc;

//--- TABLE --------------------
    #if JVET_V0056_MCTF
    const int s_range = 3;
    #else
    const int s_range = 2;
    #endif
    const double s_chromaFactor = 0.55;
    const double s_sigmaMultiplier = 9.0;
    const double s_sigmaZeroPoint = 10.0;
    const int s_motionVectorFactor = 16;
    const int s_padding = 128;
    const int s_interpolationFilter[16][8] =
    {
        {   0,   0,   0,  64,   0,   0,   0,   0 },   //0
        {   0,   1,  -3,  64,   4,  -2,   0,   0 },   //1 -->-->
        {   0,   1,  -6,  62,   9,  -3,   1,   0 },   //2 -->
        {   0,   2,  -8,  60,  14,  -5,   1,   0 },   //3 -->-->
        {   0,   2,  -9,  57,  19,  -7,   2,   0 },   //4
        {   0,   3, -10,  53,  24,  -8,   2,   0 },   //5 -->-->
        {   0,   3, -11,  50,  29,  -9,   2,   0 },   //6 -->
        {   0,   3, -11,  44,  35, -10,   3,   0 },   //7 -->-->
        {   0,   1,  -7,  38,  38,  -7,   1,   0 },   //8
        {   0,   3, -10,  35,  44, -11,   3,   0 },   //9 -->-->
        {   0,   2,  -9,  29,  50, -11,   3,   0 },   //10-->
        {   0,   2,  -8,  24,  53, -10,   3,   0 },   //11-->-->
        {   0,   2,  -7,  19,  57,  -9,   2,   0 },   //12
        {   0,   1,  -5,  14,  60,  -8,   2,   0 },   //13-->-->
        {   0,   1,  -3,   9,  62,  -6,   1,   0 },   //14-->
        {   0,   0,  -2,   4,  64,  -3,   1,   0 }    //15-->-->
    };
    #if JVET_V0056_MCTF
    const double s_refStrengths[3][4] =
    { // abs(POC offset)
    //  1,    2     3     4
    {0.85, 0.57, 0.41, 0.33},  // m_range * 2
    {1.13, 0.97, 0.81, 0.57},  // m_range
    {0.30, 0.30, 0.30, 0.30}   // otherwise
    };
    #else
    const double s_refStrengths[3][2] =
    { // abs(POC offset)
    //  1,    2
    {0.85, 0.60},  // s_range * 2
    {1.20, 1.00},  // s_range
    {0.30, 0.30}   // otherwise
    };
    #endif
};
}

#endif // ifndef X265_MCTF_H
