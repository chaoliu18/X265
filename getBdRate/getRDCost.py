#!/usr/bin/python3
#-------------------------------------------------------------------------------
    #
    #  The confidential and proprietary information contained in this file may
    #  only be used by a person authorised under and to the extent permitted
    #  by a subsisting licensing agreement from XK Silicon.
    #
    #                   (C) COPYRIGHT 2020 XK Silicon.
    #                       ALL RIGHTS RESERVED
    #
    #  This entire notice must be reproduced on all copies of this file
    #  and copies of this file may only be made by a person if such person is
    #  permitted to do so under the terms of a subsisting license agreement
    #  from XK Silicon.
    #
    #  Revision       : 112933
    #  Release        : XK265
    #
#-------------------------------------------------------------------------------
    #
    #  Filename       : getBdRate.py
    #  Author         : Liu Chao
    #  Status         : phase 003
    #  Reset          : 2022-6-3
    #  Description    : calculate Rate Distortion Cost
    #
#-------------------------------------------------------------------------------

#*** IMPORT ********************************************************************
import sys
import re
import numpy as np

#**** TABLE ********************************************************************
# s_cst_datLmd_2: 0.038 * exp(0.234 * QP) * (1 << (DATA_BIT_DEPTH - 8)) ^ 2
s_cst_datLmd_2 = [
                 0.0380,      0.0480,      0.0606,      0.0766,      0.0968,
                 0.1224,      0.1547,      0.1955,      0.2470,      0.3121,
                 0.3944,      0.4984,      0.6299,      0.7959,      1.0058,
                 1.2710,      1.6061,      2.0295,      2.5646,      3.2408,
                 4.0952,      5.1749,      6.5393,      8.2633,     10.4419,
                13.1949,     16.6736,     21.0695,     26.6244,     33.6438,
                42.5138,     53.7224,     67.8860,     85.7838,    108.4003,
               136.9794,    173.0933,    218.7284,    276.3949,    349.2649,
               441.3467,    557.7054,    704.7413,    890.5425,   1125.3291,
              1422.0160,   1796.9227,   2270.6714,   2869.3215,   3625.8023,
              4581.7251,   5789.6717,   7316.0868,   9244.9328,  11682.3084,
             14762.2847,  18654.2798,  23572.3779,  29787.1055,  37640.3119,
             47563.9728,  60103.9523,  75950.0283,  95973.8349, 121276.8079,
            153250.7703, 193654.4919, 244710.4321, 309226.9897, 390752.9823
]

#*** DICT **********************************************************************
dict_resolution = {
    "BasketballPass" : 416 * 240,
    "BQSquare" : 416 * 240,
    "BlowingBubbles" : 416 * 240,
    "RaceHorses" : 416 * 240,
    "BasketballDrill" : 832 * 480,
    "BQMall" : 832 * 480,
    "PartyScene" : 832 * 480,
    "RaceHorsesC" : 832 * 480,
    "FourPeople" : 1280 * 720,
    "Johnny" : 1280 * 720,
    "KristenAndSara" : 1280 * 720,
    "Kimono" : 1920 * 1080,
    "ParkScene" : 1920 * 1080,
    "Cactus" : 1920 * 1080,
    "BasketballDrive" : 1920 * 1080,
    "BQTerrace" : 1920 * 1080,
    "Traffic" : 2560 * 1600,
    "PeopleOnStreet" : 2560 * 1600,
}

#*** FUNCTION ******************************************************************
# getDat
def getDat(fpt, strTag):
    # pop and check type items
    strLineCur = next(fpt).rstrip()
    strTypeAll = re.split("\s{2,}", strLineCur)
    for idx in range(len(strTypeAll)):
        if strTypeAll[idx] != CSTR_TYPE[idx]:
            assert False, "\n[getBdRate.py] the {:d}(st/nd/rd/th) type item \"{:s}\" in {:s} is incorrect\n".format(idx, strTypeAll[idx], strTag)

    # pop and check info items
    strLineCur = next(fpt).rstrip()
    strInfoAll = re.split("\s{2,}", strLineCur)
    for idx in range(len(strInfoAll)):
        if strInfoAll[idx] != CSTR_INFO_BFR[idx % 4]:
            assert False, "\n[getBdRate.py] the {:d}(st/nd/rd/th) info item \"{:s}\" in {:s} is incorrect\n".format(idx, strInfoAll[idx], strTag)
    if len(strInfoAll) != 4 * len(strTypeAll):
        assert False, "\n[getBdRate.py] numbers of type and info mismatch!"

    # main body
    datFul = {}
    for strLineCur in fpt:
        # get info
        [*strDat, strSeq] = strLineCur.split()
        [strSeq, strQp] = strSeq.split(sep = "_")
        datQp = int(strQp)
        # create seq key
        if not strSeq in datFul:
            datFul[strSeq] = {}
        for idx in range(len(strDat) // 4):
            # create type key
            if not CSTR_TYPE[idx] in datFul[strSeq]:
                datFul[strSeq][CSTR_TYPE[idx]] = {}

            # create qp key
            if not datQp in datFul[strSeq][CSTR_TYPE[idx]]:
                datFul[strSeq][CSTR_TYPE[idx]][datQp] = {}
            else:
                assert False, "\n[getBdRate.py] SEQ {:s} QP {:d} of occurs more than once!\n".format(strSeq, datQp)

            # prepare data
            # !!! sure, we can go on with info key, however i don't think it is worth doing so
            dat = [float(x) for x in strDat[idx * 4: (idx + 1) * 4]]
            # here 0 for bitrate, 1 for psnr y, 2 for psnr u, 3 for psnr v, 4 for psnr average
            dat.append((dat[1] + dat[2] / datSclCh / datSclCh + dat[3] / datSclCh / datSclCh) / (1 + 1 / datSclCh / datSclCh + 1 / datSclCh / datSclCh))

            # set data
            datFul[strSeq][CSTR_TYPE[idx % 4]][datQp] = dat
            #print(datFul)

    # close
    fpt.close()

    # return
    return datFul

def gmean(list):
    if list:
        for data in list:
            if (data <= 0):
                print("Gmean input check error for ", data)
        return pow(np.product(list), 1/len(list))
    else:
        return 0

def getMse(datPsnr, datBitDepth = 8):
    return ((1 << datBitDepth) - 1) ** 2 / (10 ** (datPsnr / 10))

# getLambda
def getLambda(datQp):
    return s_cst_datLmd_2[datQp]

# getNumPixels
def getNumPixels(strSeq):
    strSeq = strSeq.split("_")[0]
    return dict_resolution[strSeq]

#*** MAIN **********************************************************************
if __name__ == '__main__':
#--- PARAMTER PREPARATION --------------
    # strings
    CSTR_TYPE     = ("average", "I frame", "P frame", "B frame")
    CSTR_INFO_BFR = ("bitrate(kb/s)", "psnr(Y)", "psnr(U)", "psnr(V)")
    CSTR_QP       = ("22", "27", "32", "37", "GMEAN")
    CSTR_USAGE    = "\n[getBdRate.py] Usage: getBdRate.py anchor.log result.log [YUV420|YUV444] > bdRate.log\n"

    # open anchor
    try:
        fptAnchor = open(sys.argv[1], "r")
    except:
        assert False, "\n[getBdRate.py] CANNOT open the anchor!" + CSTR_USAGE

    # open testor
    try:
        fptResult = open(sys.argv[2], "r")
    except:
        assert False, "\n[getBdRate.py] CANNOT open the result!" + CSTR_USAGE

    # get format
    if len(sys.argv) == 3 or sys.argv[3] == "YUV420":
        datSclCh = 2
    elif sys.argv[3] == "YUV444":
        datSclCh = 1
    else:
        assert False, "\n[getBdRate.py] unknown format \"{:s}\"\n".format(sys.argv[3]) + CSTR_USAGE

    # check redundant parameter
    if len(sys.argv) > 4:
        assert False, "\n[getBdRate.py] unknown parameter \"{:s}\"\n".format(sys.argv[4]) + CSTR_USAGE


#--- DATA PREPARATION ------------------
    # process anchor
    datAnchor = getDat(fptAnchor, "anchor")

    # process testor
    datResult = getDat(fptResult, "result")


#--- DATA PROCESS ----------------------
    # head
    print("  {:<58s} {:<60s} {:<60s} {:s}".format(*CSTR_TYPE))
    for x in range(4):
        print("{:>10s} {:>10s} {:>10s} {:>10s} {:>10s}       ".format(*CSTR_QP), end = "")
    print("{:s}".format("sequence"))

    fps = 60
    eps = 1e-10
    datRatioListAll = {}

    for strSeq in datAnchor:
        if strSeq not in datResult:
            break
        datNumPixels = getNumPixels(strSeq)
        for strTyp in CSTR_TYPE:
            datRatioList = []
            if strTyp in datAnchor[strSeq] and strTyp in datResult[strSeq]:
                for datQp in datAnchor[strSeq][strTyp]:
                    datLambda = getLambda(datQp)
                    if strTyp == "B frame":
                        datLambda *= 2
                    # stat bits and sse
                    datBitsAnchor = datAnchor[strSeq][strTyp][datQp][0] * 1000 / fps
                    datSseAnchor  = getMse(datAnchor[strSeq][strTyp][datQp][1]) * datNumPixels                       \
                                  + getMse(datAnchor[strSeq][strTyp][datQp][2]) * datNumPixels / datSclCh / datSclCh \
                                  + getMse(datAnchor[strSeq][strTyp][datQp][3]) * datNumPixels / datSclCh / datSclCh
                    datBitsResult = datResult[strSeq][strTyp][datQp][0] * 1000 / fps
                    datSseResult  = getMse(datResult[strSeq][strTyp][datQp][1]) * datNumPixels                       \
                                  + getMse(datResult[strSeq][strTyp][datQp][2]) * datNumPixels / datSclCh / datSclCh \
                                  + getMse(datResult[strSeq][strTyp][datQp][3]) * datNumPixels / datSclCh / datSclCh

                    # stat rate-distortion cost
                    datCostAnchor = datSseAnchor + datLambda * datBitsAnchor
                    datCostResult = datSseResult + datLambda * datBitsResult
                    if datAnchor[strSeq][strTyp][datQp][1] == 0 or datResult[strSeq][strTyp][datQp][1] == 0:
                        datRDCRatio = 0
                    else:
                        datRDCRatio = (datCostResult + eps) / (datCostAnchor + eps)
                    datRatioList.append(datRDCRatio)
                datRatioList.append(gmean(datRatioList))
            else:
                datRatioList = (0, 0, 0, 0, 0)

            # collect
            if not strTyp in datRatioListAll:
                datRatioListAll[strTyp] = {}
            for idxInfoAft in range(len(CSTR_QP)):
                strInfoAft = CSTR_QP[idxInfoAft]
                if not strInfoAft in datRatioListAll[strTyp]:
                    datRatioListAll[strTyp][strInfoAft] = []
                datRatioListAll[strTyp][strInfoAft].append(datRatioList[idxInfoAft])
            print("{:>10.2%} {:>10.2%} {:>10.2%} {:>10.2%} {:>10.2%}       ".format(*datRatioList), end = "")

        # dump strSeq
        print(strSeq)

    # dump datRatioListAll
    print("")
    for strStat in ("min", "AVE", "max"):
        for idxLine in range(2):
            for strTyp in CSTR_TYPE:
                if (idxLine == 0):
                    if (strTyp == CSTR_TYPE[-1]):
                        print("{:>10s} {:>10s} {:>10s} {:>10s} {:s}"    .format(*(strStat + "(" + str(x) + ")" for x in CSTR_QP)), end = "")
                    else:
                        print("{:>10s} {:>10s} {:>10s} {:>10s} {:>10s}       ".format(*(strStat + "(" + str(x) + ")" for x in CSTR_QP)), end = "")
                else:
                    func = {"min": np.min, "AVE": np.mean, "max": np.max}[strStat]
                    if (strTyp == CSTR_TYPE[-1]):
                        print("{:>10.2%} {:>10.2%} {:>10.2%} {:>10.2%} {:.3%}"    .format(*(func(datRatioListAll[strTyp][strInfoAft]) for strInfoAft in CSTR_QP)), end = "")
                    else:
                        print("{:>10.2%} {:>10.2%} {:>10.2%} {:>10.2%} {:>10.2%}       ".format(*(func(datRatioListAll[strTyp][strInfoAft]) for strInfoAft in CSTR_QP)), end = "")
            print("")
