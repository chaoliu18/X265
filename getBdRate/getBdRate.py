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
    #  Author         : Huang Leilei
    #  Status         : phase 003
    #  Reset          : 2021-10-15
    #  Description    : calculate B-D rate
    #
#-------------------------------------------------------------------------------
    #  Modified       : 2022-6-3 by Liu Chao
    #  Description    : Update Average BD-rate from 4:1:1 BD-rate to 6:1:1 PSNR, 
    #                   following JVET.
#-------------------------------------------------------------------------------

#*** IMPORT ********************************************************************
import sys
import re
import numpy as np
from getBdRateCore import getBdRateCore


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


#*** MAIN **********************************************************************
if __name__ == '__main__':
#--- PARAMTER PREPARATION --------------
    # strings
    CSTR_TYPE     = ("average", "I frame", "P frame", "B frame")
    CSTR_INFO_BFR = ("bitrate(kb/s)", "psnr(Y)", "psnr(U)", "psnr(V)")
    CSTR_INFO_AFT = ("bdrate(Y)", "bdrate(U)", "bdrate(V)", "bdrate(average)")
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
    print("{:<57s} {:<57s} {:<57s} {:s}".format(*CSTR_TYPE))
    for x in range(4):
        print("{:<12s} {:<12s} {:<12s} {:<18s} ".format(*CSTR_INFO_AFT), end = "")
    print("{:s}".format("sequence"))

    # body
    # for sequence
    datBdRtAll = {}
    for strSeq in datAnchor:
        if strSeq in datResult:

            # for type
            for strTyp in CSTR_TYPE:
                datBdRt = []
                if strTyp in datAnchor[strSeq] and strTyp in datResult[strSeq]:

                    # for info
                    for idxInfoAft in range(len(CSTR_INFO_AFT)):
                        strInfoAft = CSTR_INFO_AFT[idxInfoAft]
                        # for qp
                        datBtRtAnchor = []
                        datPsnrAnchor = []
                        datBtRtResult = []
                        datPsnrResult = []
                        if strInfoAft != "bdrate(average)":    # if enabled, average is taken after  function getBdRateCore
                            # !!! sure, the qp of anchor and result do not have to be identical
                            for datQp in datAnchor[strSeq][strTyp]:
                                # !!!                                                 1 + idxInfoAft here indicates
                                # !!! there is a fixed position constraints between CSTR_INFO_BFR and CSTR_INFO_AFT
                                datBtRtAnchor.append(datAnchor[strSeq][strTyp][datQp][0             ])
                                datPsnrAnchor.append(datAnchor[strSeq][strTyp][datQp][1 + idxInfoAft])
                            for datQp in datResult[strSeq][strTyp]:
                                datBtRtResult.append(datResult[strSeq][strTyp][datQp][0             ])
                                datPsnrResult.append(datResult[strSeq][strTyp][datQp][1 + idxInfoAft])
                        else:
                            # !!! sure, the qp of anchor and result do not have to be identical
                            for datQp in datAnchor[strSeq][strTyp]:
                                # !!!                                                 1 + idxInfoAft here indicates
                                # !!! there is a fixed position constraints between CSTR_INFO_BFR and CSTR_INFO_AFT
                                datBtRtAnchor.append(datAnchor[strSeq][strTyp][datQp][0])
                                Psnr = (6 * datAnchor[strSeq][strTyp][datQp][1]
                                          + datAnchor[strSeq][strTyp][datQp][2]
                                          + datAnchor[strSeq][strTyp][datQp][3]) / 8
                                datPsnrAnchor.append(Psnr)
                            for datQp in datResult[strSeq][strTyp]:
                                datBtRtResult.append(datResult[strSeq][strTyp][datQp][0])
                                Psnr = (6 * datResult[strSeq][strTyp][datQp][1]
                                          + datResult[strSeq][strTyp][datQp][2]
                                          + datResult[strSeq][strTyp][datQp][3]) / 8
                                datPsnrResult.append(Psnr)

                        # calculate bd rate
                        if any(datBtRtResult) and any(datPsnrResult) and any(datBtRtAnchor) and any(datPsnrAnchor):
                            datBtRtAnchor = np.array(datBtRtAnchor)
                            datPsnrAnchor = np.array(datPsnrAnchor)
                            datBtRtResult = np.array(datBtRtResult)
                            datPsnrResult = np.array(datPsnrResult)
                            datBdRt.append(getBdRateCore(datBtRtAnchor, datPsnrAnchor, datBtRtResult, datPsnrResult))
                        else:
                            datBdRt.append(0)
                else:
                    datBdRt = (0, 0, 0, 0)

                # collect
                if not strTyp in datBdRtAll:
                    datBdRtAll[strTyp] = {}
                for idxInfoAft in range(len(CSTR_INFO_AFT)):
                    strInfoAft = CSTR_INFO_AFT[idxInfoAft]
                    if not strInfoAft in datBdRtAll[strTyp]:
                        datBdRtAll[strTyp][strInfoAft] = []
                    datBdRtAll[strTyp][strInfoAft].append(datBdRt[idxInfoAft])

                # dump datBdRt
                print("{:<12.3f} {:<12.3f} {:<12.3f} {:<18.3f} ".format(*datBdRt), end = "")

            # dump strSeq
            print(strSeq)

    # dump datBdRtStat
    print("")
    for strStat in ("min", "AVE", "max"):
        for idxLine in range(2):
            for strTyp in CSTR_TYPE:
                if (idxLine == 0):
                    if (strTyp == CSTR_TYPE[-1]):
                        print("{:<12s} {:<12s} {:<12s} {:s}"    .format(*(x.replace("bdrate", strStat) for x in CSTR_INFO_AFT)), end = "")
                    else:
                        print("{:<12s} {:<12s} {:<12s} {:<18s} ".format(*(x.replace("bdrate", strStat) for x in CSTR_INFO_AFT)), end = "")
                else:
                    func = {"min": np.min, "AVE": np.mean, "max": np.max}[strStat]
                    if (strTyp == CSTR_TYPE[-1]):
                        print("{:<12.3f} {:<12.3f} {:<12.3f} {:.3f}"    .format(*(func(datBdRtAll[strTyp][strInfoAft]) for strInfoAft in CSTR_INFO_AFT)), end = "")
                    else:
                        print("{:<12.3f} {:<12.3f} {:<12.3f} {:<18.3f} ".format(*(func(datBdRtAll[strTyp][strInfoAft]) for strInfoAft in CSTR_INFO_AFT)), end = "")
            print("")
