# Raw Rfans LIDAR packet.#
char   syncHead         # 0x96 #
char   chksum           #
uint32  timeH           #
uint32  timeL           #
RfansPoint[16] block    #
