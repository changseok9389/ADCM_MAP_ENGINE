#pragma once
#pragma pack(1)

#include <vector>
#include <stdint.h>

typedef unsigned char       byte;
typedef unsigned short      WORD;
typedef unsigned int		UINT;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned int		UINT;

#if !(defined(_WIN64) || defined(_WIN32))
typedef int8_t				__int8;
typedef int16_t				__int16;
typedef int32_t				__int32;
typedef int64_t				__int64;
#endif

//------------------------------------------------------------------------------
//			???? ???
//------------------------------------------------------------------------------
enum ObjTypeInRoad
{
	OTR_UNKWON,				// 0 : ?????? ???
	OTR_PREVROAD,			// 1 : ??????? Road ID
	OTR_NEXTROAD,			// 2 : ?????? Road ID
	OTR_LANE,				// 3 : ???? Lane ID
	OTR_LINE,				// 4 : ???? Line ID
	OTR_CROSSWALK,			// 5 : ?????? CrossWalk ID
	OTR_SPEEDBUMP,			// 6 : ????????? SpeedBump ID
	OTR_TRAFFICLIGHT,		// 7 : ????? TrafficLight ID
	OTR_STOPSIGN,			// 8 : ????????? StopSign ID
	OTR_YIELDSIGN,			// 9 : ??????? YieldSign ID
	OTR_LEFTBORDER,			// 10 : ??????? Border ID
	OTR_RIGHTBORDER,		// 11 : ??????? Border ID
	OTR_NUMBER,				// 12 : ??????? ????
};

//------------------------------------------------------------------------------
//			?????? ?????
//------------------------------------------------------------------------------
typedef struct
{
	double dx;
	double dy;
	double dz;
} VERTEX_INFO;  //24

//------------------------------------------------------------------------------
//			????? ?????
//------------------------------------------------------------------------------

typedef struct
{
	__int8			nRoadRank;					// ???? ???
	__int8			nRoadType;					// ???? ????
	__int8			nLinkType;					// ???? ????
	__int8			nMaxSpeed;					// ?????????
	__int8			nLaneNo;					// ???? ???
	__int16			nVerticesNum;				// ????? ????
	float			fLength;					// ????
	__int64			nID;						// ????ID (??????)
	__int64			nR_LaneID;					// ???????? ID (??????)
	__int64			nL_LaneID;					// ???????? ID (??????)
	__int64			nFromNodeID;				// ??????? ID (??????)
	__int64			nToNodeID;					// ??????? ID (??????)
	__int64			nSectionID;					// ????/???? ID (??????)
}LANE_INFO;

//				A5_PARKINGLOT
typedef struct
{
	__int16			nVerticesNum;				// ????? ????
	__int16			nType;						// ?????? ????
	__int64			nID;						// ??????ID (??????)
	__int64			nSelectID;					// ???? ID (??????)
}PARKINGLOT_INFO;

//				B1_SURFACEMAKR
typedef struct
{
	__int16			nRefLane;					// ???? ???? ??
	__int64			nID;						// ???????ID (??????)
	__int64			nLinkID;					// ??? ID (??????)
	__int64			nPostID;					// ???? ID (??????)
}SIGN_INFO;

//				B2_SURFACELINEMAKR
typedef struct
{
	__int16			nVerticesNum;				// ????? ????
	__int16			nType;						// ?? ???, ???? ????
	__int16			nKind;						// ????, ???????????? ?? ???? ????
	__int64			nID;						// ????ID (??????)
	__int64			nR_LinkID;					// ???????? ID  (??????)
	__int64			nL_LinkID;					// ???????? ID  (??????)
}LINE_INFO;

//				B3_SURFACEMAKR
typedef struct
{
	__int8				nType;						// ??? ????
	__int16			nKind;						// ??? ????
	__int16			nVerticesNum;				// ????? ????
	__int64			nID;						// ??��?ID (??????)
	__int64			nLinkID;					// ???? ???? ID (??????)
}CROSSWALK_INFO;

//				C1_TRAFFICLIGHT
typedef struct
{
	__int16			nType;						// ???? ????? ????
	__int16			nRefLane;					// ???? ???? ??
	__int64			nID;						// ???? ?????ID (??????)
	__int64			nLinkID;					// ??? ID (??????)
	__int64			nPostID;					// ???? ID (??????)
}SIGNAL_INFO;

//				C3_VEHICLEPROTECTIONSAFTY
typedef struct
{
	__int16				nVerticesNum;			// ????? ????
	__int16				nType;					// ???????
	__int16				nIsCentral;				// ???��???
	__int16				nLowHigh;				// ???, ???
	__int64				nID;					// ??????????ID (??????)
	__int64				nRef_FID;				// ????? ID (??????)
}BORDER_INFO;

//				C4_SPEEDBUMP
typedef struct
{
	__int16			nVerticesNum;				// ????? ????
	__int16			nType;						// ????????? ????
	__int16			nRef_Lane;					// ???? ???? ??
	__int64			nID;						// ?????????ID (??????)
	__int64			nLinkID;					// ???ID (??????)
}SPEEDBUMP_INFO;

//------------------------------------------------------------------------------
//			(??? ???? + ???? ????) ?????
//------------------------------------------------------------------------------
typedef struct
{
	LANE_INFO sInfo;							// 
	std::vector<VERTEX_INFO> vGeometry;			// ????
} LANE_VTX;										// ????

typedef struct
{
	PARKINGLOT_INFO sInfo;						//
	std::vector<VERTEX_INFO> vGeometry;			// ????
} PARKINGLOT_VTX;								// ????

typedef struct
{
	SIGN_INFO sInfo;							// 
	VERTEX_INFO sGeometry;						// 24
} SIGN_VTX;										//

typedef struct
{
	LINE_INFO sInfo;							//
	std::vector<VERTEX_INFO> vGeometry;			// ????
} LINE_VTX;										// ????

typedef struct
{
	CROSSWALK_INFO sInfo;						//
	std::vector<VERTEX_INFO> vGeometry;			// ????
} CROSSWALK_VTX;								// ????

typedef struct
{
	SIGNAL_INFO sInfo;							// 
	VERTEX_INFO sGeometry;						// 24
} SIGNAL_VTX;									// 

typedef struct
{
	BORDER_INFO sInfo;							// 
	std::vector<VERTEX_INFO> vGeometry;			// ????
} BORDER_VTX;									// ????

typedef struct
{
	SPEEDBUMP_INFO sInfo;
	std::vector<VERTEX_INFO> vGeometry;
} SPEEDBUMP_VTX;


//------------------------------------------------------------------------------
//			?????? ?????? ?????
//------------------------------------------------------------------------------
typedef struct
{
	__int64 nID;
	__int64 nJunctionID;
	std::vector<__int64> nPredecessorID;
	std::vector<__int64> nSuccessorID;
	double dLength;
	__int8 nLaneCnt;
	std::vector<LANE_VTX> vLanes;
	std::vector<LINE_VTX> vLines;
	std::vector<CROSSWALK_VTX> vCrosswalks;
	std::vector<SIGNAL_VTX> vSignals;
	std::vector<SIGN_VTX> vStopsigns;
	std::vector<SIGN_VTX> vYieldsigns;
	std::vector<SPEEDBUMP_VTX> vSpeedbumps;
} ROAD;

typedef struct
{
	__int8 nFrom;
	__int8 nTo;
} LANELINK;

typedef struct
{
	__int8 nID;
	__int64 nIncomingRoadID;
	__int64 nConnectingRoadID;
	__int8 nContactPoint;
	__int8 nType;
	std::vector<LANELINK> vLaneLinkInfo;
} CONNECTION;

typedef struct
{
	__int64 nID;
	__int8	nType;
	std::vector<CONNECTION> vConnections;
} JUNCTION;

typedef struct
{
	std::vector<ROAD> vRoads;
	std::vector<JUNCTION> vJunctions;
} ROAD_JUNCTION_LIST;

typedef struct
{
	__int64 nID;
	std::vector<ROAD> vRoads;
	std::vector<JUNCTION> vJunctions;
	std::vector<LINE_VTX> vStopLines;
	std::vector<BORDER_VTX> vBorders;
	std::vector<PARKINGLOT_VTX> vParkinglots;
	std::vector<CROSSWALK_VTX> vCrosswalks_out;
} LOCAL_MAP;
