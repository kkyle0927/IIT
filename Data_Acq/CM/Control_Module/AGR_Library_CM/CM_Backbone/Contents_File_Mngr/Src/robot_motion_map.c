
#include "robot_motion_map.h"


/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

MotionMapFileInfo MotionMap_File;
MotionMapFileInfo MotionMap_File_Read;

/*************************************************************************( C Vectors )*******************************************************************************************************/
// C-Vectors
MotionSetFile_Cvector c_vectors_FF_PD_DOB =
{
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}},
		{{1, 1, 0, 1, 0, 0}}
};

/*************************************************************************( P Vectors )*******************************************************************************************************/
// P-Vectors For Demonstration

// Stand
MotionSetFile_Pvector p_vectors_stand =
{
		/*0x2: RH_COR*/ {{  +498, +3000,   +0,   +0}},
		/*0x3: LH_COR*/ {{  +498, +3000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{   +0, +3000,   +0,   +0}},
		/*0x7: LH_SAG*/ {{   +0, +3000,   +0,   +0}},
		/*0x8:     RK*/ {{   +1, +3000,   +0,   +0}},
		/*0x9:     LK*/ {{   +1, +3000,   +0,   +0}},
		/*0xA: RA_MED*/ {{   +0, +3000,   +0,   +0}},
		/*0xB: LA_MED*/ {{   +0, +3000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{   +0, +3000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{   +0, +3000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

// Squat
MotionSetFile_Pvector p_vectors_squat =
{
		/*0x2: RH_COR*/ {{ +498, +3000,   +0,   +0}},
		/*0x3: LH_COR*/ {{ +498, +3000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +40, +1800,   +0,   +0}},
		/*0x7: LH_SAG*/ {{  +40, +1800,   +0,   +0}},
		/*0x8:     RK*/ {{  +40, +3000,   +0,   +0}},
		/*0x9:     LK*/ {{  +40, +3000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ 1906, +3000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ 1906, +3000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ 1906, +3000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ 1906, +3000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_squat2 =
{
		/*0x2: RH_COR*/ {{ +498, +3000,   +0,   +0}},
		/*0x3: LH_COR*/ {{ +498, +3000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +95,  +600,   +0,   +0}, {  +70, +600,   +0,   +0}, {  +40, +1800,   +0,   +0}},
		/*0x7: LH_SAG*/ {{  +95,  +600,   +0,   +0}, {  +70, +600,   +0,   +0}, {  +40, +1800,   +0,   +0}},
		/*0x8:     RK*/ {{  +40, +3000,   +0,   +0}},
		/*0x9:     LK*/ {{  +40, +3000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ 1906, +3000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ 1906, +3000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ 1906, +3000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ 1906, +3000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};


// Air Sit
MotionSetFile_Pvector p_vectors_air_sit =
{
		/*0x2: RH_COR*/ {{  +498, +6000,   +0,   +0}},
		/*0x3: LH_COR*/ {{  +498, +6000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{   +95, +6000,   +0,   +0}}, // 110
		/*0x7: LH_SAG*/ {{   +95, +6000,   +0,   +0}},
		/*0x8:     RK*/ {{  +108, +6000,   +0,   +0}},
		/*0x9:     LK*/ {{  +108, +6000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ +1830, +6000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ +1830, +6000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ +1830, +6000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ +1830, +6000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

// Air Sit
MotionSetFile_Pvector p_vectors_sit =
{
		/*0x2: RH_COR*/ {{ +498, +9000,   +0,   +0}},
		/*0x3: LH_COR*/ {{ +498, +9000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +93, +9000,   +0,   +0}}, // 90
		/*0x7: LH_SAG*/ {{  +93, +9000,   +0,   +0}},
		/*0x8:     RK*/ {{  +91, +9000,   +0,   +0}},
		/*0x9:     LK*/ {{  +91, +9000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ -740, +9000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ -740, +9000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ -740, +9000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ -740, +9000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_sit2stand =
{
		/*0x2: RH_COR*/ {{ +498, +6000,   +0,   +0}},
		/*0x3: LH_COR*/ {{ +498, +6000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +95, +1000,   +5,   +0}, {  +80, +1400,   +0,   +0}, {  +0, +3600,   +0,   +0}},
		/*0x7: LH_SAG*/ {{  +95, +1000,   +5,   +0}, {  +80, +1400,   +0,   +0}, {  +0, +3600,   +0,   +0}},
		/*0x8:     RK*/ {{   +1, +6000,   +0,   +0}},
		/*0x9:     LK*/ {{   +1, +6000,   +0,   +0}},
		/*0xA: RA_MED*/ {{   +0, +6000,   +0,   +0}},
		/*0xB: LA_MED*/ {{   +0, +6000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{   +0, +6000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{   +0, +6000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_stand2sit =
{
		/*0x2: RH_COR*/ {{ +498, +6000,   +0,   +0}},
		/*0x3: LH_COR*/ {{ +498, +6000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +70, +3000,   +0,   +0}, { +93, +3000,   +0,   +0}},
		/*0x7: LH_SAG*/ {{  +70, +3000,   +0,   +0}, { +93, +3000,   +0,   +0}},
		/*0x8:     RK*/ {{  +91, +6000,   +0,   +0}},
		/*0x9:     LK*/ {{  +91, +6000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ -740, +6000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ -740, +6000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ -740, +6000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ -740, +6000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};


MotionSetFile_Pvector p_vectors_left_half_swing =
{
		/*0x2: RH_COR*/ {},
		/*0x3: LH_COR*/ {},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{       0,  +750,   +0,   +0}, {  -21.26, +750,   +0,   +2}},
		/*0x7: LH_SAG*/ {{  +21.26,  +750,   +0,   +0}},
		/*0x8:     RK*/ {{      +1,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0x9:     LK*/ {{  +34.45,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0xA: RA_MED*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xB: LA_MED*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xC: RA_LAT*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xD: LA_LAT*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_right_full_swing =
{
		/*0x2: RH_COR*/ {},
		/*0x3: LH_COR*/ {},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +21.26,  +750,   +0,   +0}},
		/*0x7: LH_SAG*/ {{      +0,  +750,   +0,   +0}, {  -21.26, +750,   +0,   +2}},
		/*0x8:     RK*/ {{  +34.45,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0x9:     LK*/ {{      +1,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0xA: RA_MED*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xB: LA_MED*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xC: RA_LAT*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xD: LA_LAT*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_left_full_swing =
{
		/*0x2: RH_COR*/ {},
		/*0x3: LH_COR*/ {},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{       0,  +750,   +0,   +0}, {  -21.26, +750,   +0,   +2}},
		/*0x7: LH_SAG*/ {{  +21.26,  +750,   +0,   +0}},
		/*0x8:     RK*/ {{      +1,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0x9:     LK*/ {{  +34.45,  +750,   +0,   +0}, {  +21.08, +750,   +0,   +3}},
		/*0xA: RA_MED*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xB: LA_MED*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xC: RA_LAT*/ {{       0,  +750,   +0,   +0}, {+1309.02, +750,   +0,   +3}},
		/*0xD: LA_LAT*/ {{   +1712,  +750,   +0,   +0}, {-1231.24, +750,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_dance =
{
		/*0x2: RH_COR*/ {{  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}},
		/*0x3: LH_COR*/ {{  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}},
		/*0x7: LH_SAG*/ {{   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}},
		/*0x8:     RK*/ {{    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}},
		/*0x9:     LK*/ {{   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}},
		/*0xA: RA_MED*/ {{    +0, +2000,   +0,   +0}},
		/*0xB: LA_MED*/ {{    +0, +2000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{    +0, +2000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{    +0, +2000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

/*
MotionSetFile_Pvector p_vectors_dance =
{
		0x2: RH_COR {{  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}},
		0x3: LH_COR {{  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}, {  -180, +2000,   +0,   +0}, {  +180, +2000,   +0,   +0}},
		0x4:   NONE {},
		0x5:   NONE {},
		0x6: RH_SAG {{    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}},
		0x7: LH_SAG {{   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}},
		0x8:     RK {{    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}},
		0x9:     LK {{   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}, {   +20, +2000,   +0,   +0}, {    +0, +2000,   +0,   +0}},
		0xA: RA_MED {{    +0, +2000,   +0,   +0}},
		0xB: LA_MED {{    +0, +2000,   +0,   +0}},
		0xC: RA_LAT {{    +0, +2000,   +0,   +0}},
		0xD: LA_LAT {{    +0, +2000,   +0,   +0}},
		0xE:   NONE {},
		0xF:   NONE {},
		{},
		{}
};
*/

/*MotionSetFile_Pvector p_vector_stepping =
{
		0x2: RH_COR {{    +498, +1000,   +0,   +0}},
		0x3: LH_COR {{    +498, +1000,   +0,   +0}},
		0x4:   NONE {},
		0x5:   NONE {},
		0x6: RH_SAG {{       0, +1500,   +0,   +0}, {  +30.00,  +750,   +0,   +0}, {      +0, +750,   +0,   +0}},
		0x7: LH_SAG {{  +30.00,  +750,   +0,   +0}, {      +0, +750,   +0,   +0}},
		0x8:     RK {{      +1, +1500,   +0,   +0}, {  +56.45,  +750,   +0,   +0}, {      +1, +750,   +0,   +3}},
		0x9:     LK {{  +56.45,  +750,   +0,   +0}, {      +1, +750,   +0,   +3}},
		0xA: RA_MED {{       0, +1500,   +0,   +0}, {   +1712,  +750,   +0,   +0}, {      +0, +750,   +0,   +3}},
		0xB: LA_MED {{   +1712,  +750,   +0,   +0}, {      +0, +750,   +0,   +3}},
		0xC: RA_LAT {{       0, +1500,   +0,   +0}, {   +1712,  +750,   +0,   +0}, {      +0, +750,   +0,   +3}},
		0xD: LA_LAT {{   +1712,  +750,   +0,   +0}, {      +0, +750,   +0,   +3}},
		0xE:   NONE {},
		0xF:   NONE {},
				{},
		{}

};*/

MotionSetFile_Pvector p_vector_left_foot_up =
{
		/*0x2: RH_COR*/ {{   -1513, +1500,   +0,   +3}},
		/*0x3: LH_COR*/ {{   +1571, +1500,   +0,   +3}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{       0, +1500,   +0,   +3}},
		/*0x7: LH_SAG*/ {{  +50.00, +1500,   +0,   +3}},
		/*0x8:     RK*/ {{      +1, +1500,   +0,   +3}},
		/*0x9:     LK*/ {{  +69.53, +1500,   +0,   +3}},
		/*0xA: RA_MED*/ {{    -367, +1500,   +0,   +3}},
		/*0xB: LA_MED*/ {{   +1712, +1500,   +0,   +3}},
		/*0xC: RA_LAT*/ {{    +370, +1500,   +0,   +3}},
		/*0xD: LA_LAT*/ {{   +1712, +1500,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vector_right_foot_up =
{
		/*0x2: RH_COR*/ {{   +1571, +1500,   +0,   +3}},
		/*0x3: LH_COR*/ {{   -1513, +1500,   +0,   +3}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +50.00, +1500,   +0,   +3}},
		/*0x7: LH_SAG*/ {{       0, +1500,   +0,   +3}},
		/*0x8:     RK*/ {{  +69.53, +1500,   +0,   +3}},
		/*0x9:     LK*/ {{      +1, +1500,   +0,   +3}},
		/*0xA: RA_MED*/ {{   +1712, +1500,   +0,   +3}},
		/*0xB: LA_MED*/ {{    -367, +1500,   +0,   +3}},
		/*0xC: RA_LAT*/ {{   +1712, +1500,   +0,   +3}},
		/*0xD: LA_LAT*/ {{    +370, +1500,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vector_left_tilt =
{
		/*0x2: RH_COR*/ {{   +1571, +1500,   +0,   +3}},
		/*0x3: LH_COR*/ {{   -1513, +1500,   +0,   +3}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{      +0, +1500,   +0,   +3}},
		/*0x7: LH_SAG*/ {{       0, +1500,   +0,   +3}},
		/*0x8:     RK*/ {{      +1, +1500,   +0,   +3}},
		/*0x9:     LK*/ {{      +1, +1500,   +0,   +3}},
		/*0xA: RA_MED*/ {{    +440, +1500,   +0,   +3}},
		/*0xB: LA_MED*/ {{    -367, +1500,   +0,   +3}},
		/*0xC: RA_LAT*/ {{    -295, +1500,   +0,   +3}},
		/*0xD: LA_LAT*/ {{    +370, +1500,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vector_right_tilt =
{
		/*0x2: RH_COR*/ {{   -1513, +1500,   +0,   +3}},
		/*0x3: LH_COR*/ {{   +1571, +1500,   +0,   +3}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{      +0, +1500,   +0,   +3}},
		/*0x7: LH_SAG*/ {{       0, +1500,   +0,   +3}},
		/*0x8:     RK*/ {{      +1, +1500,   +0,   +3}},
		/*0x9:     LK*/ {{      +1, +1500,   +0,   +3}},
		/*0xA: RA_MED*/ {{    -367, +1500,   +0,   +3}},
		/*0xB: LA_MED*/ {{    +440, +1500,   +0,   +3}},
		/*0xC: RA_LAT*/ {{    +370, +1500,   +0,   +3}},
		/*0xD: LA_LAT*/ {{    -295, +1500,   +0,   +3}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

//flexion - flexion
/*MotionSetFile_Pvector p_vectors_knee_test =
{
		{},
		{},
		{},
		{},
		{{-10, 3000, 0, 0}},
		{},
		{{1, 9000, 0, 0}, {10, 3000, 0, 0}, {20, 3000, 0, 0}, {30, 3000, 0, 0}, {40, 3000, 0, 0}, {50, 3000, 0, 0}, {60, 3000, 0, 0}, {70, 3000, 0, 0}, {80, 3000, 0, 0}, {90, 3000, 0, 0}, {80, 3000, 0, 0}, {70, 3000, 0, 0}, {60, 3000, 0, 0}, {50, 3000, 0, 0}, {40, 3000, 0, 0}, {30, 3000, 0, 0}, {20, 3000, 0, 0}, {10, 3000, 0, 0}, {1, 3000, 0, 0}},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{}
};*/

//flexion - flexion
MotionSetFile_Pvector p_vectors_knee_test =
{
		{},
		{},
		{},
		{},
		{{  0, 6000, 0, 0}, {5, 3000, 0, 0}, {10, 3000, 0, 0}, {15, 3000, 0, 0}, {20, 3000, 0, 0}, {25, 3000, 0, 0}, {30, 3000, 0, 0}, {35, 3000, 0, 0}, {40, 3000, 0, 0}},
		{},
		{{  1, 6000, 0, 0}},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{}
};

/*MotionSetFile_Pvector p_vectors_knee_test =
{
		0x2: RH_COR {},
		0x3: LH_COR {},
		0x5:   NONE {},
		0x6: RH_SAG {},
		0x7: LH_SAG {{0, 3000, 0, 0}, {-10, 3000, 0, 0}, {-20, 3000, 0, 0}, {-30, 3000, 0, 0}, {-40, 3000, 0, 0}, {-50, 3000, 0, 0}, {-60, 3000, 0, 0}, {-70, 3000, 0, 0}, {-80, 3000, 0, 0}, {-90, 3000, 0, 0}, {-80, 3000, 0, 0}, {-70, 3000, 0, 0}, {-60, 3000, 0, 0}, {-50, 3000, 0, 0}, {-40, 3000, 0, 0}, {-30, 3000, 0, 0}, {-20, 3000, 0, 0}, {-10, 3000, 0, 0}, {0, 3000, 0, 0}},
		0x8:     RK {},
		0x9:     LK {{1, 3000, 0, 0}, {10, 3000, 0, 0}, {20, 3000, 0, 0}, {30, 3000, 0, 0}, {40, 3000, 0, 0}, {50, 3000, 0, 0}, {60, 3000, 0, 0}, {70, 3000, 0, 0}, {80, 3000, 0, 0}, {90, 3000, 0, 0}, {80, 3000, 0, 0}, {70, 3000, 0, 0}, {60, 3000, 0, 0}, {50, 3000, 0, 0}, {40, 3000, 0, 0}, {30, 3000, 0, 0}, {20, 3000, 0, 0}, {10, 3000, 0, 0}, {1, 3000, 0, 0}},
		0xA: RA_MED {},
		0xB: LA_MED {},
		0xC: RA_LAT {},
		0xD: LA_LAT {},
		0xE:   NONE {},
		0xF:   NONE {},
				{},
		{}
};*/

/*MotionSetFile_Pvector p_vectors_knee_test =
{
		0x2: RH_COR {},
		0x3: LH_COR {},
		0x5:   NONE {},
		0x6: RH_SAG {},
		0x7: LH_SAG {},
		0x8:     RK {},
		0x9:     LK {{   +5, +3000,   +0,   +0}, {    +10, +3000,   +0,   +0}, {   +15, +2000,   +0,   +0}, {    +1, +2000,   +0,   +0}, {   +90, +1500,   +0,   +0}, {    +1, +1500,   +0,   +0}, {+90, +1000,   +0,   +0}, {1, 1000, 0, 0}},
		0xA: RA_MED {},
		0xB: LA_MED {},
		0xC: RA_LAT {},
		0xD: LA_LAT {},
		0xE:   NONE {},
		0xF:   NONE {},
				{},
		{}
};*/
// Motion: Walking
/*MotionSetFile_Pvector p_vectors_walking =
{
		0x2: RH_COR {{   +0, +100,   +0,   +0}},
		0x3: LH_COR {{   +0, +100,   +0,   +0}},
		0x4:   NONE {},
		0x5:   NONE {},
		0x6: RH_SAG {{   +0, +595,  +16,  +10}, {   +0, +108,   +8,   +5}, {   -4, +328,  +16,  +24}, {  +35, +630,   +8,   +4}, {  +24, +339,   +3,   +5}},
		0x7: LH_SAG {{  +35, +630,   +8,   +4}, {  +24, +339,   +3,   +5}, {   +0, +595,  +16,  +10}, {   +0, +108,   +8,   +5}, {   -4, +328,  +16,  +24}},
		0x8:     RK {{  +26,  +44,  +28,   +9}, {   +6, +503,  +11,  +16}, {  +65, +910,   +6,  +16}, {  +22, +490,   +4,  +11}, {  +25,  +53,   +8,  +27}},
		0x9:     LK {{  +65, +457,  +13,   +5}, {  +22, +490,   +4,  +11}, {  +26,  +98,   +1,   +2}, {   +6, +503,  +11,  +16}, {  +25, +452,  +17,  +23}},
		0xA: RA_MED {{+1125, +171,   +3,   +3}, { -297, +905,  +32,  +24}, {+1324, +297,   +1,   +4}, { +661, +297,  +15,   +3}, { +895, +153,   +1,   +8}, { +619, +177,  +11,   +0}},
		0xB: LA_MED {{ -297, +103,  +20,   +8}, {+1324,  +97,   +1,   +4}, { +665, +297,  +15,   +3}, { +895, +153,   +1,   +8}, { +625, +134,   +5,   +9}, {+1176, +168,    +3,  +3}, { -175, +848,  +34,  +23}},
		0xC: RA_LAT {{+1125, +171,   +3,   +3}, { -297, +905,  +32,  +24}, {+1324, +297,   +1,   +4}, { +661, +297,  +15,   +3}, { +895, +153,   +1,   +8}, { +619, +177,  +11,   +0}},
		0xD: LA_LAT {{ -297, +103,  +20,   +8}, {+1324,  +97,   +1,   +4}, { +665, +297,  +15,   +3}, { +895, +153,   +1,   +8}, { +625, +134,   +5,   +9}, {+1176, +168,    +3,  +3}, { -175, +848,  +34,  +23}},
		0xE:   NONE {},
		0xF:   NONE {},
				{},
		{}
};*/

// Motion: Walking
MotionSetFile_Pvector p_vectors_walking =
{
		/*0x2: RH_COR*/ {{   +0, +100,   +0,   +0}},
		/*0x3: LH_COR*/ {{   +0, +100,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{   +0, +595,  +16,  +10}, {   +0, +108,   +8,   +5}, {   -4, +328,  +16,  +24}, {  +35, +630,   +8,   +4}, {  +24, +339,   +3,   +5}},
		/*0x7: LH_SAG*/ {{  +35, +630,   +8,   +4}, {  +24, +339,   +3,   +5}, {   +0, +595,  +16,  +10}, {   +0, +108,   +8,   +5}, {   -4, +328,  +16,  +24}},
		/*0x8:     RK*/ {{  +26,  +44,  +28,   +9}, {   +6, +503,  +11,  +16}, {  +65, +910,   +6,  +16}, {  +22, +490,   +4,  +11}, {  +25,  +53,   +8,  +27}},
		/*0x9:     LK*/ {{  +65, +457,  +13,   +5}, {  +22, +490,   +4,  +11}, {  +26,  +98,   +1,   +2}, {   +6, +503,  +11,  +16}, {  +25, +452,  +17,  +23}},
		/*0xA: RA_MED*/ {{0.5*-1125, +171,   +3,   +3}, {0.5*+297, +905,  +32,  +24}, {0.5*-1324, +297,   +1,   +4}, {0.5*-661, +297,  +15,   +3}, {0.5*-895, +153,   +1,   +8}, {0.5*-619, +177,  +11,   +0}},
		/*0xB: LA_MED*/ {{0.5*+297, +103,  +20,   +8}, {0.5*-1324,  +97,   +1,   +4}, {0.5*-665, +297,  +15,   +3}, {0.5*-895, +153,   +1,   +8}, {0.5*-625, +134,   +5,   +9}, {0.5*-1176, +168,    +3,  +3}, {0.5*+175, +848,  +34,  +23}},
		/*0xC: RA_LAT*/ {{0.5*-1125, +171,   +3,   +3}, {0.5*+297, +905,  +32,  +24}, {0.5*-1324, +297,   +1,   +4}, {0.5*-661, +297,  +15,   +3}, {0.5*-895, +153,   +1,   +8}, {0.5*-619, +177,  +11,   +0}},
		/*0xD: LA_LAT*/ {{0.5*+297, +103,  +20,   +8}, {0.5*-1324,  +97,   +1,   +4}, {0.5*-665, +297,  +15,   +3}, {0.5*-895, +153,   +1,   +8}, {0.5*-625, +134,   +5,   +9}, {0.5*-1176, +168,    +3,  +3}, {0.5*+175, +848,  +34,  +23}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

MotionSetFile_Pvector p_vectors_half_walking =
{
		/*0x2: RH_COR*/ {{   +0, +2000,   +0,   +0}},
		/*0x3: LH_COR*/ {{   +0, +2000,   +0,   +0}},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {{  +24, +2000,   +0,   +0}},
		/*0x7: LH_SAG*/ {{   -4, +2000,   +0,   +0}},
		/*0x8:     RK*/ {{  +25, +2000,   +0,   +0}},
		/*0x9:     LK*/ {{  +25, +2000,   +0,   +0}},
		/*0xA: RA_MED*/ {{ -614, +2000,   +0,   +0}},
		/*0xB: LA_MED*/ {{ +175, +2000,   +0,   +0}},
		/*0xC: RA_LAT*/ {{ -614, +2000,   +0,   +0}},
		/*0xD: LA_LAT*/ {{ +175, +2000,   +0,   +0}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

/*************************************************************************( I Vectors )*******************************************************************************************************/
MotionSetFile_Ivector i_vectors_test =
{
		/*0x2: RH_COR*/ {},
		/*0x3: LH_COR*/ {},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {},
		/*0x7: LH_SAG*/ {},
		/*0x8:     RK*/ {{ 0, +5, +0, 0 ,1000}},
		/*0x9:     LK*/ {{ 0, +5, +0, 0 ,1000}},
		/*0xA: RA_MED*/ {{ 10, +2, +0, 0 ,1000}},
		/*0xB: LA_MED*/ {{ 10, +2, +0, 0 ,1000}},
		/*0xC: RA_LAT*/ {{ 10, +2, +0, 0 ,1000}},
		/*0xD: LA_LAT*/ {{ 10, +2, +0, 0 ,1000}},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};

/*************************************************************************( F Vectors )*******************************************************************************************************/
MotionSetFile_Ivector f_vectors_test =
{
		/*0x2: RH_COR*/ {},
		/*0x3: LH_COR*/ {},
		/*0x4:   NONE*/ {},
		/*0x5:   NONE*/ {},
		/*0x6: RH_SAG*/ {},
		/*0x7: LH_SAG*/ {},
		/*0x8:     RK*/ {},
		/*0x9:     LK*/ {},
		/*0xA: RA_MED*/ {},
		/*0xB: LA_MED*/ {},
		/*0xC: RA_LAT*/ {},
		/*0xD: LA_LAT*/ {},
		/*0xE:   NONE*/ {},
		/*0xF:   NONE*/ {},
		{},
		{}
};


/**
 *------------------------------------------------------------
 *                         VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */


/* default */
float extension_y_offset = 0.3;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/************************* Begin (2023-12-01) *************************/
void Make_MotionMap_Examples(void) 
{
	MotionMap_File.robot_id = 10;
	MotionMap_File.file_version_major = 20;

	for (int j = 0; j < MAX_N_MOTION_SET; j++) {
		for (int k = 0; k < MAX_N_MD; k++) {
			for (int c = 0; c < MAX_N_C_VECTORS; c++) {
				MotionMap_File.MS[j].MD[k].c_vector[c].FC_gain  = (uint8_t)(c+5);
				MotionMap_File.MS[j].MD[k].c_vector[c].IC_gain  = (uint8_t)(c+4);
				MotionMap_File.MS[j].MD[k].c_vector[c].DOB_gain = (uint8_t)(c+3);
				MotionMap_File.MS[j].MD[k].c_vector[c].FF_gain  = (uint8_t)(c+2);
				MotionMap_File.MS[j].MD[k].c_vector[c].IRC_gain = (uint8_t)(c+1);
			}
			for (int p = 0; p < MAX_N_P_VECTORS; p++) {
				MotionMap_File.MS[j].MD[k].p_vector[p].yd = (int16_t)(p+5);
				MotionMap_File.MS[j].MD[k].p_vector[p].L = (uint16_t)(p+4);
				MotionMap_File.MS[j].MD[k].p_vector[p].s0 = (uint8_t)(p+3);
				MotionMap_File.MS[j].MD[k].p_vector[p].sd = (uint8_t)(p+2);
			}
			for (int f = 0; f < MAX_N_F_VECTORS; f++) {
				MotionMap_File.MS[j].MD[k].f_vector[f].mode_idx = (uint16_t)(f+5);
				MotionMap_File.MS[j].MD[k].f_vector[f].coefficient =  (int16_t)(f+4);
				MotionMap_File.MS[j].MD[k].f_vector[f].global_variable_index =  (uint16_t)(f+3);
				MotionMap_File.MS[j].MD[k].f_vector[f].delay =    (uint16_t)(f+2);
			}
			for (int i = 0; i < MAX_N_P_VECTORS; i++) {
				MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target = (uint8_t)(i+5);
				MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target = (uint8_t)(i+4);
				MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target = (uint8_t)(i+3);
				MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target = (uint8_t)(i+2);
				MotionMap_File.MS[j].MD[k].i_vector[i].duration =  (uint16_t)(i+1);
			}
		}
		MotionMap_File.MS[j].MS_ID = (uint8_t)(j+5);
	}
}

void Save_MotionMap(void) 
{
	uint32_t writeAddr;

	IOIF_EraseFlash(IOIF_FLASH_START_MM_ADDR1, IOIF_ERASE_ONE_SECTOR);
	IOIF_EraseFlash(IOIF_FLASH_START_MM_ADDR2, IOIF_ERASE_ONE_SECTOR);
//	IOIF_EraseFlash(IOIF_FLASH_START_MM_ADDR3, IOIF_ERASE_ONE_SECTOR);
//	IOIF_EraseFlash(IOIF_FLASH_START_MM_ADDR4, IOIF_ERASE_ONE_SECTOR);
//	IOIF_EraseFlash(IOIF_FLASH_START_MM_ADDR5, IOIF_ERASE_ONE_SECTOR);

	writeAddr = IOIF_FLASH_START_MM_ADDR1;

    uint32_t memArr[4];
	memcpy(&memArr[0], &MotionMap_File.robot_id,     sizeof(MotionMap_File.robot_id));
	memcpy(&memArr[1], &MotionMap_File.file_version_major, sizeof(MotionMap_File.file_version_major));
	memcpy(&memArr[2], &MotionMap_File.file_version_minor, sizeof(MotionMap_File.file_version_minor));
	memcpy(&memArr[3], &MotionMap_File.file_version_patch, sizeof(MotionMap_File.file_version_patch));
	IOIF_WriteFlash(writeAddr, memArr);
	writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

	for (int j = 0; j < MAX_N_MOTION_SET; j++) {
		for (int k = 0; k < MAX_N_MD; k++) {

			uint32_t memArr[6];
			memcpy(&memArr[0], &MotionMap_File.MS[j].MD[k].c_vector[0].FF_gain,     sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].FF_gain));
			memcpy(&memArr[1], &MotionMap_File.MS[j].MD[k].c_vector[0].PD_gain,     sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].PD_gain));
			memcpy(&memArr[2], &MotionMap_File.MS[j].MD[k].c_vector[0].IC_gain,     sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].IC_gain));
			memcpy(&memArr[3], &MotionMap_File.MS[j].MD[k].c_vector[0].DOB_gain,    sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].DOB_gain));
			memcpy(&memArr[4], &MotionMap_File.MS[j].MD[k].c_vector[0].IRC_gain,    sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].IRC_gain));
			memcpy(&memArr[5], &MotionMap_File.MS[j].MD[k].c_vector[0].FC_gain,     sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].FC_gain));
			IOIF_WriteFlash(writeAddr, memArr);
			writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;


			for (int p = 0; p < MAX_N_P_VECTORS; p++) {
				uint32_t memArr[4];
				memcpy(&memArr[0], &MotionMap_File.MS[j].MD[k].p_vector[p].yd, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].yd));
				memcpy(&memArr[1], &MotionMap_File.MS[j].MD[k].p_vector[p].L,  sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].L));
				memcpy(&memArr[2], &MotionMap_File.MS[j].MD[k].p_vector[p].s0, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].s0));
				memcpy(&memArr[3], &MotionMap_File.MS[j].MD[k].p_vector[p].sd, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].sd));
				IOIF_WriteFlash(writeAddr, memArr);
				writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
			}
			/*
			for (int f = 0; f < MAX_N_F_VECTORS; f++) {
				uint32_t memArr[3];
				memcpy(&memArr[0], &MotionMap_File.MS[j].MD[k].f_vector[f].mode_idx, sizeof(MotionMap_File.MS[j].MD[k].f_vector[f].mode_idx));
				memcpy(&memArr[1], &MotionMap_File.MS[j].MD[k].f_vector[f].tau_max,  sizeof(MotionMap_File.MS[j].MD[k].f_vector[f].tau_max));
				memcpy(&memArr[2], &MotionMap_File.MS[j].MD[k].f_vector[f].delay,    sizeof(MotionMap_File.MS[j].MD[k].f_vector[f].delay));
				IOIF_WriteFlash(writeAddr, memArr);
				writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
			}

			for (int i = 0; i < MAX_N_I_VECTORS; i++) {
				uint32_t memArr[5];
				memcpy(&memArr[0], &MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target, sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target));
				memcpy(&memArr[1], &MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target,      sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target));
				memcpy(&memArr[2], &MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target,      sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target));
				memcpy(&memArr[3], &MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target,  sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target));
				memcpy(&memArr[4], &MotionMap_File.MS[j].MD[k].i_vector[i].duration,       sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].duration));
				IOIF_WriteFlash(writeAddr, memArr);
				writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
			}
			*/
			for (int i = 0; i < MAX_N_F_VECTORS; i++) {
				uint32_t memArr[8];
				memcpy(&memArr[0], &MotionMap_File.MS[j].MD[k].f_vector[i].mode_idx,       			sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].mode_idx));
				// memcpy(&memArr[1], &MotionMap_File.MS[j].MD[k].f_vector[i].tau_max,				sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].tau_max));
				memcpy(&memArr[3], &MotionMap_File.MS[j].MD[k].f_vector[i].delay,          			sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].delay));
				memcpy(&memArr[4], &MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target, 			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target));
				memcpy(&memArr[5], &MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target,      			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target));
				memcpy(&memArr[6], &MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target,      			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target));
				memcpy(&memArr[7], &MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target,  			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target));
//				memcpy(&memArr[8], &MotionMap_File.MS[j].MD[k].i_vector[i].duration,       			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].duration));

				IOIF_WriteFlash(writeAddr, memArr);
				writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
			}
		}
		IOIF_WriteFlash(writeAddr, &MotionMap_File.MS[j].MS_ID);              writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
	}
}

void Download_MotionMap(void) 
{
	uint32_t readAddr;

	readAddr = IOIF_FLASH_START_MM_ADDR1;

	IOIF_ReadFlash(readAddr, &MotionMap_File.robot_id,     sizeof(MotionMap_File.robot_id));      									             readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &MotionMap_File.file_version_major, sizeof(MotionMap_File.file_version_major));  								     readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &MotionMap_File.file_version_minor, sizeof(MotionMap_File.file_version_minor));  								     readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &MotionMap_File.file_version_patch, sizeof(MotionMap_File.file_version_patch));  								     readAddr += IOIF_FLASH_READ_ADDR_SIZE_20B;

	for (int j = 0; j < MAX_N_MOTION_SET; j++) {
		for (int k = 0; k < MAX_N_MD; k++) {

			IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].FF_gain,  sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].FF_gain));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
			IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].PD_gain,  sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].PD_gain));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
			IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].IC_gain,  sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].IC_gain));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
			IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].DOB_gain, sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].DOB_gain)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
			IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].IRC_gain, sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].IRC_gain)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	    	IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].c_vector[0].FC_gain,  sizeof(MotionMap_File.MS[j].MD[k].c_vector[0].FC_gain));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

			for (int p = 0; p < MAX_N_P_VECTORS; p++) {
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].p_vector[p].yd, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].yd));         readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].p_vector[p].L,  sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].L));	         readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].p_vector[p].s0, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].s0));         readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].p_vector[p].sd, sizeof(MotionMap_File.MS[j].MD[k].p_vector[p].sd));         readAddr += IOIF_FLASH_READ_ADDR_SIZE_20B;
			}
			for (int i = 0; i < MAX_N_I_VECTORS; i++) {
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].f_vector[i].mode_idx,       			sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].mode_idx));       readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				// IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].f_vector[i].tau_max,				sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].tau_max));       readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].f_vector[i].delay,          			sizeof(MotionMap_File.MS[j].MD[k].f_vector[i].delay));          readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target, 			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].epsilon_target)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target,      			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kp_target));	     readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target,      			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].Kd_target));      readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target,  			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].lambda_target));  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
				IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MD[k].i_vector[i].duration,       			sizeof(MotionMap_File.MS[j].MD[k].i_vector[i].duration));       readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
			}
		}
		IOIF_ReadFlash(readAddr, &MotionMap_File.MS[j].MS_ID,             sizeof(MotionMap_File.MS[j].MS_ID));              readAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

	}
}

int Check_MM_Save(void)
{
	int err = 0;

	if (MotionMap_File.file_version_major != MotionMap_File_Read.file_version_major) err++;
	if (MotionMap_File.file_version_minor != MotionMap_File_Read.file_version_minor) err++;
	if (MotionMap_File.file_version_patch != MotionMap_File_Read.file_version_patch) err++;
	if (MotionMap_File.robot_id     != MotionMap_File_Read.robot_id)     err++;

	for (int j = 0; j < MAX_N_MOTION_SET; j++) {

		RobotMotionSet MS_in  = MotionMap_File.MS[j];
		RobotMotionSet MS_out = MotionMap_File_Read.MS[j];

		if (MS_in.MS_ID != MS_out.MS_ID) err++;

		for (int k = 0; k < MAX_N_MD; k++) {

			SingleJointMotionSet MS_in1  = MS_in.MD[k];
			SingleJointMotionSet MS_out1 = MS_out.MD[k];

			if (MS_in1.c_vector[0].DOB_gain != MS_out1.c_vector[0].DOB_gain) {err++; return 1*1000000 + j*10000 + k*100;}
			if (MS_in1.c_vector[0].FC_gain  != MS_out1.c_vector[0].FC_gain)  {err++; return 1*1000000 + j*10000 + k*100;}
			if (MS_in1.c_vector[0].PD_gain  != MS_out1.c_vector[0].PD_gain)  {err++; return 1*1000000 + j*10000 + k*100;}
			if (MS_in1.c_vector[0].FF_gain  != MS_out1.c_vector[0].FF_gain)  {err++; return 1*1000000 + j*10000 + k*100;}
			if (MS_in1.c_vector[0].IC_gain  != MS_out1.c_vector[0].IC_gain)  {err++; return 1*1000000 + j*10000 + k*100;}
			if (MS_in1.c_vector[0].IRC_gain != MS_out1.c_vector[0].IRC_gain) {err++; return 1*1000000 + j*10000 + k*100;}

			for (int p = 0; p < MAX_N_P_VECTORS; p++) {
				if (MS_in1.p_vector[p].L  != MS_out1.p_vector[p].L)  {err++; return 2*1000000 + j*10000 + k*100 + p;}
				if (MS_in1.p_vector[p].yd != MS_out1.p_vector[p].yd) {err++; return 2*1000000 + j*10000 + k*100 + p;}
				if (MS_in1.p_vector[p].s0 != MS_out1.p_vector[p].s0) {err++; return 2*1000000 + j*10000 + k*100 + p;}
				if (MS_in1.p_vector[p].sd != MS_out1.p_vector[p].sd) {err++; return 2*1000000 + j*10000 + k*100 + p;}
			}

			for (int f = 0; f < MAX_N_F_VECTORS; f++) {
				if (MS_in1.f_vector[f].delay    != MS_out1.f_vector[f].delay)    {err++; return 3*1000000 + j*10000 + k*100 + f;}
				if (MS_in1.f_vector[f].mode_idx != MS_out1.f_vector[f].mode_idx) {err++; return 3*1000000 + j*10000 + k*100 + f;}
				// if (MS_in1.f_vector[f].tau_max  != MS_out1.f_vector[f].tau_max)  {err++; return 3*1000000 + j*10000 + k*100 + f;}
			}

			for (int i = 0; i < MAX_N_I_VECTORS; i++) {
				if (MS_in1.i_vector[i].Kd_target      != MS_out1.i_vector[i].Kd_target)      {err++; return 4*1000000 + j*10000 + k*100 + i;}
				if (MS_in1.i_vector[i].Kp_target      != MS_out1.i_vector[i].Kp_target)      {err++; return 4*1000000 + j*10000 + k*100 + i;}
				if (MS_in1.i_vector[i].duration       != MS_out1.i_vector[i].duration)       {err++; return 4*1000000 + j*10000 + k*100 + i;}
				if (MS_in1.i_vector[i].epsilon_target != MS_out1.i_vector[i].epsilon_target) {err++; return 4*1000000 + j*10000 + k*100 + i;}
				if (MS_in1.i_vector[i].lambda_target  != MS_out1.i_vector[i].lambda_target)  {err++; return 4*1000000 + j*10000 + k*100 + i;}
			}

		}
	}

	return err;
}
/************************* End (2023-12-01) *************************/

/************************* For Manual Set *************************/
void Reset_MotionMap(int8_t val)
{
	MotionMap_File.send_state = 0;

	for (int MS_idx = 0; MS_idx < MAX_N_MOTION_SET; MS_idx++)
	{
		for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_C_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].FF_gain  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].PD_gain  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].IC_gain  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].DOB_gain = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].IRC_gain = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].c_vector[VEC_idx].FC_gain  = val;
			}

			for (int VEC_idx = 0; VEC_idx < MAX_N_P_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].yd = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].L  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].s0 = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].sd = val;
			}

			for (int VEC_idx = 0; VEC_idx < MAX_N_F_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].mode_idx = val;
				// MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].tau_max  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].delay   = val;
			}

			for (int VEC_idx = 0; VEC_idx < MAX_N_I_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].epsilon_target = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].Kp_target      = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].Kd_target      = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].lambda_target  = val;
				MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].duration       = val;
			}
		}
	}
}

void Download_MotionSet(uint8_t t_MS_idx,
						MotionSetFile_Cvector MotionSet_c,
						MotionSetFile_Pvector MotionSet_p,
						MotionSetFile_Fvector MotionSet_f,
						MotionSetFile_Ivector MotionSet_i)
{
	MotionMap_File.MS[t_MS_idx].MS_ID = t_MS_idx;

	for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
	{
		if (MotionSet_c != NULL)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_C_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].FF_gain  = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][0]*255);
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].PD_gain  = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][1]*255);
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].IC_gain  = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][2]*255);
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].DOB_gain = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][3]*255);
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].IRC_gain = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][4]*255);
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].c_vector[VEC_idx].FC_gain  = (uint8_t)(MotionSet_c[MD_idx][VEC_idx][5]*255);
			}
		}

		if (MotionSet_p != NULL)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_P_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].p_vector[VEC_idx].yd = (int16_t)MotionSet_p[MD_idx][VEC_idx][0];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].p_vector[VEC_idx].L  = (uint16_t)MotionSet_p[MD_idx][VEC_idx][1];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].p_vector[VEC_idx].s0 = (uint8_t)MotionSet_p[MD_idx][VEC_idx][2];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].p_vector[VEC_idx].sd = (uint8_t)MotionSet_p[MD_idx][VEC_idx][3];
			}
		}

		if (MotionSet_f != NULL)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_F_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].f_vector[VEC_idx].mode_idx = (uint8_t)MotionSet_f[MD_idx][VEC_idx][0];
				// MotionMap_File.MS[t_MS_idx].MD[MD_idx].f_vector[VEC_idx].tau_max  = (int16_t)MotionSet_f[MD_idx][VEC_idx][1];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].f_vector[VEC_idx].delay    = (uint16_t)MotionSet_f[MD_idx][VEC_idx][2];
			}
		}

		if (MotionSet_i != NULL)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_I_VECTORS; VEC_idx++)
			{
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].i_vector[VEC_idx].epsilon_target = (uint8_t)MotionSet_i[MD_idx][VEC_idx][0];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].i_vector[VEC_idx].Kp_target      = (uint8_t)MotionSet_i[MD_idx][VEC_idx][1];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].i_vector[VEC_idx].Kd_target      = (uint8_t)MotionSet_i[MD_idx][VEC_idx][2];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].i_vector[VEC_idx].lambda_target  = (uint8_t)MotionSet_i[MD_idx][VEC_idx][3];
				MotionMap_File.MS[t_MS_idx].MD[MD_idx].i_vector[VEC_idx].duration       = (uint16_t)MotionSet_i[MD_idx][VEC_idx][4];
			}
		}
	}
}
/************************* (End)For Manual Set *************************/

void Get_Max_PFI_Vectors_Length(void)
{
	for (int MS_idx = 0; MS_idx < MAX_N_MOTION_SET; MS_idx++)
	{
		uint8_t t_p_vector_max_len = 0;
		for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_P_VECTORS; VEC_idx++)
			{
				if ((MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].yd == 0) && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].L == 0)  && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].s0 == 0) && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].p_vector[VEC_idx].sd == 0))
				{
					if (VEC_idx > t_p_vector_max_len) t_p_vector_max_len = VEC_idx;
					break;
				}
			}
		}
		MotionMap_File.MS[MS_idx].max_p_vectors_len = t_p_vector_max_len;

		uint8_t t_f_vector_max_len = 0;
		for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_F_VECTORS; VEC_idx++)
			{
				if ((MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].mode_idx == 0) && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].coefficient == 0)  && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].global_variable_index == 0)  && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].f_vector[VEC_idx].delay == 0))
				{
					if (VEC_idx > t_f_vector_max_len) t_f_vector_max_len = VEC_idx;
					break;
				}
			}
		}
		MotionMap_File.MS[MS_idx].max_f_vectors_len = t_f_vector_max_len;

		uint8_t t_i_vector_max_len = 0;
		for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
		{
			for (int VEC_idx = 0; VEC_idx < MAX_N_I_VECTORS; VEC_idx++)
			{
				if ((MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].epsilon_target == 0) && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].Kp_target == 0)      && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].Kd_target == 0)      && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].lambda_target == 0)  && \
					(MotionMap_File.MS[MS_idx].MD[MD_idx].i_vector[VEC_idx].duration == 0))

				{
					if (VEC_idx > t_i_vector_max_len) t_i_vector_max_len = VEC_idx;
					break;
				}
			}
		}
		MotionMap_File.MS[MS_idx].max_i_vectors_len = t_i_vector_max_len;
	}
}

void Send_Motion_Set(uint8_t MS_idx)
{
	//Send P-Vector
	if (MS_idx != 0)
	{
		MS_idx = MS_idx - 1;
		if (MotionMap_File.send_state == 0)
		{
			for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
			{
				if (RS_File.vec[MD_idx].usage == e_ENABLE)
				{
					uint8_t t_K_FF  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].FF_gain;
					uint8_t t_K_PD  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].PD_gain;
					uint8_t t_K_IC  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].IC_gain;
					uint8_t t_K_DOB = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].DOB_gain;
					uint8_t t_K_IRC = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].IRC_gain;
					uint8_t t_K_FC  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].c_vector[MotionMap_File.cnt].FC_gain;

					//Send_C_Vector(MD_idx, t_K_FF, t_K_PD, t_K_IC, t_K_DOB, t_K_IRC, t_K_FC);
				}
			}
			MotionMap_File.cnt=MotionMap_File.cnt % 8; //cnt 다 돌기 전에 state 천이가 일어나면 오버플로두 날때까지 cnt가 증가만 하여 초기화
			MotionMap_File.cnt++;
			if (MotionMap_File.cnt == 1) // Change later
			{
				MotionMap_File.cnt = 0;
				MotionMap_File.send_state = 1;
			}
		}

		if (MotionMap_File.send_state == 1)
		{
			if (MotionMap_File.MS[MS_idx].max_p_vectors_len == 0)
			{
				MotionMap_File.send_state = 2;
			}
			else
			{
				for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
				{
					if (RS_File.vec[MD_idx].usage == e_ENABLE)
					{
						int16_t  t_yd = MotionMap_File.MS[MS_idx].MD[MD_idx-2].p_vector[MotionMap_File.cnt].yd;
						uint16_t t_L  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].p_vector[MotionMap_File.cnt].L;
						uint8_t  t_s0 = MotionMap_File.MS[MS_idx].MD[MD_idx-2].p_vector[MotionMap_File.cnt].s0;
						uint8_t  t_sd = MotionMap_File.MS[MS_idx].MD[MD_idx-2].p_vector[MotionMap_File.cnt].sd;

						if (t_yd || t_L || t_s0 || t_sd)
						{
							if (AssistMode == STS_MODE) {
								if (IsBLEconnectedtoApp == true) {
									if (FSMMngrObj.state_curr == 70) {
										//t_L = StandtoSitDuration;	// default : Motion Map Files
										t_L = 10;	// default : Motion Map Files
									} else if (FSMMngrObj.state_curr == 74 || FSMMngrObj.state_curr == 75) {
										//t_L = SittoStandDuration; // default : Motion Map Files
										t_L = 2; // default : Motion Map Files
									}
									if (FSMMngrObj.state_curr == 67 && MD_idx == 2) {
										t_yd = (uint16_t)(RH_PositionAct*10);	// default : Motion Map Files
										t_s0 = 0;
										t_sd = 0;
									}else if(FSMMngrObj.state_curr == 67 && MD_idx == 3){
										t_yd = (uint16_t)(LH_PositionAct*10);
										t_s0 = 0;
										t_sd = 0;
									}
								}
							}
							if (AssistMode == STS_MODE && IsBLEconnectedtoApp == true &&(FSMMngrObj.state_curr == 70)) {
								if (MotionMap_File.cnt == 0 ) {
									Send_P_Vector(MD_idx, t_yd, 0.7 * t_L, t_s0, t_sd);
								} else if (MotionMap_File.cnt == 1) {
									Send_P_Vector(MD_idx, t_yd, 0.3 * t_L, t_s0, t_sd);
								}
							} else {
								Send_P_Vector(MD_idx, t_yd, t_L, t_s0, t_sd);
							}

						}
					}
				}
				MotionMap_File.cnt++;
				if (MotionMap_File.cnt == 1)
				{
					MotionMap_File.cnt = 0;
					MotionMap_File.send_state = 2;
				}
			}
		}

		// Send F-Vector
		if (MotionMap_File.send_state == 2)
		{
			if (MotionMap_File.MS[MS_idx].max_f_vectors_len == 0) // no F-vector
			{
				MotionMap_File.send_state = 3;
			}
			else
			{
//				if(MotionMap_File.cnt == 0)
//				{
//					for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
//					{
//						if (RS_File.vec[MD_idx].usage == e_ENABLE)
//						{
//							Send_F_Vector_Reset(MD_idx);
//						}
//					}
//				}

				for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
				{
					if (RS_File.vec[MD_idx].usage == e_ENABLE)
					{
						uint8_t  t_mode_idx = (uint8_t)MotionMap_File.MS[MS_idx].MD[MD_idx-2].f_vector[MotionMap_File.cnt].mode_idx;
						int16_t  t_coefficient  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].f_vector[MotionMap_File.cnt].coefficient;
						uint16_t t_global_variable_index  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].f_vector[MotionMap_File.cnt].global_variable_index;
						uint16_t t_delay    = MotionMap_File.MS[MS_idx].MD[MD_idx-2].f_vector[MotionMap_File.cnt].delay;
						uint16_t t_tau_max = 0;

						if (t_global_variable_index == 0x01) {
							t_tau_max = RightHipFlexionTorque;
						}
						if (t_global_variable_index == 0x02) {
							t_tau_max = RightHipExtensionTorque;
						}
						if (t_global_variable_index == 0x03) {
							t_tau_max = LeftHipFlexionTorque;
						}
						if (t_global_variable_index == 0x04) {
							t_tau_max = LeftHipExtensionTorque;
						}
						if (t_global_variable_index == 0x05) {
							t_tau_max = RightKneeFlexionTorque;
						}
						if (t_global_variable_index == 0x06) {
							t_tau_max = RightKneeExtensionTorque;
						}
						if (t_global_variable_index == 0x07) {
							t_tau_max = LeftKneeFlexionTorque;
						}
						if (t_global_variable_index == 0x08) {
							t_tau_max = LeftKneeExtensionTorque;
						}

						if (BLECtrlObj.data.AssistMode != SMART_ASSIST_MODE) {
													t_tau_max = 0;
												}

						if (t_mode_idx || t_coefficient || t_global_variable_index || t_delay)
						{
							if(t_mode_idx == 255)
							{
								Send_F_Vector_Reset(MD_idx);
							}
							else
							{
								#ifdef SUIT_MINICM_ENABLED
								if(t_coefficient > 0)
								{
									Send_F_Vector(MD_idx, t_mode_idx, (float)t_tau_max * assistForcePctg * 0.01 * t_coefficient, t_delay);
								}
								else if(assistForcePctg == 0)
								{
									Send_F_Vector(MD_idx, t_mode_idx, (float)t_tau_max * assistForcePctg * 0.01 * t_coefficient, t_delay);
								}
								else
								{
									Send_F_Vector(MD_idx, t_mode_idx, (float)t_tau_max * assistForcePctg * 0.01 * t_coefficient * (1 - extension_y_offset) + (float)t_tau_max * 0.01 * t_coefficient * extension_y_offset, t_delay);
								}
								#else
									Send_F_Vector(MD_idx, t_mode_idx, (float)t_tau_max * assistForcePctg * 0.01 * t_coefficient, t_delay);
								#endif
							}
						}
					}
				}

				MotionMap_File.cnt++;
				if (MotionMap_File.cnt == 3)
				{
					MotionMap_File.cnt = 0;
					MotionMap_File.send_state = 3;
				}
			}
		}

		//Send I-Vector
		if (MotionMap_File.send_state == 3)
		{
			if (MotionMap_File.MS[MS_idx].max_i_vectors_len == 0) // no I-vector
			{
				MotionMap_File.send_state = 4;
			}
			else
			{
				for (int MD_idx = 0; MD_idx < MAX_N_MD; MD_idx++)
				{
					if (RS_File.vec[MD_idx].usage == e_ENABLE)
					{
						uint8_t  t_epsilon   = MotionMap_File.MS[MS_idx].MD[MD_idx-2].i_vector[MotionMap_File.cnt].epsilon_target;
						uint8_t  t_Kp        = MotionMap_File.MS[MS_idx].MD[MD_idx-2].i_vector[MotionMap_File.cnt].Kp_target;
						uint8_t  t_Kd        = MotionMap_File.MS[MS_idx].MD[MD_idx-2].i_vector[MotionMap_File.cnt].Kd_target;
						uint8_t  t_lambda    = MotionMap_File.MS[MS_idx].MD[MD_idx-2].i_vector[MotionMap_File.cnt].lambda_target;
						uint16_t t_duration  = MotionMap_File.MS[MS_idx].MD[MD_idx-2].i_vector[MotionMap_File.cnt].duration;

						if (t_epsilon || t_Kp || t_Kd || t_lambda || t_duration)
						{
							Send_I_Vector(MD_idx, t_epsilon, t_Kp, t_Kd, t_lambda, t_duration);
						}
					}
				}
				MotionMap_File.cnt++;
				if (MotionMap_File.cnt == 1)
				{
					MotionMap_File.cnt = 0;
					MotionMap_File.send_state = 4;
				}
			}
		}
		else
		{

		}
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
