/**
 * stubs.c - host-build hardware stubs for the solver regression harness.
 *
 * lib/solver references the motion/sensor/battery layer throughout its real
 * state machine, but a headless Solver_RunSim() is pure CPU against a virtual
 * maze and calls NONE of it. These definitions exist only so the native link
 * succeeds; the bodies are no-ops. They're deliberately declared here WITHOUT
 * including the driver headers, so the linker matches purely by symbol name and
 * we don't have to reproduce ~130 exact prototypes. If the sim ever actually
 * calls one of these on the host, that's a bug in this harness's assumptions -
 * the sim is supposed to stay hardware-free.
 */
#include "main.h"      /* shim: DWT_Type_Host, SystemCoreClock, HAL_GetTick */

/* --- CMSIS bits the sim path really reads ------------------------------- */
uint32_t SystemCoreClock = 160000000u;   /* real F_CPU; only scales us timing */
uint32_t HAL_GetTick(void) { return 0; }

static DWT_Type_Host _dwt_host;
DWT_Type_Host *DWT = &_dwt_host;

/* --- hardware layer: never called in the sim, link-by-name no-ops ------- */
int Control_StartAdvance()        { return 0; }
int Control_StartPivot()          { return 0; }
int Control_GetState()            { return 0; }
int Control_GetFollow()           { return 0; }
int Control_SetFollow()           { return 0; }
int Control_RunBegin()            { return 0; }
int Control_RunEnd()              { return 0; }
int Control_SetPivotSnappy()      { return 0; }
int Control_SetReachCreep()       { return 0; }
int Control_SetFrontStop()        { return 0; }
int Control_Stop()                { return 0; }
int Control_AdvanceBrakeNow()     { return 0; }
int Control_GetArcGeom()          { return 0; }
int Control_StartFlowTurnInline() { return 0; }
int Control_StartContactProbe()   { return 0; }
int Control_PickupDetected()      { return 0; }
int Control_StartFrontAlign()     { return 0; }
int Control_IsActive()            { return 0; }
int Control_PopAlignDone()        { return 0; }
int Control_PopTurnDone()         { return 0; }
int Control_PopCellMark()         { return 0; }
int Control_GetTurnMode()         { return 0; }
int Control_PopMoveDone()         { return 0; }
int Control_LastMoveHitWall()     { return 0; }
int Control_LastMoveStalled()     { return 0; }
int Control_StartRecenter()       { return 0; }
int Control_PopRecenterDone()     { return 0; }
int Control_PopContactDone()      { return 0; }

int Walls_Front()          { return 0; }
int Walls_FrontConfident() { return 0; }
int Walls_FrontContact()   { return 0; }
int Walls_FrontRaw()       { return 0; }
int Walls_Right()          { return 0; }
int Walls_Left()           { return 0; }

/* Sensors_Raw returns a pointer to the 6 IR readings; the sim's log_append()
   DOES call it (to tag each logged cell with what the sensors "saw"). Return a
   valid zero array so the deref is safe - values are irrelevant in the sim. */
static int _sens_raw[8];
const int *Sensors_Raw(void) { return _sens_raw; }

/* Sensors_GetCal writes thresholds through its pointer args; harmless no-op
   (only feeds logged diagnostic values, not the flood-fill). */
int Sensors_GetCal() { return 0; }

int Battery_GetSnapshot() { return 0; }
int Battery_SetVacuum()   { return 0; }

/* mazestore: the search auto-saves its learned map to flash at the end; on host
   there's no flash, so this is a no-op (the sim doesn't need persistence). */
int MazeStore_Save() { return 0; }
