<<<<<<< HEAD
/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/odeconfig.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include "config.h"
#include "odemath.h"
#include "matrix.h"
#include "objects.h"
#include "joints/joint.h"
#include "lcp.h"
#include "util.h"
#include "threadingutils.h"

#include <new>


#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((B)>(A) ? (B) : (A))

//****************************************************************************
// misc defines

//#define TIMING


#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif


struct dJointWithInfo1
{
    dxJoint *joint;
    dxJoint::Info1 info;
};

enum dxRHSCFMElement
{
    RCE_RHS = dxJoint::GI2_RHS,
    RCE_CFM = dxJoint::GI2_CFM,
    
    // Elements for array reuse
    RLE_RHS = RCE_RHS,
    RLE_LAMBDA = RCE_CFM,

    RCE__RHS_CFM_MAX = dxJoint::GI2__RHS_CFM_MAX,
    RLE__RHS_LAMBDA_MAX = RCE__RHS_CFM_MAX,
};

enum dxLoHiElement
{
    LHE_LO = dxJoint::GI2_LO,
    LHE_HI = dxJoint::GI2_HI,

    LHE__LO_HI_MAX = dxJoint::GI2__LO_HI_MAX,
};

enum dxJacobiVectorElement
{
    JVE__MIN,

    JVE__L_MIN = JVE__MIN + dDA__L_MIN,

    JVE_LX = JVE__L_MIN + dSA_X,
    JVE_LY = JVE__L_MIN + dSA_Y,
    JVE_LZ = JVE__L_MIN + dSA_Z,

    JVE__L_MAX = JVE__L_MIN + dSA__MAX,

    JVE__A_MIN = JVE__MIN + dDA__A_MIN,

    JVE_AX = JVE__A_MIN + dSA_X,
    JVE_AY = JVE__A_MIN + dSA_Y,
    JVE_AZ = JVE__A_MIN + dSA_Z,

    JVE__A_MAX = JVE__A_MIN + dSA__MAX,

    JVE__MAX = JVE__MIN + dDA__MAX,

    JVE__L_COUNT = JVE__L_MAX - JVE__L_MIN,
    JVE__A_COUNT = JVE__A_MAX - JVE__A_MIN,
};


enum dxJacobiMatrixElement
{
    JME__MIN,

    JME__J_MIN = JME__MIN,
    JME__JL_MIN = JME__J_MIN + JVE__L_MIN,

    JME_JLX = JME__J_MIN + JVE_LX,
    JME_JLY = JME__J_MIN + JVE_LY,
    JME_JLZ = JME__J_MIN + JVE_LZ,

    JME__JL_MAX = JME__J_MIN + JVE__L_MAX,

    JME__JA_MIN = JME__J_MIN + JVE__A_MIN,

    JME_JAX = JME__J_MIN + JVE_AX,
    JME_JAY = JME__J_MIN + JVE_AY,
    JME_JAZ = JME__J_MIN + JVE_AZ,

    JME__JA_MAX = JME__J_MIN + JVE__A_MAX,
    JME__J_MAX = JME__J_MIN + JVE__MAX,

    JME__MAX = JME__J_MAX,

    JME__J_COUNT = JME__J_MAX - JME__J_MIN,
};

enum dxJInvMElement
{
    JIM__MIN,

    JIM__L_MIN = JIM__MIN + dMD_LINEAR * dV3E__MAX,

    JIM__L_AXES_MIN = JIM__L_MIN + dV3E__AXES_MIN,

    JIM_LX = JIM__L_MIN + dV3E_X,
    JIM_LY = JIM__L_MIN + dV3E_Y,
    JIM_LZ = JIM__L_MIN + dV3E_Z,

    JIM__L_AXES_MAX = JIM__L_MIN + dV3E__AXES_MAX,

    JIM_LPAD = JIM__L_MIN + dV3E_PAD,

    JIM__L_MAX = JIM__L_MIN + dV3E__MAX,

    JIM__A_MIN = JIM__MIN + dMD_ANGULAR * dV3E__MAX,

    JIM__A_AXES_MIN = JIM__A_MIN + dV3E__AXES_MIN,

    JIM_AX = JIM__A_MIN + dV3E_X,
    JIM_AY = JIM__A_MIN + dV3E_Y,
    JIM_AZ = JIM__A_MIN + dV3E_Z,

    JIM__A_AXES_MAX = JIM__A_MIN + dV3E__AXES_MAX,

    JIM_APAD = JIM__A_MIN + dV3E_PAD,

    JIM__A_MAX = JIM__A_MIN + dV3E__MAX,

    JIM__MAX = JIM__MIN + dMD__MAX * dV3E__MAX,
};

enum dxContactForceElement
{
    CFE__MIN,

    CFE__DYNAMICS_MIN = CFE__MIN,

    CFE__L_MIN = CFE__DYNAMICS_MIN + dDA__L_MIN,

    CFE_LX = CFE__DYNAMICS_MIN + dDA_LX,
    CFE_LY = CFE__DYNAMICS_MIN + dDA_LY,
    CFE_LZ = CFE__DYNAMICS_MIN + dDA_LZ,

    CFE__L_MAX = CFE__DYNAMICS_MIN + dDA__L_MAX,

    CFE__A_MIN = CFE__DYNAMICS_MIN + dDA__A_MIN,

    CFE_AX = CFE__DYNAMICS_MIN + dDA_AX,
    CFE_AY = CFE__DYNAMICS_MIN + dDA_AY,
    CFE_AZ = CFE__DYNAMICS_MIN + dDA_AZ,

    CFE__A_MAX = CFE__DYNAMICS_MIN + dDA__A_MAX,

    CFE__DYNAMICS_MAX = CFE__DYNAMICS_MIN + dDA__MAX,

    CFE__MAX = CFE__DYNAMICS_MAX,
};


#define AMATRIX_ALIGNMENT   dMAX(64, EFFICIENT_ALIGNMENT)
#define INVI_ALIGNMENT      dMAX(32, EFFICIENT_ALIGNMENT)
#define JINVM_ALIGNMENT     dMAX(64, EFFICIENT_ALIGNMENT)

struct dxStepperStage0Outputs
{
    size_t                          ji_start;
    size_t                          ji_end;
    unsigned int                    m;
    unsigned int                    nub;
};

struct dxStepperStage1CallContext
{
    void Initialize(const dxStepperProcessingCallContext *stepperCallContext, void *stageMemArenaState, dReal *invI, dJointWithInfo1 *jointinfos)
    {
        m_stepperCallContext = stepperCallContext;
        m_stageMemArenaState = stageMemArenaState;
        m_invI = invI;
        m_jointinfos = jointinfos;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    void                            *m_stageMemArenaState;
    dReal                           *m_invI;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          m_stage0Outputs;
};

struct dxStepperStage0BodiesCallContext
{
    void Initialize(const dxStepperProcessingCallContext *stepperCallContext, dReal *invI)
    {
        m_stepperCallContext = stepperCallContext;
        m_invI = invI;
        m_tagsTaken = 0;
        m_gravityTaken = 0;
        m_inertiaBodyIndex = 0;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    dReal                           *m_invI;
    atomicord32                     m_tagsTaken;
    atomicord32                     m_gravityTaken;
    volatile atomicord32            m_inertiaBodyIndex;
};

struct dxStepperStage0JointsCallContext
{
    void Initialize(const dxStepperProcessingCallContext *stepperCallContext, dJointWithInfo1 *jointinfos, dxStepperStage0Outputs *stage0Outputs)
    {
        m_stepperCallContext = stepperCallContext;
        m_jointinfos = jointinfos;
        m_stage0Outputs = stage0Outputs;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          *m_stage0Outputs;
};

static int dxStepIsland_Stage0_Bodies_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
// static int dxStepIsland_Stage0_Joints_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage1_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

static void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext);
static void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext);
static void dxStepIsland_Stage1(dxStepperStage1CallContext *callContext);


struct dxStepperLocalContext
{
    void Initialize(dReal *invI, dJointWithInfo1 *jointinfos, unsigned int nj, 
        unsigned int m, unsigned int nub, const unsigned int *mindex, int *findex, 
        dReal *J, dReal *A, dReal *pairsRhsCfm, dReal *pairsLoHi, 
        atomicord32 *bodyStartJoints, atomicord32 *bodyJointLinks)
    {
        m_invI = invI;
        m_jointinfos = jointinfos;
        m_nj = nj;
        m_m = m;
        m_nub = nub;
        m_mindex = mindex;
        m_findex = findex; 
        m_J = J;
        m_A = A;
        m_pairsRhsCfm = pairsRhsCfm;
        m_pairsLoHi = pairsLoHi;
        m_bodyStartJoints = bodyStartJoints;
        m_bodyJointLinks = bodyJointLinks;
    }

    dReal                           *m_invI;
    dJointWithInfo1                 *m_jointinfos;
    unsigned int                    m_nj;
    unsigned int                    m_m;
    unsigned int                    m_nub;
    const unsigned int              *m_mindex;
    int                             *m_findex;
    dReal                           *m_J;
    dReal                           *m_A;
    dReal                           *m_pairsRhsCfm;
    dReal                           *m_pairsLoHi;
    atomicord32                     *m_bodyStartJoints;
    atomicord32                     *m_bodyJointLinks;
};

struct dxStepperStage2CallContext
{
    void Initialize(const dxStepperProcessingCallContext *callContext, const dxStepperLocalContext *localContext, 
        dReal *JinvM, dReal *rhs_tmp)
    {
        m_stepperCallContext = callContext;
        m_localContext = localContext;
        m_JinvM = JinvM;
        m_rhs_tmp = rhs_tmp;
        m_ji_J = 0;
        m_ji_Ainit = 0;
        m_ji_JinvM = 0;
        m_ji_Aaddjb = 0;
        m_bi_rhs_tmp = 0;
        m_ji_rhs = 0;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    const dxStepperLocalContext     *m_localContext;
    dReal                           *m_JinvM;
    dReal                           *m_rhs_tmp;
    volatile atomicord32            m_ji_J;
    volatile atomicord32            m_ji_Ainit;
    volatile atomicord32            m_ji_JinvM;
    volatile atomicord32            m_ji_Aaddjb;
    volatile atomicord32            m_bi_rhs_tmp;
    volatile atomicord32            m_ji_rhs;
};

struct dxStepperStage3CallContext
{
    void Initialize(const dxStepperProcessingCallContext *callContext, const dxStepperLocalContext *localContext, 
        void *stage1MemArenaState)
    {
        m_stepperCallContext = callContext;
        m_localContext = localContext;
        m_stage1MemArenaState = stage1MemArenaState;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    const dxStepperLocalContext     *m_localContext;
    void                            *m_stage1MemArenaState;
};

struct dxStepperStage4CallContext
{
    void Initialize(const dxStepperProcessingCallContext *callContext, const dxStepperLocalContext *localContext/*, 
        void *stage3MemarenaState*/)
    {
        m_stepperCallContext = callContext;
        m_localContext = localContext;
        // m_stage3MemarenaState = stage3MemarenaState;
        m_bi_constrForce = 0;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    const dxStepperLocalContext     *m_localContext;
    // void                            *m_stage3MemarenaState;
    volatile atomicord32            m_bi_constrForce;
};

static int dxStepIsland_Stage2a_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2aSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2b_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2bSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2c_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage3_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

static void dxStepIsland_Stage2a(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage2b(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage2c(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage3(dxStepperStage3CallContext *callContext);

static int dxStepIsland_Stage4_Callback(void *_stage4CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static void dxStepIsland_Stage4(dxStepperStage4CallContext *stage4CallContext);


//****************************************************************************
// special matrix multipliers


// this assumes the 4th and 8th rows of B and C are zero.

static inline 
void MultiplyAddJinvMxJToA (dReal *Arow, const dReal *JinvMRow, const dReal *JRow,
    unsigned int infomJinvM, unsigned int infomJ, unsigned int mskip)
{
    dIASSERT (infomJinvM > 0 && infomJ > 0 && Arow && JinvMRow && JRow);
    const unsigned int mskip_munus_infomJ_plus_1 = mskip - infomJ + 1;
    dIASSERT(mskip >= infomJ);
    dReal *currA = Arow;
    const dReal *currJinvM = JinvMRow;
    for (unsigned int i = infomJinvM; ; ) {
        dReal JiM0 = currJinvM[JIM_LX];
        dReal JiM1 = currJinvM[JIM_LY];
        dReal JiM2 = currJinvM[JIM_LZ];
        dReal JiM4 = currJinvM[JIM_AX];
        dReal JiM5 = currJinvM[JIM_AY];
        dReal JiM6 = currJinvM[JIM_AZ];
        const dReal *currJ = JRow;
        for (unsigned int j = infomJ; ; ) {
            dReal sum;
            sum  = JiM0 * currJ[JME_JLX];
            sum += JiM1 * currJ[JME_JLY];
            sum += JiM2 * currJ[JME_JLZ];
            sum += JiM4 * currJ[JME_JAX];
            sum += JiM5 * currJ[JME_JAY];
            sum += JiM6 * currJ[JME_JAZ];
            *currA += sum; 
            if (--j == 0) {
                break;
            }
            ++currA;
            currJ += JME__MAX;
        }
        if (--i == 0) {
            break;
        }
        currJinvM += JIM__MAX;
        currA += mskip_munus_infomJ_plus_1;
    }
}


// this assumes the 4th and 8th rows of B are zero.

static inline 
void MultiplySubJxRhsTmpFromRHS (dReal *rowRhsCfm, const dReal *JRow, const dReal *rowRhsTmp, unsigned int infom)
{
    dIASSERT (infom > 0 && rowRhsCfm && JRow && rowRhsTmp);
    dReal *currRhs = rowRhsCfm + RCE_RHS;
    const dReal *currJ = JRow;
    const dReal RT_LX = rowRhsTmp[dDA_LX], RT_LY = rowRhsTmp[dDA_LY], RT_LZ = rowRhsTmp[dDA_LZ];
    const dReal RT_AX = rowRhsTmp[dDA_AX], RT_AY = rowRhsTmp[dDA_AY], RT_AZ = rowRhsTmp[dDA_AZ];
    for (unsigned int i = infom; ; ) {
        dReal sum;
        sum  = currJ[JME_JLX] * RT_LX;
        sum += currJ[JME_JLY] * RT_LY;
        sum += currJ[JME_JLZ] * RT_LZ;
        sum += currJ[JME_JAX] * RT_AX;
        sum += currJ[JME_JAY] * RT_AY;
        sum += currJ[JME_JAZ] * RT_AZ;
        *currRhs -= sum;
        if (--i == 0) {
            break;
        }
        currRhs += RCE__RHS_CFM_MAX;
        currJ += JME__MAX;
    }
}


static inline 
void MultiplyAddJxLambdaToCForce(dReal cforce[CFE__MAX], 
    const dReal *JRow, const dReal *rowRhsLambda, unsigned int infom, 
    dJointFeedback *fb/*=NULL*/, unsigned jointBodyIndex)
{
    dIASSERT (infom > 0 && cforce && JRow && rowRhsLambda);
    dReal sumLX = 0, sumLY = 0, sumLZ = 0, sumAX=0, sumAY = 0, sumAZ = 0;
    const dReal *currJ = JRow, *currLambda = rowRhsLambda + RLE_LAMBDA;
    for (unsigned int k = infom; ; ) {
        const dReal lambda = *currLambda;
        sumLX += currJ[JME_JLX] * lambda;
        sumLY += currJ[JME_JLY] * lambda;
        sumLZ += currJ[JME_JLZ] * lambda;
        sumAX += currJ[JME_JAX] * lambda;
        sumAY += currJ[JME_JAY] * lambda;
        sumAZ += currJ[JME_JAZ] * lambda;
        if (--k == 0) {
            break;
        }
        currJ += JME__MAX;
        currLambda += RLE__RHS_LAMBDA_MAX;
    }
    if (fb != NULL) {
        if (jointBodyIndex == dJCB__MIN) {
            fb->f1[dV3E_X] = sumLX;
            fb->f1[dV3E_Y] = sumLY;
            fb->f1[dV3E_Z] = sumLZ;
            fb->t1[dV3E_X] = sumAX;
            fb->t1[dV3E_Y] = sumAY;
            fb->t1[dV3E_Z] = sumAZ;
        }
        else {
            dIASSERT(jointBodyIndex == dJCB__MIN + 1);
            dSASSERT(dJCB__MAX == 2);

            fb->f2[dV3E_X] = sumLX;
            fb->f2[dV3E_Y] = sumLY;
            fb->f2[dV3E_Z] = sumLZ;
            fb->t2[dV3E_X] = sumAX;
            fb->t2[dV3E_Y] = sumAY;
            fb->t2[dV3E_Z] = sumAZ;
        }
    }
    cforce[CFE_LX] += sumLX;
    cforce[CFE_LY] += sumLY;
    cforce[CFE_LZ] += sumLZ;
    cforce[CFE_AX] += sumAX;
    cforce[CFE_AY] += sumAY;
    cforce[CFE_AZ] += sumAZ;
}


//****************************************************************************

/*extern */
void dxStepIsland(const dxStepperProcessingCallContext *callContext)
{
    IFTIMING(dTimerStart("preprocessing"));

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    dxWorld *world = callContext->m_world;
    unsigned int nb = callContext->m_islandBodiesCount;
    unsigned int _nj = callContext->m_islandJointsCount;

    dReal *invI = memarena->AllocateOveralignedArray<dReal>(dM3E__MAX * (size_t)nb, INVI_ALIGNMENT);
    // Reserve twice as much memory and start from the middle so that regardless of 
    // what direction the array grows to there would be sufficient room available.
    const size_t ji_reserve_count = 2 * (size_t)_nj;
    dJointWithInfo1 *const jointinfos = memarena->AllocateArray<dJointWithInfo1>(ji_reserve_count);

    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
    dIASSERT(allowedThreads != 0);

    void *stagesMemArenaState = memarena->SaveState();

    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage1CallContext));
    stage1CallContext->Initialize(callContext, stagesMemArenaState, invI, jointinfos);

    dxStepperStage0BodiesCallContext *stage0BodiesCallContext = (dxStepperStage0BodiesCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0BodiesCallContext));
    stage0BodiesCallContext->Initialize(callContext, invI);
    
    dxStepperStage0JointsCallContext *stage0JointsCallContext = (dxStepperStage0JointsCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0JointsCallContext));
    stage0JointsCallContext->Initialize(callContext, jointinfos, &stage1CallContext->m_stage0Outputs);

    if (allowedThreads == 1)
    {
        dxStepIsland_Stage0_Bodies(stage0BodiesCallContext);
        dxStepIsland_Stage0_Joints(stage0JointsCallContext);
        dxStepIsland_Stage1(stage1CallContext);
    }
    else
    {
        unsigned bodyThreads = allowedThreads;
        unsigned jointThreads = 1;

        dCallReleaseeID stage1CallReleasee;
        world->PostThreadedCallForUnawareReleasee(NULL, &stage1CallReleasee, bodyThreads + jointThreads, callContext->m_finalReleasee, 
            NULL, &dxStepIsland_Stage1_Callback, stage1CallContext, 0, "StepIsland Stage1");

        world->PostThreadedCallsGroup(NULL, bodyThreads, stage1CallReleasee, &dxStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, "StepIsland Stage0-Bodies");

        dxStepIsland_Stage0_Joints(stage0JointsCallContext);
        world->AlterThreadedCallDependenciesCount(stage1CallReleasee, -1);
        dIASSERT(jointThreads == 1);
    }
}    

static 
int dxStepIsland_Stage0_Bodies_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage0BodiesCallContext *callContext = (dxStepperStage0BodiesCallContext *)_callContext;
    dxStepIsland_Stage0_Bodies(callContext);
    return 1;
}

static 
void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext)
{
    dxBody * const *body = callContext->m_stepperCallContext->m_islandBodiesStart;
    unsigned int nb = callContext->m_stepperCallContext->m_islandBodiesCount;

    if (ThrsafeExchange(&callContext->m_tagsTaken, 1) == 0)
    {
        // number all bodies in the body list - set their tag values
        for (unsigned int i=0; i<nb; i++) body[i]->tag = i;
    }

    if (ThrsafeExchange(&callContext->m_gravityTaken, 1) == 0)
    {
        dxWorld *world = callContext->m_stepperCallContext->m_world;

        // add the gravity force to all bodies
        // since gravity does normally have only one component it's more efficient
        // to run three loops for each individual component
        dxBody *const *const bodyend = body + nb;
        dReal gravity_x = world->gravity[0];
        if (gravity_x) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[dV3E_X] += b->mass.mass * gravity_x;
                }
            }
        }
        dReal gravity_y = world->gravity[1];
        if (gravity_y) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[dV3E_Y] += b->mass.mass * gravity_y;
                }
            }
        }
        dReal gravity_z = world->gravity[2];
        if (gravity_z) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[dV3E_Z] += b->mass.mass * gravity_z;
                }
            }
        }
    }

    // for all bodies, compute the inertia tensor and its inverse in the global
    // frame, and compute the rotational force and add it to the torque
    // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
    {
        dReal *invIrow = callContext->m_invI;
        unsigned int bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);

        for (unsigned int i = 0; i != nb; invIrow += dM3E__MAX, ++i) {
            if (i == bodyIndex) {
                dMatrix3 tmp;
                dxBody *b = body[i];

                // compute inverse inertia tensor in global frame
                dMultiply2_333 (tmp, b->invI, b->posr.R);
                dMultiply0_333 (invIrow, b->posr.R, tmp);

                // Don't apply gyroscopic torques to bodies
                // if not flagged or the body is kinematic
                if ((b->flags & dxBodyGyroscopic) && (b->invMass > 0)) {
                    dMatrix3 I;
                    // compute inertia tensor in global frame
                    dMultiply2_333 (tmp,b->mass.I,b->posr.R);
                    dMultiply0_333 (I,b->posr.R,tmp);
                    // compute rotational force
#if 0
                    // Explicit computation
                    dMultiply0_331 (tmp,I,b->avel);
                    dSubtractVectorCross3(b->tacc,b->avel,tmp);
#else
                    // Do the implicit computation based on 
                    //"Stabilizing Gyroscopic Forces in Rigid Multibody Simulations"
                    // (Lacoursière 2006)
                    dReal h = callContext->m_stepperCallContext->m_stepSize; // Step size
                    dVector3 L; // Compute angular momentum
                    dMultiply0_331(L, I, b->avel);
                    
                    // Compute a new effective 'inertia tensor'
                    // for the implicit step: the cross-product 
                    // matrix of the angular momentum plus the
                    // old tensor scaled by the timestep.  
                    // Itild may not be symmetric pos-definite, 
                    // but we can still use it to compute implicit
                    // gyroscopic torques.
                    dMatrix3 Itild = { 0 };  
                    dSetCrossMatrixMinus(Itild, L, dV3E__MAX);
                    for (int ii = dM3E__MIN; ii != dM3E__MAX; ++ii) {
                      Itild[ii] = Itild[ii] * h + I[ii];
                    }

                    // Scale momentum by inverse time to get 
                    // a sort of "torque"
                    dScaleVector3(L, dRecip(h)); 
                    // Invert the pseudo-tensor
                    dMatrix3 itInv;
                    // This is a closed-form inversion.
                    // It's probably not numerically stable
                    // when dealing with small masses with
                    // a large asymmetry.
                    // An LU decomposition might be better.
                    if (dInvertMatrix3(itInv, Itild) != 0) {
                        // "Divide" the original tensor
                        // by the pseudo-tensor (on the right)
                        dMultiply0_333(Itild, I, itInv);
                        // Subtract an identity matrix
                        Itild[dM3E_XX] -= 1; Itild[dM3E_YY] -= 1; Itild[dM3E_ZZ] -= 1;

                        // This new inertia matrix rotates the 
                        // momentum to get a new set of torques
                        // that will work correctly when applied
                        // to the old inertia matrix as explicit
                        // torques with a semi-implicit update
                        // step.
                        dVector3 tau0;
                        dMultiply0_331(tau0,Itild,L);
                        
                        // Add the gyro torques to the torque 
                        // accumulator
                        for (int ii = dSA__MIN; ii != dSA__MAX; ++ii) {
                          b->tacc[dV3E__AXES_MIN + ii] += tau0[dV3E__AXES_MIN + ii];
                        }
                    }
#endif
                }

                bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);
            }
        }
    }
}

// static 
// int dxStepIsland_Stage0_Joints_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
// {
//     (void)callInstanceIndex; // unused
//     (void)callThisReleasee; // unused
//     dxStepperStage0JointsCallContext *callContext = (dxStepperStage0JointsCallContext *)_callContext;
//     dxStepIsland_Stage0_Joints(callContext);
//     return 1;
// }

static 
void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext)
{
    dxJoint * const *_joint = callContext->m_stepperCallContext->m_islandJointsStart;
    dJointWithInfo1 *jointinfos = callContext->m_jointinfos;
    unsigned int _nj = callContext->m_stepperCallContext->m_islandJointsCount;

    // get m = total constraint dimension, nub = number of unbounded variables.
    // create constraint offset array and number-of-rows array for all joints.
    // the constraints are re-ordered as follows: the purely unbounded
    // constraints, the mixed unbounded + LCP constraints, and last the purely
    // LCP constraints. this assists the LCP solver to put all unbounded
    // variables at the start for a quick factorization.
    //
    // joints with m=0 are inactive and are removed from the joints array
    // entirely, so that the code that follows does not consider them.
    // also number all active joints in the joint list (set their tag values).
    // inactive joints receive a tag value of -1.

    size_t ji_start, ji_end;
    {
        unsigned int mcurr = 0;
        size_t unb_start, mix_start, mix_end, lcp_end;
        unb_start = mix_start = mix_end = lcp_end = _nj;

        dJointWithInfo1 *jicurr = jointinfos + lcp_end;
        dxJoint *const *const _jend = _joint + _nj;
        dxJoint *const *_jcurr = _joint;
        while (true) {
            // -------------------------------------------------------------------------
            // Switch to growing array forward
            {
                bool fwd_end_reached = false;
                dJointWithInfo1 *jimixend = jointinfos + mix_end;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        lcp_end = jicurr - jointinfos;
                        fwd_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == 0) { // A lcp info - a correct guess!!!
                            jicurr->joint = j;
                            ++jicurr;
                        } else if (jicurr->info.nub < jicurr->info.m) { // A mixed case
                            if (unb_start == mix_start) { // no unbounded infos yet - just move to opposite side of mixed-s
                                unb_start = mix_start = mix_start - 1;
                                dJointWithInfo1 *jimixstart = jointinfos + mix_start;
                                jimixstart->info = jicurr->info;
                                jimixstart->joint = j;
                            } else if (jimixend != jicurr) { // have to swap to the tail of mixed-s
                                dxJoint::Info1 tmp_info = jicurr->info;
                                *jicurr = *jimixend;
                                jimixend->info = tmp_info;
                                jimixend->joint = j;
                                ++jimixend; ++jicurr;
                            } else { // no need to swap as there are no LCP info-s yet
                                jicurr->joint = j;
                                jimixend = jicurr = jicurr + 1;
                            }
                        } else { // A purely unbounded case -- break out and proceed growing in opposite direction
                            unb_start = unb_start - 1;
                            dJointWithInfo1 *jiunbstart = jointinfos + unb_start;
                            jiunbstart->info = jicurr->info;
                            jiunbstart->joint = j;
                            lcp_end = jicurr - jointinfos;
                            mix_end = jimixend - jointinfos;
                            jicurr = jiunbstart - 1;
                            break;
                        }
                    } else {
                        j->tag = -1;
                    }
                }
                if (fwd_end_reached) {
                    break;
                }
            }
            // -------------------------------------------------------------------------
            // Switch to growing array backward
            {
                bool bkw_end_reached = false;
                dJointWithInfo1 *jimixstart = jointinfos + mix_start - 1;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        unb_start = (jicurr + 1) - jointinfos;
                        mix_start = (jimixstart + 1) - jointinfos;
                        bkw_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == jicurr->info.m) { // An unbounded info - a correct guess!!!
                            jicurr->joint = j;
                            --jicurr;
                        } else if (jicurr->info.nub != 0) { // A mixed case
                            if (mix_end == lcp_end) { // no lcp infos yet - just move to opposite side of mixed-s
                                dJointWithInfo1 *jimixend = jointinfos + mix_end;
                                lcp_end = mix_end = mix_end + 1;
                                jimixend->info = jicurr->info;
                                jimixend->joint = j;
                            } else if (jimixstart != jicurr) { // have to swap to the head of mixed-s
                                dxJoint::Info1 tmp_info = jicurr->info;
                                *jicurr = *jimixstart;
                                jimixstart->info = tmp_info;
                                jimixstart->joint = j;
                                --jimixstart; --jicurr;
                            } else { // no need to swap as there are no unbounded info-s yet
                                jicurr->joint = j;
                                jimixstart = jicurr = jicurr - 1;
                            }
                        } else { // A purely lcp case -- break out and proceed growing in opposite direction
                            dJointWithInfo1 *jilcpend = jointinfos + lcp_end;
                            lcp_end = lcp_end + 1;
                            jilcpend->info = jicurr->info;
                            jilcpend->joint = j;
                            unb_start = (jicurr + 1) - jointinfos;
                            mix_start = (jimixstart + 1) - jointinfos;
                            jicurr = jilcpend + 1;
                            break;
                        }
                    } else {
                        j->tag = -1;
                    }
                }
                if (bkw_end_reached) {
                    break;
                }
            }
        }

        callContext->m_stage0Outputs->m = mcurr;
        callContext->m_stage0Outputs->nub = (unsigned)(mix_start - unb_start);
        dIASSERT((size_t)(mix_start - unb_start) <= (size_t)UINT_MAX);
        ji_start = unb_start;
        ji_end = lcp_end;
    }

    {
        const dJointWithInfo1 *jicurr = jointinfos + ji_start;
        const dJointWithInfo1 *const jiend = jointinfos + ji_end;
        for (unsigned int i = 0; jicurr != jiend; i++, ++jicurr) {
            jicurr->joint->tag = i;
        }
    }

    callContext->m_stage0Outputs->ji_start = ji_start;
    callContext->m_stage0Outputs->ji_end = ji_end;
}

static 
int dxStepIsland_Stage1_Callback(void *_stage1CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)_stage1CallContext;
    dxStepIsland_Stage1(stage1CallContext);
    return 1;
}

static 
void dxStepIsland_Stage1(dxStepperStage1CallContext *stage1CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage1CallContext->m_stepperCallContext;
    dJointWithInfo1 *_jointinfos = stage1CallContext->m_jointinfos;
    dReal *invI = stage1CallContext->m_invI;
    size_t ji_start = stage1CallContext->m_stage0Outputs.ji_start;
    size_t ji_end = stage1CallContext->m_stage0Outputs.ji_end;
    unsigned int m = stage1CallContext->m_stage0Outputs.m;
    unsigned int nub = stage1CallContext->m_stage0Outputs.nub;

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    {
        memarena->RestoreState(stage1CallContext->m_stageMemArenaState);
        stage1CallContext = NULL; // WARNING! _stage1CallContext is not valid after this point!
        dIVERIFY(stage1CallContext == NULL); // To suppress compiler warnings about unused variable assignment

        unsigned int _nj = callContext->m_islandJointsCount;
        const size_t ji_reserve_count = 2 * (size_t)_nj;
        memarena->ShrinkArray<dJointWithInfo1>(_jointinfos, ji_reserve_count, ji_end);
    }

    dJointWithInfo1 *jointinfos = _jointinfos + ji_start;
    unsigned int nj = (unsigned int)(ji_end - ji_start);
    dIASSERT((size_t)(ji_end - ji_start) <= (size_t)UINT_MAX);

    unsigned int *mindex = NULL;
    dReal *J = NULL, *A = NULL, *pairsRhsCfm = NULL, *pairsLoHi = NULL;
    int *findex = NULL;
    atomicord32 *bodyStartJoints = NULL, *bodyJointLinks = NULL;

    // if there are constraints, compute constrForce
    if (m > 0) {
        mindex = memarena->AllocateArray<unsigned int>((size_t)(nj + 1));
        {
            unsigned int *mcurr = mindex;
            unsigned int moffs = 0;
            mcurr[0] = moffs;
            mcurr += 1;

            const dJointWithInfo1 *const jiend = jointinfos + nj;
            for (const dJointWithInfo1 *jicurr = jointinfos; jicurr != jiend; ++jicurr) {
                //dxJoint *joint = jicurr->joint;
                moffs += jicurr->info.m;
                mcurr[0] = moffs;
                mcurr += 1;
            }
        }

        // create a constraint equation right hand side vector `c', a constraint
        // force mixing vector `cfm', and LCP low and high bound vectors, and an
        // 'findex' vector.
        findex = memarena->AllocateArray<int>(m);
        J = memarena->AllocateArray<dReal>((size_t)m * (2 * JME__MAX));
        A = memarena->AllocateOveralignedArray<dReal>((size_t)m * dPAD(m), AMATRIX_ALIGNMENT);
        pairsRhsCfm = memarena->AllocateArray<dReal>((size_t)m * RCE__RHS_CFM_MAX);
        pairsLoHi = memarena->AllocateArray<dReal>((size_t)m * LHE__LO_HI_MAX);
        const unsigned int nb = callContext->m_islandBodiesCount;
        bodyStartJoints = memarena->AllocateArray<atomicord32>(nb);
        bodyJointLinks = memarena->AllocateArray<atomicord32>((size_t)nj * dJCB__MAX);
        dICHECK(nj < ~((atomicord32)0) / dJCB__MAX); // If larger joint counts are to be used, pointers (or size_t) need to be stored rather than atomicord32 indices
    }

    dxStepperLocalContext *localContext = (dxStepperLocalContext *)memarena->AllocateBlock(sizeof(dxStepperLocalContext));
    localContext->Initialize(invI, jointinfos, nj, m, nub, mindex, findex, J, A, pairsRhsCfm, pairsLoHi, bodyStartJoints, bodyJointLinks);

    void *stage1MemarenaState = memarena->SaveState();
    dxStepperStage3CallContext *stage3CallContext = (dxStepperStage3CallContext*)memarena->AllocateBlock(sizeof(dxStepperStage3CallContext));
    stage3CallContext->Initialize(callContext, localContext, stage1MemarenaState);

    if (m > 0) {
        dReal *JinvM = memarena->AllocateOveralignedArray<dReal>((size_t)m * (2 * JIM__MAX), JINVM_ALIGNMENT);
        const unsigned int nb = callContext->m_islandBodiesCount;
        dReal *rhs_tmp = memarena->AllocateArray<dReal>((size_t)nb * dDA__MAX);

        dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage2CallContext));
        stage2CallContext->Initialize(callContext, localContext, JinvM, rhs_tmp);

        const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
        dIASSERT(allowedThreads != 0);

        if (allowedThreads == 1) {
            IFTIMING(dTimerNow("create J"));
            dxStepIsland_Stage2a(stage2CallContext);
            IFTIMING(dTimerNow("compute Adiag, JinvM and rhs_tmp"));
            dxStepIsland_Stage2b(stage2CallContext);
            IFTIMING(dTimerNow("compute A and rhs"));
            dxStepIsland_Stage2c(stage2CallContext);
            dxStepIsland_Stage3(stage3CallContext);
        }
        else {
            dxWorld *world = callContext->m_world;
            dCallReleaseeID stage3CallReleasee;
            world->PostThreadedCallForUnawareReleasee(NULL, &stage3CallReleasee, 1, callContext->m_finalReleasee, 
                NULL, &dxStepIsland_Stage3_Callback, stage3CallContext, 0, "StepIsland Stage3");

            dCallReleaseeID stage2bSyncReleasee;
            world->PostThreadedCall(NULL, &stage2bSyncReleasee, 1, stage3CallReleasee, 
                NULL, &dxStepIsland_Stage2bSync_Callback, stage2CallContext, 0, "StepIsland Stage2b Sync");

            dCallReleaseeID stage2aSyncReleasee;
            world->PostThreadedCall(NULL, &stage2aSyncReleasee, allowedThreads, stage2bSyncReleasee, 
                NULL, &dxStepIsland_Stage2aSync_Callback, stage2CallContext, 0, "StepIsland Stage2a Sync");

            dIASSERT(allowedThreads > 1); /*if (allowedThreads > 1) */{
                world->PostThreadedCallsGroup(NULL, allowedThreads - 1, stage2aSyncReleasee, &dxStepIsland_Stage2a_Callback, stage2CallContext, "StepIsland Stage2a");
            }
            dxStepIsland_Stage2a(stage2CallContext);
            world->AlterThreadedCallDependenciesCount(stage2aSyncReleasee, -1);
        }
    }
    else {
        dxStepIsland_Stage3(stage3CallContext);
    }
}


static 
int dxStepIsland_Stage2a_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2a(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2a(dxStepperStage2CallContext *stage2CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    const dReal stepsizeRecip = dRecip(callContext->m_stepSize);
    dxWorld *world = callContext->m_world;

    {
        int *findex = localContext->m_findex;
        dReal *J = localContext->m_J;
        dReal *pairsRhsCfm = localContext->m_pairsRhsCfm;
        dReal *pairsLoHi = localContext->m_pairsLoHi;

        // get jacobian data from constraints. a (2*m)x8 matrix will be created
        // to store the two jacobian blocks from each constraint. it has this
        // format:
        //
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
        //   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
        //   etc...
        //
        //   (lll) = linear jacobian data
        //   (aaa) = angular jacobian data
        //

        const dReal worldERP = world->global_erp;
        const dReal worldCFM = world->global_cfm;

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_J, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *const JRow = J + (size_t)ofsi * (2 * JME__MAX);
            dReal *rowRhsCfm = pairsRhsCfm + (size_t)ofsi * RCE__RHS_CFM_MAX;
            dReal *rowLoHi = pairsLoHi + (size_t)ofsi * LHE__LO_HI_MAX;
            {
                dSetZero (JRow, infom * (2 * JME__MAX));

                dReal *const endRhsCfm = rowRhsCfm + infom * RCE__RHS_CFM_MAX;
                for (dReal *currRhsCfm = rowRhsCfm; currRhsCfm != endRhsCfm; currRhsCfm += RCE__RHS_CFM_MAX) {
                    currRhsCfm[RCE_RHS] = REAL(0.0);
                    currRhsCfm[RCE_CFM] = worldCFM;
                }

                dReal *const endLoHi = rowLoHi + infom * LHE__LO_HI_MAX;
                for (dReal *currLoHi = rowLoHi; currLoHi != endLoHi; currLoHi += LHE__LO_HI_MAX) {
                    currLoHi[LHE_LO] = -dInfinity;
                    currLoHi[LHE_HI] = dInfinity;
                }
            }
            int *findexRow = findex + ofsi;
            dSetValue(findexRow, infom, -1);

            dxJoint *joint = jointinfos[ji].joint;
            joint->getInfo2(stepsizeRecip, worldERP, JME__MAX, JRow + JME__J_MIN, JRow + infom * JME__MAX + JME__J_MIN, RCE__RHS_CFM_MAX, rowRhsCfm, rowLoHi, findexRow);
            dSASSERT((int)LHE__LO_HI_MAX == RCE__RHS_CFM_MAX); // To make sure same step fits for both pairs in the call to dxJoint::getInfo2() above

            // findex iteration is compact and is not going to pollute caches - do it first
            {
                // adjust returned findex values for global index numbering
                int *const findicesEnd = findexRow + infom;
                for (int *findexCurr = findexRow; findexCurr != findicesEnd; ++findexCurr) {
                    int fival = *findexCurr;
                    if (fival != -1) {
                        *findexCurr = fival + ofsi;
                    }
                }
            }
            {
                dReal *const endRhsCfm = rowRhsCfm + infom * RCE__RHS_CFM_MAX;
                for (dReal *currRhsCfm = rowRhsCfm; currRhsCfm != endRhsCfm; currRhsCfm += RCE__RHS_CFM_MAX) {
                    currRhsCfm[RCE_RHS] *= stepsizeRecip;
                    currRhsCfm[RCE_CFM] *= stepsizeRecip;
                }
            }
        }
    }
}

static 
int dxStepIsland_Stage2aSync_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;

    dIASSERT(allowedThreads > 1); /*if (allowedThreads > 1) */{ // The allowed thread count is greater than one as otherwise current function would not be scheduled for execution from the previous stage
        dxWorld *world = callContext->m_world;
        world->AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads - 1);
        world->PostThreadedCallsGroup(NULL, allowedThreads - 1, callThisReleasee, &dxStepIsland_Stage2b_Callback, stage2CallContext, "StepIsland Stage2b");
    }
    dxStepIsland_Stage2b(stage2CallContext);

    return 1;
}

static 
int dxStepIsland_Stage2b_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2b(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2b(dxStepperStage2CallContext *stage2CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    {
        // Warning!!!
        // This code depends on cfm elements and therefore must be in different sub-stage 
        // from Jacobian construction in Stage2a to ensure proper synchronization 
        // and avoid accessing numbers being modified.
        // Warning!!!
        dReal *A = localContext->m_A;
        const dReal *pairsRhsCfm = localContext->m_pairsRhsCfm;
        const unsigned m = localContext->m_m;

        const unsigned int mskip = dPAD(m);

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_Ainit, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Arow = A + (size_t)mskip * ofsi;
            dSetZero(Arow, (size_t)mskip * infom);
            dReal *Adiag = Arow + ofsi;
            const dReal *rowRfsCrm = pairsRhsCfm + (size_t)ofsi * RCE__RHS_CFM_MAX;
            for (unsigned int i = 0; i != infom; Adiag += mskip, ++i) {
                Adiag[i] = (rowRfsCrm + i * RCE__RHS_CFM_MAX)[RCE_CFM];
            }
        }
    }

    {
        // Warning!!!
        // This code depends on J elements and therefore must be in different sub-stage 
        // from Jacobian construction in Stage2a to ensure proper synchronization 
        // and avoid accessing numbers being modified.
        // Warning!!!
        const dReal *invI = localContext->m_invI;
        const dReal *J = localContext->m_J;
        dReal *JinvM = stage2CallContext->m_JinvM;

        // compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
        // format as J so we just go through the constraints in J multiplying by
        // the appropriate scalars and matrices.
        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_JinvM, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Jdst = JinvM + (size_t)ofsi * (2 * JIM__MAX);
            dSetZero(Jdst, infom * (2 * JIM__MAX));

            const dReal *Jsrc = J + (size_t)ofsi * (2 * JME__MAX);
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                dReal body_invMass0 = jb0->invMass;
                const dReal *body_invI0 = invI + (size_t)(unsigned int)jb0->tag * dM3E__MAX;
                for (unsigned int j = infom; j != 0; --j) {
                    for (unsigned int k = dSA__MIN; k != dSA__MAX; ++k) Jdst[JIM__L_AXES_MIN + k] = Jsrc[JME__JL_MIN + k] * body_invMass0;
                    dMultiply0_133(Jdst + JIM__A_AXES_MIN, Jsrc + JME__JA_MIN, body_invI0);
                    Jsrc += JME__MAX;
                    Jdst += JIM__MAX;
                }
            }

            dxBody *jb1 = joint->node[1].body;
            if (jb1 != NULL) {
                dReal body_invMass1 = jb1->invMass;
                const dReal *body_invI1 = invI + (size_t)(unsigned int)jb1->tag * dM3E__MAX;
                for (unsigned int j = infom; j != 0; --j) {
                    for (unsigned int k = dSA__MIN; k != dSA__MAX; ++k) Jdst[JIM__L_AXES_MIN + k] = Jsrc[JME__JL_MIN + k] * body_invMass1;
                    dMultiply0_133 (Jdst + JIM__A_AXES_MIN, Jsrc + JME__JA_MIN, body_invI1);
                    Jsrc += JME__MAX;
                    Jdst += JIM__MAX;
                }
            }
        }
    }

    {
        // Warning!!!
        // This code reads facc/tacc fields of body objects which (the fields)
        // may be modified by dxJoint::getInfo2(). Therefore the code must be
        // in different sub-stage from Jacobian construction in Stage2a 
        // to ensure proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        dxBody * const *const body = callContext->m_islandBodiesStart;
        const unsigned int nb = callContext->m_islandBodiesCount;
        const dReal *invI = localContext->m_invI;
        atomicord32 *bodyStartJoints = localContext->m_bodyStartJoints;
        dReal *rhs_tmp = stage2CallContext->m_rhs_tmp;

        // compute the right hand side `rhs'
        const dReal stepsizeRecip = dRecip(callContext->m_stepSize);

        // put v/h + invM*fe into rhs_tmp
        unsigned bi;
        while ((bi = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_bi_rhs_tmp, nb)) != nb) {
            dReal *tmp1curr = rhs_tmp + (size_t)bi * dDA__MAX;
            const dReal *invIrow = invI + (size_t)bi * dM3E__MAX;
            dxBody *b = body[bi];
            // dSetZero(tmp1curr, 8); -- not needed
            for (unsigned int j = dSA__MIN; j != dSA__MAX; ++j) tmp1curr[dDA__L_MIN + j] = b->facc[dV3E__AXES_MIN + j] * b->invMass + b->lvel[dV3E__AXES_MIN + j] * stepsizeRecip;
            dMultiply0_331 (tmp1curr + dDA__A_MIN, invIrow, b->tacc);
            for (unsigned int k = dSA__MIN; k != dSA__MAX; ++k) tmp1curr[dDA__A_MIN + k] += b->avel[dV3E__AXES_MIN + k] * stepsizeRecip;
            // Initialize body start joint indices -- this will be needed later for building body related joint list in dxStepIsland_Stage2c
            bodyStartJoints[bi] = 0;
        }
    }
}

static 
int dxStepIsland_Stage2bSync_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;

    dIASSERT(allowedThreads > 1); /*if (allowedThreads > 1) */{ // The allowed thread count is greater than one as otherwise current function would not be scheduled for execution from the previous stage
        dxWorld *world = callContext->m_world;
        world->AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads - 1);
        world->PostThreadedCallsGroup(NULL, allowedThreads - 1, callThisReleasee, &dxStepIsland_Stage2c_Callback, stage2CallContext, "StepIsland Stage2c");
    }
    dxStepIsland_Stage2c(stage2CallContext);

    return 1;
}


static 
int dxStepIsland_Stage2c_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2c(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2c(dxStepperStage2CallContext *stage2CallContext)
{
    //const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    {
        // Warning!!!
        // This code depends on A elements and JinvM elements and therefore 
        // must be in different sub-stage from A initialization and JinvM calculation in Stage2b 
        // to ensure proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        dReal *A = localContext->m_A;
        const dReal *JinvM = stage2CallContext->m_JinvM;
        const dReal *J = localContext->m_J;
        const unsigned m = localContext->m_m;

        // now compute A = JinvM * J'. A's rows and columns are grouped by joint,
        // i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
        // if joints i and j have at least one body in common. 
        const unsigned int mskip = dPAD(m);

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_Aaddjb, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Arow = A + (size_t)mskip * ofsi;
            const dReal *JinvMRow = JinvM + (size_t)ofsi * (2 * JIM__MAX);
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                // compute diagonal block of A
                const dReal *JRow = J + (size_t)ofsi * (2 * JME__MAX);
                MultiplyAddJinvMxJToA (Arow + ofsi, JinvMRow, JRow, infom, infom, mskip);

                for (dxJointNode *n0 = (ji != 0 ? jb0->firstjoint : NULL); n0; n0 = n0->next) {
                    // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                    // joint that should not be considered
                    int j0 = n0->joint->tag;
                    if (j0 != -1 && (unsigned)j0 < ji) {
                        const unsigned int jiother_ofsi = mindex[j0];
                        const unsigned int jiother_infom = mindex[j0 + 1] - jiother_ofsi;
                        const dJointWithInfo1 *jiother = jointinfos + j0;
                        unsigned int smart_infom = (jiother->joint->node[1].body == jb0) ? jiother_infom : 0;
                        // set block of A
                        const dReal *JOther = J + ((size_t)jiother_ofsi * 2 + smart_infom) * JME__MAX;
                        MultiplyAddJinvMxJToA (Arow + jiother_ofsi, JinvMRow, JOther, infom, jiother_infom, mskip);
                    }
                }
            }

            dxBody *jb1 = joint->node[1].body;
            dIASSERT(jb1 != jb0);
            if (jb1 != NULL) {
                const dReal *JinvMOther = JinvMRow + infom * JIM__MAX;
                // compute diagonal block of A
                const dReal *JRow = J + ((size_t)ofsi * 2 + infom) * JME__MAX;
                MultiplyAddJinvMxJToA (Arow + ofsi, JinvMOther, JRow, infom, infom, mskip);

                for (dxJointNode *n1 = (ji != 0 ? jb1->firstjoint : NULL); n1; n1 = n1->next) {
                    // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                    // joint that should not be considered
                    int j1 = n1->joint->tag;
                    if (j1 != -1 && (unsigned)j1 < ji) {
                        const unsigned int jiother_ofsi = mindex[j1];
                        const unsigned int jiother_infom = mindex[j1 + 1] - jiother_ofsi;
                        const dJointWithInfo1 *jiother = jointinfos + j1;
                        unsigned int smart_infom = (jiother->joint->node[1].body == jb1) ? jiother_infom : 0;
                        // set block of A
                        const dReal *JOther = J + ((size_t)jiother_ofsi * 2 + smart_infom) * JME__MAX;
                        MultiplyAddJinvMxJToA (Arow + jiother_ofsi, JinvMOther, JOther, infom, jiother_infom, mskip);
                    }
                }
            }
        }
    }

    {
        // Warning!!!
        // This code depends on rhs_tmp elements and therefore must be in 
        // different sub-stage from rhs_tmp calculation in Stage2b to ensure 
        // proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        const dReal *J = localContext->m_J;
        const dReal *rhs_tmp = stage2CallContext->m_rhs_tmp;
        dReal *pairsRhsCfm = localContext->m_pairsRhsCfm;
        atomicord32 *bodyStartJoints = localContext->m_bodyStartJoints;
        atomicord32 *bodyJointLinks = localContext->m_bodyJointLinks;

        // compute the right hand side `rhs'
        // put J*rhs_tmp into rhs
        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_rhs, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *currRhsCfm = pairsRhsCfm + (size_t)ofsi * RCE__RHS_CFM_MAX;
            const dReal *JRow = J + (size_t)ofsi * (2 * JME__MAX);
            
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                unsigned bodyIndex = (unsigned)jb0->tag;
                MultiplySubJxRhsTmpFromRHS (currRhsCfm, JRow, rhs_tmp + (size_t)bodyIndex * dDA__MAX, infom);

                // Link joints connected to each body into a list to be used on results incorporation. The bodyStartJoints have been initialized in dxStepIsland_Stage2b.
                const atomicord32 linkIndex = (atomicord32)((size_t)ji * dJCB__MAX + dJCB_FIRST_BODY); // It is asserted at links buffer allocation that the indices can't overflow atomicord32
                for (atomicord32 oldStartIndex = bodyStartJoints[bodyIndex]; ; oldStartIndex = bodyStartJoints[bodyIndex]) {
                    bodyJointLinks[linkIndex] = oldStartIndex;
                    if (ThrsafeCompareExchange(&bodyStartJoints[bodyIndex], oldStartIndex, linkIndex + 1)) { // The link index is stored incremented to allow 0 as end indicator
                        break;
                    }
                }
            }

            dxBody *jb1 = joint->node[1].body;
            if (jb1 != NULL) {
                unsigned bodyIndex = (unsigned)jb1->tag;
                MultiplySubJxRhsTmpFromRHS (currRhsCfm, JRow + infom * JME__MAX, rhs_tmp + (size_t)bodyIndex * dDA__MAX, infom);

                // Link joints connected to each body into a list to be used on results incorporation. The bodyStartJoints have been initialized in dxStepIsland_Stage2b
                const atomicord32 linkIndex = (atomicord32)((size_t)ji * dJCB__MAX + dJCB_SECOND_BODY); // It is asserted at links buffer allocation that the indices can't overflow atomicord32
                for (atomicord32 oldStartIndex = bodyStartJoints[bodyIndex]; ; oldStartIndex = bodyStartJoints[bodyIndex]) {
                    bodyJointLinks[linkIndex] = oldStartIndex;
                    if (ThrsafeCompareExchange(&bodyStartJoints[bodyIndex], oldStartIndex, linkIndex + 1)) { // The link index is stored incremented to allow 0 as end indicator
                        break;
                    }
                }
            }
        }
    }
}


static 
int dxStepIsland_Stage3_Callback(void *_stage3CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage3CallContext *stage3CallContext = (dxStepperStage3CallContext *)_stage3CallContext;
    dxStepIsland_Stage3(stage3CallContext);
    return 1;
}

static 
void dxStepIsland_Stage3(dxStepperStage3CallContext *stage3CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage3CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage3CallContext->m_localContext;

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    memarena->RestoreState(stage3CallContext->m_stage1MemArenaState);
    stage3CallContext = NULL; // WARNING! stage3CallContext is not valid after this point!
    dIVERIFY(stage3CallContext == NULL); // To suppress unused variable assignment warnings

    unsigned int m = localContext->m_m;
    unsigned int nub = localContext->m_nub;
    //const unsigned int *mindex = localContext->m_mindex;
    int *findex = localContext->m_findex;
    dReal *A = localContext->m_A;
    dReal *pairsRhsLambda = localContext->m_pairsRhsCfm; // Reuse cfm buffer for lambdas as the former values are not needed any more
    dReal *pairsLoHi = localContext->m_pairsLoHi;

    if (m > 0) {
        BEGIN_STATE_SAVE(memarena, lcpstate) {
            IFTIMING(dTimerNow ("solve LCP problem"));

            // solve the LCP problem and get lambda.
            // this will destroy A but that's OK
            dxSolveLCP (memarena, m, A, pairsRhsLambda, NULL, nub, pairsLoHi, findex);
            dSASSERT((int)RLE__RHS_LAMBDA_MAX == PBX__MAX && (int)RLE_RHS == PBX_B && (int)RLE_LAMBDA == PBX_X);
            dSASSERT((int)LHE__LO_HI_MAX == PLH__MAX && (int)LHE_LO == PLH_LO && (int)LHE_HI == PLH_HI);

        } END_STATE_SAVE(memarena, lcpstate);
    }

    // void *stage3MemarenaState = memarena->SaveState();

    dxStepperStage4CallContext *stage4CallContext = (dxStepperStage4CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage4CallContext));
    stage4CallContext->Initialize(callContext, localContext/*, stage3MemarenaState*/);

    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
    dIASSERT(allowedThreads != 0);

    if (allowedThreads == 1) {
        IFTIMING(dTimerNow ("compute and apply constraint force"));
        dxStepIsland_Stage4(stage4CallContext);
        IFTIMING(dTimerEnd());

        if (m > 0) {
            IFTIMING(dTimerReport(stdout,1));
        }
    }
    else {
        dCallReleaseeID finalReleasee = callContext->m_finalReleasee;
        dxWorld *world = callContext->m_world;
        world->AlterThreadedCallDependenciesCount(finalReleasee, allowedThreads - 1);
        world->PostThreadedCallsGroup(NULL, allowedThreads - 1, finalReleasee, &dxStepIsland_Stage4_Callback, stage4CallContext, "StepIsland Stage4");
        // Note: Adding another dependency for the finalReleasee is not necessary as it already depends on the current call
        dxStepIsland_Stage4(stage4CallContext);
    }
}

static 
int dxStepIsland_Stage4_Callback(void *_stage4CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage4CallContext *stage4CallContext = (dxStepperStage4CallContext *)_stage4CallContext;
    dxStepIsland_Stage4(stage4CallContext);
    return 1;
}

static 
void dxStepIsland_Stage4(dxStepperStage4CallContext *stage4CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage4CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage4CallContext->m_localContext;

    const dReal stepSize = callContext->m_stepSize;
    dxBody *const *bodies = callContext->m_islandBodiesStart;
    dReal *invI = localContext->m_invI;
    dJointWithInfo1 *jointInfos = localContext->m_jointinfos;
    dReal *J = localContext->m_J;
    dReal *pairsRhsLambda = localContext->m_pairsRhsCfm;
    const unsigned int *mIndex = localContext->m_mindex;
    atomicord32 *bodyStartJoints = localContext->m_bodyStartJoints;
    atomicord32 *bodyJointLinks = localContext->m_bodyJointLinks;
    const unsigned int nb = callContext->m_islandBodiesCount;

    unsigned bi;
    while ((bi = ThrsafeIncrementIntUpToLimit(&stage4CallContext->m_bi_constrForce, nb)) != nb) {
        dVector3 angularForceAccumulator;
        dxBody *b = bodies[bi];
        const dReal *invIrow = invI + (size_t)bi * dM3E__MAX;
        dReal body_invMass_mul_stepSize = stepSize * b->invMass;

        dReal bodyConstrForce[CFE__MAX];
        bool constrForceAvailable = false;
        
        unsigned linkIndex = bodyStartJoints != NULL ? bodyStartJoints[bi] : 0;
        if (linkIndex != 0) {
            dSetZero(bodyConstrForce, dARRAY_SIZE(bodyConstrForce));
        }

        // compute the constraint force as constrForce = J'*lambda
        for (; linkIndex != 0; constrForceAvailable = true, linkIndex = bodyJointLinks[linkIndex - 1]) {
            unsigned jointIndex = (linkIndex - 1) / dJCB__MAX;
            unsigned jointBodyIndex = (linkIndex - 1) % dJCB__MAX;

            const dJointWithInfo1 *currJointInfo = jointInfos + jointIndex;
            unsigned ofsi = mIndex[jointIndex];
            dIASSERT(dIN_RANGE(jointIndex, 0, localContext->m_nj));

            const dReal *JRow = J + (size_t)ofsi * (2 * JME__MAX);
            const dReal *rowRhsLambda = pairsRhsLambda + (size_t)ofsi * RLE__RHS_LAMBDA_MAX;

            dxJoint *joint = currJointInfo->joint;
            const unsigned int infom = currJointInfo->info.m;

            // unsigned jRowExtraOffset = jointBodyIndex * infom * JME__MAX;
            unsigned jRowExtraOffset = jointBodyIndex != dJCB__MIN ? infom * JME__MAX : 0;
            dSASSERT(dJCB__MAX == 2);

            dJointFeedback *fb = joint->feedback;
            MultiplyAddJxLambdaToCForce(bodyConstrForce, JRow + jRowExtraOffset, rowRhsLambda, infom, fb, jointBodyIndex);
        }

        // compute the velocity update
        if (constrForceAvailable) {
            // add fe to cforce and multiply cforce by stepSize
            for (unsigned int j = dSA__MIN; j != dSA__MAX; ++j) {
                b->lvel[dV3E__AXES_MIN + j] += (bodyConstrForce[CFE__L_MIN + j] + b->facc[dV3E__AXES_MIN + j]) * body_invMass_mul_stepSize;
            }
            for (unsigned int k = dSA__MIN; k != dSA__MAX; ++k) {
                angularForceAccumulator[dV3E__AXES_MIN + k] = (bodyConstrForce[CFE__A_MIN + k] + b->tacc[dV3E__AXES_MIN + k]) * stepSize;
            }
        }
        else {
            // add fe to cforce and multiply cforce by stepSize
            dAddVectorScaledVector3(b->lvel, b->lvel, b->facc, body_invMass_mul_stepSize);
            dCopyScaledVector3(angularForceAccumulator, b->tacc, stepSize);
        }

        dMultiplyAdd0_331 (b->avel, invIrow, angularForceAccumulator + dV3E__AXES_MIN);

        // update the position and orientation from the new linear/angular velocity
        // (over the given time step)
        dxStepBody (b, stepSize);

        // zero all force accumulators
        dZeroVector3(b->facc);
        dZeroVector3(b->tacc);
    }
}


//****************************************************************************

/*extern */
size_t dxEstimateStepMemoryRequirements (dxBody * const *body, unsigned int nb, dxJoint * const *_joint, unsigned int _nj)
{
    (void)body; // unused
    unsigned int nj, m;

    {
        unsigned int njcurr = 0, mcurr = 0;
        dxJoint::SureMaxInfo info;
        dxJoint *const *const _jend = _joint + _nj;
        for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; ++_jcurr) {	
            dxJoint *j = *_jcurr;
            j->getSureMaxInfo (&info);

            unsigned int jm = info.max_m;
            if (jm > 0) {
                njcurr++;

                mcurr += jm;
            }
        }
        nj = njcurr; m = mcurr;
    }

    size_t res = 0;

    res += dOVERALIGNED_SIZE(sizeof(dReal) * dM3E__MAX * nb, INVI_ALIGNMENT); // for invI

    {
        size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * _nj); // for initial jointinfos

        // The array can't grow right more than by nj
        size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * ((size_t)_nj + (size_t)nj)); // for shrunk jointinfos
        sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperLocalContext)); //for dxStepperLocalContext
        if (m > 0) {
            sub1_res2 += dEFFICIENT_SIZE(sizeof(unsigned int) * (nj + 1)); // for mindex
            sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * JME__MAX * m); // for J
            unsigned int mskip = dPAD(m);
            sub1_res2 += dOVERALIGNED_SIZE(sizeof(dReal) * mskip * m, AMATRIX_ALIGNMENT); // for A
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * RCE__RHS_CFM_MAX * m); // for pairsRhsCfm
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * LHE__LO_HI_MAX * m); // for pairsLoHi
            sub1_res2 += dEFFICIENT_SIZE(sizeof(atomicord32) * nb); // for bodyStartJoints
            sub1_res2 += dEFFICIENT_SIZE(sizeof(atomicord32)* dJCB__MAX * nj); // for bodyJointLinks
        }

        {
            size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); // for dxStepperStage3CallContext

            size_t sub2_res2 = 0;

            size_t sub2_res3 = dEFFICIENT_SIZE(sizeof(dxStepperStage4CallContext)); // for dxStepperStage4CallContext

            if (m > 0) {
                sub2_res1 += dOVERALIGNED_SIZE(sizeof(dReal) * 2 * JIM__MAX * m, JINVM_ALIGNMENT); // for JinvM
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * dDA__MAX * nb); // for rhs_tmp
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dxStepperStage2CallContext)); // for dxStepperStage2CallContext

                sub2_res2 += dxEstimateSolveLCPMemoryReq(m, false);
            }

            sub1_res2 += dMAX(sub2_res1, dMAX(sub2_res2, sub2_res3));
        }

        size_t sub1_res12_max = dMAX(sub1_res1, sub1_res2);
        size_t stage01_contexts = dEFFICIENT_SIZE(sizeof(dxStepperStage0BodiesCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage0JointsCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage1CallContext));
        res += dMAX(sub1_res12_max, stage01_contexts);
    }

    return res;
}


/*extern */
unsigned dxEstimateStepMaxCallCount(
    unsigned /*activeThreadCount*/, unsigned allowedThreadCount)
{
    unsigned result = 1 // dxStepIsland itself
        + (2 * allowedThreadCount + 2) // (dxStepIsland_Stage2a + dxStepIsland_Stage2b) * allowedThreadCount + 2 * dxStepIsland_Stage2?_Sync
        + 1; // dxStepIsland_Stage3
    return result;
}
=======
/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include "objects.h"
#include "joints/joint.h"
#include <ode/odeconfig.h>
#include "config.h"
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include "lcp.h"
#include "util.h"

//****************************************************************************
// misc defines

#define FAST_FACTOR
//#define TIMING

// memory allocation system
#ifdef dUSE_MALLOC_FOR_ALLOCA
unsigned int dMemoryFlag;
#define REPORT_OUT_OF_MEMORY fprintf(stderr, "Insufficient memory to complete rigid body simulation.  Results will not be accurate.\n")

#define CHECK(p)                                \
  if (!p) {                                     \
    dMemoryFlag = d_MEMORY_OUT_OF_MEMORY;       \
    return;                                     \
  }

#define ALLOCA(t,v,s)                           \
  Auto<t> v(malloc(s));                         \
  CHECK(v)

#else // use alloca()

#define ALLOCA(t,v,s)                           \
  Auto<t> v( dALLOCA16(s) );

#endif



/* This template should work almost like std::auto_ptr
 */
template<class T>
struct Auto {
  T *p;
  Auto(void * q) :
    p(reinterpret_cast<T*>(q))
  { }

  ~Auto()
  {
#ifdef dUSE_MALLOC_FOR_ALLOCA
    free(p);
#endif
  }

  operator T*() 
  {
    return p;
  }
  T& operator[] (int i)
  {
    return p[i];
  }
private:
  // intentionally undefined, don't use this
  template<class U>
  Auto& operator=(const Auto<U>&) const;
};





//****************************************************************************
// debugging - comparison of various vectors and matrices produced by the
// slow and fast versions of the stepper.

//#define COMPARE_METHODS

#ifdef COMPARE_METHODS
#include "testing.h"
dMatrixComparison comparator;
#endif

// undef to use the fast decomposition
#define DIRECT_CHOLESKY
#undef REPORT_ERROR

//****************************************************************************
// special matrix multipliers

// this assumes the 4th and 8th rows of B and C are zero.

static void Multiply2_p8r (dReal *A, dReal *B, dReal *C,
			   int p, int r, int Askip)
{
  int i,j;
  dReal sum,*bb,*cc;
  dIASSERT (p>0 && r>0 && A && B && C);
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      sum = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(A++) = sum; 
      cc += 8;
    }
    A += Askip - r;
    bb += 8;
  }
}


// this assumes the 4th and 8th rows of B and C are zero.

static void MultiplyAdd2_p8r (dReal *A, dReal *B, dReal *C,
			      int p, int r, int Askip)
{
  int i,j;
  dReal sum,*bb,*cc;
  dIASSERT (p>0 && r>0 && A && B && C);
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      sum = bb[0]*cc[0];
      sum += bb[1]*cc[1];
      sum += bb[2]*cc[2];
      sum += bb[4]*cc[4];
      sum += bb[5]*cc[5];
      sum += bb[6]*cc[6];
      *(A++) += sum; 
      cc += 8;
    }
    A += Askip - r;
    bb += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void Multiply0_p81 (dReal *A, dReal *B, dReal *C, int p)
{
  int i;
  dIASSERT (p>0 && A && B && C);
  dReal sum;
  for (i=p; i; i--) {
    sum =  B[0]*C[0];
    sum += B[1]*C[1];
    sum += B[2]*C[2];
    sum += B[4]*C[4];
    sum += B[5]*C[5];
    sum += B[6]*C[6];
    *(A++) = sum;
    B += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd0_p81 (dReal *A, dReal *B, dReal *C, int p)
{
  int i;
  dIASSERT (p>0 && A && B && C);
  dReal sum;
  for (i=p; i; i--) {
    sum =  B[0]*C[0];
    sum += B[1]*C[1];
    sum += B[2]*C[2];
    sum += B[4]*C[4];
    sum += B[5]*C[5];
    sum += B[6]*C[6];
    *(A++) += sum;
    B += 8;
  }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd1_8q1 (dReal *A, dReal *B, dReal *C, int q)
{
  int k;
  dReal sum;
  dIASSERT (q>0 && A && B && C);
  sum = 0;
  for (k=0; k<q; k++) sum += B[k*8] * C[k];
  A[0] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[1+k*8] * C[k];
  A[1] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[2+k*8] * C[k];
  A[2] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[4+k*8] * C[k];
  A[4] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[5+k*8] * C[k];
  A[5] += sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[6+k*8] * C[k];
  A[6] += sum;
}


// this assumes the 4th and 8th rows of B are zero.

static void Multiply1_8q1 (dReal *A, dReal *B, dReal *C, int q)
{
  int k;
  dReal sum;
  dIASSERT (q>0 && A && B && C);
  sum = 0;
  for (k=0; k<q; k++) sum += B[k*8] * C[k];
  A[0] = sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[1+k*8] * C[k];
  A[1] = sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[2+k*8] * C[k];
  A[2] = sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[4+k*8] * C[k];
  A[4] = sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[5+k*8] * C[k];
  A[5] = sum;
  sum = 0;
  for (k=0; k<q; k++) sum += B[6+k*8] * C[k];
  A[6] = sum;
}

//****************************************************************************
// the slow, but sure way
// note that this does not do any joint feedback!

// given lists of bodies and joints that form an island, perform a first
// order timestep.
//
// `body' is the body array, `nb' is the size of the array.
// `_joint' is the body array, `nj' is the size of the array.

void dInternalStepIsland_x1 (dxWorld *world, dxBody * const *body, int nb,
			     dxJoint * const *_joint, int nj, dReal stepsize)
{
  int i,j,k;
  int n6 = 6*nb;

#ifdef TIMING
  dTimerStart("preprocessing");
#endif

  // number all bodies in the body list - set their tag values
  for (i=0; i<nb; i++) body[i]->tag = i;

  // make a local copy of the joint array, because we might want to modify it.
  // (the "dxJoint *const*" declaration says we're allowed to modify the joints
  // but not the joint array, because the caller might need it unchanged).
  ALLOCA(dxJoint*,joint,nj*sizeof(dxJoint*));
  memcpy (joint,_joint,nj * sizeof(dxJoint*));

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator.
  // @@@ check computation of rotational force.
  ALLOCA(dReal,I,3*nb*4*sizeof(dReal));
  ALLOCA(dReal,invI,3*nb*4*sizeof(dReal));

  //dSetZero (I,3*nb*4);
  //dSetZero (invI,3*nb*4);
  for (i=0; i<nb; i++) {
    dReal tmp[12];
    // compute inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->mass.I,body[i]->posr.R);
    dMULTIPLY0_333 (I+i*12,body[i]->posr.R,tmp);
    // compute inverse inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->invI,body[i]->posr.R);
    dMULTIPLY0_333 (invI+i*12,body[i]->posr.R,tmp);
    // compute rotational force
    dMULTIPLY0_331 (tmp,I+i*12,body[i]->avel);
    dCROSS (body[i]->tacc,-=,body[i]->avel,tmp);
  }

  // add the gravity force to all bodies
  for (i=0; i<nb; i++) {
    if ((body[i]->flags & dxBodyNoGravity)==0) {
      body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
      body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
      body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
    }
  }

  // get m = total constraint dimension, nub = number of unbounded variables.
  // create constraint offset array and number-of-rows array for all joints.
  // the constraints are re-ordered as follows: the purely unbounded
  // constraints, the mixed unbounded + LCP constraints, and last the purely
  // LCP constraints.
  //
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  int m = 0;
  ALLOCA(dxJoint::Info1,info,nj*sizeof(dxJoint::Info1));
  ALLOCA(int,ofs,nj*sizeof(int));

  for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
    joint[j]->getInfo1 (info+i);
    dIASSERT (info[i].m >= 0 && info[i].m <= 6 &&
	      info[i].nub >= 0 && info[i].nub <= info[i].m);
    if (info[i].m > 0) {
      joint[i] = joint[j];
      i++;
    }
  }
  nj = i;

  // the purely unbounded constraints
  for (i=0; i<nj; i++) if (info[i].nub == info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  //int nub = m;
  // the mixed unbounded + LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub > 0 && info[i].nub < info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  // the purely LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub == 0) {
    ofs[i] = m;
    m += info[i].m;
  }

  // create (6*nb,6*nb) inverse mass matrix `invM', and fill it with mass
  // parameters
#ifdef TIMING
  dTimerNow ("create mass matrix");
#endif
  int nskip = dPAD (n6);
  ALLOCA(dReal, invM, n6*nskip*sizeof(dReal));

  dSetZero (invM,n6*nskip);
  for (i=0; i<nb; i++) {
    dReal *MM = invM+(i*6)*nskip+(i*6);
    MM[0] = body[i]->invMass;
    MM[nskip+1] = body[i]->invMass;
    MM[2*nskip+2] = body[i]->invMass;
    MM += 3*nskip+3;
    for (j=0; j<3; j++) for (k=0; k<3; k++) {
      MM[j*nskip+k] = invI[i*12+j*4+k];
    }
  }

  // assemble some body vectors: fe = external forces, v = velocities
  ALLOCA(dReal,fe,n6*sizeof(dReal));
  ALLOCA(dReal,v,n6*sizeof(dReal));

  //dSetZero (fe,n6);
  //dSetZero (v,n6);
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) fe[i*6+j] = body[i]->facc[j];
    for (j=0; j<3; j++) fe[i*6+3+j] = body[i]->tacc[j];
    for (j=0; j<3; j++) v[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; j++) v[i*6+3+j] = body[i]->avel[j];
  }

  // this will be set to the velocity update
  ALLOCA(dReal,vnew,n6*sizeof(dReal));
  dSetZero (vnew,n6);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c', a constraint
    // force mixing vector `cfm', and LCP low and high bound vectors, and an
    // 'findex' vector.
    ALLOCA(dReal,c,m*sizeof(dReal));
    ALLOCA(dReal,cfm,m*sizeof(dReal));
    ALLOCA(dReal,lo,m*sizeof(dReal));
    ALLOCA(dReal,hi,m*sizeof(dReal));
    ALLOCA(int,findex,m*sizeof(int));
    dSetZero (c,m);
    dSetValue (cfm,m,world->global_cfm);
    dSetValue (lo,m,-dInfinity);
    dSetValue (hi,m, dInfinity);
    for (i=0; i<m; i++) findex[i] = -1;

    // create (m,6*nb) jacobian mass matrix `J', and fill it with constraint
    // data. also fill the c vector.
#   ifdef TIMING
    dTimerNow ("create J");
#   endif
    ALLOCA(dReal,J,m*nskip*sizeof(dReal));
    dSetZero (J,m*nskip);
    dxJoint::Info2 Jinfo;
    Jinfo.rowskip = nskip;
    Jinfo.fps = dRecip(stepsize);
    Jinfo.erp = world->global_erp;
    for (i=0; i<nj; i++) {
      Jinfo.J1l = J + nskip*ofs[i] + 6*joint[i]->node[0].body->tag;
      Jinfo.J1a = Jinfo.J1l + 3;
      if (joint[i]->node[1].body) {
	Jinfo.J2l = J + nskip*ofs[i] + 6*joint[i]->node[1].body->tag;
	Jinfo.J2a = Jinfo.J2l + 3;
      }
      else {
	Jinfo.J2l = 0;
	Jinfo.J2a = 0;
      }
      Jinfo.c = c + ofs[i];
      Jinfo.cfm = cfm + ofs[i];
      Jinfo.lo = lo + ofs[i];
      Jinfo.hi = hi + ofs[i];
      Jinfo.findex = findex + ofs[i];
      joint[i]->getInfo2 (&Jinfo);
      // adjust returned findex values for global index numbering
      for (j=0; j<info[i].m; j++) {
	if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
      }
    }

    // compute A = J*invM*J'
#   ifdef TIMING
    dTimerNow ("compute A");
#   endif
    ALLOCA(dReal,JinvM,m*nskip*sizeof(dReal));
    //dSetZero (JinvM,m*nskip);
    dMultiply0 (JinvM,J,invM,m,n6,n6);
    int mskip = dPAD(m);
    ALLOCA(dReal,A,m*mskip*sizeof(dReal));
    //dSetZero (A,m*mskip);
    dMultiply2 (A,JinvM,J,m,n6,m);

    // add cfm to the diagonal of A
    for (i=0; i<m; i++) A[i*mskip+i] += cfm[i] * Jinfo.fps;

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (A,m,m,1,"A");
#   endif

    // compute `rhs', the right hand side of the equation J*a=c
#   ifdef TIMING
    dTimerNow ("compute rhs");
#   endif
    ALLOCA(dReal,tmp1,n6*sizeof(dReal));
    //dSetZero (tmp1,n6);
    dMultiply0 (tmp1,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) tmp1[i] += v[i]/stepsize;
    ALLOCA(dReal,rhs,m*sizeof(dReal));
    //dSetZero (rhs,m);
    dMultiply0 (rhs,J,tmp1,m,n6,1);
    for (i=0; i<m; i++) rhs[i] = c[i]/stepsize - rhs[i];

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (c,m,1,0,"c");
    comparator.nextMatrix (rhs,m,1,0,"rhs");
#   endif



 

#ifndef DIRECT_CHOLESKY
    // solve the LCP problem and get lambda.
    // this will destroy A but that's okay
#   ifdef TIMING
    dTimerNow ("solving LCP problem");
#   endif
    ALLOCA(dReal,lambda,m*sizeof(dReal));
    ALLOCA(dReal,residual,m*sizeof(dReal));
    dSolveLCP (m,A,lambda,rhs,residual,nub,lo,hi,findex);

#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY)
      return;
#endif


#else

    // OLD WAY - direct factor and solve

    // factorize A (L*L'=A)
#   ifdef TIMING
    dTimerNow ("factorize A");
#   endif
    ALLOCA(dReal,L,m*mskip*sizeof(dReal));
    memcpy (L,A,m*mskip*sizeof(dReal));
    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");

    // compute lambda
#   ifdef TIMING
    dTimerNow ("compute lambda");
#   endif
    ALLOCA(dReal,lambda,m*sizeof(dReal));
    memcpy (lambda,rhs,m * sizeof(dReal));
    dSolveCholesky (L,lambda,m);
#endif

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (lambda,m,1,0,"lambda");
#   endif

    // compute the velocity update `vnew'
#   ifdef TIMING
    dTimerNow ("compute velocity update");
#   endif
    dMultiply1 (tmp1,J,lambda,n6,m,1);
    for (i=0; i<n6; i++) tmp1[i] += fe[i];
    dMultiply0 (vnew,invM,tmp1,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];

#ifdef REPORT_ERROR
    // see if the constraint has worked: compute J*vnew and make sure it equals
    // `c' (to within a certain tolerance).
#   ifdef TIMING
    dTimerNow ("verify constraint equation");
#   endif
    dMultiply0 (tmp1,J,vnew,m,n6,1);
    dReal err = 0;
    for (i=0; i<m; i++) {
		err += dFabs(tmp1[i]-c[i]);
    }
	printf ("total constraint error=%.6e\n",err);
#endif

  }
  else {
    // no constraints
    dMultiply0 (vnew,invM,fe,n6,n6,1);
    for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];
  }

#ifdef COMPARE_METHODS
  comparator.nextMatrix (vnew,n6,1,0,"vnew");
#endif

  // apply the velocity update to the bodies
#ifdef TIMING
  dTimerNow ("update velocity");
#endif
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) body[i]->lvel[j] = vnew[i*6+j];
    for (j=0; j<3; j++) body[i]->avel[j] = vnew[i*6+3+j];
  }

  // update the position and orientation from the new linear/angular velocity
  // (over the given timestep)
#ifdef TIMING
  dTimerNow ("update position");
#endif
  for (i=0; i<nb; i++) dxStepBody (body[i],stepsize);

#ifdef TIMING
  dTimerNow ("tidy up");
#endif

  // zero all force accumulators
  for (i=0; i<nb; i++) {
    body[i]->facc[0] = 0;
    body[i]->facc[1] = 0;
    body[i]->facc[2] = 0;
    body[i]->facc[3] = 0;
    body[i]->tacc[0] = 0;
    body[i]->tacc[1] = 0;
    body[i]->tacc[2] = 0;
    body[i]->tacc[3] = 0;
  }

#ifdef TIMING
  dTimerEnd();
  if (m > 0) dTimerReport (stdout,1);
#endif

}

//****************************************************************************
// an optimized version of dInternalStepIsland1()

void dInternalStepIsland_x2 (dxWorld *world, dxBody * const *body, int nb,
			     dxJoint * const *_joint, int nj, dReal stepsize)
{
  int i,j,k;
#ifdef TIMING
  dTimerStart("preprocessing");
#endif

  dReal stepsize1 = dRecip(stepsize);

  // number all bodies in the body list - set their tag values
  for (i=0; i<nb; i++) body[i]->tag = i;

  // make a local copy of the joint array, because we might want to modify it.
  // (the "dxJoint *const*" declaration says we're allowed to modify the joints
  // but not the joint array, because the caller might need it unchanged).
  ALLOCA(dxJoint*,joint,nj*sizeof(dxJoint*));
  memcpy (joint,_joint,nj * sizeof(dxJoint*));

  // for all bodies, compute the inertia tensor and its inverse in the global
  // frame, and compute the rotational force and add it to the torque
  // accumulator. invI are vertically stacked 3x4 matrices, one per body.
  // @@@ check computation of rotational force.

  ALLOCA(dReal,invI,3*nb*4*sizeof(dReal));

  //dSetZero (I,3*nb*4);
  //dSetZero (invI,3*nb*4);
  for (i=0; i<nb; i++) {
    dReal tmp[12];

    // compute inverse inertia tensor in global frame
    dMULTIPLY2_333 (tmp,body[i]->invI,body[i]->posr.R);
    dMULTIPLY0_333 (invI+i*12,body[i]->posr.R,tmp);

    if (body[i]->flags & dxBodyGyroscopic) {
        dMatrix3 I;

        // compute inertia tensor in global frame
        dMULTIPLY2_333 (tmp,body[i]->mass.I,body[i]->posr.R);
        dMULTIPLY0_333 (I,body[i]->posr.R,tmp);

        // compute rotational force
        dMULTIPLY0_331 (tmp,I,body[i]->avel);
        dCROSS (body[i]->tacc,-=,body[i]->avel,tmp);
    }
  }

  // add the gravity force to all bodies
  for (i=0; i<nb; i++) {
    if ((body[i]->flags & dxBodyNoGravity)==0) {
      body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
      body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
      body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
    }
  }

  // get m = total constraint dimension, nub = number of unbounded variables.
  // create constraint offset array and number-of-rows array for all joints.
  // the constraints are re-ordered as follows: the purely unbounded
  // constraints, the mixed unbounded + LCP constraints, and last the purely
  // LCP constraints. this assists the LCP solver to put all unbounded
  // variables at the start for a quick factorization.
  //
  // joints with m=0 are inactive and are removed from the joints array
  // entirely, so that the code that follows does not consider them.
  // also number all active joints in the joint list (set their tag values).
  // inactive joints receive a tag value of -1.

  int m = 0;
  ALLOCA(dxJoint::Info1,info,nj*sizeof(dxJoint::Info1));
  ALLOCA(int,ofs,nj*sizeof(int));
  for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
    joint[j]->getInfo1 (info+i);
    dIASSERT (info[i].m >= 0 && info[i].m <= 6 &&
	      info[i].nub >= 0 && info[i].nub <= info[i].m);
    if (info[i].m > 0) {
      joint[i] = joint[j];
      joint[i]->tag = i;
      i++;
    }
    else {
      joint[j]->tag = -1;
    }
  }
  nj = i;

  // the purely unbounded constraints
  for (i=0; i<nj; i++) if (info[i].nub == info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  int nub = m;
  // the mixed unbounded + LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub > 0 && info[i].nub < info[i].m) {
    ofs[i] = m;
    m += info[i].m;
  }
  // the purely LCP constraints
  for (i=0; i<nj; i++) if (info[i].nub == 0) {
    ofs[i] = m;
    m += info[i].m;
  }

  // this will be set to the force due to the constraints
  ALLOCA(dReal,cforce,nb*8*sizeof(dReal));
  dSetZero (cforce,nb*8);

  // if there are constraints, compute cforce
  if (m > 0) {
    // create a constraint equation right hand side vector `c', a constraint
    // force mixing vector `cfm', and LCP low and high bound vectors, and an
    // 'findex' vector.
    ALLOCA(dReal,c,m*sizeof(dReal));
    ALLOCA(dReal,cfm,m*sizeof(dReal));
    ALLOCA(dReal,lo,m*sizeof(dReal));
    ALLOCA(dReal,hi,m*sizeof(dReal));
    ALLOCA(int,findex,m*sizeof(int));
    dSetZero (c,m);
    dSetValue (cfm,m,world->global_cfm);
    dSetValue (lo,m,-dInfinity);
    dSetValue (hi,m, dInfinity);
    for (i=0; i<m; i++) findex[i] = -1;

    // get jacobian data from constraints. a (2*m)x8 matrix will be created
    // to store the two jacobian blocks from each constraint. it has this
    // format:
    //
    //   l l l 0 a a a 0  \    .
    //   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
    //   l l l 0 a a a 0  /
    //   l l l 0 a a a 0  \    .
    //   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
    //   l l l 0 a a a 0  /
    //   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
    //   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
    //   etc...
    //
    //   (lll) = linear jacobian data
    //   (aaa) = angular jacobian data
    //
#   ifdef TIMING
    dTimerNow ("create J");
#   endif
    ALLOCA(dReal,J,2*m*8*sizeof(dReal));
    dSetZero (J,2*m*8);
    dxJoint::Info2 Jinfo;
    Jinfo.rowskip = 8;
    Jinfo.fps = stepsize1;
    Jinfo.erp = world->global_erp;
    for (i=0; i<nj; i++) {
      Jinfo.J1l = J + 2*8*ofs[i];
      Jinfo.J1a = Jinfo.J1l + 4;
      Jinfo.J2l = Jinfo.J1l + 8*info[i].m;
      Jinfo.J2a = Jinfo.J2l + 4;
      Jinfo.c = c + ofs[i];
      Jinfo.cfm = cfm + ofs[i];
      Jinfo.lo = lo + ofs[i];
      Jinfo.hi = hi + ofs[i];
      Jinfo.findex = findex + ofs[i];
      joint[i]->getInfo2 (&Jinfo);
      // adjust returned findex values for global index numbering
      for (j=0; j<info[i].m; j++) {
	if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
      }
    }

    // compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
    // format as J so we just go through the constraints in J multiplying by
    // the appropriate scalars and matrices.
#   ifdef TIMING
    dTimerNow ("compute A");
#   endif
    ALLOCA(dReal,JinvM,2*m*8*sizeof(dReal));
    dSetZero (JinvM,2*m*8);
    for (i=0; i<nj; i++) {
      int b = joint[i]->node[0].body->tag;
      dReal body_invMass = body[b]->invMass;
      dReal *body_invI = invI + b*12;
      dReal *Jsrc = J + 2*8*ofs[i];
      dReal *Jdst = JinvM + 2*8*ofs[i];
      for (j=info[i].m-1; j>=0; j--) {
	for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
	dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);
	Jsrc += 8;
	Jdst += 8;
      }
      if (joint[i]->node[1].body) {
	b = joint[i]->node[1].body->tag;
	body_invMass = body[b]->invMass;
	body_invI = invI + b*12;
	for (j=info[i].m-1; j>=0; j--) {
	  for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
	  dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);
	  Jsrc += 8;
	  Jdst += 8;
	}
      }
    }

    // now compute A = JinvM * J'. A's rows and columns are grouped by joint,
    // i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
    // if joints i and j have at least one body in common. this fact suggests
    // the algorithm used to fill A:
    //
    //    for b = all bodies
    //      n = number of joints attached to body b
    //      for i = 1..n
    //        for j = i+1..n
    //          ii = actual joint number for i
    //          jj = actual joint number for j
    //          // (ii,jj) will be set to all pairs of joints around body b
    //          compute blockwise: A(ii,jj) += JinvM(ii) * J(jj)'
    //
    // this algorithm catches all pairs of joints that have at least one body
    // in common. it does not compute the diagonal blocks of A however -
    // another similar algorithm does that.

    int mskip = dPAD(m);
    ALLOCA(dReal,A,m*mskip*sizeof(dReal));
    dSetZero (A,m*mskip);
    for (i=0; i<nb; i++) {
      for (dxJointNode *n1=body[i]->firstjoint; n1; n1=n1->next) {
	for (dxJointNode *n2=n1->next; n2; n2=n2->next) {
	  // get joint numbers and ensure ofs[j1] >= ofs[j2]
	  int j1 = n1->joint->tag;
	  int j2 = n2->joint->tag;
	  if (ofs[j1] < ofs[j2]) {
	    int tmp = j1;
	    j1 = j2;
	    j2 = tmp;
	  }

	  // if either joint was tagged as -1 then it is an inactive (m=0)
	  // joint that should not be considered
	  if (j1==-1 || j2==-1) continue;

	  // determine if body i is the 1st or 2nd body of joints j1 and j2
	  int jb1 = (joint[j1]->node[1].body == body[i]);
	  int jb2 = (joint[j2]->node[1].body == body[i]);
	  // jb1/jb2 must be 0 for joints with only one body
	  dIASSERT(joint[j1]->node[1].body || jb1==0);
	  dIASSERT(joint[j2]->node[1].body || jb2==0);

	  // set block of A
	  MultiplyAdd2_p8r (A + ofs[j1]*mskip + ofs[j2],
			    JinvM + 2*8*ofs[j1] + jb1*8*info[j1].m,
			    J     + 2*8*ofs[j2] + jb2*8*info[j2].m,
			    info[j1].m,info[j2].m, mskip);
	}
      }
    }
    // compute diagonal blocks of A
    for (i=0; i<nj; i++) {
      Multiply2_p8r (A + ofs[i]*(mskip+1),
		     JinvM + 2*8*ofs[i],
		     J + 2*8*ofs[i],
		     info[i].m,info[i].m, mskip);
      if (joint[i]->node[1].body) {
	MultiplyAdd2_p8r (A + ofs[i]*(mskip+1),
			  JinvM + 2*8*ofs[i] + 8*info[i].m,
			  J + 2*8*ofs[i] + 8*info[i].m,
			  info[i].m,info[i].m, mskip);
      }
    }

    // add cfm to the diagonal of A
    for (i=0; i<m; i++) A[i*mskip+i] += cfm[i] * stepsize1;

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (A,m,m,1,"A");
#   endif

    // compute the right hand side `rhs'
#   ifdef TIMING
    dTimerNow ("compute rhs");
#   endif
    ALLOCA(dReal,tmp1,nb*8*sizeof(dReal));
    //dSetZero (tmp1,nb*8);
    // put v/h + invM*fe into tmp1
    for (i=0; i<nb; i++) {
      dReal body_invMass = body[i]->invMass;
      dReal *body_invI = invI + i*12;
      for (j=0; j<3; j++) tmp1[i*8+j] = body[i]->facc[j] * body_invMass +
			    body[i]->lvel[j] * stepsize1;
      dMULTIPLY0_331 (tmp1 + i*8 + 4,body_invI,body[i]->tacc);
      for (j=0; j<3; j++) tmp1[i*8+4+j] += body[i]->avel[j] * stepsize1;
    }
    // put J*tmp1 into rhs
    ALLOCA(dReal,rhs,m*sizeof(dReal));
    //dSetZero (rhs,m);
    for (i=0; i<nj; i++) {
      dReal *JJ = J + 2*8*ofs[i];
      Multiply0_p81 (rhs+ofs[i],JJ,
		     tmp1 + 8*joint[i]->node[0].body->tag, info[i].m);
      if (joint[i]->node[1].body) {
	MultiplyAdd0_p81 (rhs+ofs[i],JJ + 8*info[i].m,
			  tmp1 + 8*joint[i]->node[1].body->tag, info[i].m);
      }
    }
    // complete rhs
    for (i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (c,m,1,0,"c");
    comparator.nextMatrix (rhs,m,1,0,"rhs");
#   endif

    // solve the LCP problem and get lambda.
    // this will destroy A but that's okay
#   ifdef TIMING
    dTimerNow ("solving LCP problem");
#   endif
    ALLOCA(dReal,lambda,m*sizeof(dReal));
    ALLOCA(dReal,residual,m*sizeof(dReal));
    dSolveLCP (m,A,lambda,rhs,residual,nub,lo,hi,findex);

#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY)
      return;
#endif


//  OLD WAY - direct factor and solve
//
//    // factorize A (L*L'=A)
//#   ifdef TIMING
//    dTimerNow ("factorize A");
//#   endif
//    dReal *L = (dReal*) ALLOCA (m*mskip*sizeof(dReal));
//    memcpy (L,A,m*mskip*sizeof(dReal));
//#   ifdef FAST_FACTOR
//    dFastFactorCholesky (L,m);  // does not report non positive definiteness
//#   else
//    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");
//#   endif
//
//    // compute lambda
//#   ifdef TIMING
//    dTimerNow ("compute lambda");
//#   endif
//    dReal *lambda = (dReal*) ALLOCA (m * sizeof(dReal));
//    memcpy (lambda,rhs,m * sizeof(dReal));
//    dSolveCholesky (L,lambda,m);

#   ifdef COMPARE_METHODS
    comparator.nextMatrix (lambda,m,1,0,"lambda");
#   endif

    // compute the constraint force `cforce'
#   ifdef TIMING
    dTimerNow ("compute constraint force");
#   endif
    // compute cforce = J'*lambda
    for (i=0; i<nj; i++) {
      dReal *JJ = J + 2*8*ofs[i];
      dxBody* b1 = joint[i]->node[0].body;
      dxBody* b2 = joint[i]->node[1].body;
      dJointFeedback *fb = joint[i]->feedback;

      if (fb) {
        // the user has requested feedback on the amount of force that this
        // joint is applying to the bodies. we use a slightly slower
        // computation that splits out the force components and puts them
        // in the feedback structure.
        dReal data[8];

        Multiply1_8q1 (data, JJ, lambda+ofs[i], info[i].m);
        dReal *cf1 = cforce + 8*b1->tag;
        cf1[0] += (fb->f1[0] = data[0]);
        cf1[1] += (fb->f1[1] = data[1]);
        cf1[2] += (fb->f1[2] = data[2]);
        cf1[4] += (fb->t1[0] = data[4]);
        cf1[5] += (fb->t1[1] = data[5]);
        cf1[6] += (fb->t1[2] = data[6]);
        if (b2){
          Multiply1_8q1 (data, JJ + 8*info[i].m, lambda+ofs[i], info[i].m);
          dReal *cf2 = cforce + 8*b2->tag;
          cf2[0] += (fb->f2[0] = data[0]);
          cf2[1] += (fb->f2[1] = data[1]);
          cf2[2] += (fb->f2[2] = data[2]);
          cf2[4] += (fb->t2[0] = data[4]);
          cf2[5] += (fb->t2[1] = data[5]);
          cf2[6] += (fb->t2[2] = data[6]);
	}
      }
      else {
	// no feedback is required, let's compute cforce the faster way
	MultiplyAdd1_8q1 (cforce + 8*b1->tag,JJ, lambda+ofs[i], info[i].m);
	if (b2) {
	  MultiplyAdd1_8q1 (cforce + 8*b2->tag,
			    JJ + 8*info[i].m, lambda+ofs[i], info[i].m);
	}
      }
    }
  }

  // compute the velocity update
#ifdef TIMING
  dTimerNow ("compute velocity update");
#endif

  // add fe to cforce
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) cforce[i*8+j] += body[i]->facc[j];
    for (j=0; j<3; j++) cforce[i*8+4+j] += body[i]->tacc[j];
  }
  // multiply cforce by stepsize
  for (i=0; i < nb*8; i++) cforce[i] *= stepsize;
  // add invM * cforce to the body velocity
  for (i=0; i<nb; i++) {
    dReal body_invMass = body[i]->invMass;
    dReal *body_invI = invI + i*12;
    for (j=0; j<3; j++) body[i]->lvel[j] += body_invMass * cforce[i*8+j];
    dMULTIPLYADD0_331 (body[i]->avel,body_invI,cforce+i*8+4);
  }

  // update the position and orientation from the new linear/angular velocity
  // (over the given timestep)
# ifdef TIMING
  dTimerNow ("update position");
# endif
  for (i=0; i<nb; i++) dxStepBody (body[i],stepsize);

#ifdef COMPARE_METHODS
  ALLOCA(dReal,tmp, nb*6*sizeof(dReal));
  for (i=0; i<nb; i++) {
    for (j=0; j<3; j++) tmp_vnew[i*6+j] = body[i]->lvel[j];
    for (j=0; j<3; j++) tmp_vnew[i*6+3+j] = body[i]->avel[j];
  }
  comparator.nextMatrix (tmp_vnew,nb*6,1,0,"vnew");
#endif

#ifdef TIMING
  dTimerNow ("tidy up");
#endif

  // zero all force accumulators
  for (i=0; i<nb; i++) {
    body[i]->facc[0] = 0;
    body[i]->facc[1] = 0;
    body[i]->facc[2] = 0;
    body[i]->facc[3] = 0;
    body[i]->tacc[0] = 0;
    body[i]->tacc[1] = 0;
    body[i]->tacc[2] = 0;
    body[i]->tacc[3] = 0;
  }

#ifdef TIMING
  dTimerEnd();
  if (m > 0) dTimerReport (stdout,1);
#endif

}

//****************************************************************************

void dInternalStepIsland (dxWorld *world, dxBody * const *body, int nb,
			  dxJoint * const *joint, int nj, dReal stepsize)
{

#ifdef dUSE_MALLOC_FOR_ALLOCA
  dMemoryFlag = d_MEMORY_OK;
#endif

#ifndef COMPARE_METHODS
  dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);

#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
      REPORT_OUT_OF_MEMORY;
      return;
    }
#endif

#endif

#ifdef COMPARE_METHODS
  int i;

  // save body state
  ALLOCA(dxBody,state,nb*sizeof(dxBody));

  for (i=0; i<nb; i++) memcpy (state+i,body[i],sizeof(dxBody));

  // take slow step
  comparator.reset();
  dInternalStepIsland_x1 (world,body,nb,joint,nj,stepsize);
  comparator.end();
#ifdef dUSE_MALLOC_FOR_ALLOCA
  if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
    REPORT_OUT_OF_MEMORY;
    return;
  }
#endif

  // restore state
  for (i=0; i<nb; i++) memcpy (body[i],state+i,sizeof(dxBody));

  // take fast step
  dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);
  comparator.end();
#ifdef dUSE_MALLOC_FOR_ALLOCA
    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
      REPORT_OUT_OF_MEMORY;
      return;
    }
#endif

  //comparator.dump();
  //_exit (1);
#endif
}


// Local Variables:
// c-basic-offset:2
// End:
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
