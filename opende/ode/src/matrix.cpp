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

#include <ode/common.h>
#include "config.h"
#include "matrix.h"
#include "util.h"

#include "matrix_impl.h"


// misc defines
#define ALLOCA dALLOCA16


/*extern */
void dxMultiply0 (dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned qskip = dPAD(q);
    const unsigned rskip = dPAD(r);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned i = p; i != 0; aa+=rskip, bb+=qskip, --i) {
        dReal *a = aa;
        const dReal *cc = C, *ccend = C + r;
        for (; cc != ccend; ++a, ++cc) {
            dReal sum = REAL(0.0);
            const dReal *c = cc;
            const dReal *b = bb, *bend = bb + q;
            for (; b != bend; c+=rskip, ++b) {
                sum += (*b) * (*c);
            }
            (*a) = sum; 
        }
    }
}


/*extern */
void dxMultiply1 (dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned pskip = dPAD(p);
    const unsigned rskip = dPAD(r);
    dReal *aa = A;
    const dReal *bb = B, *bbend = B + p;
    for (; bb != bbend; aa += rskip, ++bb) {
        dReal *a = aa;
        const dReal *cc = C, *ccend = C + r;
        for (; cc != ccend; ++a, ++cc) {
            dReal sum = REAL(0.0);
            const dReal *b = bb, *c = cc;
            for (unsigned k = q; k != 0; b += pskip, c += rskip, --k) {
                sum += (*b) * (*c);
            }
            (*a) = sum;
        }
    }
}


/*extern */
void dxMultiply2 (dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned rskip = dPAD(r);
    const unsigned qskip = dPAD(q);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned i = p; i != 0; aa += rskip, bb += qskip, --i) {
        dReal *a = aa, *aend = aa + r;
        const dReal *cc = C;
        for (; a != aend; cc+=qskip, ++a) {
            dReal sum = REAL(0.0);
            const dReal *b = bb, *c = cc, *cend = cc + q;
            for (; c != cend; ++b, ++c) {
                sum += (*b) * (*c);
            }
            (*a) = sum; 
        }
    }
}


/*extern */
int dxFactorCholesky (dReal *A, unsigned n, void *tmpbuf/*[n]*/)
{
    dAASSERT (n > 0 && A);
    bool failure = false;
    const unsigned nskip = dPAD (n);
    dReal *recip = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (n * sizeof(dReal));
    dReal *aa = A;
    for (unsigned i = 0; i < n; aa += nskip, ++i) {
        dReal *cc = aa;
        {
            const dReal *bb = A;
            for (unsigned j = 0; j < i; bb += nskip, ++cc, ++j) {
                dReal sum = *cc;
                const dReal *a = aa, *b = bb, *bend = bb + j;
                for (; b != bend; ++a, ++b) {
                    sum -= (*a) * (*b);
                }
                *cc = sum * recip[j];
            }
        }
        {
            dReal sum = *cc;
            dReal *a = aa, *aend = aa + i;
            for (; a != aend; ++a) {
                sum -= (*a)*(*a);
            }
            if (sum <= REAL(0.0)) {
                failure = true;
                break;
            }
            dReal sumsqrt = dSqrt(sum);
            *cc = sumsqrt;
            recip[i] = dRecip (sumsqrt);
        }
    }
    return failure ? 0 : 1;
}


/*extern */
void dxSolveCholesky (const dReal *L, dReal *b, unsigned n, void *tmpbuf/*[n]*/)
{
    dAASSERT (n > 0 && L && b);
    const unsigned nskip = dPAD (n);
    dReal *y = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (n * sizeof(dReal));
    {
        const dReal *ll = L;
        for (unsigned i = 0; i < n; ll += nskip, ++i) {
            dReal sum = REAL(0.0);
            for (unsigned k = 0; k < i; ++k) {
                sum += ll[k] * y[k];
            }
            dIASSERT(ll[i] != dReal(0.0));
            y[i] = (b[i] - sum) / ll[i];
        }
    }
    {
        const dReal *ll = L + (n - 1) * (nskip + 1);
        for (unsigned i = n; i > 0; ll -= nskip + 1) {
            --i;
            dReal sum = REAL(0.0);
            const dReal *l = ll + nskip;
            for (unsigned k = i + 1; k < n; l += nskip, ++k) {
                sum += (*l) * b[k];
            }
            dIASSERT(*ll != dReal(0.0));
            b[i] = (y[i] - sum) / (*ll);
        }
    }
}


/*extern */
int dxInvertPDMatrix (const dReal *A, dReal *Ainv, unsigned n, void *tmpbuf/*[nskip*(n+2)]*/)
{
    dAASSERT (n > 0 && A && Ainv);
    bool success = false;
    size_t FactorCholesky_size = dxEstimateFactorCholeskyTmpbufSize(n);
    size_t SolveCholesky_size = dxEstimateSolveCholeskyTmpbufSize(n);
    size_t MaxCholesky_size = FactorCholesky_size > SolveCholesky_size ? FactorCholesky_size : SolveCholesky_size;
    dIASSERT(MaxCholesky_size % sizeof(dReal) == 0);
    const unsigned nskip = dPAD (n);
    const size_t nskip_mul_n = (size_t)nskip * n;
    dReal *tmp = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (MaxCholesky_size + (nskip + nskip_mul_n) * sizeof(dReal));
    dReal *X = (dReal *)((char *)tmp + MaxCholesky_size);
    dReal *L = X + nskip;
    memcpy (L, A, nskip_mul_n * sizeof(dReal));
    if (dFactorCholesky (L, n, tmp)) {
        dSetZero (Ainv, nskip_mul_n);	// make sure all padding elements set to 0
        dReal *aa = Ainv, *xi = X, *xiend = X + n;
        for (; xi != xiend; ++aa, ++xi) {
            dSetZero(X, n);
            *xi = REAL(1.0);
            dSolveCholesky (L, X, n, tmp);
            dReal *a = aa;
            const dReal *x = X, *xend = X + n;
            for (; x != xend; a += nskip, ++x) {
                *a = *x;
            }
        }
        success = true;
    }
    return success ? 1 : 0;  
}


/*extern */
int dxIsPositiveDefinite (const dReal *A, unsigned n, void *tmpbuf/*[nskip*(n+1)]*/)
{
    dAASSERT (n > 0 && A);
    size_t FactorCholesky_size = dxEstimateFactorCholeskyTmpbufSize(n);
    dIASSERT(FactorCholesky_size % sizeof(dReal) == 0);
    const unsigned nskip = dPAD (n);
    const size_t nskip_mul_n = (size_t)nskip * n;
    dReal *tmp = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (FactorCholesky_size + nskip_mul_n * sizeof(dReal));
    dReal *Acopy = (dReal *)((char *)tmp + FactorCholesky_size);
    memcpy (Acopy, A, nskip_mul_n * sizeof(dReal));
    return dFactorCholesky (Acopy, n, tmp);
}

/*extern */
void dxVectorScale (dReal *a, const dReal *d, unsigned n)
{
    dxtVectorScale<1, 1> (a, d, n);
}

/*extern */
void dxSolveLDLT (const dReal *L, const dReal *d, dReal *b, unsigned n, int nskip)
{
    dIASSERT(n != 0);

    dxtSolveLDLT<1, 1>(L, d, b, n, nskip);
}

/*extern */
void dxLDLTAddTL (dReal *L, dReal *d, const dReal *a, unsigned n, unsigned nskip, void *tmpbuf/*[2*nskip]*/)
{
    dAASSERT (L && d && a && n > 0 && nskip >= n);

    if (n < 2) return;
    dReal *W1 = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (nskip * (2 * sizeof(dReal)));
    dReal *W2 = W1 + nskip;

    W1[0] = REAL(0.0);
    W2[0] = REAL(0.0);
    for (unsigned j = 1; j < n; ++j) {
        W1[j] = W2[j] = (dReal) (a[j] * M_SQRT1_2);
    }
    dReal W11 = (dReal) ((REAL(0.5)*a[0]+1)*M_SQRT1_2);
    dReal W21 = (dReal) ((REAL(0.5)*a[0]-1)*M_SQRT1_2);

    dReal alpha1 = REAL(1.0);
    dReal alpha2 = REAL(1.0);

    {
        dReal dee = d[0];
        dReal alphanew = alpha1 + (W11*W11)*dee;
        dIASSERT(alphanew != dReal(0.0));
        dee /= alphanew;
        dReal gamma1 = W11 * dee;
        dee *= alpha1;
        alpha1 = alphanew;
        alphanew = alpha2 - (W21*W21)*dee;
        dee /= alphanew;
        //dReal gamma2 = W21 * dee;
        alpha2 = alphanew;
        dReal k1 = REAL(1.0) - W21*gamma1;
        dReal k2 = W21*gamma1*W11 - W21;
        dReal *ll = L + nskip;
        for (unsigned p = 1; p < n; ll += nskip, ++p) {
            dReal Wp = W1[p];
            dReal ell = *ll;
            W1[p] =    Wp - W11*ell;
            W2[p] = k1*Wp +  k2*ell;
        }
    }

    dReal *ll = L + (nskip + 1);
    for (unsigned j = 1; j < n; ll += nskip + 1, ++j) {
        dReal k1 = W1[j];
        dReal k2 = W2[j];

        dReal dee = d[j];
        dReal alphanew = alpha1 + (k1*k1)*dee;
        dIASSERT(alphanew != dReal(0.0));
        dee /= alphanew;
        dReal gamma1 = k1 * dee;
        dee *= alpha1;
        alpha1 = alphanew;
        alphanew = alpha2 - (k2*k2)*dee;
        dee /= alphanew;
        dReal gamma2 = k2 * dee;
        dee *= alpha2;
        d[j] = dee;
        alpha2 = alphanew;

        dReal *l = ll + nskip;
        for (unsigned p = j + 1; p < n; l += nskip, ++p) {
            dReal ell = *l;
            dReal Wp = W1[p] - k1 * ell;
            ell += gamma1 * Wp;
            W1[p] = Wp;
            Wp = W2[p] - k2 * ell;
            ell -= gamma2 * Wp;
            W2[p] = Wp;
            *l = ell;
        }
    }
}


// macros for dLDLTRemove() for accessing A - either access the matrix
// directly or access it via row pointers. we are only supposed to reference
// the lower triangle of A (it is symmetric), but indexes i and j come from
// permutation vectors so they are not predictable. so do a test on the
// indexes - this should not slow things down too much, as we don't do this
// in an inner loop.

#define _GETA(i,j) (A[i][j])
//#define _GETA(i,j) (A[(i)*nskip+(j)])
#define GETA(i,j) ((i > j) ? _GETA(i,j) : _GETA(j,i))


/*extern */
void dxLDLTRemove (dReal **A, const unsigned *p, dReal *L, dReal *d,
                   unsigned n1, unsigned n2, unsigned r, unsigned nskip, void *tmpbuf/*n2 + 2*nskip*/)
{
    dAASSERT(A && p && L && d && n1 > 0 && n2 > 0 /*&& r >= 0 */&& r < n2 &&
        n1 >= n2 && nskip >= n1);
#ifndef dNODEBUG
    for (unsigned i = 0; i < n2; ++i) dIASSERT(p[i] >= 0 && p[i] < n1);
#endif

    if (r == n2 - 1) {
        return;		// deleting last row/col is easy
    }
    else {
        size_t LDLTAddTL_size = dxEstimateLDLTAddTLTmpbufSize(nskip);
        dIASSERT(LDLTAddTL_size % sizeof(dReal) == 0);
        dReal *tmp = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (LDLTAddTL_size + n2 * sizeof(dReal));
        if (r == 0) {
            dReal *a = (dReal *)((char *)tmp + LDLTAddTL_size);
            const unsigned p_0 = p[0];
            for (unsigned i = 0; i < n2; ++i) {
                a[i] = -GETA(p[i],p_0);
            }
            a[0] += REAL(1.0);
            dLDLTAddTL (L, d, a, n2, nskip, tmp);
        }
        else {
            dReal *t = (dReal *)((char *)tmp + LDLTAddTL_size);
            {
                dReal *Lcurr = L + r*nskip;
                for (unsigned i = 0; i < r; ++Lcurr, ++i) {
                    dIASSERT(d[i] != dReal(0.0));
                    t[i] = *Lcurr / d[i];
                }
            }
            dReal *a = t + r;
            {
                dReal *Lcurr = L + r * nskip;
                const unsigned *pp_r = p + r, p_r = *pp_r;
                const unsigned n2_minus_r = n2 - r;
                for (unsigned i = 0; i < n2_minus_r; Lcurr += nskip, ++i) {
                    a[i] = dDot(Lcurr, t, r) - GETA(pp_r[i], p_r);
                }
            }
            a[0] += REAL(1.0);
            dLDLTAddTL (L + (size_t)(nskip + 1) * r, d + r, a, n2 - r, nskip, tmp);
        }
    }

    // snip out row/column r from L and d
    dRemoveRowCol (L, n2, nskip, r);
    if (r < (n2 - 1)) memmove (d + r, d + r + 1, (n2 - r - 1) * sizeof(dReal));
}


/*extern */
void dxRemoveRowCol (dReal *A, unsigned n, unsigned nskip, unsigned r)
{
    dAASSERT(A && n > 0 && nskip >= n && r >= 0 && r < n);
    if (r >= n - 1) return;
    if (r > 0) {
        {
            const size_t move_size = (n - r - 1) * sizeof(dReal);
            dReal *Adst = A + r;
            for (unsigned i = 0; i < r; Adst += nskip, ++i) {
                dReal *Asrc = Adst + 1;
                memmove (Adst, Asrc, move_size);
            }
        }
        {
            const size_t cpy_size = r * sizeof(dReal);
            dReal *Adst = A + (size_t)nskip * r;
            unsigned n1 = n - 1;
            for (unsigned i = r; i < n1; ++i) {
                dReal *Asrc = Adst + nskip;
                memcpy (Adst, Asrc, cpy_size);
                Adst = Asrc;
            }
        }
    }
    {
        const size_t cpy_size = (n - r - 1) * sizeof(dReal);
        dReal *Adst = A + (size_t)(nskip + 1) * r;
        unsigned n1 = n - 1;
        for (unsigned i = r; i < n1; ++i) {
            dReal *Asrc = Adst + (nskip + 1);
            memcpy (Adst, Asrc, cpy_size);
            Adst = Asrc - 1;
        }
    }
}


#undef dSetZero
#undef dSetValue
//#undef dDot
#undef dMultiply0
#undef dMultiply1
#undef dMultiply2
#undef dFactorCholesky
#undef dSolveCholesky
#undef dInvertPDMatrix
#undef dIsPositiveDefinite
//#undef dFactorLDLT
//#undef dSolveL1
//#undef dSolveL1T
#undef dVectorScale
#undef dSolveLDLT
#undef dLDLTAddTL
#undef dLDLTRemove
#undef dRemoveRowCol


/*extern */
void dSetZero (dReal *a, int n)
{
    dxSetZero (a, n);
}

/*extern */
void dSetValue (dReal *a, int n, dReal value)
{
    dxSetValue (a, n, value);
}

// dReal dDot (const dReal *a, const dReal *b, int n);

/*extern */
void dMultiply0 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply0 (A, B, C, p, q, r);
}

/*extern */
void dMultiply1 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply1 (A, B, C, p, q, r);
}

/*extern */
void dMultiply2 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply2 (A, B, C, p, q, r);
}

/*extern */
int dFactorCholesky (dReal *A, int n)
{
    return dxFactorCholesky (A, n, NULL);
}

/*extern */
void dSolveCholesky (const dReal *L, dReal *b, int n)
{
    dxSolveCholesky (L, b, n, NULL);
}

/*extern */
int dInvertPDMatrix (const dReal *A, dReal *Ainv, int n)
{
    return dxInvertPDMatrix (A, Ainv, n, NULL);
}

/*extern */
int dIsPositiveDefinite (const dReal *A, int n)
{
    return dxIsPositiveDefinite (A, n, NULL);
}

// void dFactorLDLT (dReal *A, dReal *d, int n, int nskip);
// void dSolveL1 (const dReal *L, dReal *b, int n, int nskip);
// void dSolveL1T (const dReal *L, dReal *b, int n, int nskip);

/*extern */
void dVectorScale (dReal *a, const dReal *d, int n)
{
    dxVectorScale (a, d, n);
}

/*extern */
void dSolveLDLT (const dReal *L, const dReal *d, dReal *b, int n, int nskip)
{
    dAASSERT(n != 0);

    if (n != 0)
    {
        dAASSERT(L != NULL);
        dAASSERT(d != NULL);
        dAASSERT(b != NULL);

        dxSolveLDLT(L, d, b, n, nskip);
    }
}

/*extern */
void dLDLTAddTL (dReal *L, dReal *d, const dReal *a, int n, int nskip)
{
    dxLDLTAddTL (L, d, a, n, nskip, NULL);
}

/*extern */
void dLDLTRemove (dReal **A, const int *p, dReal *L, dReal *d, int n1, int n2, int r, int nskip)
{
    dxLDLTRemove (A, (const unsigned *)p, L, d, n1, n2, r, nskip, NULL);
    dSASSERT(sizeof(unsigned) == sizeof(*p));
}

/*extern */
void dRemoveRowCol (dReal *A, int n, int nskip, int r)
{
    dxRemoveRowCol (A, n, nskip, r);
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

#include <ode/common.h>
#include <ode/matrix.h>
#include "util.h"
#include "config.h"

// misc defines
#define ALLOCA dALLOCA16


void dSetZero (dReal *a, int n)
{
  dAASSERT (a && n >= 0);
  while (n > 0) {
    *(a++) = 0;
    n--;
  }
}


void dSetValue (dReal *a, int n, dReal value)
{
  dAASSERT (a && n >= 0);
  while (n > 0) {
    *(a++) = value;
    n--;
  }
}


void dMultiply0 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,qskip,rskip,rpad;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  qskip = dPAD(q);
  rskip = dPAD(r);
  rpad = rskip - r;
  dReal sum;
  const dReal *b,*c,*bb;
  bb = B;
  for (i=p; i; i--) {
    for (j=0 ; j<r; j++) {
      c = C + j;
      b = bb;
      sum = 0;
      for (k=q; k; k--, c+=rskip) sum += (*(b++))*(*c);
      *(A++) = sum; 
    }
    A += rpad;
    bb += qskip;
  }
}


void dMultiply1 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,pskip,rskip;
  dReal sum;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  pskip = dPAD(p);
  rskip = dPAD(r);
  for (i=0; i<p; i++) {
    for (j=0; j<r; j++) {
      sum = 0;
      for (k=0; k<q; k++) sum += B[i+k*pskip] * C[j+k*rskip];
      A[i*rskip+j] = sum;
    }
  }
}


void dMultiply2 (dReal *A, const dReal *B, const dReal *C, int p, int q, int r)
{
  int i,j,k,z,rpad,qskip;
  dReal sum;
  const dReal *bb,*cc;
  dAASSERT (A && B && C && p>0 && q>0 && r>0);
  rpad = dPAD(r) - r;
  qskip = dPAD(q);
  bb = B;
  for (i=p; i; i--) {
    cc = C;
    for (j=r; j; j--) {
      z = 0;
      sum = 0;
      for (k=q; k; k--,z++) sum += bb[z] * cc[z];
      *(A++) = sum; 
      cc += qskip;
    }
    A += rpad;
    bb += qskip;
  }
}


int dFactorCholesky (dReal *A, int n)
{
  int i,j,k,nskip;
  dReal sum,*a,*b,*aa,*bb,*cc,*recip;
  dAASSERT (n > 0 && A);
  nskip = dPAD (n);
  recip = (dReal*) ALLOCA (n * sizeof(dReal));
  aa = A;
  for (i=0; i<n; i++) {
    bb = A;
    cc = A + i*nskip;
    for (j=0; j<i; j++) {
      sum = *cc;
      a = aa;
      b = bb;
      for (k=j; k; k--) sum -= (*(a++))*(*(b++));
      *cc = sum * recip[j];
      bb += nskip;
      cc++;
    }
    sum = *cc;
    a = aa;
    for (k=i; k; k--, a++) sum -= (*a)*(*a);
    if (sum <= REAL(0.0)) return 0;
    *cc = dSqrt(sum);
    recip[i] = dRecip (*cc);
    aa += nskip;
  }
  return 1;
}


void dSolveCholesky (const dReal *L, dReal *b, int n)
{
  int i,k,nskip;
  dReal sum,*y;
  dAASSERT (n > 0 && L && b);
  nskip = dPAD (n);
  y = (dReal*) ALLOCA (n*sizeof(dReal));
  for (i=0; i<n; i++) {
    sum = 0;
    for (k=0; k < i; k++) sum += L[i*nskip+k]*y[k];
    y[i] = (b[i]-sum)/L[i*nskip+i];
  }
  for (i=n-1; i >= 0; i--) {
    sum = 0;
    for (k=i+1; k < n; k++) sum += L[k*nskip+i]*b[k];
    b[i] = (y[i]-sum)/L[i*nskip+i];
  }
}


int dInvertPDMatrix (const dReal *A, dReal *Ainv, int n)
{
  int i,j,nskip;
  dReal *L,*x;
  dAASSERT (n > 0 && A && Ainv);
  nskip = dPAD (n);
  L = (dReal*) ALLOCA (nskip*n*sizeof(dReal));
  memcpy (L,A,nskip*n*sizeof(dReal));
  x = (dReal*) ALLOCA (n*sizeof(dReal));
  if (dFactorCholesky (L,n)==0) return 0;
  dSetZero (Ainv,n*nskip);	// make sure all padding elements set to 0
  for (i=0; i<n; i++) {
    for (j=0; j<n; j++) x[j] = 0;
    x[i] = 1;
    dSolveCholesky (L,x,n);
    for (j=0; j<n; j++) Ainv[j*nskip+i] = x[j];
  }
  return 1;  
}


int dIsPositiveDefinite (const dReal *A, int n)
{
  dReal *Acopy;
  dAASSERT (n > 0 && A);
  int nskip = dPAD (n);
  Acopy = (dReal*) ALLOCA (nskip*n * sizeof(dReal));
  memcpy (Acopy,A,nskip*n * sizeof(dReal));
  return dFactorCholesky (Acopy,n);
}


/***** this has been replaced by a faster version
void dSolveL1T (const dReal *L, dReal *b, int n, int nskip)
{
  int i,j;
  dAASSERT (L && b && n >= 0 && nskip >= n);
  dReal sum;
  for (i=n-2; i>=0; i--) {
    sum = 0;
    for (j=i+1; j<n; j++) sum += L[j*nskip+i]*b[j];
    b[i] -= sum;
  }
}
*/


void dVectorScale (dReal *a, const dReal *d, int n)
{
  dAASSERT (a && d && n >= 0);
  for (int i=0; i<n; i++) a[i] *= d[i];
}


void dSolveLDLT (const dReal *L, const dReal *d, dReal *b, int n, int nskip)
{
  dAASSERT (L && d && b && n > 0 && nskip >= n);
  dSolveL1 (L,b,n,nskip);
  dVectorScale (b,d,n);
  dSolveL1T (L,b,n,nskip);
}


void dLDLTAddTL (dReal *L, dReal *d, const dReal *a, int n, int nskip)
{
  int j,p;
  dReal *W1,*W2,W11,W21,alpha1,alpha2,alphanew,gamma1,gamma2,k1,k2,Wp,ell,dee;
  dAASSERT (L && d && a && n > 0 && nskip >= n);

  if (n < 2) return;
  W1 = (dReal*) ALLOCA (n*sizeof(dReal));
  W2 = (dReal*) ALLOCA (n*sizeof(dReal));

  W1[0] = 0;
  W2[0] = 0;
  for (j=1; j<n; j++) W1[j] = W2[j] = (dReal) (a[j] * M_SQRT1_2);
  W11 = (dReal) ((REAL(0.5)*a[0]+1)*M_SQRT1_2);
  W21 = (dReal) ((REAL(0.5)*a[0]-1)*M_SQRT1_2);

  alpha1=1;
  alpha2=1;

  dee = d[0];
  alphanew = alpha1 + (W11*W11)*dee;
  dee /= alphanew;
  gamma1 = W11 * dee;
  dee *= alpha1;
  alpha1 = alphanew;
  alphanew = alpha2 - (W21*W21)*dee;
  dee /= alphanew;
  gamma2 = W21 * dee;
  alpha2 = alphanew;
  k1 = REAL(1.0) - W21*gamma1;
  k2 = W21*gamma1*W11 - W21;
  for (p=1; p<n; p++) {
    Wp = W1[p];
    ell = L[p*nskip];
    W1[p] =    Wp - W11*ell;
    W2[p] = k1*Wp +  k2*ell;
  }

  for (j=1; j<n; j++) {
    dee = d[j];
    alphanew = alpha1 + (W1[j]*W1[j])*dee;
    dee /= alphanew;
    gamma1 = W1[j] * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - (W2[j]*W2[j])*dee;
    dee /= alphanew;
    gamma2 = W2[j] * dee;
    dee *= alpha2;
    d[j] = dee;
    alpha2 = alphanew;

    k1 = W1[j];
    k2 = W2[j];
    for (p=j+1; p<n; p++) {
      ell = L[p*nskip+j];
      Wp = W1[p] - k1 * ell;
      ell += gamma1 * Wp;
      W1[p] = Wp;
      Wp = W2[p] - k2 * ell;
      ell -= gamma2 * Wp;
      W2[p] = Wp;
      L[p*nskip+j] = ell;
    }
  }
}


// macros for dLDLTRemove() for accessing A - either access the matrix
// directly or access it via row pointers. we are only supposed to reference
// the lower triangle of A (it is symmetric), but indexes i and j come from
// permutation vectors so they are not predictable. so do a test on the
// indexes - this should not slow things down too much, as we don't do this
// in an inner loop.

#define _GETA(i,j) (A[i][j])
//#define _GETA(i,j) (A[(i)*nskip+(j)])
#define GETA(i,j) ((i > j) ? _GETA(i,j) : _GETA(j,i))


void dLDLTRemove (dReal **A, const int *p, dReal *L, dReal *d,
		  int n1, int n2, int r, int nskip)
{
  int i;
  dAASSERT(A && p && L && d && n1 > 0 && n2 > 0 && r >= 0 && r < n2 &&
	   n1 >= n2 && nskip >= n1);
  #ifndef dNODEBUG
  for (i=0; i<n2; i++) dIASSERT(p[i] >= 0 && p[i] < n1);
  #endif

  if (r==n2-1) {
    return;		// deleting last row/col is easy
  }
  else if (r==0) {
    dReal *a = (dReal*) ALLOCA (n2 * sizeof(dReal));
    for (i=0; i<n2; i++) a[i] = -GETA(p[i],p[0]);
    a[0] += REAL(1.0);
    dLDLTAddTL (L,d,a,n2,nskip);
  }
  else {
    dReal *t = (dReal*) ALLOCA (r * sizeof(dReal));
    dReal *a = (dReal*) ALLOCA ((n2-r) * sizeof(dReal));
    for (i=0; i<r; i++) t[i] = L[r*nskip+i] / d[i];
    for (i=0; i<(n2-r); i++)
      a[i] = dDot(L+(r+i)*nskip,t,r) - GETA(p[r+i],p[r]);
    a[0] += REAL(1.0);
    dLDLTAddTL (L + r*nskip+r, d+r, a, n2-r, nskip);
  }

  // snip out row/column r from L and d
  dRemoveRowCol (L,n2,nskip,r);
  if (r < (n2-1)) memmove (d+r,d+r+1,(n2-r-1)*sizeof(dReal));
}


void dRemoveRowCol (dReal *A, int n, int nskip, int r)
{
  int i;
  dAASSERT(A && n > 0 && nskip >= n && r >= 0 && r < n);
  if (r >= n-1) return;
  if (r > 0) {
    for (i=0; i<r; i++)
      memmove (A+i*nskip+r,A+i*nskip+r+1,(n-r-1)*sizeof(dReal));
    for (i=r; i<(n-1); i++)
      memcpy (A+i*nskip,A+i*nskip+nskip,r*sizeof(dReal));
  }
  for (i=r; i<(n-1); i++)
    memcpy (A+i*nskip+r,A+i*nskip+nskip+r+1,(n-r-1)*sizeof(dReal));
}
>>>>>>> parent of c757c4e1... renamed globally ode to ode-dbl
