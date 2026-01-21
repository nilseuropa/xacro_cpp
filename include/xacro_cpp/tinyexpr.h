/// NowTechnologies Zrt. All rights reserved.
/// Embedded tinyexpr interface adapted for xacro_cpp.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20
/// Based on tinyexpr (public domain): https://github.com/codeplea/tinyexpr
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TeExpr {
  int mType;
  union {
    double mValue;
    const void* mBound;
    const void* mFunction;
  } mU;
  struct TeExpr* mParameters[1];
} TeExpr;

typedef double (*TeFun2)(double, double);
typedef double (*TeFun1)(double);

typedef struct TeVariable {
  const char* mName;
  const void* mAddress;
  int mType;
  void* mContext;
} TeVariable;

enum { cTeVariable = 0x0100, cTeFunction1 = 0x0200, cTeFunction2 = 0x0400, cTeFlagPure = 0x0800 };

double teInterp(const char* expression, int* error);
TeExpr* teCompile(const char* expression, const TeVariable* variables, int varCount, int* error);
double teEval(const TeExpr* n);
void teFree(TeExpr* n);

#ifdef __cplusplus
}
#endif
