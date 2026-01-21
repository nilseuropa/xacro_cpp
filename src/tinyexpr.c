/// NowTechnologies Zrt. All rights reserved.
/// Embedded tinyexpr implementation adapted for xacro_cpp.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20
/// Based on tinyexpr (public domain): https://github.com/codeplea/tinyexpr
#include "xacro_cpp/tinyexpr.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/// This is a very small subset of tinyexpr sufficient for basic math.

static double evalFunc(const char* name, int len, double arg, int* handled) {
  /// Recognize a handful of math functions used by xacro: sin, cos, exp, abs, fabs.
  *handled = 1;
  if (len == 3 && name[0] == 's' && name[1] == 'i' && name[2] == 'n') {
    return sin(arg);
  }
  if (len == 3 && name[0] == 'c' && name[1] == 'o' && name[2] == 's') {
    return cos(arg);
  }
  if (len == 3 && name[0] == 'e' && name[1] == 'x' && name[2] == 'p') {
    return exp(arg);
  }
  if (len == 3 && name[0] == 'a' && name[1] == 'b' && name[2] == 's') {
    return fabs(arg);
  }
  if (len == 4 && name[0] == 'f' && name[1] == 'a' && name[2] == 'b' && name[3] == 's') {
    return fabs(arg);
  }
  *handled = 0;
  return 0.0;
}

typedef struct State {
  const char* mStart;
  const char* mNext;
  int mType;
  TeVariable* mLookup;
  int mLookupLen;
} State;

enum {
  cTokNull = 0,
  cTokNumber = 1,
  cTokVariable,
  cTokPlus = '+',
  cTokMinus = '-',
  cTokMul = '*',
  cTokDiv = '/',
  cTokOpen = '(',
  cTokClose = ')'
};

static void nextToken(State* s) {
  s->mType = cTokNull;
  while (*s->mNext && isspace((unsigned char)*s->mNext)) {
    s->mNext++;
  }
  if (!*s->mNext) {
    return;
  }
  char c = *s->mNext;
  if (c == '+' || c == '-' || c == '*' || c == '/' || c == '(' || c == ')') {
    s->mType = c;
    s->mNext++;
    return;
  }
  if (isdigit((unsigned char)c) || c == '.') {
    s->mStart = s->mNext;
    while (isdigit((unsigned char)*s->mNext) || *s->mNext == '.' || *s->mNext == 'e' || *s->mNext == 'E'
           || *s->mNext == '+' || *s->mNext == '-') {
      if ((s->mNext == s->mStart || (*(s->mNext - 1) != 'e' && *(s->mNext - 1) != 'E'))
          && (*s->mNext == '+' || *s->mNext == '-')) {
        break;
      }
      s->mNext++;
    }
    s->mType = cTokNumber;
    return;
  }
  if (isalpha((unsigned char)c) || c == '_') {
    s->mStart = s->mNext;
    while (isalnum((unsigned char)*s->mNext) || *s->mNext == '_') {
      s->mNext++;
    }
    s->mType = cTokVariable;
    return;
  }
  s->mNext++;
}

static TeExpr* newExpr(int type, TeExpr* a, TeExpr* b) {
  TeExpr* e = (TeExpr*)malloc(sizeof(TeExpr) + (b ? 2 : 1) * sizeof(TeExpr*));
  e->mType = type;
  e->mU.mValue = 0;
  e->mParameters[0] = a;
  if (b) {
    e->mParameters[1] = b;
  }
  return e;
}

static TeExpr* newValue(double v) {
  TeExpr* e = newExpr(cTokNumber, NULL, NULL);
  e->mU.mValue = v;
  return e;
}

static int varIndex(State* s, const char* name, int len) {
  for (int i = 0; i < s->mLookupLen; i++) {
    const char* n = s->mLookup[i].mName;
    if ((int)strlen(n) == len && strncmp(n, name, len) == 0) {
      return i;
    }
  }
  return -1;
}

static TeExpr* parseExpr(State* s);

static TeExpr* parsePrimary(State* s) {
  if (s->mType == cTokNumber) {
    double v = strtod(s->mStart, NULL);
    TeExpr* e = newValue(v);
    nextToken(s);
    return e;
  } else if (s->mType == cTokVariable) {
    const char* name = s->mStart;
    int len = (int)(s->mNext - s->mStart);
    const char* after = s->mNext;
    while (*after && isspace((unsigned char)*after)) {
      ++after;
    }
    if (*after == '(') {
      /// Function call with a single argument.
      s->mNext = after;
      /// consume '('
      nextToken(s);
      /// move to first token in arg
      nextToken(s);
      TeExpr* arg = parseExpr(s);
      if (s->mType == cTokClose) {
        nextToken(s);
      }
      int handled = 0;
      double v = evalFunc(name, len, arg ? arg->mU.mValue : 0.0, &handled);
      TeExpr* e = newValue(handled ? v : 0.0);
      if (arg) {
        free(arg);
      }
      return e;
    } else {
      int idx = varIndex(s, name, len);
      double v = 0.0;
      if (idx >= 0) {
        v = *(const double*)s->mLookup[idx].mAddress;
      }
      TeExpr* e = newValue(v);
      nextToken(s);
      return e;
    }
  } else if (s->mType == cTokOpen) {
    nextToken(s);
    TeExpr* e = parseExpr(s);
    if (s->mType == cTokClose) {
      nextToken(s);
    }
    return e;
  } else if (s->mType == cTokMinus) {
    nextToken(s);
    TeExpr* e = parsePrimary(s);
    e->mU.mValue = -e->mU.mValue;
    return e;
  } else if (s->mType == cTokPlus) {
    nextToken(s);
    TeExpr* e = parsePrimary(s);
    return e;
  }
  return newValue(0.0);
}

static TeExpr* parseTerm(State* s) {
  TeExpr* e = parsePrimary(s);
  for (;;) {
    if (s->mType == cTokMul) {
      nextToken(s);
      TeExpr* r = parsePrimary(s);
      e->mU.mValue *= r->mU.mValue;
      free(r);
    } else if (s->mType == cTokDiv) {
      nextToken(s);
      TeExpr* r = parsePrimary(s);
      e->mU.mValue /= r->mU.mValue;
      free(r);
    } else {
      break;
    }
  }
  return e;
}

static TeExpr* parseExpr(State* s) {
  TeExpr* e = parseTerm(s);
  for (;;) {
    if (s->mType == cTokPlus) {
      nextToken(s);
      TeExpr* r = parseTerm(s);
      e->mU.mValue += r->mU.mValue;
      free(r);
    } else if (s->mType == cTokMinus) {
      nextToken(s);
      TeExpr* r = parseTerm(s);
      e->mU.mValue -= r->mU.mValue;
      free(r);
    } else {
      break;
    }
  }
  return e;
}

double teInterp(const char* expression, int* error) {
  TeVariable vars[1];
  (void)vars;
  State s = {expression, expression, 0, NULL, 0};
  nextToken(&s);
  TeExpr* e = parseExpr(&s);
  double v = e->mU.mValue;
  free(e);
  if (error) {
    *error = (*s.mNext) ? 1 : 0;
  }
  return v;
}

TeExpr* teCompile(const char* expression, const TeVariable* variables, int varCount, int* error) {
  State s;
  s.mStart = expression;
  s.mNext = expression;
  s.mLookup = (TeVariable*)variables;
  s.mLookupLen = varCount;
  nextToken(&s);
  TeExpr* e = parseExpr(&s);
  if (error) {
    *error = (*s.mNext) ? 1 : 0;
  }
  return e;
}

double teEval(const TeExpr* n) {
  return n ? n->mU.mValue : 0.0;
}
void teFree(TeExpr* n) {
  if (n) {
    free(n);
  }
}
