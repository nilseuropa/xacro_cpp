// Minimal embed of tinyexpr (public domain): https://github.com/codeplea/tinyexpr
// Slightly namespaced to avoid symbol clashes.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct te_expr {
    int type;
    union { double value; const void *bound; const void *function; } u;
    struct te_expr *parameters[1];
} te_expr;

typedef double (*te_fun2)(double, double);
typedef double (*te_fun1)(double);

typedef struct te_variable {
    const char *name;
    const void *address;
    int type;
    void *context;
} te_variable;

enum {TE_VARIABLE = 0x0100, TE_FUNCTION1 = 0x0200, TE_FUNCTION2 = 0x0400, TE_FLAG_PURE = 0x0800};

double te_interp(const char *expression, int *error);
te_expr *te_compile(const char *expression, const te_variable *variables, int var_count, int *error);
double te_eval(const te_expr *n);
void te_free(te_expr *n);

#ifdef __cplusplus
}
#endif

