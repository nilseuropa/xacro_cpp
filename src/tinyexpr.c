#include "xacro_cpp/tinyexpr.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* This is a very small subset of tinyexpr sufficient for basic math. */

typedef struct state {
  const char* start;
  const char* next;
  int type;
  te_variable* lookup;
  int lookup_len;
} state;

enum {
  TOK_NULL = 0,
  TOK_NUMBER = 1,
  TOK_VARIABLE,
  TOK_PLUS = '+',
  TOK_MINUS = '-',
  TOK_MUL = '*',
  TOK_DIV = '/',
  TOK_OPEN = '(',
  TOK_CLOSE = ')'
};

static void next_token(state* s) {
  s->type = TOK_NULL;
  while (*s->next && isspace((unsigned char)*s->next))
    s->next++;
  if (!*s->next)
    return;
  char c = *s->next;
  if (c == '+' || c == '-' || c == '*' || c == '/' || c == '(' || c == ')') {
    s->type = c;
    s->next++;
    return;
  }
  if (isdigit((unsigned char)c) || c == '.') {
    s->start = s->next;
    while (isdigit((unsigned char)*s->next) || *s->next == '.' || *s->next == 'e' || *s->next == 'E' || *s->next == '+'
           || *s->next == '-') {
      if ((s->next == s->start || (*(s->next - 1) != 'e' && *(s->next - 1) != 'E'))
          && (*s->next == '+' || *s->next == '-'))
        break;
      s->next++;
    }
    s->type = TOK_NUMBER;
    return;
  }
  if (isalpha((unsigned char)c) || c == '_') {
    s->start = s->next;
    while (isalnum((unsigned char)*s->next) || *s->next == '_')
      s->next++;
    s->type = TOK_VARIABLE;
    return;
  }
  s->next++;
}

static te_expr* new_expr(int type, te_expr* a, te_expr* b) {
  te_expr* e = (te_expr*)malloc(sizeof(te_expr) + (b ? 2 : 1) * sizeof(te_expr*));
  e->type = type;
  e->u.value = 0;
  e->parameters[0] = a;
  if (b)
    e->parameters[1] = b;
  return e;
}

static te_expr* new_value(double v) {
  te_expr* e = new_expr(TOK_NUMBER, NULL, NULL);
  e->u.value = v;
  return e;
}

static int var_index(state* s, const char* name, int len) {
  for (int i = 0; i < s->lookup_len; i++) {
    const char* n = s->lookup[i].name;
    if ((int)strlen(n) == len && strncmp(n, name, len) == 0)
      return i;
  }
  return -1;
}

static te_expr* parse_expr(state* s);

static te_expr* parse_primary(state* s) {
  if (s->type == TOK_NUMBER) {
    double v = strtod(s->start, NULL);
    te_expr* e = new_value(v);
    next_token(s);
    return e;
  } else if (s->type == TOK_VARIABLE) {
    const char* name = s->start;
    int len = (int)(s->next - s->start);
    int idx = var_index(s, name, len);
    double v = 0.0;
    if (idx >= 0)
      v = *(const double*)s->lookup[idx].address;
    te_expr* e = new_value(v);
    next_token(s);
    return e;
  } else if (s->type == TOK_OPEN) {
    next_token(s);
    te_expr* e = parse_expr(s);
    if (s->type == TOK_CLOSE)
      next_token(s);
    return e;
  } else if (s->type == TOK_MINUS) {
    next_token(s);
    te_expr* e = parse_primary(s);
    e->u.value = -e->u.value;
    return e;
  }
  return new_value(0.0);
}

static te_expr* parse_term(state* s) {
  te_expr* e = parse_primary(s);
  for (;;) {
    if (s->type == TOK_MUL) {
      next_token(s);
      te_expr* r = parse_primary(s);
      e->u.value *= r->u.value;
      free(r);
    } else if (s->type == TOK_DIV) {
      next_token(s);
      te_expr* r = parse_primary(s);
      e->u.value /= r->u.value;
      free(r);
    } else
      break;
  }
  return e;
}

static te_expr* parse_expr(state* s) {
  te_expr* e = parse_term(s);
  for (;;) {
    if (s->type == TOK_PLUS) {
      next_token(s);
      te_expr* r = parse_term(s);
      e->u.value += r->u.value;
      free(r);
    } else if (s->type == TOK_MINUS) {
      next_token(s);
      te_expr* r = parse_term(s);
      e->u.value -= r->u.value;
      free(r);
    } else
      break;
  }
  return e;
}

double te_interp(const char* expression, int* error) {
  te_variable vars[1];
  (void)vars;
  state s = {expression, expression, 0, NULL, 0};
  next_token(&s);
  te_expr* e = parse_expr(&s);
  double v = e->u.value;
  free(e);
  if (error)
    *error = (*s.next) ? 1 : 0;
  return v;
}

te_expr* te_compile(const char* expression, const te_variable* variables, int var_count, int* error) {
  state s;
  s.start = expression;
  s.next = expression;
  s.lookup = (te_variable*)variables;
  s.lookup_len = var_count;
  next_token(&s);
  te_expr* e = parse_expr(&s);
  if (error)
    *error = (*s.next) ? 1 : 0;
  return e;
}

double te_eval(const te_expr* n) {
  return n ? n->u.value : 0.0;
}
void te_free(te_expr* n) {
  if (n)
    free(n);
}
