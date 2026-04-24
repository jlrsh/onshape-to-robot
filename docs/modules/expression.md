# `expression.py` — Onshape parametric expressions

Evaluates dimension formulas and variable references that Onshape
returns alongside parameters.

## `ExpressionParser`

```python
class ExpressionParser:
    def __init__(self)
    self.variables: dict[str, float]
    self.variables_lazy_loading: Callable | None
```

- `variables` is seeded with `pi`, `Pi`, `PI`.
- `variables_lazy_loading` is an optional callback triggered the first
  time the parser needs to resolve an unknown identifier.
  `Assembly.__init__` sets it to `self.load_variables`, which hits
  `/variables/...` on the Onshape API.

### Supported operators

`+`, `-`, `*`, `/`, `**`/`^`, `%`, unary `-`.

### Supported functions

Trig: `cos`, `sin`, `tan`, `acos`, `asin`, `atan`, `atan2`, `cosh`,
`sinh`, `tanh`, `asinh`, `acosh`, `atanh`.

Rounding: `ceil`, `floor`, `round`.

Misc: `exp`, `sqrt`, `abs`, `log`, `log10`, `max`, `min`.

All delegate to numpy. Unknown function → `ValueError`.

### Unit suffixes

Units are replaced with multiplicative factors before parsing, so
`"5 mm" → "5*0.001"`. Length:

| Suffix | Factor |
|--------|--------|
| `millimeter` / `mm` | 1e-3 |
| `centimeter` / `cm` | 1e-2 |
| `meter` / `m` | 1.0 |
| `inch` / `in` | 0.0254 |
| `foot` / `ft` | 0.3048 |
| `yard` / `yd` | 0.9144 |

Angle:

| Suffix | Factor |
|--------|--------|
| `radian` / `rad` | 1.0 |
| `degree` / `deg` | π/180 |

### Methods

- `eval_expr(expr: str) -> float` — preprocesses (`^` → `**`, strip `#`,
  unit replacement), parses via `ast.parse(..., mode="eval")`, evaluates.
- `eval_(node)` — internal AST evaluator. Handles `Constant`, `BinOp`,
  `UnaryOp`, `Name`, `Call`. Unknown names trigger `variables_lazy_loading`
  once, then error if still unresolved.

### Example

```python
parser = ExpressionParser()
parser.variables["x"] = 5
parser.eval_expr("(cos(5 deg)) mm + x inch")
```
