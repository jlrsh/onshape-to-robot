# `message.py` — Colorized console output

Thin wrappers around `colorama`. All return the input string wrapped in
ANSI colour codes; call sites embed them in `print(...)`.

| Function       | Colour       |
|----------------|--------------|
| `error(text)`  | Red          |
| `warning(text)`| Yellow       |
| `success(text)`| Green        |
| `info(text)`   | Blue         |
| `bright(text)` | Bold         |
| `dim(text)`    | Dim          |

Calls `colorama.just_fix_windows_console()` at import for Windows
compatibility.
