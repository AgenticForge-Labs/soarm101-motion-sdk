# Contributing

Use Python 3.12 and `uv`.

```bash
uv sync --extra dev
uv run ruff check .
uv run ruff format --check .
uv run mypy src
uv run pytest
uv build
```

Hardware tests must be opt-in, clearly marked, and safe to skip in CI.
