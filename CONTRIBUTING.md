# Contributing

```bash
uv sync --extra dev --extra simulation
uv run ruff check .
uv run ruff format --check .
uv run mypy src
uv run pytest
uv build
```

Physical tests must be marked `hardware`, start with conservative motion, and document the exact arm, calibration, firmware, payload, and result.
