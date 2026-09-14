"""PyInstaller entry point: import lazypx4 as a top-level package and run its CLI."""

from lazypx4.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
