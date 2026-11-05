"""CLI entry point for running module via `python -m gcs`."""

import sys

from gcs.app import main

if __name__ == "__main__":
    sys.exit(main())
