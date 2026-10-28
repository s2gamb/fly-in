export PATH := $(HOME)/.local/bin:$(PATH)

PYTHON ?= python3
PIP ?= pip3
ARGS ?= maps/easy/01_linear_path.txt

install:
	$(PIP) install -r requirements.txt

run:
	$(PYTHON) main.py $(ARGS)

debug:
	$(PYTHON) -m pdb main.py $(ARGS)

clean:
	rm -rf __pycache__ src/__pycache__ .mypy_cache
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict

.PHONY: install run debug clean lint lint-strict
