PYTHON ?= python3
ARGS ?=

.PHONY: build play test

build:
	$(PYTHON) -m pip install --editable . --no-build-isolation

play:
	mars-rover-play $(ARGS)

test:
	$(PYTHON) -m unittest discover -s tests -v
