PYTHON ?= python3
ARGS ?=

.PHONY: build train check submission gui play gui-test

build:
	$(PYTHON) -m pip install --editable . --no-build-isolation

train:
	$(PYTHON) train.py $(ARGS)

check:
	docker build --tag mars-rover-submission-check .

submission:
	$(PYTHON) package_submission.py

gui:
	$(PYTHON) -m pip install --editable gui --no-build-isolation

play: gui
	mars-rover-play $(ARGS)

gui-test:
	$(PYTHON) -m unittest discover -s gui/tests -v
