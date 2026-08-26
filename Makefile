ARGS ?= maps/easy/01_linear_path.txt

install:
	@echo "Use make run"

run:
	python3 main.py $(ARGS)

clean:
	rm -rf __pycache__
	rm -rf src/*/__pycache__

.PHONY: install, run, clean
