.PHONY: help
help:
	@echo "make cpp|all"

.PHONY: cpp
cpp:
	@echo "Building C++ documentation..."
	doxygen Doxyfile doc_cpp

.PHONY: mkdocs
mkdocs:
	@echo "Building MkDocs documentation..."
	cd .. && mkdocs build

.PHONY: all
all: cpp mkdocs
	@echo "All documentation built."

.PHONY: deploy
deploy:
	@echo "Deploying documentation..."
	cd .. && mkdocs gh-deploy --force
