# Makefile for Sphinx documentation

SPHINXOPTS    ?= -W --keep-going -j auto
SPHINXBUILD  ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = _build

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile clean api examples

# Generate API docs before building
api:
	@echo "Generating API documentation..."
	cd $(SOURCEDIR) && python generate_api_docs.py

# Generate examples docs before building
examples:
	@echo "Generating examples documentation..."
	cd $(SOURCEDIR) && python generate_examples_docs.py

# Build HTML with API and examples generation
html: api examples
	@echo "Building HTML documentation..."
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@echo "✓ Documentation built in _build/html/"

# Clean everything including API and examples docs
clean:
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@rm -rf $(SOURCEDIR)/api/
	@rm -rf $(SOURCEDIR)/examples/
	@echo "✓ Cleaned all build files"

# Quick build without API regeneration
html-fast:
	@echo "Building HTML documentation (fast mode)..."
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
