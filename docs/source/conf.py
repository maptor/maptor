import sys
from pathlib import Path


project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

project = "MAPTOR"
copyright = "2025, MAPTOR"
author = "David Timothy"

try:
    import maptor

    version = maptor.__version__
    release = version
except ImportError:
    version = "0.2.0"
    release = version

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.coverage",
    "sphinx.ext.mathjax",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx_autodoc_typehints",
    "myst_parser",
    "sphinx_copybutton",
    "sphinx_design",
]

myst_enable_extensions = [
    "dollarmath",
    "amsmath",
    "deflist",
    "fieldlist",
    "html_admonition",
    "html_image",
    "colon_fence",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

myst_dmath_double_inline = True
myst_dmath_allow_labels = True

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "myst",
}

master_doc = "index"
default_role = "autolink"

autosummary_generate = True
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
    "special-members": "__init__",
}

autodoc_typehints = "description"
autodoc_member_order = "bysource"

exclude_patterns = [
    "**.ipynb",
]

suppress_warnings = [
    "autodoc.import_object",
    "myst.header",
    "autosummary.import_cycle",
    "autodoc.import_cycle",
    "sphinx.docstring.indent",
]
nitpicky = False

autodoc_warningiserror = False

html_theme = "pydata_sphinx_theme"

html_logo = "_static/MAPTOR_banner.svg"

html_static_path = ["_static"]
html_css_files = ["maptor_brand.css"]

html_title = f"{project} v{version} Documentation"

html_sidebars = {"index": ["search-button-field"], "**": ["search-button-field", "sidebar-nav-bs"]}

html_theme_options = {
    "header_links_before_dropdown": 6,
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/maptor/maptor",
            "icon": "fa-brands fa-github",
        },
        {
            "name": "PyPI",
            "url": "https://pypi.org/project/maptor/",
            "icon": "fa-solid fa-box",
        },
    ],
    "navbar_start": ["navbar-logo"],
    "navbar_end": ["theme-switcher", "navbar-icon-links"],
    "navbar_persistent": [],
    "secondary_sidebar_items": ["page-toc"],
    "footer_start": ["copyright"],
    "footer_end": ["theme-version"],
    "primary_sidebar_end": [],
    "show_toc_level": 2,
    "navigation_with_keys": True,
    "show_version_warning_banner": True,
}

# Intersphinx mapping - SciPy standard
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
    "scipy": ("https://docs.scipy.org/doc/scipy/", None),
    "matplotlib": ("https://matplotlib.org/stable/", None),
}

# Professional copying settings - SciPy pattern
html_use_modindex = True
html_domain_indices = False
html_copy_source = False
html_file_suffix = ".html"

mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"

copybutton_prompt_text = r">>> |\.\.\. |\$ |In \[\d*\]: | {2,5}\.{3,}: | {5,8}: "
copybutton_prompt_is_regexp = True

napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False

add_function_parentheses = False
html_last_updated_fmt = "%b %d, %Y"


def setup(app):
    """Auto-generate API and examples docs on build - SciPy pattern."""
    import subprocess

    api_script_path = Path(__file__).parent / "generate_api_docs.py"
    if api_script_path.exists():
        try:
            subprocess.run([sys.executable, str(api_script_path)], check=True)
            print("✓ API documentation generated")
        except subprocess.CalledProcessError:
            print("⚠ API generation had issues")

    examples_script_path = Path(__file__).parent / "generate_examples_docs.py"
    if examples_script_path.exists():
        try:
            subprocess.run([sys.executable, str(examples_script_path)], check=True)
            print("Examples documentation generated")
        except subprocess.CalledProcessError:
            print("Examples generation had issues")
