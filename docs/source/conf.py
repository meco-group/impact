# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import sphinx_rtd_theme

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../..'))


# -- Project information -----------------------------------------------------

project = 'impact'
copyright = '2021 - 2023, MECO Research Team, KU Leuven'
author = 'MECO Research Team, KU Leuven'

# The full version, including alpha/beta/rc tags
release = '0.0.1'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.coverage',
'sphinx.ext.intersphinx',
'sphinx.ext.viewcode',
'sphinx.ext.todo',
'sphinx.ext.mathjax',
'sphinx_gallery.gen_gallery',
'sphinx_inline_tabs',
]

todo_include_todos = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

autodoc_default_options = {
    'special-members': '__init__',
    'exclude-members': '__weakref__'
}

# from sphinx_gallery.sorting import ExampleTitleSortKey

# Sphinx-Gallery configuration
sphinx_gallery_conf = {
    'examples_dirs': '../../examples',
    'gallery_dirs': 'examples',
    # defaults for local builds
    'filename_pattern': r'/cart_pendulum.py',
    'ignore_pattern': r'/cart_pendulum_|/double_integrator|/generate_models',
    'abort_on_example_error': False,
    'run_stale_examples': False,
}

# On CI, don't execute any examples at all
if os.environ.get('CI') == 'true' or os.environ.get('GITLAB_CI'):
    sphinx_gallery_conf['filename_pattern'] = r'^$'      # match nothing
    sphinx_gallery_conf['ignore_pattern'] = r'.*'        # ignore everything
    sphinx_gallery_conf['abort_on_example_error'] = False
    sphinx_gallery_conf['run_stale_examples'] = False



# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'
html_theme = "sphinx_rtd_theme"

html_theme_options = {
    'logo_only': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

html_logo = "../figures/logo/impact_logo.svg"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

# Do not order alphabetically
autodoc_member_order = 'bysource'

