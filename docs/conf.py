# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'dVRK'
copyright = '2012-2024, Johns Hopkins University (Baltimore, USA)'
author = 'Anton Deguet, Peter Kazanzides'

# The full version, including alpha/beta/rc tags
release = '2.3.0'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx_tabs.tabs'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

master_doc = 'index'

# -- Hook to download and compile json schemas
import json
import os
import shutil
from json_schema_for_humans.generate import generate_from_schema, generate_from_filename, GenerationConfiguration
import urllib.request

schema_dir = '_build/html/pages/schemas'

if not os.path.exists(schema_dir):
  os.makedirs(schema_dir)

dvrk_version = '2.3.0'
for file in ['cisst-component-manager', 
             'cisst-matrices',
             'dvrk-arm',
             'dvrk-console',
             'dvrk-ecm',
             'dvrk-interface-button',
             'dvrk-mtm',
             'dvrk-psm',
             'dvrk-teleop-ecm',
             'dvrk-teleop-psm',
             'dvrk-tool-list']:
    print(f'retrieving JSON schema {file}')
    url = f'https://raw.githubusercontent.com/jhu-dvrk/sawIntuitiveResearchKit/refs/tags/{dvrk_version}/share/schemas/{file}.schema.json'
    dest = f'{schema_dir}/{file}.schema.json'
    urllib.request.urlretrieve(url, dest)

# the following is to be able to locate all possible $ref in schema
schema_store = {}
schema_files = []

# find all files in directory
all_files = os.listdir(schema_dir)
for that_file in all_files:
    that_file_full_path = os.path.join(schema_dir, that_file)
    extension = '.schema.json'
    if that_file_full_path[-len(extension):] == extension:
        print(f'Found possible schema file: {that_file}')
        with open(that_file_full_path, 'r') as schema_file_descriptor:
            that_schema = json.load(schema_file_descriptor)
            if '$id' in that_schema:
                print(f" - $id: {that_schema['$id']}")
                schema_store[that_schema['$id']] = that_schema
                schema_files.append(that_file_full_path)
            else:
                print(f'WARNING: ignoring file {that_file}, $id is missing')

# # now iterate through the same files to generate the html documentation
config = GenerationConfiguration(copy_css = True,
                                 copy_js = True,
                                 expand_buttons = True,
                                 collapse_long_descriptions = False)
                                 
# dummy generate to get a copy of css and js
generate_from_filename(schema_file_name = schema_files[0], result_file_name = 'dummy.html')
# copy css and js files to directory with all files
shutil.copy('schema_doc.css', schema_dir)
shutil.copy('schema_doc.min.js', schema_dir)

for schema_file in schema_files:
    html = generate_from_schema(schema_file = schema_file, loaded_schemas = schema_store, config = config)
                                 
    # file name with version and without .schema.json
    html_file = os.path.normpath('{}.html'.format(schema_file)).replace('.schema.json', '')
    print('Generating {}'.format(html_file))
    with open(html_file, 'w') as file_descriptor:
        file_descriptor.write(html)

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

html_css_files = [
    'custom.css',
]

html_title = project + ' ' + release
