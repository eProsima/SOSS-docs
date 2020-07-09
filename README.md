# SOSS Documentation

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Linux Build Status](http://jenkins.eprosima.com:8080/job/nightly_soss-docs_master/badge/icon)](http://jenkins.eprosima.com:8080/job/nightly_soss-docs_master)

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

*SOSS* is a *System-Of-Systems Synthesizer* that allows communication among an arbitrary number of protocols that
speak different languages.

If one has a number of complex systems and wills to combine them to create a larger, even more
complex system, *SOSS* can act as an intermediate message-passing tool that, by speaking a common
language, centralizes and mediates the integration.

For more information, check out the
[SOSS documentation](https://soss.docs.eprosima.com/en/latest/).
You can find all the source code on our [GitHub repository](https://github.com/eProsima/soss_v2).

1. [Installation Guide](#installation-guide)
1. [Getting Started](#getting-started)
1. [Generating documentation in other formats](#generating-documentation-in-other-formats)
1. [Running documentation tests](#running-documentation-tests)

## Installation Guide

1. In order to build and test the documentation, some dependencies must be installed beforehand:

    ```bash
    sudo apt update
    sudo apt install -y \
        git \
        python3 \
        python3-pip \
        python3-venv \
        python3-sphinxcontrib.spelling \
    ```

1. Clone the repository

    ```bash
    cd ~
    git clone https://github.com/eProsima/SOSS-docs soss-docs
    ```

1. Create a virtual environment and install python3 dependencies

    ```bash
    cd ~/soss-docs
    python3 -m venv soss-docs-venv
    source soss-docs-venv/bin/activate
    pip3 install -r docs/requirements.txt
    ```

## Getting Started

To generate the documentation in a HTML format for a specific branch of *SOSS* run:

```bash
cd ~/soss-docs
source soss-docs-venv/bin/activate
make html
```

## Generating documentation in other formats

The documentation can be generated in several formats such as HTML, PDF, LaTex, etc. For a complete list of targets run:

```bash
cd ~/soss-docs
make help
```

Once you have selected a format, generate the documentation with:

```bash
cd ~/soss-docs
source soss-docs-venv/bin/activate
make <output_format>
```

## Running documentation tests

This repository provides a set of tests that verify that:

1. The RST follows the style guidelines
1. There are no spelling errors
1. The HTML is built correctly

Run the tests by:

```bash
cd ~/soss-docs
source soss-docs-venv/bin/activate
make test
```
## Contributing

If you are interested in making some contributions, either in the form of an issue or a pull request, please refer to
our [Contribution Guidelines](https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md).
