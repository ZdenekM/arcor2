# ARCOR2

[![CircleCI](https://circleci.com/gh/robofit/arcor2.svg?style=svg&circle-token=190cc70ee7baa7b6b1335f85ff71a553cf2c50a2)](https://circleci.com/gh/robofit/arcor2)

**ARCOR** stands for **A**ugmented **R**eality **C**ollaborative **R**obot. It is a system and user interface for simplified programming of collaborative robots based on augmented reality. This is the second evolution of this system developed by [Robo@FIT](https://www.fit.vut.cz/research/group/robo/.en). The first one was [ARCOR(1)](https://github.com/robofit/arcor). 

This repository contains the backend solution (server, build, execution units). It can be easily tested out or deployed using prepared [docker images](https://github.com/robofit/arcor2/tree/master/docker). Unity-based client application is available [here](https://github.com/robofit/arcor2_editor).

Development is supported by [Test-it-off: Robotic offline product testing](https://www.fit.vut.cz/research/project/1308/) project (Ministry of Industry and Trade of the Czech Republic).

## Overview

Purpose of the following text is to introduce basic concepts of ARCOR2.

### Scene

TBD

### Project

TBD

### Objects, services

TBD

## For developers

Expected environment:
  * Linux (tested on Ubuntu)
  * Python 3.8
  * ...dependencies from setup.py
  
Installation (for development):
```bash
pip install -e .
```

Before running ```arcor2_execution```, please set ```ARCOR2_PROJECT_PATH``` to a directory in ```PYTHONPATH```.

How to run tests:
```bash
mypy arcor2
flake8 arcor2
py.test --cov arcor2
```

After any commit, coverage should not be worse than before.

