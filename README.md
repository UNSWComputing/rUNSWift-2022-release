[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Welcome to the rUNSWift git repository.


## Directory structure

The directory structure is:

* **bin**:
    This is where any executables are stored, such as configuration scripts (setup-build, nao_sync)
* **image**:
    The contents of this directory are synced to the robot with nao_sync, put custom configuration files
    or libraries here. Python code that handles behaviour decision making exists here.
* **robot**:
    This is the source code for the rUNSWift binaries, including our core architecture.
* **utils**:
    This is the source code for any off-nao utilities, such as colour
    calibration or offline debugging utilities.


## Documentation

Documentation can be found in several places:
 - On [ReadTheDocs][read-the-docs] for heaps of high-level overviews on different areas, of note:
    - [Setup](https://runswift.readthedocs.io/en/latest/setup/index.html)
    - [Architecture](https://runswift.readthedocs.io/en/latest/architecture.html)
    - [Code Release / Team Reports](https://runswift.readthedocs.io/en/latest/code_releases_team_reports.html)

 - On the [CSE RoboCup Site](https://cgi.cse.unsw.edu.au/~robocup/) for game videos, older team reports and code releases

 <!-- search link is relative to repo home.  won't work when looking at `README.md` as a blob. -->
 - [In README.md files in many directories](../../search?q=filename%3AREADME) such as for [pos files](image/home/nao/data/pos/README.md) and [vision regions](robot/perception/vision/Region/README.md), great for implementation overviews
 
 - In docstrings or multiline comments at the top of files, such as explaining how [behaviour.py](image/home/nao/data/behaviours/behaviour.py#L1-L17) bridges C++ and Python

 - In code comments as appropriate to the language
 
 <!-- wiki link is relative to repo home.  won't work when looking at `README.md` as a blob. -->
 - In the [wiki](../../wiki), typically just for private/secret things (most of the public stuff has been migrated to [ReadTheDocs][read-the-docs], though if you find something that hasn't been please help migrate it)

[read-the-docs]: https://runswift.readthedocs.io/
