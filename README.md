# Ground Concept Implementation

## Introduction
This concept implementation is written in Python.

The code consists of two significant parts: strategic and tactical conflict resolution parts.

The strategic conflict resolution part is located in `ground_routing` folder and responsible for computing conflict-free 4D trajectories.

The tactical conflict resolution part is located in `ground_tactical` folder and deals with managing conflicts that occur during simulation. It is implemented as a plugin for BlueSky.

## Installation

### Strategic deconfliction only
To run only strategic deconfliction it is enough to install requirements from `requrements.txt` file and some IP solver supported by `python-mip` library (opensource free CBC-solver works just fine).
Then it is OK to look at code examples in `main.py` as how to resolve several conflicts.

### Tactical deconfliction plugin for bluesky
First run `python setup.py install` to install centralized deconfliction code as a library. 
This should automatically install all necessary dependencies.

Then, create a symlink for the `ground_tactical` folder in BlueSky plugins folder. Linux users may run something like `ln -s /path to ground concept/ground_tactical/ ground_tactical` inside of `bluesky/plugins/` folder to do that.

It should also be possible to copy-paste the files to the BlueSky plugins directory without creating a symlink.
