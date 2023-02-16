# HelixNavigator Autos

The file structure is designed to be as flexible as possible to account for
rapid iteration at competition.

The system uses the software
[HelixNavigator (HN)](https://github.com/TripleHelixProgramming/HelixNavigator)
which inputs paths and generates trajectories.

## Definitions

A path describes a starting and ending pose with constraints on how to get
from one to the other.

A trajectory describes exactly *how* to get to the ending pose.

## Quick examples

HN Document:
`blue-south-1cone_place1_pick1.json`

- `blue-south-1cone`: auto routine name
  - Blue side of field
  - South side of field
  - In routine, robot picks up one cone
- `place1`: first path
  - place a cone on the southmost grid slot
- `pick1`: second path
  - pick up the cone at the southmost preplaced location

## File types

- Paths are contained within HelixNavigator documents (`.json`)
- Trajectories are contained within trajectory files (`.json`)

## File naming

All file names contain only lowercase letters. The auto routine name comes
first, followed by the paths it is composed of in order. `_` separates paths
and trajectories, and `-` may be used to separate words in names. Here are
some examples:

`red-south-1cone_place1_pick1`
`blue-north-1cone_place9`

## Path file structure

Paths go in `autos`. Each auto routine has a directory for each of its paths.

Here's an example file structure for a two cone auto:

```
.
└── autos/
    └── blue-north-2cone/
        ├── blue-north-2cone_place9.json
        ├── blue-north-2cone_pick4.json
        └── blue-north-2cone_place7.json
    └── another-auto/
        ├── another-auto_part1.json
        └── another-auto_part2.json

```

## Trajectory file structure

Trajectories go in `src/main/deploy`. Examples:

```
.
└── src/
    └── main/
        └── deploy/
            ├── red-north-2cone-chgstat_place9.json
            ├── red-north-2cone-chgstat_pick4.json
            ├── red-north-2cone-chgstat_place7.json
            ├── red-north-2cone-chgstat_chgstat.json
            ├── blue-south-1cone-place1.json
            └── another-trajectory.json
```

## Naming

### Sides of field

To allow for variance in field setups, paths are created for red and blue
separately.

- `blue`: blue side, -x
- `red`: red side, +x

### Field Directions (look at the field so blue is to left)

- `north`: Upper side of field, +y
- `south`: Lower side of field, -y

### Objectives

- `pick#`: drive to a cone and pick it up
  - `#` is which preplaced cone picked up, 1-4, from south to north
  - Ex: `pick4` is the farthest north cone pickable
- `place#`: drive to a node and place a cone
  - `#` is which node to place on, 1-9 from south to north
  - Ex: `place9` is the first cone placement if starting all the way north
- `chgstat-LEVEL`: charging station
  - `LEVEL` is which part of the station to drive onto, `south`, `mid`, or
    `north`.
  - Ex: `chgstat-mid` if robot sits in middle of charging station
