# Pathfinder (Alpha)
Defold extension wave path finder

## Installation

You can use the Pathfinder extension in your own project by adding this project as a [Defold library dependency](http://www.defold.com/manuals/libraries/).
Open your game.project file and in the dependencies field under project add:

>https://github.com/KorolevSoftware/Pathfinder/archive/master.zip

git release coming soon

## Using

-1 is block

map only two dimension
```lua
local start = vmath.vector3(1, 1, 1)
local finish = vmath.vector3(4, 4, 4)
local map = {
	{0, 0,0,0},
	{-1,0,0,0},
	{0, 0,0,0},
	{0, 0,0,0}
	};
local path = pathfinder.solve(map, start, finish)
```
