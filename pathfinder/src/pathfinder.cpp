// pathfinder.cpp
// Extension lib defines

#define LIB_NAME "Pathfinder"
#define MODULE_NAME "pathfinder"
#include <dmsdk/dlib/array.h>
#include <dmsdk/script/script.h>
#include <dmsdk/sdk.h>

// C/C++ start index by 1, Lua start index 1
#define luaIndexToC(i) i - 1 
#define cIndexToLua(i) i + 1

typedef bool (*intComparator)(int value);

int globalCurrentValue;
int mapWidth;

bool ValueEqualZero(int value) {
    return value == 0;
}

bool ValueLessCurrent(int value) {
    return globalCurrentValue > value && value > 0;
}

void BoxFilter(dmArray<int>& outIndexes, dmArray<int> const& data, int linearIndex, int dataWidth, intComparator comparator) {
    int xpositive = linearIndex + 1;
    int xnegative = linearIndex - 1;
    int ypositive = linearIndex + dataWidth;
    int ynegative = linearIndex - dataWidth;
    // x+
    if (linearIndex % dataWidth + 1 < dataWidth && comparator(data[xpositive])) {
        outIndexes.Push(xpositive);
    }

    // x-
    if (linearIndex % dataWidth - 1 >= 0 && comparator(data[xnegative])) {
        outIndexes.Push(xnegative);
    }

    // y+
    if (ypositive < data.Size() && comparator(data[ypositive])) {
        outIndexes.Push(ypositive);
    }

    // y-
    if (ynegative >= 0 && comparator(data[ynegative])) {
        outIndexes.Push(ynegative);
    }

    //return outIndexes;
}

bool WavePropagation(int startPosition, int endPosition, dmArray<int>& mapPropagation) {
    int step = 1;
    int startLinearIndex = startPosition;
    mapPropagation[startLinearIndex] = step;
    bool hasEmptyCell;

    // Box filter
    do {
        hasEmptyCell = false;
        for (int linearIndex = 0; linearIndex < mapPropagation.Size(); linearIndex++) {
            if (mapPropagation[linearIndex] != step) {
                continue;
            }
            dmArray<int> indexes;
            indexes.SetCapacity(4);
            BoxFilter(indexes, mapPropagation, linearIndex, mapWidth, ValueEqualZero);
            if (!indexes.Empty()) {
                hasEmptyCell = true;
            }

            for (int* valuePtr = indexes.Begin(); valuePtr != indexes.End(); valuePtr++) {
                mapPropagation[*valuePtr] = step + 1; 
            }
        }
        step++;
    } while (hasEmptyCell);
    return mapPropagation[endPosition] > 0;
}

void RestorePath(dmArray<int>& path, int endPosition, dmArray<int> const& mapPropagation) {
    path.SetCapacity(mapPropagation.Size());
    int endLinearIndex = endPosition;
    globalCurrentValue = mapPropagation[endLinearIndex];
    path.Push(endPosition);
    while (true) {
        // _1 < currentValue && _1 > 0;
        dmArray<int> indexes;
        indexes.SetCapacity(4);
        BoxFilter(indexes, mapPropagation, endLinearIndex, mapWidth, ValueLessCurrent);

        if (indexes.Empty()) {
            break;
        }

        int firstIndex = indexes.Front();
        globalCurrentValue = mapPropagation[firstIndex];
        path.Push(firstIndex);
        endLinearIndex = firstIndex;
    }
}

// http://lua-users.org/lists/lua-l/2004-04/msg00201.html
static void RecursiveToDimToLinear(lua_State* L, dmArray<int>& linearArray){
    linearArray.OffsetCapacity(lua_objlen(L, -1));
    mapWidth = lua_objlen(L, -1);
    lua_pushnil(L);
    while (lua_next(L, -2)) {
        if (lua_isnumber(L, -1)) {
            linearArray.Push(lua_tointeger(L, -1));
        }
        if (lua_istable(L, -1)) {
            RecursiveToDimToLinear(L, linearArray);
        }
        lua_pop(L, 1);
    }
}

int TwoDimToLinear(int x, int y) {
    return y * mapWidth + x;
}

void LinearIndexToTwoDim(int linearIndex, int& x, int& y) {
    y = linearIndex/mapWidth;
    x = linearIndex % mapWidth;
}

bool CheckBound(int mapWidth, int mapSize, Vectormath::Aos::Vector3* value) {
    int linearIndex = TwoDimToLinear(luaIndexToC(value->getX()), luaIndexToC(value->getY()));
    if (linearIndex >= mapSize){
        return false;
    }

    if (luaIndexToC(value->getX()) >= mapWidth){
        return false;
    }

    if(value->getX() <= 0 || value->getY() <= 0 ) {
        return false;
    }

    return true;
}

static int solve(lua_State* L) {
    Vectormath::Aos::Vector3* start = dmScript::ToVector3(L, 2);
    Vectormath::Aos::Vector3* end = dmScript::ToVector3(L, 3);
    lua_pop(L, 2);

    dmArray<int> linearArray;
    RecursiveToDimToLinear(L, linearArray);

    int mapSize = linearArray.Size();
    int startLinearIndex = TwoDimToLinear(luaIndexToC(start->getX()), luaIndexToC(start->getY()));
    int endLinearIndex = TwoDimToLinear(luaIndexToC(end->getX()), luaIndexToC(end->getY()));

    if(!CheckBound(mapWidth, mapSize, start) || !CheckBound(mapWidth, mapSize, end)) {
        return 0;
    }
    
    if (!WavePropagation(startLinearIndex, endLinearIndex, linearArray)) {
        return 0;
    }
    
    dmArray<int> path;
    RestorePath(path, endLinearIndex, linearArray);
    lua_createtable(L, 0, path.Size());
    int index = 1;
    for (int* valuePtr = path.End()-1; valuePtr != path.Begin()-1; valuePtr--, index++) {
        int x, y;
        LinearIndexToTwoDim(*valuePtr, x, y);
        Vectormath::Aos::Vector3 point(cIndexToLua(x), cIndexToLua(y), 0);
        lua_pushinteger(L, index);
        dmScript::PushVector3(L, point);
        lua_settable(L, -3);
    }
    
    // Return table path
    return 1;
}

// Functions exposed to Lua
static const luaL_reg Module_methods[] = {
    {"solve", solve},
    {0, 0}
};

static void LuaInit(lua_State* L) {
    int top = lua_gettop(L);

    // Register lua names
    luaL_register(L, MODULE_NAME, Module_methods);

    lua_pop(L, 1);
    assert(top == lua_gettop(L));
}
dmExtension::Result AppInitializePathfinder(dmExtension::AppParams *params) {
    return dmExtension::RESULT_OK;
}

dmExtension::Result InitializePathfinder(dmExtension::Params *params) {
    // Init Lua
    LuaInit(params->m_L);
    printf("Registered %s Extension\n", MODULE_NAME);
    return dmExtension::RESULT_OK;
}

dmExtension::Result AppFinalizePathfinder(dmExtension::AppParams *params) {
    return dmExtension::RESULT_OK;
}

dmExtension::Result FinalizePathfinder(dmExtension::Params *params) {
    return dmExtension::RESULT_OK;
}


// MyExtension is the C++ symbol that holds all relevant extension data.
// It must match the name field in the `ext.manifest`
DM_DECLARE_EXTENSION(Pathfinder, LIB_NAME, AppInitializePathfinder, AppFinalizePathfinder, InitializePathfinder, 0, 0, FinalizePathfinder)