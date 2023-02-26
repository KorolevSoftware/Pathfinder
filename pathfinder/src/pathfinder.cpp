// pathfinder.cpp
// Extension lib defines

#define LIB_NAME "Pathfinder"
#define MODULE_NAME "pathfinder"
#include <dmsdk/dlib/array.h>
#include <dmsdk/script/script.h>
#include <dmsdk/sdk.h>

// C/C++ start index by 0, Lua start index 1
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
}

bool WavePropagation(int startPosition, int endPosition, dmArray<int>& mapPropagation, dmArray<int>& index_by_steps, int steps_count = -1) {
    int step = 1;
    int startLinearIndex = startPosition;
    mapPropagation[startLinearIndex] = step;
    bool hasEmptyCell;
    if (steps_count == -1) {
        steps_count = mapPropagation.Size();
    }
    // dmArray<int> index_by_steps;
    index_by_steps.SetCapacity(mapPropagation.Size());
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
                index_by_steps.Push(*valuePtr);
                
            }
            if(!indexes.Empty()) {
                steps_count--;
            }
            if (steps_count == 0) {
                break;
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
static void RecursiveParseToLinear(lua_State* L, dmArray<int>& linearArray){
    linearArray.OffsetCapacity(lua_objlen(L, -1));
    mapWidth = lua_objlen(L, -1);
    lua_pushnil(L);
    while (lua_next(L, -2)) {
        if (lua_isnumber(L, -1)) {
            linearArray.Push(lua_tointeger(L, -1));
        }
        if (lua_istable(L, -1)) {
            RecursiveParseToLinear(L, linearArray);
        }
        lua_pop(L, 1);
    }
}

int TwoDimToLinear(int x, int y) {
    return y * mapWidth + x;
}

void LinearIndexToTwoDim(int linearIndex, int& x, int& y) {
    y = linearIndex / mapWidth;
    x = linearIndex % mapWidth;
}

int MapHeight(int size, int mapWidth) {
    return size / mapWidth;
}

bool CheckBound(int mapWidth, int mapHeight, dmVMath::Vector3* value) {
    if (luaIndexToC(value->getX()) >= mapWidth || luaIndexToC(value->getY()) >= mapHeight) { // UP bound
        return false;
    }

    if(value->getX() <= 0 || value->getY() <= 0 ) { // Down bound
        return false;
    }

    return true;
}

//lua_State:
// 1 table
// 2 Vector3
// 3 Vector3

int MakeEmptyTable(lua_State* L) {
    lua_newtable(L);
    return 1;
}

static int solve(lua_State* L) {
    dmVMath::Vector3* start = dmScript::CheckVector3(L, 2);
    dmVMath::Vector3* end = dmScript::CheckVector3(L, 3);
    lua_pop(L, 2);


    dmArray<int> linearArray;
    RecursiveParseToLinear(L, linearArray);

    int mapSize = linearArray.Size();
    int mapHeight = MapHeight(mapSize, mapWidth);
    int startLinearIndex = TwoDimToLinear(luaIndexToC(start->getX()), luaIndexToC(start->getY()));
    int endLinearIndex = TwoDimToLinear(luaIndexToC(end->getX()), luaIndexToC(end->getY()));

    if(!CheckBound(mapWidth, mapHeight, start) || !CheckBound(mapWidth, mapHeight, end)) {
        return MakeEmptyTable(L);
    }
    dmArray<int> index_by_steps;
    if (!WavePropagation(startLinearIndex, endLinearIndex, linearArray, index_by_steps)) {
        return MakeEmptyTable(L);
    }
    
    dmArray<int> path;
    RestorePath(path, endLinearIndex, linearArray);
    lua_createtable(L, 0, path.Size());
    int index = 1;
    for (int* valuePtr = path.End()-1; valuePtr != path.Begin()-1; valuePtr--, index++) {
        int x, y;
        LinearIndexToTwoDim(*valuePtr, x, y);
        lua_pushinteger(L, index);
        dmVMath::Vector3 point(cIndexToLua(x), cIndexToLua(y), 0);
        dmScript::PushVector3(L, point);
        lua_settable(L, -3);
    }
    
    // Return table path
    return 1;
}

static int solve_near(lua_State* L) {
    dmVMath::Vector3* start = dmScript::CheckVector3(L, 2);
    int steps_count = luaL_checknumber(L, 3);
    lua_pop(L, 2);

    dmArray<int> linearArray;
    RecursiveParseToLinear(L, linearArray);

    int mapSize = linearArray.Size();
    int mapHeight = MapHeight(mapSize, mapWidth);
    int startLinearIndex = TwoDimToLinear(luaIndexToC(start->getX()), luaIndexToC(start->getY()));

    if(!CheckBound(mapWidth, mapHeight, start)) {
        return MakeEmptyTable(L);
    }

    dmArray<int> index_by_steps;
    WavePropagation(startLinearIndex, 0, linearArray, index_by_steps, steps_count);

    lua_createtable(L, 0, index_by_steps.Size());
    for (int index = 0; index < index_by_steps.Size(); index++) {
        int x, y;
        LinearIndexToTwoDim(index_by_steps[index], x, y);
        lua_pushinteger(L, index + 1);
        dmVMath::Vector3 point(cIndexToLua(x), cIndexToLua(y), 0);
        dmScript::PushVector3(L, point);
        lua_settable(L, -3);
    }
    
    // Return table path
    return 1;
}

// Functions exposed to Lua
static const luaL_reg Module_methods[] = {
    {"solve", solve},
    {"solve_near", solve_near},
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
    dmLogInfo("Registered %s Extension\n", MODULE_NAME);
    return dmExtension::RESULT_OK;
}

dmExtension::Result AppFinalizePathfinder(dmExtension::AppParams *params) {
    return dmExtension::RESULT_OK;
}

dmExtension::Result FinalizePathfinder(dmExtension::Params *params) {
    return dmExtension::RESULT_OK;
}

DM_DECLARE_EXTENSION(Pathfinder, LIB_NAME, AppInitializePathfinder, AppFinalizePathfinder, InitializePathfinder, NULL, NULL, FinalizePathfinder)