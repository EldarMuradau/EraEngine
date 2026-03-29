// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "ai/navigation.h"

#include "core/memory.h"
#include "core/hash.h"

namespace era_engine::ai
{
    static bool is_destination(const vec2& pos, const NavNode& dest)
    {
        return pos == dest.position;
    }

    static float calculate_h(const vec2& pos, const NavNode& dest)
    {
        return length(pos - dest.position);
    }

    // TODO: Obstacle checking
    static bool is_valid(const vec2& pos)
    {
        return true;
    }

    NODISCARD std::vector<NavNode> PathFinder::a_star_make_path(std::unordered_map<vec2, NavNode>& map, NavNode& dest)
    {
        try
        {
            std::cout << "Found a path\n";
            int x = dest.position.x;
            int y = dest.position.y;
            std::stack<NavNode> path;
            std::vector<NavNode> usablePath;

            while (!(map[vec2(x, y)].parent_position == vec2(x, y))
                && map[vec2(x, y)].position != NAV_INF_POS)
            {
                path.push(map[vec2(x, y)]);
                int tempX = map[vec2(x, y)].parent_position.x;
                int tempY = map[vec2(x, y)].parent_position.y;
                x = tempX;
                y = tempY;
            }
            path.push(map[vec2(x, y)]);

            while (!path.empty())
            {
                NavNode top = path.top();
                path.pop();
                usablePath.emplace_back(top);
            }

            return usablePath;
        }
        catch (const std::exception& e)
        {
            std::cout << e.what() << "\n";
            return {};
        }
    }

    NODISCARD std::vector<NavNode> AStarNavigation::navigate(NavNode& from, NavNode& to)
    {
        if (is_valid(to.position) == false)
        {
            std::cout << "Destination is an obstacle\n";
            return {};
        }
        if (is_destination(from.position, to))
        {
            std::cout << "You are the destination\n";
            return {};
        }

        std::unordered_map<vec2, bool> closedList;
        closedList.reserve((NAV_X_MAX / NAV_X_STEP) * (NAV_Y_MAX / NAV_Y_STEP) + 1);

        std::unordered_map<vec2, NavNode> allMap;
        allMap.reserve((NAV_X_MAX / NAV_X_STEP) * (NAV_Y_MAX / NAV_Y_STEP) + 1);
        for (int x = -1; x < (NAV_X_MAX / NAV_X_STEP) + 1; x++)
        {
            for (int y = -1; y < (NAV_Y_MAX / NAV_Y_STEP) + 1; y++)
            {
                allMap[vec2(x, y)].f_cost = FLT_MAX;
                allMap[vec2(x, y)].g_cost = FLT_MAX;
                allMap[vec2(x, y)].h_cost = FLT_MAX;
                allMap[vec2(x, y)].parent_position = NAV_INF_POS;
                allMap[vec2(x, y)].position = vec2(x, y);

                closedList.emplace(vec2(x, y), false);
            }
        }

        int x = from.position.x;
        int y = from.position.y;
        allMap[vec2(x, y)].f_cost = 0.0;
        allMap[vec2(x, y)].g_cost = 0.0;
        allMap[vec2(x, y)].h_cost = 0.0;
        allMap[vec2(x, y)].parent_position = vec2(x, y);

        std::vector<NavNode> openList;
        openList.emplace_back(allMap[vec2(x, y)]);
        bool destinationFound = false;

        while (!openList.empty() && openList.size() < (NAV_X_MAX / NAV_X_STEP) * (NAV_Y_MAX / NAV_Y_STEP))
        {
            NavNode node;
            do
            {
                float temp = FLT_MAX;
                std::vector<NavNode>::iterator itNode;
                for (auto it = openList.begin(); it != openList.end(); it = next(it))
                {
                    NavNode n = *it;
                    if (n.f_cost < temp)
                    {
                        temp = n.f_cost;
                        itNode = it;
                    }
                }
                node = *itNode;
                openList.erase(itNode);
            } while (is_valid(node.position) == false);

            x = node.position.x;
            y = node.position.y;
            closedList[vec2(x, y)] = true;

            for (int newX = -1; newX <= 1; newX++)
            {
                for (int newY = -1; newY <= 1; newY++)
                {
                    double gNew, hNew, fNew;
                    if (is_valid(vec2(x + newX, y + newY)))
                    {
                        if (is_destination(vec2(x + newX, y + newY), to))
                        {
                            allMap[vec2(x + newX, y + newY)].parent_position = vec2(x, y);
                            destinationFound = true;
                            return PathFinder::a_star_make_path(allMap, to);;
                        }
                        else if (closedList[vec2(x + newX, y + newY)] == false)
                        {
                            gNew = node.g_cost + 1.0;
                            hNew = calculate_h(vec2(x + newX, y + newY), to);
                            fNew = gNew + hNew;

                            if (allMap[vec2(x + newX, y + newY)].f_cost == FLT_MAX ||
                                allMap[vec2(x + newX, y + newY)].f_cost > fNew)
                            {
                                allMap[vec2(x + newX, y + newY)].f_cost = fNew;
                                allMap[vec2(x + newX, y + newY)].g_cost = gNew;
                                allMap[vec2(x + newX, y + newY)].h_cost = hNew;
                                allMap[vec2(x + newX, y + newY)].parent_position = vec2(x, y);
                                openList.emplace_back(allMap[vec2(x + newX, y + newY)]);
                            }
                        }
                    }
                }
            }
        }

        if (destinationFound == false)
        {
            std::cout << "Destination not found\n";
            return {};
        }

        return {};
    }
}