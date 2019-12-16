--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2019 Peter Vaiko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
]]

PathfinderUtil = {}

---Size/turn radius all other information on the vehicle
---@class PathfinderUtil.VehicleData
PathfinderUtil.VehicleData = CpObject()

function PathfinderUtil.VehicleData:init(vehicle, withImplements, buffer)
    self.turnRadius = vehicle.cp and vehicle.cp.turnDiameter and vehicle.cp.turnDiameter / 2 or 10
    self.name = vehicle.getName and vehicle:getName() or 'N/A'
    self.dFront, self.dRear, self.dLeft, self.dRight = 0, 0, 0, 0
    self:calculateSizeOfObjectList(vehicle, {{object = vehicle}}, buffer)
    if withImplements then
        self:calculateSizeOfObjectList(vehicle, vehicle:getAttachedImplements(), buffer)
    end
end

--- calculate the bounding box of all objects in the implement list. This is not a very good way to figure out how
--- big a vehicle is as the sizes of foldable implements seem to be in the folded state but should be ok for
--- now.
function PathfinderUtil.VehicleData:calculateSizeOfObjectList(vehicle, implements, buffer)
    for _, implement in ipairs(implements) do
        --print(implement.object:getName())
        local _, _, rootToDirectionNodeDistance = localToLocal(AIDriverUtil.getDirectionNode(vehicle), implement.object.rootNode, 0, 0, 0)
        self.dFront = math.max(self.dFront, implement.object.sizeLength / 2 + implement.object.lengthOffset - rootToDirectionNodeDistance + (buffer or 0))
        self.dRear = math.max(self.dRear, implement.object.sizeLength / 2 - implement.object.lengthOffset + rootToDirectionNodeDistance + (buffer or 0))
        self.dLeft = math.max(self.dLeft, implement.object.sizeWidth / 2)
        self.dRight = math.max(self.dRight, implement.object.sizeWidth / 2)
    end
    --courseplay.debugVehicle(7, vehicle, 'Size: dFront %.1f, dRear %.1f, dLeft = %.1f, dRight = %.1f',
    --        self.dFront, self.dRear, self.dLeft, self.dRight)
end

--- Field info for pathfinding
---@class PathfinderUtil.FieldData
PathfinderUtil.FieldData = CpObject()

function PathfinderUtil.FieldData:init(fieldNum)
    if not fieldNum or fieldNum == 0 then
        -- do not restrict search to the field when none given
        self.minX, self.maxX, self.minY, self.maxY, self.minZ, self.maxZ =
            -math.huge, math.huge, -math.huge, math.huge, -math.huge, math.huge
        return
    end
    self.minX, self.maxX, self.minZ, self.maxZ = math.huge, -math.huge, math.huge, -math.huge
    if courseplay.fields.fieldData[fieldNum] then
        for _, point in ipairs(courseplay.fields.fieldData[fieldNum].points) do
            if ( point.cx < self.minX ) then self.minX = point.cx end
            if ( point.cz < self.minZ ) then self.minZ = point.cz end
            if ( point.cx > self.maxX ) then self.maxX = point.cx end
            if ( point.cz > self.maxZ ) then self.maxZ = point.cz end
        end
    end
    self.maxY = -self.minZ + 20
    self.minY = -self.maxZ - 20
    self.maxX = self.maxX + 20
    self.minX = self.minX - 20
end

--- Pathfinder context
---@class PathfinderUtil.Context
PathfinderUtil.Context = CpObject()
function PathfinderUtil.Context:init(vehicleData, fieldData)
    self.vehicleData = vehicleData
    self.fieldData = fieldData
end

--- Calculate the four corners of a rectangle around a node (for example the area covered by a vehicle)
function PathfinderUtil.getCollisionData(node, vehicleData)
    local x, y, z
    local corners = {}
    x, y, z = localToWorld(node, - vehicleData.dRight, 0, - vehicleData.dRear)
    table.insert(corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, - vehicleData.dRight, 0, vehicleData.dFront)
    table.insert(corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, vehicleData.dLeft, 0, vehicleData.dFront)
    table.insert(corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, vehicleData.dLeft, 0, - vehicleData.dRear)
    table.insert(corners, {x = x, y = y, z = z})
    x, y, z = localToWorld(node, 0, 0, 0)
    local center = {x = x, y = y, z = z}
    return {name = vehicleData.name, center = center, corners = corners}
end

--- Find all other vehicles and add them to our list of vehicles to avoid. Must be called before each pathfinding to
--- have the current position of the vehicles.
function PathfinderUtil.setUpVehicleCollisionData(myVehicle)
    PathfinderUtil.vehicleCollisionData = {}
    local myRootVehicle = myVehicle and myVehicle:getRootVehicle() or nil
    for _, vehicle in pairs(g_currentMission.vehicles) do
        if vehicle:getRootVehicle() ~= myRootVehicle and vehicle.rootNode and vehicle.sizeWidth and vehicle.sizeLength then
            courseplay.debugVehicle(14, myVehicle, 'othervehicle %s, otherroot %s, myroot %s', vehicle:getName(), vehicle:getRootVehicle():getName(), tostring(myRootVehicle))
            table.insert(PathfinderUtil.vehicleCollisionData, PathfinderUtil.getCollisionData(vehicle.rootNode, PathfinderUtil.VehicleData(vehicle)))
        end
    end
end

function PathfinderUtil.findCollidingVehicles(myCollisionData)
    if not PathfinderUtil.vehicleCollisionData then return false end
    for _, collisionData in pairs(PathfinderUtil.vehicleCollisionData) do
        if PathfinderUtil.doRectanglesOverlap(myCollisionData.corners, collisionData.corners) then
            -- courseplay.debugFormat(7, 'x = %.1f, z = %.1f, %s', myCollisionData.center.x, myCollisionData.center.z, collisionData.name)
            return true, collisionData.name
        end
    end
    return false
end

function PathfinderUtil.findCollidingShapes(myCollisionData, yRot, vehicleData)
    local center = myCollisionData.center
    local collidingShapes = overlapBox(
            center.x, center.y + 1, center.z,
            0, yRot, 0,
            vehicleData.dRight + vehicleData.dLeft, 1, vehicleData.dFront + vehicleData.dRear,
            'collisionCallback', PathfinderUtil, AIVehicleUtil.COLLISION_MASK, true, true, true)
    if collidingShapes > 0 then
        --courseplay.debugFormat(7, 'x = %.1f, z = %.1f, %d', center.x, center.z, collidingShapes)
    end
    return collidingShapes
end

function PathfinderUtil.collisionCallback(self, transformId)
    local object = g_currentMission:getNodeObject(transformId)
    if object then
        print(getName(object))
    end
end

function PathfinderUtil.hasFruit(x, z, length, width)
    local fruitsToIgnore = {13, 14} -- GRASS, DRYGRASS
    for _, fruitType in ipairs(g_fruitTypeManager.fruitTypes) do
        local ignoreThis = false
        for _, fruitToIgnore in ipairs(fruitsToIgnore) do
            if fruitType.index == fruitToIgnore then
                ignoreThis = true
                break
            end
        end
        if not ignoreThis then
            local fruitValue, _, _, _ = FSDensityMapUtil.getFruitArea(fruitType.index, x - width / 2, z - length / 2, x + width / 2, z, x, z + length / 2, nil, false)
            if fruitValue > 0 then
                return true, fruitValue, g_fruitTypeManager:getFruitTypeByIndex(fruitType.index).name
            end
        end
    end
    return false
end


--- This is a simplified implementation of the Separating Axis Test, based on Stephan Schloesser's code in AutoDrive.
--- The implementation assumes that a and b are rectangles (not any polygon)
--- We use this during the pathfinding to drive around other vehicles
function PathfinderUtil.doRectanglesOverlap(a, b)

    if math.abs(a[1].x - b[1].x )> 50 then return false end

    for _, rectangle in pairs({a, b}) do

        -- leverage the fact that rectangles have parallel edges, only need to check the first two
        for i = 1, 3 do
            --grab 2 vertices to create an edge
            local p1 = rectangle[i]
            local p2 = rectangle[i + 1]

            -- find the line perpendicular to this edge
            local normal = {x = p2.z - p1.z, z = p1.x - p2.x}

            local minA = math.huge
            local maxA = -math.huge

            -- for each vertex in the first shape, project it onto the line perpendicular to the edge
            -- and keep track of the min and max of these values
            for _, corner in pairs(a) do
                local projected = normal.x * corner.x + normal.z * corner.z
                if projected < minA then
                    minA = projected
                end
                if projected > maxA then
                    maxA = projected
                end
            end

            --for each vertex in the second shape, project it onto the line perpendicular to the edge
            --and keep track of the min and max of these values
            local minB = math.huge
            local maxB = -math.huge
            for _, corner in pairs(b) do
                local projected = normal.x * corner.x + normal.z * corner.z
                if projected < minB then
                    minB = projected
                end
                if projected > maxB then
                    maxB = projected
                end
            end
            -- if there is no overlap between the projections, the edge we are looking at separates the two
            -- rectangles, and we know there is no overlap
            if maxA < minB or maxB < minA then
                return false
            end
        end
    end
    return true
end

--- Calculate penalty for this node. The penalty will be added to the cost of the node. This allows for
--- obstacle avoidance or forcing the search to remain in certain areas.
---@param node State3D
function PathfinderUtil.getNodePenalty(node)
    -- tweak these two parameters to set up how far the path will be from the field or fruit boundary
    -- size of the area to check for field/fruit
    local areaSize = 3
    -- minimum ratio of the area checked must be on field/clear of fruit
    local minRequiredAreaRatio = 0.8
    local penalty = 0
    local isField, area, totalArea = courseplay:isField(node.x, -node.y, areaSize, areaSize)
    if area / totalArea < minRequiredAreaRatio then
        penalty = penalty + 5
    end
    if isField then
        local hasFruit, fruitValue = PathfinderUtil.hasFruit(node.x, -node.y, areaSize, areaSize)
        penalty = penalty + (hasFruit and (10 + fruitValue / 2) or 0)
    end
    return penalty
end

--- Check if node is valid: would we collide with another vehicle here?
---@param node State3D
---@param userData PathfinderUtil.Context
function PathfinderUtil.isValidNode(node, context)
    if node.x < context.fieldData.minX or node.x > context.fieldData.maxX or node.y < context.fieldData.minY or node.y > context.fieldData.maxY then
        --courseplay.debugFormat(7, '%.1f/%.1f is out of field (%.1f, %.1f - %.1f, %.1f', node.x, node.y,
          --  context.fieldData.minX, context.fieldData.minY, context.fieldData.maxX, context.fieldData.maxY)
        return false
    end
    if not PathfinderUtil.helperNode then
        PathfinderUtil.helperNode = courseplay.createNode('pathfinderHelper', node.x, -node.y, 0)
    end
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, node.x, 0, -node.y);
    setTranslation(PathfinderUtil.helperNode, node.x, y, -node.y)
    local heading = context.reverseHeading and courseGenerator.toCpAngle(node:getReverseHeading()) or courseGenerator.toCpAngle(node.t)
    setRotation(PathfinderUtil.helperNode, 0, heading, 0)
    local myCollisionData = PathfinderUtil.getCollisionData(PathfinderUtil.helperNode, context.vehicleData, 'me')
    local _, _, yRot = PathfinderUtil.getNodePositionAndDirection(PathfinderUtil.helperNode)
    local collidingShapes = PathfinderUtil.findCollidingShapes(myCollisionData, yRot, context.vehicleData)
    return (not PathfinderUtil.findCollidingVehicles(myCollisionData) and collidingShapes == 0)
end

--- Interface function to start the pathfinder
---@param start State3D start node
---@param goal State3D goal node
---@param context PathfinderUtil.Context
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfinding(start, goal, context, allowReverse)
    local pathfinder = HybridAStarWithAStarInTheMiddle(20)
    local done, path = pathfinder:start(start, goal, context.vehicleData.turnRadius, context, allowReverse, PathfinderUtil.getNodePenalty, PathfinderUtil.isValidNode)
    return pathfinder, done, path
end

function PathfinderUtil.getNodePositionAndDirection(node, sideOffset)
    local x, _, z = localToWorld(node, sideOffset or 0, 0, 0)
    local lx, _, lz = localDirectionToWorld(node, 0, 0, 1)
    local yRot = math.atan2(lx, lz)
    return x, z, yRot
end

--- Interface function to start the pathfinder in the game
---@param vehicle table, will be used as the start location/heading, turn radius and size
---@param goalWaypoint Waypoint The destination waypoint (x, z, angle)
---@param allowReverse boolean allow reverse driving
---@param fieldNum number if other than 0 or nil the pathfinding is restricted to the given field and its vicinity
function PathfinderUtil.startPathfindingFromVehicleToWaypoint(vehicle, goalWaypoint, allowReverse, fieldNum)
    local x, z, yRot = PathfinderUtil.getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    local goal = State3D(goalWaypoint.x, -goalWaypoint.z, courseGenerator.fromCpAngleDeg(goalWaypoint.angle))
    PathfinderUtil.setUpVehicleCollisionData(vehicle)
    local context = PathfinderUtil.Context(PathfinderUtil.VehicleData(vehicle, true, 1), PathfinderUtil.FieldData(fieldNum))
    return PathfinderUtil.startPathfinding(start, goal, context, allowReverse)
end

--- Interface function to start the pathfinder in the game. The goal is a point at sideOffset meters from the goal node
--- (sideOffset > 0 is left)
---@param vehicle table, will be used as the start location/heading, turn radius and size
---@param goalNode table The goal node
---@param sideOffset number side offset of the goal from the goal node
---@param allowReverse boolean allow reverse driving
---@param fieldNum number if other than 0 or nil the pathfinding is restricted to the given field and its vicinity
function PathfinderUtil.startPathfindingFromVehicleToNode(vehicle, goalNode, sideOffset, allowReverse, fieldNum)
    local x, z, yRot = PathfinderUtil.getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    x, z, yRot = PathfinderUtil.getNodePositionAndDirection(goalNode, sideOffset)
    local goal = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    PathfinderUtil.setUpVehicleCollisionData(vehicle)
    local context = PathfinderUtil.Context(PathfinderUtil.VehicleData(vehicle, true, 1), PathfinderUtil.FieldData(fieldNum))
    return PathfinderUtil.startPathfinding(start, goal, context, allowReverse)
end

function PathfinderUtil.showNodes(pathfinder)
    if pathfinder and pathfinder.nodes then
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pathfinder.startNode.x, 0, -pathfinder.startNode.y)
        cpDebug:drawLineRGB(pathfinder.startNode.x, y + 4, -pathfinder.startNode.y, 0, 255, 0, pathfinder.goalNode.x, y + 4, -pathfinder.goalNode.y)
        y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pathfinder.goalNode.x, 0, -pathfinder.goalNode.y)
        for _, row in pairs(pathfinder.nodes.nodes) do
            for _, column in pairs(row) do
                for _, cell in pairs(column) do
                    local range = pathfinder.nodes.highestCost - pathfinder.nodes.lowestCost
                    local color = (cell.cost - pathfinder.nodes.lowestCost) * 250 / range
                    local r, g, b
                    if cell:isClosed() or true then
                        r, g, b = 100 + color, 250 - color, 0
                    else
                        r, g, b = cell.cost *3, 80, 0
                    end
                    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cell.x, 0, -cell.y)
                    if cell.pred then
                        cpDebug:drawLineRGB(cell.x, y + 1, -cell.y, r, g, b, cell.pred.x, y + 1, -cell.pred.y)
                    end
                end
            end
        end
    end
end