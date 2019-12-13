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

---@param name string name of the vehicle
---@param turnRadius number turning radius of the vehicle
---@param dFront number distance of the vehicle's front from the node
---@param dRear number distance of the vehicle's rear from the node
---@param dLeft number distance of the vehicle's left side from the node
---@param dRight number distance of the vehicle's right side from the node
function PathfinderUtil.VehicleData:init(name, turnRadius, dFront, dRear, dLeft, dRight)
    self.name = name
    self.turnRadius = turnRadius
    self.dFront = dFront
    self.dRear = dRear
    self.dLeft = dLeft
    self.dRight = dRight
end

--- Field info for pathfinding
---@class PathfinderUtil.FieldData
PathfinderUtil.FieldData = CpObject()

function PathfinderUtil.FieldData:init(fieldNum)
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
    return {name = vehicleData.name, corners = corners}
end

function PathfinderUtil.findCollidingVehicles(myCollisionData)
    if not PathfinderUtil.vehicleCollisionData then return false end
    for _, collisionData in pairs(PathfinderUtil.vehicleCollisionData) do
        if PathfinderUtil.doRectanglesOverlap(myCollisionData.corners, collisionData.corners) then
            return true, collisionData.name
        end
    end
    return false
end

function PathfinderUtil.findCollidingShapes(myCollisionData, yRot, vehicleData)
    local rearLeftCorner = myCollisionData.corners[4]
    local collidingShapes = overlapBox(
            rearLeftCorner.x, rearLeftCorner.y + 1, rearLeftCorner.z,
            0, yRot, 0,
            vehicleData.dRight + vehicleData.dLeft, 1, vehicleData.dFront + vehicleData.dRear,
            "dummy", nil, AIVehicleUtil.COLLISION_MASK, true, true, true)
    return 0
end

--- Find all other vehicles and add them to our list of vehicles to avoid. Must be called before each pathfinding to 
--- have the current position of the vehicles.
function PathfinderUtil.setUpVehicleCollisionData(myVehicle)
    PathfinderUtil.vehicleCollisionData = {}
    local myRootVehicle = myVehicle and myVehicle:getRootVehicle() or nil
    for _, vehicle in pairs(g_currentMission.vehicles) do
        if vehicle:getRootVehicle() ~= myRootVehicle and vehicle.rootNode and vehicle.sizeWidth and vehicle.sizeLength then
            courseplay.debugVehicle(14, myVehicle, 'othervehicle %s, otherroot %s, myroot %s', vehicle:getName(), vehicle:getRootVehicle():getName(), tostring(myRootVehicle))
            table.insert(PathfinderUtil.vehicleCollisionData, PathfinderUtil.getCollisionData(vehicle.rootNode, PathfinderUtil.getVehicleData(vehicle)))
        end
    end
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
        local hasFruit
        hasFruit, _, area, totalArea = courseplay:areaHasFruit(node.x, -node.y, nil, areaSize, areaSize)
        if hasFruit and area / totalArea > 1 - minRequiredAreaRatio then
            penalty = penalty + 50
        end
    end
    return penalty
end

--- Check if node is valid: would we collide with another vehicle here?
---@param node State3D
---@param userData PathfinderUtil.Context
function PathfinderUtil.isValidNode(node, context)
    if node.x < context.fieldData.minX or node.x > context.fieldData.maxX or node.y < context.fieldData.minY or node.y > context.fieldData.maxY then
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

---@return HybridAStar.VehicleData
function PathfinderUtil.getVehicleData(vehicle, buffer)
    local _, _, rootToDirectionNodeDistance = localToLocal(AIDriverUtil.getDirectionNode(vehicle), vehicle.rootNode, 0, 0, 0)
    local turnRadius = vehicle.cp and vehicle.cp.turnDiameter and vehicle.cp.turnDiameter / 2 or 10
    local name = vehicle.getName and vehicle:getName() or 'N/A'
    return PathfinderUtil.VehicleData(
            name,
            turnRadius,
            vehicle.sizeLength / 2 + vehicle.lengthOffset - rootToDirectionNodeDistance + (buffer or 0),
            vehicle.sizeLength / 2 - vehicle.lengthOffset + rootToDirectionNodeDistance + (buffer or 0),
            vehicle.sizeWidth / 2,
            vehicle.sizeWidth / 2
    )
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
---@param vehicle  vehicle, will be used as the start location/heading, turn radius and size
---@param goalWaypoint Waypoint The destination waypoint (x, z, angle)
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfindingFromVehicleToWaypoint(vehicle, goalWaypoint, allowReverse)
    local x, z, yRot = PathfinderUtil.getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    local goal = State3D(goalWaypoint.x, -goalWaypoint.z, courseGenerator.fromCpAngleDeg(goalWaypoint.angle))
    PathfinderUtil.setUpVehicleCollisionData(vehicle)
    local fieldNum = courseplay.fields:onWhichFieldAmI(vehicle)
    local context = PathfinderUtil.Context(PathfinderUtil.getVehicleData(vehicle, 1), PathfinderUtil.FieldData(fieldNum))
    return PathfinderUtil.startPathfinding(start, goal, context, allowReverse)
end

--- Interface function to start the pathfinder in the game. The goal is a point at sideOffset meters from the goal node
--- (sideOffset > 0 is left)
---@param vehicle  vehicle, will be used as the start location/heading, turn radius and size
---@param goalNode node The goal node
---@param sideOffset number side offset of the goal from the goal node
---@param allowReverse boolean allow reverse driving
function PathfinderUtil.startPathfindingFromVehicleToNode(vehicle, goalNode, sideOffset, allowReverse)
    local x, z, yRot = PathfinderUtil.getNodePositionAndDirection(AIDriverUtil.getDirectionNode(vehicle))
    local start = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    x, z, yRot = PathfinderUtil.getNodePositionAndDirection(goalNode, sideOffset)
    local goal = State3D(x, -z, courseGenerator.fromCpAngle(yRot))
    PathfinderUtil.setUpVehicleCollisionData(vehicle)
    local fieldNum = courseplay.fields:onWhichFieldAmI(vehicle)
    local context = PathfinderUtil.Context(PathfinderUtil.getVehicleData(vehicle, 1), PathfinderUtil.FieldData(fieldNum))
    return PathfinderUtil.startPathfinding(start, goal, context, allowReverse)
end