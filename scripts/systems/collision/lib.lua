-- Contains helper functions for the collision system.
local f = {}
-- elements of list: {{x=x, y=y, minx=x, miny=y, maxx=x, maxy=y, }}

--- Get a circle around an entity. Stays valid when polygon rotates around origin (trivially true)
f.circle_around = function(entity, list)
    local poly = entity.collision.polygon
    local x, y = entity.position.x, entity.position.y
    local radius = 0

    for k, v in ipairs(poly) do
        local xz = v.x
        local yz = v.y
        radius = math.max(radius, xz * xz + yz * yz)
    end
    list[entity] = math.sqrt(radius)
    return x, y, math.sqrt(radius)
end

--- assume: has at least one point. Important: does NOT auto-update when polygon rotates.
f.aabb_around = function(entity, list)
    local poly = {}
    for k, v in ipairs(entity.collision.polygon) do
        poly[#poly + 1] = core.rotate_point(v, entity.position.rotation)
    end
    local x, y = entity.position.x, entity.position.y
    local maxx = poly[1].x
    local minx = poly[1].x
    local maxy = poly[1].y
    local miny = poly[1].y
    for k, v in ipairs(poly) do
        maxx = math.max(maxx, v.x)
        minx = math.min(minx, v.x)
        maxy = math.max(maxy, v.y)
        miny = math.min(miny, v.y)
    end
    list[entity] = { minx = minx, miny = miny, maxx = maxx, maxy = maxy }
    return x, y, minx, miny, maxx, maxy
end


local rules = {}
f.add_rule = function(type1, type2, func)
    rules[type1] = rules[type1] or {}
    rules[type1][type2] = func
end
f.clear_rules = function()
    rules = {}
end
f.rem_rule = function(type1, type2)
    if rules[type1] then
        rules[type1][type2] = nil
    end
end

f.check_rule = function(entity1, entity2)
    if rules[entity1.collision.type] and rules[entity1.collision.type][entity2.collision.type] then
        return true
    end
    if rules[entity2.collision.type] and rules[entity2.collision.type][entity1.collision.type] then
        return true
    end
    return false
end

f.execute_if_rule = function(entity1, entity2, prev, collision_type, collision_info)
    local f = rules[entity1.collision.type][entity2.collision.type]
    if f then
        return f(entity1, entity2, prev, collision_type, collision_info)
    end
    local f = rules[entity2.collision.type][entity1.collision.type]
    if f then
        return f(entity2, entity1, prev, collision_type, collision_info)
    end
    print("Warning: No collision function found!")
end

local function point_in_polygon(polygon, point, position, position2)
    local odd = false
    local prev = #polygon

    local y = point.y + position2.y - position.y
    local x = point.x + position2.x - position.x
    for k, v in ipairs(polygon) do
        local w = polygon[prev]
        if (v.y < y and w.y >= y) or (w.y < y and v.y >= y) then
            if x < ((w.x - v.x) * (y - v.y)) / ((w.y - v.y)) + v.x then
                odd = not odd
            end
        end
        prev = k
    end
    return odd, point
end


local function segmentIntersects(x1, y1, x2, y2, x3, y3, x4, y4)
    local d = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    local Ua_n = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))
    local Ub_n = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))
    if d == 0 then

        return false
    end
    local Ua = Ua_n / d
    local Ub = Ub_n / d
    if Ua >= 0 and Ua <= 1 and Ub >= 0 and Ub <= 1 then
        return true
    end
    return false
end

f.segmentIntersects = segmentIntersects
local function line_in_polygon(polygon, start, finish, position, position2)
    local old = polygon[#polygon]

    for k, v in ipairs(polygon) do
        if segmentIntersects(v.x + position.x, v.y + position.y,
            old.x + position.x, old.y + position.y,
            start.x + position2.x, start.y + position2.y,
            finish.x + position2.x, finish.y + position2.y) then
            return true, v.x, v.y, old.x, old.y
        end
        old = v
    end
    return false
end

f.line_in_polygon = line_in_polygon

--- Checks if two polygons intersect
-- @param polygon2 The polygon that won't be resolved for this collision
-- @param polygon The moving polygon that possibly needs resolving in this
-- collision
-- @param position2
-- @param position
-- @return false if there is no intersection, 1 if there is an intersection of
-- a point from polygon 1 against polygon 2, and 2 if there is a point
-- intersection from polygon 2 against polygon 1. Along with that, it also
-- returns an object containing type 1 for a point intersect or type 2 for
-- a line intersect. For type 1, it returns the point and for type 2, returns
-- the line.
f.polygon_in_polygon = function(polygon2, polygon, position2, position)

    local old = polygon2[#polygon2]
    local in_poly, point = point_in_polygon(polygon, old, position, position2)
    if in_poly then
        return 1, { type = 1, point = point }
    end

    for k, v in ipairs(polygon2) do
        local in_poly, point = point_in_polygon(polygon, v, position, position2)
        if in_poly then
            return 1, { type = 1, point = point }
        end
        local intersects, x0, y0, x1, y1 = line_in_polygon(polygon, old, v, position, position2)
        if intersects then
            return 2, { type = 2, x0 = x0, y0 = y0, x1 = x1, y1 = y1 }
        end
        old = v
    end
    for k, v in ipairs(polygon) do
        local in_poly, point = point_in_polygon(polygon2, v, position2, position)
        if in_poly then
            return 2, {type = 1, point = point}
        end
    end
    return false
end


f.rotate_poly = function(entity)
    local poly = {}
    for k, v in ipairs(entity.collision.polygon) do
        poly[#poly + 1] = core.rotate_point(v, entity.position.rotation)
    end
    return poly
end

f.trivial_solve = function(entity1, _, prev)
    entity1.position = prev
end

--- Solves a collision using normal vectors and correction for direction.
--
-- <h3>Principle of operation</h3>
-- <p>
-- This solver tries to accurately direct an entity into a direction that
-- would be logical according to the laws of physics. It first takes either
-- the normal vector for collision onto a line, or the  vector for a
-- collision onto a point, with entity1 having a line on that collsion point.
-- There is always one of these that match, if either one could match
-- using a simple resolve, an assumption is made and solved "however".
-- </p>
-- <p>
-- When it has this vector, it takes a 90 degree angle of that and projects
-- the movement vector onto that vector. It then tries to move entity1
-- again, using that vector.
-- </p>
--
-- @param entity1 The moving entity that initiated the collision, and for
-- which, a collision needs to be solved
-- @param entity2 The stationary entity or entity which is not affected by this solve
-- @param prev The current location of entity1
-- @param collision_type The return value of <code>f.polygon_in_polygon</code>.
-- It is either 1 for a point-to-line collision, or 2 for a line-to-point collision.
--
-- @see f.trivial_solve
-- @see f.polygon_in_polygon
f.normal_solve = function(entity1, entity2, prev, collision_type, collision_info)
    -- Move the minimum not to collide.
    if collision_info.type == 1 then
        -- Point collision

        if collision_type == 1 then
            -- Find the line on entity1, return the direction of that line.
        else
            -- Find the line on entity2, return the direction of that line.
        end
    else
        -- We have a line, just return a 90 degree vector of its normal vector.
    end

    -- Project the movement vector onto the collision line vector and move along
    -- the resulting vector.
end


return f
